#include "opendoge_deploy/controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace opendoge
{
namespace
{

/// Convert a logical (URDF) joint target to the EL05 motor frame and clamp it
/// to the joint's physical range. The clamp is done in logical space but applied
/// to the converted motor value, so a wrong reduction/offset can never push the
/// joint past its hard stops.
double clampMotorTarget(double logical_target, const JointCalibration & calib)
{
  const double motor = motorPosition(logical_target, calib);
  const double lo = std::min(motorPosition(calib.lower, calib), motorPosition(calib.upper, calib));
  const double hi = std::max(motorPosition(calib.lower, calib), motorPosition(calib.upper, calib));
  return std::clamp(motor, lo, hi);
}

}  // namespace

double rateLimit(double desired, double previous, double max_step)
{
  return previous + std::clamp(desired - previous, -max_step, max_step);
}

bool allFeedbackReceived(const std::array<MotorState, kNumJoints> & states)
{
  return std::all_of(states.begin(), states.end(), [](const auto & state) {
    return state.received;
  });
}

void sendDampingBurst(
  El05SocketCan & can,
  const std::array<JointMap, kNumJoints> & joints,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  double safe_kd)
{
  for (int repeat = 0; repeat < 20; ++repeat) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double clamped_kd = std::min(safe_kd, calibration[i].max_kd);
      MotorCommand damp{states[i].position, 0.0, 0.0, 0.0, clamped_kd};
      can.sendMotion(joints[i], damp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void updateStateMachine(
  RuntimeState & runtime_state,
  bool & rl_fallback_active,
  OperatorCommand & command,
  const Options & opt,
  const std::array<MotorState, kNumJoints> & states,
  const DeployConfig & config,
  const ImuSample & imu,
  double t,
  double & pc_startup_start_s,
  double & feedback_wait_start_s,
  double & standup_start_s,
  std::string & fault_reason)
{
  // ── Mode priority: RL beats position control ──
  if (command.rl_inference && command.position_control) {
    command.position_control = false;
  }

  // ── WaitFeedback → Ready ──
  if (runtime_state == RuntimeState::WaitFeedback) {
    if (allFeedbackReceived(states)) {
      runtime_state = RuntimeState::Ready;
      feedback_wait_start_s = 0.0;
    } else if (config.feedback_wait_timeout_s > 0.0) {
      if (feedback_wait_start_s == 0.0) {
        feedback_wait_start_s = t;
      } else if (t - feedback_wait_start_s > config.feedback_wait_timeout_s) {
        fault_reason = "feedback wait timeout after " + std::to_string(t - feedback_wait_start_s) + "s";
        runtime_state = RuntimeState::DampingFault;
        return;
      }
    }
  }

  // ── Ready ↔ LowGainTest ↔ Active transitions ──
  if (runtime_state == RuntimeState::Ready && command.low_gain_mode) {
    runtime_state = RuntimeState::LowGainTest;
  }
  if (runtime_state == RuntimeState::LowGainTest && !command.low_gain_mode) {
    runtime_state = RuntimeState::Ready;
  }

  if (runtime_state == RuntimeState::Ready && command.active && (imu.valid || opt.allow_missing_imu)) {
    // If the robot is far from the default pose (e.g. prone), stand up first
    // with a position ramp and no policy. The policy is out of its training
    // distribution there, and EnteringPosition's deviation check would fault.
    bool far_from_default = false;
    const auto defaults = defaultJointPosition();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double pos = logicalPosition(states[i].position, config.joints[i]);
      if (std::abs(pos - defaults[i]) > config.pc_startup_max_deviation) {
        far_from_default = true;
        break;
      }
    }
    if (far_from_default) {
      runtime_state = RuntimeState::StandingUp;
      standup_start_s = t;
    } else if (command.rl_inference) {
      runtime_state = RuntimeState::ActiveRL;
    } else {
      runtime_state = RuntimeState::EnteringPosition;
      pc_startup_start_s = t;
    }
  }

  // ── EnteringPosition: verify joint positions + check ramp completion ──
  if (runtime_state == RuntimeState::EnteringPosition) {
    if (!command.active) {
      runtime_state = RuntimeState::Ready;
    } else if (command.rl_inference) {
      runtime_state = RuntimeState::ActiveRL;
    } else {
      bool positions_valid = true;
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double pos = logicalPosition(states[i].position, config.joints[i]);
        if (std::abs(pos - defaultJointPosition()[i]) > config.pc_startup_max_deviation) {
          positions_valid = false;
          break;
        }
      }
      if (!positions_valid) {
        fault_reason = "position control startup: joint deviation exceeds limit";
        runtime_state = RuntimeState::DampingFault;
      } else if (t - pc_startup_start_s >= config.pc_startup_ramp_s) {
        runtime_state = RuntimeState::ActivePC;
      }
    }
  }

  // ── StandingUp: ramp from the measured pose to the default pose ──
  if (runtime_state == RuntimeState::StandingUp) {
    if (!command.active) {
      runtime_state = RuntimeState::Ready;
    } else if (t - standup_start_s >=
               config.standup_duration_s + config.standup_settle_s) {
      // The target ramp has reached the default pose and the settle window
      // has elapsed (tracking/torque safety would have faulted otherwise);
      // hand over to the policy/PC.
      runtime_state = command.rl_inference ? RuntimeState::ActiveRL
                                           : RuntimeState::ActivePC;
    }
  }

  // ── ActivePC / ActiveRL → Ready ──
  if ((runtime_state == RuntimeState::ActivePC || runtime_state == RuntimeState::ActiveRL) && !command.active) {
    runtime_state = RuntimeState::Ready;
    rl_fallback_active = false;
  }

  // ── ActivePC ↔ ActiveRL switching ──
  if (runtime_state == RuntimeState::ActivePC && command.rl_inference) {
    runtime_state = RuntimeState::ActiveRL;
    rl_fallback_active = false;
  }
  if (runtime_state == RuntimeState::ActiveRL && !command.rl_inference && command.active) {
    runtime_state = RuntimeState::ActivePC;
    rl_fallback_active = false;
  }
}

void computeMotorCommands(
  std::array<MotorCommand, kNumJoints> & commands,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calib,
  const DeployConfig & config,
  RuntimeState runtime_state,
  const std::array<double, kNumJoints> & limited_target,
  const std::array<double, kNumJoints> & default_pos,
  double t,
  double pc_startup_start_s,
  double standup_start_s)
{
  const bool is_active = runtime_state == RuntimeState::ActivePC
                      || runtime_state == RuntimeState::ActiveRL;
  const bool is_ramping = runtime_state == RuntimeState::EnteringPosition;
  const bool low_gain = runtime_state == RuntimeState::LowGainTest;
  const bool is_standing_up = runtime_state == RuntimeState::StandingUp;
  (void)default_pos;  // target comes from limited_target in every non-damping mode

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint_cfg = calib[i];
    if (!is_active && !is_ramping && !low_gain && !is_standing_up) {
      // 阻尼模式：kp=0, kd=safe_kd (clamped to per-joint max)
      commands[i] = {states[i].position, 0.0, 0.0, 0.0, std::min(config.safe_kd, joint_cfg.max_kd)};
      continue;
    }

    double effective_kp;
    double effective_kd;
    if (is_ramping) {
      // 斜坡：kp/kd 从阻尼值平滑过渡到满 PD 值
      const double ramp_elapsed = t - pc_startup_start_s;
      const double ramp_frac = std::min(
        ramp_elapsed / std::max(config.pc_startup_ramp_s, 1.0e-6), 1.0);
      effective_kp = ramp_frac * std::min(config.kp, joint_cfg.max_kp);
      effective_kd = config.safe_kd + ramp_frac *
        (std::min(config.kd, joint_cfg.max_kd) - config.safe_kd);
    } else if (low_gain) {
      // Reduced gains + hold default standing pose for safety.
      // limited_target is ramped by updateTargets() from the pose captured on
      // entry, so this is no longer an instantaneous step.
      effective_kp = std::min(config.kp * 0.3, joint_cfg.max_kp);
      effective_kd = std::min(config.kd * 0.3, joint_cfg.max_kd);
    } else if (is_standing_up) {
      // Stand-up from prone: start at 60% stiffness and ramp to full over
      // standup_duration_s. A 30% start made the calf lag under gravity
      // (tracking error 0.62 rad > 0.5) and tripped the safety check.
      const double frac = std::min(
        (t - standup_start_s) / std::max(config.standup_duration_s, 1.0e-6), 1.0);
      effective_kp = std::min(config.kp * (0.6 + 0.4 * frac), joint_cfg.max_kp);
      effective_kd = std::min(
        config.safe_kd + frac * (config.kd - config.safe_kd), joint_cfg.max_kd);
    } else {
      effective_kp = std::min(config.kp, joint_cfg.max_kp);
      effective_kd = std::min(config.kd, joint_cfg.max_kd);
    }

    // Training actuator gains are joint-side (LainLab BuiltinPositionActuator-
    // Cfg on the joint: stiffness=12, damping=0.5, calf effort_limit=9 =
    // EL05 6 Nm * reduction 1.5). The EL05 Kp/Kd act on the motor output; with
    // an external reduction r, joint stiffness/damping scale by r^2:
    //   kp_joint = kp_motor * r^2,  kd_joint = kd_motor * r^2
    // Sending the joint-side gains directly made the calf 2.25x too stiff and
    // too damped (legs could not lift at small commands).
    const double gain_scale =
      1.0 / (joint_cfg.reduction * joint_cfg.reduction);
    // limited_target is ALWAYS in logical (URDF) space. Convert to the EL05
    // motor frame and clamp. The old active branch sent logical radians
    // directly as motor radians, ignoring direction/offset/reduction.
    commands[i] = {
      clampMotorTarget(limited_target[i], joint_cfg),
      0.0,
      0.0,
      std::min(effective_kp * gain_scale, joint_cfg.max_kp),
      std::min(effective_kd * gain_scale, joint_cfg.max_kd)};
  }
}

void updateTargets(
  std::array<double, kNumJoints> & logical_target,
  std::array<double, kNumJoints> & limited_target,
  std::array<double, kNumJoints> & last_action,
  const std::array<double, kNumJoints> & action,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<JointCalibration, kNumJoints> & calib,
  const DeployConfig & config,
  RuntimeState runtime_state,
  double t,
  double standup_handover_start_s)
{
  // One-shot blend after a supervised stand-up. The first policy action can
  // be large; without this the ActiveRL branch jumps the target instantly and
  // the calf can snap (large gz spike that trips the fall check). The blend
  // starts at the default pose and ramps to the full policy target.
  const bool handover =
    (runtime_state == RuntimeState::ActiveRL ||
     runtime_state == RuntimeState::ActivePC) &&
    standup_handover_start_s >= 0.0 &&
    t - standup_handover_start_s < config.standup_handover_s;
  const double handover_alpha = handover
    ? std::clamp(
        (t - standup_handover_start_s) / std::max(config.standup_handover_s, 1.0e-6),
        0.0, 1.0)
    : 1.0;

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint_cfg = calib[i];
    // LainLab / mjlab JointPositionAction: processed = raw * scale + default,
    // with no clipping. The raw policy output is also what the actor observes
    // as `last_action`, so store it unmodified.
    last_action[i] = action[i];
    if (runtime_state == RuntimeState::LowGainTest ||
        runtime_state == RuntimeState::StandingUp) {
      logical_target[i] = default_pos[i];
    } else {
      // action is in training convention; map it back to firmware convention.
      logical_target[i] =
        default_pos[i] + kJointSign[i] * last_action[i] * config.action_scale;
    }
    logical_target[i] = std::clamp(logical_target[i], joint_cfg.lower, joint_cfg.upper);
    if (handover) {
      // Blend from the settled default pose to the policy target and keep the
      // rate limiter active even in ActiveRL for this one-shot window.
      const double blended = default_pos[i] + handover_alpha *
        (logical_target[i] - default_pos[i]);
      limited_target[i] = rateLimit(blended, limited_target[i], joint_cfg.max_position_step);
    } else if (runtime_state == RuntimeState::ActiveRL) {
      // RL mode: skip rate_limit (policy trained with instantaneous target changes)
      limited_target[i] = logical_target[i];
    } else {
      // LowGainTest uses a slower slew so the stand-up from the measured pose
      // is a ramp, not a step. main.cpp snapshots limited_target on entry.
      double max_step = joint_cfg.max_position_step;
      if (runtime_state == RuntimeState::LowGainTest) {
        max_step = std::min(max_step, std::max(config.low_gain_max_position_step, 0.0));
      } else if (runtime_state == RuntimeState::StandingUp) {
        max_step = std::min(max_step, std::max(config.standup_max_position_step, 0.0));
      }
      limited_target[i] = rateLimit(logical_target[i], limited_target[i], max_step);
    }
  }
}

}  // namespace opendoge
