#include "opendoge_deploy/safety.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace opendoge
{

const char * stateName(RuntimeState state)
{
  switch (state) {
    case RuntimeState::WaitFeedback:
      return "wait_feedback";
    case RuntimeState::Ready:
      return "ready";
    case RuntimeState::EnteringPosition:
      return "entering_position";
    case RuntimeState::ActivePC:
      return "active_pc";
    case RuntimeState::ActiveRL:
      return "active_rl";
    case RuntimeState::LowGainTest:
      return "low_gain_test";
    case RuntimeState::DampingFault:
      return "damping_fault";
    case RuntimeState::StandingUp:
      return "standing_up";
  }
  return "unknown";
}

namespace
{

std::string hexValue(std::uint64_t value, int width = 0)
{
  std::ostringstream ss;
  ss << "0x" << std::uppercase << std::hex;
  if (width > 0) {
    ss << std::setw(width) << std::setfill('0');
  }
  ss << value;
  return ss.str();
}

std::string describeBits(std::uint32_t value)
{
  if (value == 0) {
    return "none";
  }
  std::ostringstream ss;
  bool first = true;
  for (int bit = 0; bit < 32; ++bit) {
    if ((value & (1u << bit)) == 0) {
      continue;
    }
    if (!first) {
      ss << ",";
    }
    ss << "bit" << bit;
    first = false;
  }
  return ss.str();
}

}  // namespace

bool safetyFault(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointMap, kNumJoints> & joints,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const DeployConfig & config,
  RuntimeState runtime_state,
  const std::array<double, kNumJoints> & logical_target,
  const ImuSample & imu,
  std::array<JointSafetyState, kNumJoints> & safety_state,
  double now_s,
  double standup_start_s,
  std::string & reason)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    // ── existing checks ──
    if (!states[i].received) {
      reason = joints[i].name + ": missing feedback";
      return true;
    }
    if (now_s - states[i].last_feedback_s > config.state_timeout_s) {
      reason = joints[i].name + ": feedback timeout";
      return true;
    }
    if (states[i].fault != 0) {
      reason = joints[i].name + ": status fault bits=" +
        hexValue(states[i].fault, 2) + " [" + describeBits(states[i].fault) + "]";
      return true;
    }
    if (states[i].param_fault != 0) {
      reason = joints[i].name + ": faultSta=" +
        hexValue(states[i].param_fault, 8) + " [" + describeBits(states[i].param_fault) + "]";
      return true;
    }
    if (states[i].temperature >= config.over_temperature_c) {
      reason = joints[i].name + ": over temperature";
      return true;
    }
    // Early temperature warning (once per joint per threshold crossing)
    static std::array<bool, kNumJoints> temp_warned{};
    if (states[i].temperature >= config.temp_warn_c &&
        states[i].temperature < config.over_temperature_c &&
        !temp_warned[i]) {
      std::cerr << "Warning: " << joints[i].name << " temperature "
                << states[i].temperature << " C (limit " << config.over_temperature_c << ")\n";
      temp_warned[i] = true;
    }
    if (states[i].temperature < config.temp_warn_c - 5.0) {
      temp_warned[i] = false;  // reset when temperature drops
    }

    // ── sustained torque monitoring ──
    // Convert motor-side torque to joint-side via reduction ratio
    const double max_torque_val = calibration[i].max_torque > 0.0
      ? calibration[i].max_torque : config.torque_threshold;
    const double joint_torque = states[i].torque * calibration[i].reduction;
    const double abs_torque = std::abs(joint_torque);
    if (abs_torque > max_torque_val) {
      if (safety_state[i].torque_exceeded_since_s == 0.0) {
        safety_state[i].torque_exceeded_since_s = now_s;
      } else if (now_s - safety_state[i].torque_exceeded_since_s > config.torque_timeout_s) {
        reason = joints[i].name + ": torque " + std::to_string(abs_torque)
          + " Nm > " + std::to_string(max_torque_val) + " Nm for "
          + std::to_string(now_s - safety_state[i].torque_exceeded_since_s) + "s";
        return true;
      }
    } else {
      safety_state[i].torque_exceeded_since_s = 0.0;
    }

    // ── sustained joint tracking error (only when sending targets) ──
    if (runtime_state == RuntimeState::ActivePC
        || runtime_state == RuntimeState::ActiveRL
        || runtime_state == RuntimeState::EnteringPosition
        || runtime_state == RuntimeState::LowGainTest
        || runtime_state == RuntimeState::StandingUp) {
      // The stand-up target is a deliberate ramp, so a larger transient error
      // is expected; use the wider standup thresholds there.
      const bool standing_up = runtime_state == RuntimeState::StandingUp;
      const double tracking_threshold = standing_up
        ? config.standup_tracking_error_threshold
        : config.tracking_error_threshold;
      const double tracking_timeout = standing_up
        ? config.standup_tracking_error_timeout_s
        : config.tracking_error_timeout_s;
      const double logical_actual = logicalPosition(states[i].position, calibration[i]);
      const double error = std::abs(logical_target[i] - logical_actual);
      if (error > tracking_threshold) {
        if (safety_state[i].tracking_error_since_s == 0.0) {
          safety_state[i].tracking_error_since_s = now_s;
        } else if (now_s - safety_state[i].tracking_error_since_s > tracking_timeout) {
          reason = joints[i].name + ": tracking error " + std::to_string(error)
            + " rad > " + std::to_string(tracking_threshold) + " rad for "
            + std::to_string(now_s - safety_state[i].tracking_error_since_s) + "s";
          return true;
        }
      } else {
        safety_state[i].tracking_error_since_s = 0.0;
      }
    }
  }

  // ── IMU-based fall detection ──
  static double fall_since_s = 0.0;
  static double fall_imu_invalid_since_s = 0.0;
  // Supervised stand-up: the robot starts from the measured (often prone) pose,
  // so projected_gravity.z ≈ 0 would trip the fall check immediately. This is
  // a TIME WINDOW from entering StandingUp/LowGainTest, not tied to the current
  // state, so it also covers the Stand-up → ActiveRL/PC handover. The old
  // state-tied version re-armed exactly at the handover, so a stand-up jerk
  // (calf snap) could trip the fall check.
  const bool standup_grace =
    standup_start_s > 0.0 &&
    now_s - standup_start_s < config.standup_grace_s;
  // Fall detection only matters while the controller is actively driving the
  // joints. In Ready / DampingFault the motors are only damping, so a tilted
  // or inverted robot on the bench must not trip a fall fault.
  const bool fall_monitored =
    runtime_state == RuntimeState::ActivePC ||
    runtime_state == RuntimeState::ActiveRL ||
    runtime_state == RuntimeState::EnteringPosition ||
    runtime_state == RuntimeState::LowGainTest ||
    runtime_state == RuntimeState::StandingUp;
  if (imu.valid) {
    fall_imu_invalid_since_s = 0.0;
    // projected_gravity.z = -1 when upright (gravity points down along body -z).
    // A fall/tilt makes it move toward 0 (or +1 when upside down), so the
    // threshold is applied on the negative side: z > -threshold.
    if (!fall_monitored || standup_grace) {
      fall_since_s = 0.0;
    } else if (imu.projected_gravity[2] > -config.fall_gravity_z_threshold) {
      if (fall_since_s == 0.0) {
        fall_since_s = now_s;
      } else if (now_s - fall_since_s > config.fall_timeout_s) {
        reason = "fall detected: gravity.z=" + std::to_string(imu.projected_gravity[2])
          + " > " + std::to_string(-config.fall_gravity_z_threshold);
        return true;
      }
    } else {
      fall_since_s = 0.0;
    }
  } else {
    // IMU invalid: track duration; reset fall timer after a gap to avoid
    // stale fall_since_s from before the invalid window
    if (fall_imu_invalid_since_s == 0.0) {
      fall_imu_invalid_since_s = now_s;
    } else if (now_s - fall_imu_invalid_since_s > 0.5) {
      fall_since_s = 0.0;
    }
  }

  return false;
}

}  // namespace opendoge
