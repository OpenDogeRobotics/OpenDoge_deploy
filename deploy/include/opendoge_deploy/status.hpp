#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

struct LoopStats
{
  std::uint64_t control_ticks{0};
  std::uint64_t inference_ticks{0};
  std::uint64_t target_ticks{0};
  std::uint64_t missed_control_deadlines{0};
  double max_control_late_s{0.0};
  // Ages of the freshest IMU/joint samples used by the last policy
  // observation (after the pre-inference refresh). Last-value diagnostics,
  // not per-window counters: resetWindow() deliberately does not clear them.
  double last_obs_imu_age_s{-1.0};
  double last_obs_joint_age_s{-1.0};

  void resetWindow()
  {
    control_ticks = 0;
    inference_ticks = 0;
    target_ticks = 0;
    missed_control_deadlines = 0;
    max_control_late_s = 0.0;
  }
};

std::string escapeJson(const std::string & input);

void waitUntilNextDeadline(double next_deadline_s);

/// Print 1Hz status line to stdout and optionally write JSON snapshot to file.
/// Updates last_can_sent/last_can_received from current can_stats.
void emitStatus(
  RuntimeState runtime_state,
  const OperatorCommand & command,
  const std::string & status_file,
  LoopStats & loop_stats,
  std::uint64_t & last_can_sent,
  std::uint64_t & last_can_received,
  const CanStats & can_stats,
  const std::string & fault_reason,
  bool rl_fallback_active,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<double, kNumJoints> & limited_target,
  const std::array<JointCalibration, kNumJoints> & calib,
  const std::array<JointMap, kNumJoints> & joints,
  const ImuSample & imu,
  double pc_startup_start_s,
  double pc_startup_ramp_s,
  double t);

}  // namespace opendoge
