#pragma once

#include <array>
#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

/// Check all safety conditions. Returns true (with reason) if a fault is detected.
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
  std::string & reason);

}  // namespace opendoge
