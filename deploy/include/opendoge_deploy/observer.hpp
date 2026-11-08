#pragma once

#include <array>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

/// Compute adaptive gait phase matching UniLab training.
/// cmd_speed = norm([vx, vy, vyaw]); freq ∈ [1.2, 2.5] Hz; phase wraps at 1.0.
/// dt is the time step between phase advances (1.0 / inference_hz).
double advancePhase(const OperatorCommand & command, double phase, double dt);

/// Build the 48-dim LainLab actor observation:
///   base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
///   + joint_pos_rel(12) + joint_vel_rel(12) + last_action(12) + command(3)
/// Joint terms are expressed in the training (LainLab) sign convention via
/// kJointSign. `default_pos` is in firmware URDF convention.
std::array<double, kObsDim> buildObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu);

/// Legacy 49-dim observation (with feet_phase, firmware sign convention).
/// Debug/regression tooling only — not compatible with the LainLab policy.
std::array<double, 49> buildLegacyObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu,
  double phase);

}  // namespace opendoge
