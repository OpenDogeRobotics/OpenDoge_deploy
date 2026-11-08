#include "opendoge_deploy/observer.hpp"

#include <algorithm>
#include <cmath>

namespace opendoge
{

double advancePhase(const OperatorCommand & command, double phase, double dt)
{
  const double cmd_speed = std::sqrt(
    command.vx * command.vx + command.vy * command.vy + command.yaw_rate * command.yaw_rate);
  const double freq = std::clamp(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5);
  return std::fmod(phase + dt * freq, 1.0);
}

std::array<double, kObsDim> buildObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu)
{
  // 48-dim LainLab actor observation (mjlab velocity task):
  //   0:3   base_lin_vel
  //   3:6   base_ang_vel
  //   6:9   projected_gravity
  //   9:21  joint_pos_rel (training convention)
  //   21:33 joint_vel_rel (training convention)
  //   33:45 last_action (raw policy output, no clamp / no sign flip)
  //   45:48 command (vx, vy, yaw_rate)
  //
  // Joint mapping: logical position/velocity are firmware-URDF convention.
  // kJointSign converts them to the training (LainLab) convention. The default
  // pose is stored in firmware convention, hence the same sign flip.
  std::array<double, kObsDim> obs{};
  std::size_t offset = 0;

  // 0:3 base_lin_vel
  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.linear_velocity[i];
  }
  offset += 3;

  // 3:6 base_ang_vel
  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.angular_velocity[i];
  }
  offset += 3;

  // 6:9 projected_gravity
  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.projected_gravity[i];
  }
  offset += 3;

  // 9:21 joint_pos_rel
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto pos = logicalPosition(states[i].position, calibration[i]);
    obs[offset + i] = kJointSign[i] * (pos - default_pos[i]);
  }
  offset += kNumJoints;

  // 21:33 joint_vel_rel (default velocity is zero)
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] =
      kJointSign[i] * logicalVelocity(states[i].velocity, calibration[i]);
  }
  offset += kNumJoints;

  // 33:45 last_action — raw ONNX output, matching mjlab `last_action`
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = last_action[i];
  }
  offset += kNumJoints;

  // 45:48 command
  obs[offset + 0] = command.vx;
  obs[offset + 1] = command.vy;
  obs[offset + 2] = command.yaw_rate;

  return obs;
}

std::array<double, 49> buildLegacyObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu,
  double phase)
{
  // Legacy 49-dim layout (gyro + gravity + q + dq + action + cmd + phase),
  // retained for regression/debug tooling only. It does NOT match the
  // LainLab 48-dim actor observation and must not be used for inference.
  std::array<double, 49> obs{};
  std::size_t offset = 0;

  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.angular_velocity[i];
  }
  offset += 3;

  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.projected_gravity[i];
  }
  offset += 3;

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = logicalPosition(states[i].position, calibration[i]) - default_pos[i];
  }
  offset += kNumJoints;

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = logicalVelocity(states[i].velocity, calibration[i]);
  }
  offset += kNumJoints;

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = last_action[i];
  }
  offset += kNumJoints;

  obs[offset + 0] = command.vx;
  obs[offset + 1] = command.vy;
  obs[offset + 2] = command.yaw_rate;
  offset += 3;

  obs[offset + 0] = phase;
  obs[offset + 1] = std::fmod(phase + 0.5, 1.0);
  obs[offset + 2] = std::fmod(phase + 0.5, 1.0);
  obs[offset + 3] = phase;

  return obs;
}

}  // namespace opendoge
