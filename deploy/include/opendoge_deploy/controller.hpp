#pragma once

#include <array>
#include <string>

#include "opendoge_deploy/cli.hpp"
#include "el05_socketcan.hpp"
#include "opendoge_deploy/types.hpp"

namespace opendoge
{

double rateLimit(double desired, double previous, double max_step);

bool allFeedbackReceived(const std::array<MotorState, kNumJoints> & states);

void sendDampingBurst(
  El05SocketCan & can,
  const std::array<JointMap, kNumJoints> & joints,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  double safe_kd);

/// State-machine update: evaluates operator commands and transitions runtime_state.
/// Mutates runtime_state, rl_fallback_active, command.position_control (priority),
/// pc_startup_start_s, feedback_wait_start_s, and fault_reason on fault.
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
  std::string & fault_reason);

/// Per-tick PD control: computes MotorCommand for each joint based on
/// current runtime_state, limited_target, and config.
/// Returns the number of commands actually sent (or would have been sent in dry-run).
/// On CAN write failure, sets fault_reason and transitions to DampingFault.
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
  double standup_start_s);

/// Target update block (200 Hz): computes logical targets from actions,
/// applies rate limiting and joint limits.
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
  double standup_handover_start_s);

}  // namespace opendoge
