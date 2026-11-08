#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace opendoge
{

constexpr std::size_t kNumJoints = 12;

// Single-frame actor observation (LainLab mjlab velocity task, 48 dims):
//   base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
//   + joint_pos_rel(12) + joint_vel_rel(12) + last_action(12) + command(3) = 48
// Observation terms are ordered exactly as the mjlab actor observation group.
constexpr std::size_t kObsDim = 48;

// Training URDF ↔ firmware URDF sign convention.
// The LainLab model mirrors the FR/RR legs (thigh/calf axis 0 -1 0) and the
// RL/RR hips (axis -1 0 0); the firmware URDF uses +axis for every joint.
//   q_train[i]      = kJointSign[i] * q_firmware[i]
//   obs_joint_pos[i] = kJointSign[i] * (q_firmware[i] - default_firmware[i])
//   target_firmware[i] = default_firmware[i]
//                        + kJointSign[i] * action_scale * action_train[i]
// With this map the training default pose becomes the uniform firmware pose
// below: {0, 0.8, -1.6} × 4.
constexpr std::array<double, kNumJoints> kJointSign = {
  +1, +1, +1,   // FL hip/thigh/calf
  +1, -1, -1,   // FR
  -1, +1, +1,   // RL
  -1, -1, -1,   // RR
};

// Default standing pose in firmware URDF convention (LainLab
// OPENDOGE_INIT_STATE mapped through kJointSign: all legs become uniform).
inline std::array<double, kNumJoints> defaultJointPosition()
{
  return {0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6};
}

struct JointMap
{
  std::string name;
  std::string can;
  int motor_id;
};

struct MotorState
{
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double temperature{0.0};
  std::uint16_t fault{0};
  std::uint32_t param_fault{0};
  // Startup self-check fields (read back from the motor):
  //   run_mode: 0x7005, -1 = not read yet
  //   mode:     type-2 CAN-ID bits 22..23 (0 reset / 1 cali / 2 motor running)
  int run_mode{-1};
  std::uint8_t mode{0};
  bool fault_status_received{false};
  // 0x7028 canTimeout read-back (50 us units; 20000 = 1 s, 0 = disabled).
  std::uint32_t can_timeout_units{0};
  bool can_timeout_received{false};
  bool received{false};
  double last_feedback_s{0.0};
  // Per-signal freshness. A type-17 READ_PARAM reply refreshes mechPos/Vel and
  // the joint-wide age, but does NOT refresh the type-2 status frame that
  // carries torque/temperature/fault/mode. Keep the timestamps separate so a
  // dead type-2 path is visible even while q/dq look fresh.
  double last_position_s{0.0};
  double last_velocity_s{0.0};
  double last_status_s{0.0};
  std::uint64_t status_frames{0};
  std::uint64_t param_frames{0};
  double last_gap_s{0.0};
  double max_gap_s{0.0};
};

struct MotorCommand
{
  double q{0.0};
  double dq{0.0};
  double tau{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct BusCanStats
{
  std::uint64_t tx{0};
  std::uint64_t rx{0};
  std::uint64_t read_errors{0};
  std::uint64_t write_errors{0};
  std::uint64_t tx_queue_full{0};
  std::uint64_t error_frames{0};
  std::uint64_t bus_off{0};
  std::uint64_t restarts{0};
  std::uint32_t tec{0};
  std::uint32_t rec{0};
  // "UNKNOWN", "ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF".
  std::string state{"UNKNOWN"};
};

struct CanStats
{
  std::uint64_t frames_sent{0};
  std::uint64_t frames_received{0};
  std::uint64_t read_errors{0};
  std::uint64_t write_errors{0};
  std::uint64_t tx_queue_full{0};
  // Per-joint expected type-2 replies (incremented on sendMotion, excluding
  // fault polls/retries). status_frames / expected_status gives the RX ratio.
  std::array<std::uint64_t, kNumJoints> expected_status{};
  // Bus order is can0..can3.
  std::array<BusCanStats, 4> buses{};
};

/// Per-joint safety state for sustained-condition monitoring.
/// Timers reset when condition clears; a fault is raised only when
/// the condition persists beyond the configured timeout.
struct JointSafetyState
{
  double torque_exceeded_since_s{0.0};
  double tracking_error_since_s{0.0};
};

enum class RuntimeState
{
  WaitFeedback,
  Ready,
  EnteringPosition,
  ActivePC,
  ActiveRL,
  LowGainTest,
  DampingFault,
  StandingUp,
};

/// Human-readable name for RuntimeState enum values (used in status output).
const char * stateName(RuntimeState state);

struct JointCalibration
{
  double direction{1.0};
  double offset{0.0};
  double reduction{1.0};  // motor-to-joint gear ratio (>1: motor spins faster)
  double lower{-12.57};
  double upper{12.57};
  double max_position_step{0.015};
  double max_velocity{20.0};
  double max_torque{3.0};
  double max_kp{50.0};
  double max_kd{5.0};  // Matches EL05 spec: kd ∈ [0, 5]
};

struct DeployConfig
{
  double inference_hz{100.0};
  double target_hz{200.0};
  double control_hz{1000.0};
  // LainLab BuiltinPositionActuatorCfg: stiffness=12.0, damping=0.5
  double kp{12.0};
  double kd{0.5};
  double safe_kd{2.0};
  double action_scale{0.25};
  // base_lin_vel source for the 48-dim actor observation:
  //   "zero"    → zeros (worst case, no state estimate)
  //   "command" → [vx, vy, 0] body-frame command approximation
  //   "imu"     → vx/vy/vz keys from the IMU state file (if the bridge provides them)
  std::string lin_vel_source{"command"};
  double state_timeout_s{0.02};
  // Scan-like feedback recovery: for joints whose last feedback is stale,
  // send a READ_PARAM (mechPos/mechVel) instead of waiting for the next
  // motion-control reply. The type-17 reply refreshes the state and counts
  // as valid feedback.
  double feedback_retry_hz{50.0};
  double over_temperature_c{80.0};
  double fault_poll_hz{10.0};
  double pc_startup_ramp_s{2.0};
  double pc_startup_max_deviation{0.25};
  // LowGainTest target slew limit (rad per target tick). The target starts at
  // the measured pose on entry, so this is the stand-up ramp speed. At 200 Hz,
  // 0.002 rad/tick = 0.4 rad/s (~2.6 s for the calf's 1.03 rad).
  double low_gain_max_position_step{0.002};
  // Stand-up from a prone/far pose (no policy): target slew and total time.
  // At 200 Hz, 0.002 rad/tick = 0.4 rad/s (~2.6 s for the calf's 1.03 rad).
  double standup_max_position_step{0.002};
  double standup_duration_s{4.0};
  // After the ramp reaches the default pose, hold it for this long before
  // handing over to the policy/PC. Gives the legs and IMU time to settle.
  double standup_settle_s{1.0};
  // After the handover, blend the policy target from the default pose to the
  // full action over this duration. Prevents the first large policy action
  // from stepping the calf target and causing a jerk/fall-check spike.
  double standup_handover_s{1.0};
  // During the stand-up ramp the target is intentionally moving, so allow a
  // larger tracking error than the normal 0.5 rad / 0.3 s check.
  double standup_tracking_error_threshold{1.0};
  double standup_tracking_error_timeout_s{0.8};
  // EL05 hardware torque limit written at startup (0 = leave motor default).
  // This is the real output-torque cap; it is RAM-only and reverts on power cycle.
  double torque_limit_nm{0.0};
  // EL05 CAN watchdog (0x7028 canTimeout), in seconds; 0 = leave unchanged.
  // Written as RAM-only at startup: if a motor receives no CAN command for this
  // long (bridge dead, deploy crash, lost COMM_STOP), it resets and stops
  // applying torque. 1.0 s is a good default; do not use < 0.5 s.
  double can_timeout_s{0.0};
  // IMU/heartbeat staleness. 0 disables the check. The bridge writes in-band
  // t=/seq= stamps; a frozen file (rewritten cache, unchanged seq) is caught
  // by age. warn = status/monitor only; fault = DampingFault after debounce.
  double imu_timeout_s{0.0};
  std::string imu_stale_action{"warn"};
  // Observation-side motor feedback staleness (0 disables). Applies to active
  // states; hold = current behaviour, freeze = skip inference this tick,
  // fault = DampingFault. The hard safety timeout stays state_timeout_s.
  double feedback_hold_max_s{0.03};
  std::string stale_action{"hold"};
  // CAN controller error policy: warn|fault. BUS-OFF always faults.
  std::string can_error_action{"warn"};
  // Safety monitoring thresholds
  double torque_threshold{3.0};
  double torque_timeout_s{0.5};
  double tracking_error_threshold{0.5};
  double tracking_error_timeout_s{0.3};
  double command_timeout_s{0.5};
  double fall_gravity_z_threshold{0.3};
  double fall_timeout_s{0.3};
  // During a supervised stand-up the robot starts prone, where
  // projected_gravity.z ≈ 0 would trip the fall check immediately. This is a
  // time window measured from entering StandingUp/LowGainTest (NOT tied to the
  // state), so it also covers the handover to ActiveRL/PC. It should be at
  // least standup_duration_s + standup_settle_s + standup_handover_s.
  double standup_grace_s{6.5};
  // Command smoothing (0 = disabled)
  double command_smoothing_alpha{0.0};
  // WaitFeedback overall timeout (0 = no timeout)
  double feedback_wait_timeout_s{5.0};
  // Early temperature warning threshold (C)
  double temp_warn_c{65.0};
  // IMU consecutive invalid reads before dropping to Ready
  int imu_debounce_count{10};
  std::array<JointCalibration, kNumJoints> joints{};
};

struct OperatorCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool active{false};
  bool estop{false};
  bool position_control{false};
  bool rl_inference{false};
  bool clear_fault{false};
  bool low_gain_mode{false};
};

struct ImuSample
{
  // Body-frame linear velocity (m/s). Optional: only the IMU bridge can fill
  // it (vx/vy/vz keys). Otherwise main.cpp synthesizes it according to
  // DeployConfig::lin_vel_source.
  std::array<double, 3> linear_velocity{0.0, 0.0, 0.0};
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
  std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};
  bool valid{false};
  double last_update_s{0.0};
  // In-band monotonic receipt stamps from the bridge (t=, gyro_t=, quat_t=).
  // -1 means the source file did not provide that stamp (legacy bridge).
  double source_stamp_s{-1.0};
  double gyro_stamp_s{-1.0};
  double quat_stamp_s{-1.0};
  std::uint64_t gyro_seq{0};
  std::uint64_t quat_seq{0};
};

inline std::array<JointMap, kNumJoints> defaultJointMap()
{
  return {{
    {"FL_hip_joint", "can0", 1}, {"FL_thigh_joint", "can0", 2}, {"FL_calf_joint", "can0", 3},
    {"FR_hip_joint", "can1", 4}, {"FR_thigh_joint", "can1", 5}, {"FR_calf_joint", "can1", 6},
    {"RL_hip_joint", "can2", 7}, {"RL_thigh_joint", "can2", 8}, {"RL_calf_joint", "can2", 9},
    {"RR_hip_joint", "can3", 10}, {"RR_thigh_joint", "can3", 11}, {"RR_calf_joint", "can3", 12},
  }};
}

inline std::array<JointCalibration, kNumJoints> defaultJointCalibration()
{
  // Physically reasonable defaults matching OpenDoge URDF constraints.
  // Config file overrides these — these are safety fallbacks only.
  //
  // ⚠️ Field order must match JointCalibration exactly:
  //    {direction, offset, reduction, lower, upper, ...}
  // `reduction` is written explicitly here. When it was omitted, the old
  // {direction, offset, lower, upper} rows silently assigned `lower` to
  // `reduction`, giving hips/thighs negative ratios (-0.785 / -0.26) and
  // driving the rear joints past their hard stops. Keep reduction explicit.
  return {{
    {1.0, 0.0, 1.0, -0.785, 0.26},  {1.0, 0.0, 1.0, -0.785, 1.134}, {1.0, 0.0, 1.0, -2.68, -1.04},
    {1.0, 0.0, 1.0, -0.26, 0.785},  {1.0, 0.0, 1.0, -0.785, 1.134}, {1.0, 0.0, 1.0, -2.68, -1.04},
    {1.0, 0.0, 1.0, -0.785, 0.26},  {1.0, 0.0, 1.0, -0.785, 1.134}, {1.0, 0.0, 1.0, -2.68, -1.04},
    {1.0, 0.0, 1.0, -0.26, 0.785},  {1.0, 0.0, 1.0, -0.785, 1.134}, {1.0, 0.0, 1.0, -2.68, -1.04},
  }};
}

inline double logicalPosition(double motor_position, const JointCalibration & calibration)
{
  return calibration.direction * (motor_position / calibration.reduction - calibration.offset);
}

inline double logicalVelocity(double motor_velocity, const JointCalibration & calibration)
{
  return calibration.direction * motor_velocity / calibration.reduction;
}

inline double motorPosition(double logical_position, const JointCalibration & calibration)
{
  return (calibration.offset + calibration.direction * logical_position) * calibration.reduction;
}

}  // namespace opendoge
