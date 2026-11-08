#include "opendoge_deploy/status.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <unistd.h>

namespace opendoge
{

std::string escapeJson(const std::string & input)
{
  std::string out;
  out.reserve(input.size() + 4);
  for (char c : input) {
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out += "\\u00";
          out += "0123456789abcdef"[static_cast<unsigned char>(c) >> 4];
          out += "0123456789abcdef"[static_cast<unsigned char>(c) & 0x0F];
        } else {
          out += c;
        }
    }
  }
  return out;
}

void waitUntilNextDeadline(double next_deadline_s)
{
  namespace chr = std::chrono;
  const double remaining_s = next_deadline_s - chr::duration<double>(
    chr::steady_clock::now().time_since_epoch()).count();
  if (remaining_s > 0.0003) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  } else if (remaining_s > 0.00005) {
    std::this_thread::yield();
  }
}

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
  double t)
{
  const auto sent_delta = can_stats.frames_sent - last_can_sent;
  const auto recv_delta = can_stats.frames_received - last_can_received;
  // In-band IMU receipt ages (ms). -1 = legacy bridge with no stamp.
  const double imu_age_ms = (imu.valid && imu.source_stamp_s > 0.0)
    ? std::max(0.0, (t - imu.source_stamp_s) * 1000.0) : -1.0;
  const double imu_gyro_age_ms = (imu.valid && imu.gyro_stamp_s > 0.0)
    ? std::max(0.0, (t - imu.gyro_stamp_s) * 1000.0) : -1.0;
  const double imu_quat_age_ms = (imu.valid && imu.quat_stamp_s > 0.0)
    ? std::max(0.0, (t - imu.quat_stamp_s) * 1000.0) : -1.0;
  // Interactive terminal: print a watch-like table. Redirected output keeps
  // the compact one-line format (logs/scripts).
  if (::isatty(STDOUT_FILENO) != 0) {
    std::cout << "state=" << stateName(runtime_state)
              << " active=" << (command.active ? 1 : 0)
              << " pos_ctrl=" << (command.position_control ? 1 : 0)
              << " rl=" << (command.rl_inference ? 1 : 0)
              << " low_gain=" << (command.low_gain_mode ? 1 : 0)
              << " estop=" << (command.estop ? 1 : 0)
              << " imu=" << (imu.valid ? 1 : 0)
              << " imu_age_ms=" << static_cast<long long>(imu_age_ms)
              << " can_tx=" << sent_delta
              << " can_rx=" << recv_delta
              << " err=" << (can_stats.read_errors + can_stats.write_errors)
              << " late_us=" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6)
              << " missed=" << loop_stats.missed_control_deadlines
              << " ramp_pct=" << (runtime_state == RuntimeState::EnteringPosition
                ? static_cast<int>(100.0 * (t - pc_startup_start_s) / pc_startup_ramp_s) : 100);
    if (runtime_state == RuntimeState::DampingFault) {
      std::cout << " fault=\"" << fault_reason << "\"";
    }
    std::cout << "\n";
    std::cout << "  " << std::left << std::setw(16) << "joint"
              << std::right << std::setw(10) << "q"
              << std::setw(10) << "q_cmd"
              << std::setw(9) << "dq"
              << std::setw(9) << "tau"
              << std::setw(9) << "tau_j"
              << std::setw(7) << "temp"
              << std::setw(7) << "fault"
              << std::setw(9) << "age_ms"
              << std::setw(6) << "recv" << "\n";
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double q = logicalPosition(states[i].position, calib[i]);
      const double dq = logicalVelocity(states[i].velocity, calib[i]);
      const double tau = states[i].torque;
      const double tau_j = tau * calib[i].reduction;  // same scaling as the safety check
      const double age_ms = states[i].received
        ? (t - states[i].last_feedback_s) * 1000.0 : -1.0;
      std::cout << "  " << std::left << std::setw(16) << joints[i].name
                << std::right << std::fixed << std::setprecision(4)
                << std::setw(10) << q
                << std::setw(10) << limited_target[i]
                << std::setprecision(3)
                << std::setw(9) << dq
                << std::setw(9) << tau
                << std::setw(9) << tau_j
                << std::setprecision(1)
                << std::setw(7) << states[i].temperature
                << std::setw(7) << states[i].fault
                << std::setw(9) << static_cast<long long>(age_ms)
                << std::setw(6) << (states[i].received ? "yes" : "no") << "\n";
    }
    std::cout << std::defaultfloat << std::setprecision(6);
  } else {
    std::cout << "state=" << stateName(runtime_state)
              << " active_cmd=" << (command.active ? 1 : 0)
              << " pos_ctrl=" << (command.position_control ? 1 : 0)
              << " rl_infer=" << (command.rl_inference ? 1 : 0)
              << " low_gain=" << (command.low_gain_mode ? 1 : 0)
              << " imu=" << (imu.valid ? 1 : 0)
              << " imu_age_ms=" << static_cast<long long>(imu_age_ms)
              << " obs_imu_age_ms="
              << (loop_stats.last_obs_imu_age_s >= 0.0
                ? static_cast<long long>(loop_stats.last_obs_imu_age_s * 1000.0) : -1)
              << " obs_joint_age_ms="
              << (loop_stats.last_obs_joint_age_s >= 0.0
                ? static_cast<long long>(loop_stats.last_obs_joint_age_s * 1000.0) : -1)
              << " ctrl_ticks=" << loop_stats.control_ticks
              << " infer_ticks=" << loop_stats.inference_ticks
              << " target_ticks=" << loop_stats.target_ticks
              << " max_late_us=" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6)
              << " missed_ctrl=" << loop_stats.missed_control_deadlines
              << " can_tx=" << sent_delta
              << " can_rx=" << recv_delta
              << " can_err=" << (can_stats.read_errors + can_stats.write_errors)
              << " ramp_pct=" << (runtime_state == RuntimeState::EnteringPosition
                ? static_cast<int>(100.0 * (t - pc_startup_start_s) / pc_startup_ramp_s) : 100)
              << " rl_fb=" << (rl_fallback_active ? 1 : 0);
    if (runtime_state == RuntimeState::DampingFault) {
      std::cout << " fault=\"" << fault_reason << "\"";
    }
    std::cout << "\n";
  }

  // Write JSON status snapshot for external consumers (web console, etc.)
  if (!status_file.empty()) {
    std::ofstream sf(status_file + ".tmp");
    if (sf) {
      sf << "{";
      sf << "\"t\":" << t << ",";
      sf << "\"state\":\"" << stateName(runtime_state) << "\",";
      sf << "\"active_cmd\":" << (command.active ? "true" : "false") << ",";
      sf << "\"estop\":" << (command.estop ? "true" : "false") << ",";
      sf << "\"position_control\":" << (command.position_control ? "true" : "false") << ",";
      sf << "\"rl_inference\":" << (command.rl_inference ? "true" : "false") << ",";
      sf << "\"low_gain\":" << (command.low_gain_mode ? "true" : "false") << ",";
      sf << "\"imu_valid\":" << (imu.valid ? "true" : "false") << ",";
      sf << "\"imu_age_ms\":" << static_cast<long long>(imu_age_ms) << ",";
      sf << "\"imu_gyro_age_ms\":" << static_cast<long long>(imu_gyro_age_ms) << ",";
      sf << "\"imu_quat_age_ms\":" << static_cast<long long>(imu_quat_age_ms) << ",";
      sf << "\"imu_stamp\":" << imu.source_stamp_s << ",";
      sf << "\"imu_gyro_seq\":" << imu.gyro_seq << ",";
      sf << "\"imu_quat_seq\":" << imu.quat_seq << ",";
      sf << "\"obs_imu_age_ms\":"
         << (loop_stats.last_obs_imu_age_s >= 0.0
           ? static_cast<long long>(loop_stats.last_obs_imu_age_s * 1000.0) : -1) << ",";
      sf << "\"obs_joint_age_ms\":"
         << (loop_stats.last_obs_joint_age_s >= 0.0
           ? static_cast<long long>(loop_stats.last_obs_joint_age_s * 1000.0) : -1) << ",";
      sf << "\"fault_reason\":\""
         << (runtime_state == RuntimeState::DampingFault ? escapeJson(fault_reason) : "")
         << "\",";
      sf << "\"ctrl_ticks\":" << loop_stats.control_ticks << ",";
      sf << "\"infer_ticks\":" << loop_stats.inference_ticks << ",";
      sf << "\"max_late_us\":" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6) << ",";
      sf << "\"missed_ctrl\":" << loop_stats.missed_control_deadlines << ",";
      sf << "\"can_tx\":" << sent_delta << ",";
      sf << "\"can_rx\":" << recv_delta << ",";
      sf << "\"can_err\":" << (can_stats.read_errors + can_stats.write_errors) << ",";
      sf << "\"can_tx_queue_full\":" << can_stats.tx_queue_full << ",";
      sf << "\"can_buses\":[";
      for (std::size_t b = 0; b < can_stats.buses.size(); ++b) {
        if (b > 0) sf << ",";
        const auto & bs = can_stats.buses[b];
        sf << "{\"name\":\"can" << b << "\","
           << "\"tx\":" << bs.tx << ","
           << "\"rx\":" << bs.rx << ","
           << "\"err\":" << (bs.read_errors + bs.write_errors) << ","
           << "\"qfull\":" << bs.tx_queue_full << ","
           << "\"error_frames\":" << bs.error_frames << ","
           << "\"bus_off\":" << bs.bus_off << ","
           << "\"state\":\"" << bs.state << "\"}";
      }
      sf << "],";
      sf << "\"command\":[" << command.vx << "," << command.vy << "," << command.yaw_rate << "],";
      // Per-joint state
      sf << "\"joints\":[";
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        if (i > 0) sf << ",";
        const double logical_pos = logicalPosition(states[i].position, calib[i]);
        const auto expected = can_stats.expected_status[i];
        const auto status_frames = states[i].status_frames;
        const auto missed = expected > status_frames ? expected - status_frames : 0;
        const double rx_ratio = expected > 0
          ? std::min(1.0, static_cast<double>(status_frames) / expected) : -1.0;
        const double pos_age_ms = states[i].last_position_s > 0.0
          ? std::max(0.0, (t - states[i].last_position_s) * 1000.0) : -1.0;
        const double vel_age_ms = states[i].last_velocity_s > 0.0
          ? std::max(0.0, (t - states[i].last_velocity_s) * 1000.0) : -1.0;
        const double status_age_ms = states[i].last_status_s > 0.0
          ? std::max(0.0, (t - states[i].last_status_s) * 1000.0) : -1.0;
        sf << "{\"n\":\"" << joints[i].name << "\","
           << "\"q\":" << logical_pos << ","
           << "\"target\":" << limited_target[i] << ","
           << "\"dq\":" << logicalVelocity(states[i].velocity, calib[i]) << ","
           << "\"tau\":" << states[i].torque << ","
           << "\"temp\":" << states[i].temperature << ","
           << "\"fault\":" << states[i].fault << ","
           << "\"fault_sta\":" << states[i].param_fault << ","
           << "\"age_ms\":" << static_cast<long long>(
                states[i].received ? (t - states[i].last_feedback_s) * 1000.0 : -1.0) << ","
           << "\"pos_age_ms\":" << static_cast<long long>(pos_age_ms) << ","
           << "\"vel_age_ms\":" << static_cast<long long>(vel_age_ms) << ","
           << "\"status_age_ms\":" << static_cast<long long>(status_age_ms) << ","
           << "\"status_frames\":" << status_frames << ","
           << "\"expected\":" << expected << ","
           << "\"missed\":" << missed << ","
           << "\"max_gap_ms\":" << static_cast<long long>(states[i].max_gap_s * 1000.0) << ","
           << "\"last_gap_ms\":" << static_cast<long long>(states[i].last_gap_s * 1000.0) << ","
           << "\"rx_ratio\":" << rx_ratio << ","
           << "\"recv\":" << (states[i].received ? "true" : "false") << "}";
      }
      sf << "],";
      // IMU
      sf << "\"imu\":{";
      sf << "\"wx\":" << imu.angular_velocity[0] << ","
         << "\"wy\":" << imu.angular_velocity[1] << ","
         << "\"wz\":" << imu.angular_velocity[2] << ","
         << "\"gx\":" << imu.projected_gravity[0] << ","
         << "\"gy\":" << imu.projected_gravity[1] << ","
         << "\"gz\":" << imu.projected_gravity[2];
      sf << "}";
      sf << "}\n";
      sf.close();
      std::rename((status_file + ".tmp").c_str(), status_file.c_str());
    }
  }

  loop_stats.resetWindow();
  last_can_sent = can_stats.frames_sent;
  last_can_received = can_stats.frames_received;
}

}  // namespace opendoge
