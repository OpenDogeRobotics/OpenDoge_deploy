#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <sys/stat.h>

#include "opendoge_deploy/cli.hpp"
#include "opendoge_deploy/controller.hpp"
#include "el05_socketcan.hpp"
#include "opendoge_deploy/observer.hpp"
#include "opendoge_deploy/policy.hpp"
#include "opendoge_deploy/runtime_io.hpp"
#include "opendoge_deploy/safety.hpp"
#include "opendoge_deploy/status.hpp"
#include "opendoge_deploy/types.hpp"

namespace
{
std::atomic_bool g_stop{false};

void signalHandler(int)
{
  g_stop.store(true);
}

double nowSeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path);
  return static_cast<bool>(file);
}
}  // namespace

int main(int argc, char ** argv)
{
  opendoge::Options opt;
  if (!opendoge::parseArgs(argc, argv, opt)) {
    return 1;
  }

  if (opt.config_path.empty()) {
    const std::array<std::string, 3> default_config_paths{
      "configs/opendoge_deploy.conf",
      "deploy/configs/opendoge_deploy.conf",
      "install/opendoge_deploy/share/opendoge_deploy/configs/opendoge_deploy.conf"};
    for (const auto & path : default_config_paths) {
      if (fileExists(path)) {
        opt.config_path = path;
        break;
      }
    }
  }
  if (opt.dry_run) {
    opt.allow_missing_imu = true;
  }
  opendoge::applyRuntimeTuning(opt);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto joints = opendoge::defaultJointMap();
  const auto default_pos = opendoge::defaultJointPosition();
  opendoge::DeployConfig config;
  std::string error;
  if (!opendoge::loadDeployConfig(opt.config_path, joints, config, error)) {
    std::cerr << "Config load failed: " << error << "\n";
    return 1;
  }
  if (!opt.lin_vel_source.empty()) {
    config.lin_vel_source = opt.lin_vel_source;
  }
  if (config.lin_vel_source != "zero" && config.lin_vel_source != "command" &&
      config.lin_vel_source != "imu")
  {
    std::cerr << "Invalid lin_vel_source: " << config.lin_vel_source
              << " (expected zero|command|imu)\n";
    return 1;
  }
  if (opt.kp_override >= 0.0) {
    config.kp = opt.kp_override;
  }
  if (opt.kd_override >= 0.0) {
    config.kd = opt.kd_override;
  }

  // Effective calibration dump — this is the first thing to check when a joint
  // moves the wrong way. A non-positive reduction or a swapped lower/upper is
  // rejected by loadDeployConfig before we get here.
  std::cout << "Effective calibration (dir offset reduction lower upper):\n"
            << std::fixed << std::setprecision(4);
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    const auto & c = config.joints[i];
    std::cout << "  " << std::left << std::setw(16) << joints[i].name
              << " dir=" << std::showpos << c.direction << std::noshowpos
              << " offset=" << c.offset
              << " reduction=" << c.reduction
              << " lower=" << c.lower
              << " upper=" << c.upper
              << " max_torque=" << c.max_torque
              << " kp=" << config.kp << " kd=" << config.kd << "\n";
  }
  std::cout << "EL05 safety: torque_limit_nm=" << config.torque_limit_nm
            << " can_timeout_s=" << config.can_timeout_s
            << " (0x7028 units="
            << (config.can_timeout_s > 0.0
                  ? std::to_string(static_cast<std::uint32_t>(
                      std::lround(config.can_timeout_s / 50.0e-6)))
                  : std::string("off"))
            << ")\n";
  // Restore the default stream formatting for the status/JSON output.
  std::cout << std::defaultfloat << std::setprecision(6);
  if (opt.dump_config) {
    return 0;
  }

  std::array<opendoge::MotorState, opendoge::kNumJoints> states{};
  std::array<opendoge::MotorCommand, opendoge::kNumJoints> commands{};
  std::array<double, opendoge::kNumJoints> action{};
  std::array<double, opendoge::kNumJoints> last_action{};
  std::array<double, opendoge::kObsDim> obs{};
  std::array<double, opendoge::kNumJoints> logical_target = default_pos;
  std::array<double, opendoge::kNumJoints> limited_target = default_pos;

  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    states[i].position = opendoge::motorPosition(default_pos[i], config.joints[i]);
    commands[i] = {states[i].position, 0.0, 0.0, 0.0, config.safe_kd};
  }

  auto policy = opendoge::makePolicy(opt.policy_backend);
  if (!policy) {
    std::cerr << "Unknown policy backend: " << opt.policy_backend << "\n";
    return 1;
  }
  if (!policy->load(opt.policy_path, error)) {
    std::cerr << "Policy load failed: " << error << "\n";
    return 1;
  }

  opendoge::El05SocketCan can;
  if (!opt.dry_run) {
    if (!can.open(joints)) {
      std::cerr << "CAN open failed: " << can.lastError() << "\n";
      return 1;
    }
    // Best-effort COMM_STOP for early-exit paths where some motors may already
    // be enabled. SocketCAN writes are not acknowledged by the adapter, so this is not proof of
    // delivery; the motor-side 0x7028 canTimeout is the real guarantee.
    auto stop_all_motors = [&](int repeats) {
      bool written = true;
      for (int r = 0; r < repeats; ++r) {
        for (const auto & joint : joints) {
          written = can.sendStop(joint, false) && written;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return written;
    };
    if (config.torque_limit_nm > 0.0) {
      for (const auto & joint : joints) {
        if (!can.sendTorqueLimit(joint, config.torque_limit_nm)) {
          std::cerr << "Torque limit write failed: " << can.lastError() << "\n";
          opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
          stop_all_motors(3);
          can.close();
          return 1;
        }
      }
      std::cout << "EL05 torque limit set to " << config.torque_limit_nm << " Nm\n";
    }
    // ── EL05 CAN watchdog (0x7028 canTimeout, RAM-only) ──
    // If this motor's CAN link dies (adapter unplugged, deploy
    // crash, COMM_STOP lost), it stops receiving commands and resets after
    // this timeout instead of holding the last damping command forever.
    // Written before enable, so a reset during the read-back is harmless.
    if (config.can_timeout_s > 0.0) {
      constexpr double kTimeoutUnitS = 50.0e-6;  // 20000 units = 1 s
      const auto timeout_units = static_cast<std::uint32_t>(
        std::lround(config.can_timeout_s / kTimeoutUnitS));
      std::cout << "EL05 CAN watchdog: " << config.can_timeout_s
                << " s (0x7028=" << timeout_units << ")\n";
      bool all_timeout_ok = true;
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
          if (!can.sendWriteParamUint32(joints[i], 0x7028, timeout_units)) {
            continue;
          }
          states[i].can_timeout_received = false;
          states[i].can_timeout_units = 0;
          can.sendReadParam(joints[i], 0x7028);
          const double deadline = nowSeconds() + 0.3;
          while (nowSeconds() < deadline) {
            can.drain(states, nowSeconds());
            if (states[i].can_timeout_received) {
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
          ok = states[i].can_timeout_received &&
               states[i].can_timeout_units == timeout_units;
        }
        if (!ok) {
          std::cerr << "CAN watchdog write/verify failed: " << joints[i].name
                    << " (id=" << joints[i].motor_id << "), expected "
                    << timeout_units << ", read "
                    << (states[i].can_timeout_received
                          ? std::to_string(states[i].can_timeout_units)
                          : std::string("none"))
                    << "\n";
          all_timeout_ok = false;
        }
      }
      if (!all_timeout_ok) {
        std::cerr << "Refusing to enable: 0x7028 canTimeout could not be verified "
                     "on all motors. Check the USB-CAN adapter / wiring.\n";
        opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
        stop_all_motors(3);
        can.close();
        return 1;
      }
    }
    if (opt.enable) {
      for (const auto & joint : joints) {
        if (opt.clear_fault) {
          if (!can.sendStop(joint, true)) {
            std::cerr << "Clear fault failed: " << can.lastError() << "\n";
            opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
            stop_all_motors(3);
            can.close();
            return 1;
          }
        }
        if (!can.sendMotionMode(joint) || !can.sendEnable(joint)) {
          std::cerr << "Motor startup failed: " << can.lastError() << "\n";
          opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
          stop_all_motors(3);
          can.close();
          return 1;
        }
        // Spread the enable inrush; 12 motors enabled back-to-back can cause a
        // supply/ground dip and a motor may miss the subsequent reads.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // ── Motor self-check ──
      // sendMotionMode()/sendEnable() only prove the CAN write succeeded.
      // Read back run_mode (0x7005), faultSta (0x3022) and the type-2 mode bits
      // (0 reset / 1 cali / 2 motor running). A motor left in PP/CSP, disabled,
      // or with limit_torque/limit_cur=0 will command no torque even though the
      // deploy thinks it is ready.
      //
      // Reads are per-motor and sequential: USB-CAN adapters can drop replies
      // when 48 frames are burst out at once, which made run_mode look like -1.
      if (!opt.skip_self_check) {
        // Let the motors' CAN controllers settle after the enable burst.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        constexpr int kSelfCheckAttempts = 5;
        // 0x7028 canTimeout is already active. The self-check reads run_mode and
        // faultSta sequentially (12 joints x ~0.7 s), so round-robin one damping
        // frame every 30 ms: each motor gets a command every ~360 ms, well
        // inside the 1 s watchdog, without bursting all 12 at once.
        std::size_t keepalive_next = 0;
        double next_keepalive_s = 0.0;
        auto keepalive_tick = [&]() {
          const double now = nowSeconds();
          if (now < next_keepalive_s) {
            return;
          }
          const std::size_t k = keepalive_next % opendoge::kNumJoints;
          const double kd = std::min(config.safe_kd, config.joints[k].max_kd);
          opendoge::MotorCommand damp{states[k].position, 0.0, 0.0, 0.0, kd};
          can.sendMotion(joints[k], damp);
          ++keepalive_next;
          next_keepalive_s = now + 0.03;
        };
        auto read_run_mode = [&](std::size_t i, double timeout_s) -> bool {
          states[i].run_mode = -1;
          can.sendReadParam(joints[i], 0x7005);
          const double deadline = nowSeconds() + timeout_s;
          while (nowSeconds() < deadline) {
            can.drain(states, nowSeconds());
            keepalive_tick();
            if (states[i].run_mode >= 0) {
              return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
          return false;
        };
        auto read_fault = [&](std::size_t i, double timeout_s) -> bool {
          states[i].fault_status_received = false;
          states[i].param_fault = 0;
          can.sendReadFaultStatus(joints[i]);
          const double deadline = nowSeconds() + timeout_s;
          while (nowSeconds() < deadline) {
            can.drain(states, nowSeconds());
            keepalive_tick();
            if (states[i].fault_status_received) {
              return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
          return false;
        };
        auto motor_ok = [&](std::size_t i) {
          return states[i].run_mode == 0 &&
                 states[i].fault_status_received &&
                 states[i].param_fault == 0 &&
                 states[i].mode == 2;
        };
        bool all_ok = false;
        for (int attempt = 0; attempt < kSelfCheckAttempts && !all_ok; ++attempt) {
          // One damping frame per motor so the type-2 mode bits refresh.
          for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
            const double kd = std::min(config.safe_kd, config.joints[i].max_kd);
            opendoge::MotorCommand damp{states[i].position, 0.0, 0.0, 0.0, kd};
            can.sendMotion(joints[i], damp);
          }
          for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
            read_run_mode(i, 0.4);
            read_fault(i, 0.3);
          }
          all_ok = true;
          for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
            if (!motor_ok(i)) {
              all_ok = false;
              break;
            }
          }
          if (!all_ok && attempt + 1 < kSelfCheckAttempts) {
            for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
              if (motor_ok(i)) {
                continue;
              }
              can.sendStop(joints[i], true);
              can.sendMotionMode(joints[i]);
              can.sendEnable(joints[i]);
            }
          }
        }
        if (!all_ok) {
          for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
            if (motor_ok(i)) {
              continue;
            }
            std::cerr << "Self-check failed: " << joints[i].name
                      << " (id=" << joints[i].motor_id << "): run_mode="
                      << states[i].run_mode << " (expected 0), faultSta=0x"
                      << std::hex << states[i].param_fault << std::dec
                      << (states[i].fault_status_received ? "" : " (not read)")
                      << ", mode=" << static_cast<int>(states[i].mode)
                      << " (expected 2 running)\n";
          }
          std::cerr << "Motor self-check failed; refusing to enable control. "
                       "Check run_mode / limit_torque / limit_cur / wiring, "
                       "or use --skip-self-check.\n";
          opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
          stop_all_motors(3);
          can.close();
          return 1;
        }
        std::cout << "Motor self-check passed: run_mode=0, faultSta=0, mode=running.\n";
      }
    }
  }

  std::cout << "OpenDoge deploy running: "
            << (opt.dry_run ? "dry-run" : "real CAN")
            << ", policy=" << opt.policy_backend
            << ", control=" << config.control_hz << "Hz"
            << ", config=" << (opt.config_path.empty() ? "defaults" : opt.config_path)
            << "\n";

  const double start_s = nowSeconds();
  double next_control_s = start_s;
  double next_infer_s = start_s;
  double next_target_s = start_s;
  double next_fault_poll_s = start_s;
  double next_status_s = start_s + 1.0;
  double next_feedback_retry_s = start_s;
  double next_enable_retry_s = start_s;
  std::size_t feedback_retry_next = 0;
  opendoge::RuntimeState runtime_state = opt.dry_run ? opendoge::RuntimeState::Ready : opendoge::RuntimeState::WaitFeedback;
  opendoge::OperatorCommand command = opt.static_command;
  // With require_fresh_activation, a latched active=true from the joystick
  // service is ignored until the operator has released it once (B/START).
  bool fresh_activation_ready = !opt.require_fresh_activation;
  opendoge::ImuSample imu;
  imu.valid = opt.allow_missing_imu;
  std::string fault_reason;
  double next_input_s = start_s;
  double last_command_warn_s = 0.0;
  opendoge::LoopStats loop_stats;
  double pc_startup_start_s{0.0};
  bool rl_fallback_active{false};
  double feedback_wait_start_s{0.0};
  double standup_start_s{0.0};
  double standup_handover_start_s{-1.0};
  int imu_invalid_count{0};
  int imu_stale_count{0};
  std::array<opendoge::JointSafetyState, opendoge::kNumJoints> safety_state{};
  std::uint64_t last_can_sent = 0;
  std::uint64_t last_can_received = 0;

  while (!g_stop.load()) {
    const double t = nowSeconds();
    if (opt.duration_s > 0.0 && t - start_s >= opt.duration_s) {
      break;
    }

    // ── CAN drain ──
    if (!opt.dry_run) {
      can.drain(states, t);
      if (!can.ok()) {
        fault_reason = can.lastError();
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      // CAN controller error state. BUS-OFF always faults; ERROR-PASSIVE is
      // configurable (default warn, visible in the status JSON).
      const auto & can_stats = can.stats();
      for (std::size_t b = 0; b < can_stats.buses.size(); ++b) {
        const auto & bus = can_stats.buses[b];
        if (bus.state == "BUS-OFF") {
          fault_reason = "CAN BUS-OFF on can" + std::to_string(b);
          runtime_state = opendoge::RuntimeState::DampingFault;
          break;
        }
        if (config.can_error_action == "fault" && bus.state == "ERROR-PASSIVE") {
          fault_reason = "CAN ERROR-PASSIVE on can" + std::to_string(b);
          runtime_state = opendoge::RuntimeState::DampingFault;
          break;
        }
      }
    }

    // ── Fault polling ──
    if (t >= next_fault_poll_s && !opt.dry_run) {
      for (const auto & joint : joints) {
        can.sendReadFaultStatus(joint);
      }
      next_fault_poll_s = t + 1.0 / std::max(config.fault_poll_hz, 1.0);
    }

    // ── Feedback recovery (scan-like retry) ──
    // The USB-CAN path can drop replies in bursts. Instead of faulting on
    // the first missed deadline, refresh stale joints with a type-17 read
    // (mechPos/mechVel); those replies count as valid feedback. In
    // WaitFeedback, also re-send motion mode + enable to motors that have
    // not answered yet.
    if (!opt.dry_run) {
      if (runtime_state == opendoge::RuntimeState::WaitFeedback && t >= next_enable_retry_s) {
        for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
          if (states[i].received) {
            continue;
          }
          can.sendMotionMode(joints[i]);
          can.sendEnable(joints[i]);
        }
        next_enable_retry_s = t + 0.5;
      }
      if (config.feedback_retry_hz > 0.0 && t >= next_feedback_retry_s) {
        const double stale_after = std::max(config.state_timeout_s * 0.5, 0.01);
        constexpr std::size_t kRetryBatch = 3;
        std::size_t sent = 0;
        for (std::size_t k = 0; k < opendoge::kNumJoints && sent < kRetryBatch; ++k) {
          const std::size_t i = (feedback_retry_next + k) % opendoge::kNumJoints;
          if (!states[i].received || t - states[i].last_feedback_s > stale_after) {
            can.sendReadParam(joints[i], 0x7019);  // mechPos
            can.sendReadParam(joints[i], 0x701B);  // mechVel
            ++sent;
          }
        }
        feedback_retry_next = (feedback_retry_next + 1) % opendoge::kNumJoints;
        next_feedback_retry_s = t + 1.0 / std::max(config.feedback_retry_hz, 1.0);
      }
    }

    // ── Input polling ──
    if (t >= next_input_s) {
      if (!opt.command_file.empty() && !fileExists(opt.command_file)) {
        // The joystick bridge may not have started yet (or was stopped). Treat
        // a missing command file as "no operator input" instead of faulting;
        // the motors stay in damping. Malformed files still fault below.
        command.vx = 0.0;
        command.vy = 0.0;
        command.yaw_rate = 0.0;
        command.active = false;
        command.estop = false;
        command.position_control = false;
        command.rl_inference = false;
        command.clear_fault = false;
        command.low_gain_mode = false;
        if (t - last_command_warn_s >= 5.0) {
          std::cerr << "Warning: command file not found: " << opt.command_file
                    << " — using neutral command (damping). Start opendoge-joystick"
                       " or create the file.\n";
          last_command_warn_s = t;
        }
      } else if (!opendoge::readCommandFile(opt.command_file, command, error)) {
        fault_reason = error;
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      if (opt.require_fresh_activation && !fresh_activation_ready) {
        if (!command.active) {
          fresh_activation_ready = true;
        } else {
          // The joystick bridge keeps active=true across deploy restarts.
          // Ignore it until the operator has released active once.
          command.active = false;
        }
      }
      command.active = command.active || opt.start_active;
      command.low_gain_mode = command.low_gain_mode || opt.low_gain;
      if (command.estop) {
        fault_reason = "operator estop";
        runtime_state = opendoge::RuntimeState::DampingFault;
      }

      // Command timeout: stale command file while active → zero commands
      if (!opt.dry_run && command.active && !opt.command_file.empty()) {
        struct stat cmd_stat {};
        if (::stat(opt.command_file.c_str(), &cmd_stat) == 0) {
          const double file_age_s = t - std::max(
            static_cast<double>(cmd_stat.st_mtime),
            static_cast<double>(cmd_stat.st_ctime));
          if (file_age_s > config.command_timeout_s) {
            std::cerr << "Warning: command file stale for "
                      << file_age_s << "s, zeroing commands\n";
            command.vx = 0.0;
            command.vy = 0.0;
            command.yaw_rate = 0.0;
            command.active = false;
          }
        }
      }

      // Command smoothing (EMA low-pass)
      if (config.command_smoothing_alpha > 0.0) {
        static opendoge::OperatorCommand smooth_cmd = command;
        const double a = config.command_smoothing_alpha;
        smooth_cmd.vx = a * command.vx + (1.0 - a) * smooth_cmd.vx;
        smooth_cmd.vy = a * command.vy + (1.0 - a) * smooth_cmd.vy;
        smooth_cmd.yaw_rate = a * command.yaw_rate + (1.0 - a) * smooth_cmd.yaw_rate;
        command.vx = smooth_cmd.vx;
        command.vy = smooth_cmd.vy;
        command.yaw_rate = smooth_cmd.yaw_rate;
        smooth_cmd.active = command.active;
        smooth_cmd.estop = command.estop;
        smooth_cmd.position_control = command.position_control;
        smooth_cmd.rl_inference = command.rl_inference;
        smooth_cmd.clear_fault = command.clear_fault;
        smooth_cmd.low_gain_mode = command.low_gain_mode;
      }

      if (!opendoge::readImuFile(opt.imu_file, imu, t, error)) {
        fault_reason = error;
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      // base_lin_vel is not measured by the stock IMU bridge. Resolve the
      // configured source: zero (worst case), command approximation, or the
      // optional vx/vy/vz fields from the IMU state file.
      if (config.lin_vel_source == "zero") {
        imu.linear_velocity = {0.0, 0.0, 0.0};
      } else if (config.lin_vel_source == "command") {
        imu.linear_velocity = {command.vx, command.vy, 0.0};
      }
      if (!opt.allow_missing_imu && !imu.valid) {
        ++imu_invalid_count;
      } else {
        imu_invalid_count = 0;
      }
      if (!opt.allow_missing_imu && imu_invalid_count > config.imu_debounce_count) {
        runtime_state = opendoge::RuntimeState::Ready;
      }
      // IMU age from the bridge's in-band stamp. A frozen bridge that keeps
      // rewriting its cached sample still shows a growing age here.
      const double imu_age_s = (imu.valid && imu.source_stamp_s > 0.0)
        ? (t > imu.source_stamp_s ? t - imu.source_stamp_s : 0.0) : -1.0;
      if (config.imu_timeout_s > 0.0 && imu_age_s > config.imu_timeout_s) {
        ++imu_stale_count;
      } else {
        imu_stale_count = 0;
      }
      if (config.imu_timeout_s > 0.0 && config.imu_stale_action == "fault" &&
          imu_stale_count > config.imu_debounce_count) {
        fault_reason = "IMU stale: age=" + std::to_string(imu_age_s) + "s";
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      next_input_s = t + 0.005;
    }

    // ── Safety checks (non-WaitFeedback states) ──
    if (!opt.dry_run && runtime_state != opendoge::RuntimeState::WaitFeedback) {
      std::string safety_reason;
      const bool fault_now = opendoge::safetyFault(states, joints, config.joints, config, runtime_state,
            limited_target, imu, safety_state, t, standup_start_s, safety_reason);
      if (fault_now) {
        fault_reason = safety_reason;
        runtime_state = opendoge::RuntimeState::DampingFault;
      } else if (runtime_state == opendoge::RuntimeState::DampingFault && command.clear_fault) {
        const bool can_ok = !opt.dry_run ? can.ok() : true;
        bool writes_ok = can_ok;
        if (can_ok) {
          for (const auto & joint : joints) {
            writes_ok = can.sendStop(joint, true) && writes_ok;
            writes_ok = can.sendMotionMode(joint) && writes_ok;
            writes_ok = can.sendEnable(joint) && writes_ok;
          }
        }
        command.clear_fault = false;
        if (!can_ok || !writes_ok) {
          // A SocketCAN write can succeed while the adapter/driver is dead. Never
          // report "fault cleared" without a live link — otherwise the operator
          // thinks the robot is safe while the motors are still unresponsive.
          std::cerr << "Fault clear not confirmed (CAN link down); staying in "
                       "DampingFault. Check the USB-CAN adapter/driver.\n";
          fault_reason = "fault clear failed: CAN link down";
        } else {
          std::cout << "Fault cleared, transitioning to WaitFeedback\n";
          fault_reason.clear();
          runtime_state = opendoge::RuntimeState::WaitFeedback;
          for (auto & ss : safety_state) {
            ss = opendoge::JointSafetyState{};
          }
          for (auto & st : states) {
            st.received = false;
          }
        }
      }
    }

    // ── State transitions (non-fault paths) ──
    const opendoge::RuntimeState prev_runtime_state = runtime_state;
    if (runtime_state != opendoge::RuntimeState::DampingFault) {
      opendoge::updateStateMachine(
        runtime_state, rl_fallback_active, command, opt,
        states, config, imu, t,
        pc_startup_start_s, feedback_wait_start_s, standup_start_s, fault_reason);
    }
    const bool entered_low_gain =
      prev_runtime_state != opendoge::RuntimeState::LowGainTest &&
      runtime_state == opendoge::RuntimeState::LowGainTest;
    const bool entered_standup =
      prev_runtime_state != opendoge::RuntimeState::StandingUp &&
      runtime_state == opendoge::RuntimeState::StandingUp;
    if (entered_low_gain || entered_standup) {
      // Start the ramp from the measured pose. Without this the rate limiter
      // would ramp from the pre-initialised default pose (a step).
      standup_start_s = t;
      standup_handover_start_s = -1.0;
      bool outside_limits = false;
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        const double q = opendoge::logicalPosition(states[i].position, config.joints[i]);
        const double margin = 0.25;  // rad; generous, the URDF limits are conservative
        if (q < config.joints[i].lower - margin || q > config.joints[i].upper + margin) {
          fault_reason = joints[i].name + ": measured position " + std::to_string(q) +
            " rad outside joint limits [" + std::to_string(config.joints[i].lower) + ", " +
            std::to_string(config.joints[i].upper) +
            "] — possible 2pi branch/offset mismatch; refusing to ramp into it";
          runtime_state = opendoge::RuntimeState::DampingFault;
          outside_limits = true;
          break;
        }
        limited_target[i] = q;
      }
      (void)outside_limits;  // DampingFault is set; the damping branch ignores limited_target
    }

    // Record the Stand-up → policy/PC handover so updateTargets() can blend
    // the first policy action in instead of stepping the target.
    const bool left_standup =
      prev_runtime_state == opendoge::RuntimeState::StandingUp &&
      (runtime_state == opendoge::RuntimeState::ActiveRL ||
       runtime_state == opendoge::RuntimeState::ActivePC);
    if (left_standup) {
      standup_handover_start_s = t;
    }

    // Observation-side feedback staleness: a type-2 status path can die while
    // a type-17 retry keeps q/dq fresh. Default hold keeps the old behaviour;
    // freeze skips inference; fault enters DampingFault.
    bool feedback_stale = false;
    if (config.feedback_hold_max_s > 0.0 &&
        (runtime_state == opendoge::RuntimeState::ActiveRL ||
         runtime_state == opendoge::RuntimeState::ActivePC ||
         runtime_state == opendoge::RuntimeState::EnteringPosition)) {
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        // No type-2 frame yet at all: let the missing-feedback/state-timeout
        // safety handle it, not this observation-side check.
        const double status_age =
          (states[i].status_frames > 0 && states[i].last_status_s > 0.0)
          ? t - states[i].last_status_s : 0.0;
        if (status_age > config.feedback_hold_max_s) {
          feedback_stale = true;
          break;
        }
      }
      if (feedback_stale && config.stale_action == "fault") {
        fault_reason = "motor feedback stale > " +
          std::to_string(config.feedback_hold_max_s) + "s";
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
    }

    // ── Inference block (100 Hz) ──
    if (t >= next_infer_s) {
      ++loop_stats.inference_ticks;
      if ((runtime_state == opendoge::RuntimeState::ActiveRL || runtime_state == opendoge::RuntimeState::EnteringPosition
           || runtime_state == opendoge::RuntimeState::ActivePC) && command.active &&
          !(feedback_stale && config.stale_action == "freeze")) {
        // Refresh observation inputs at the inference tick. The 200 Hz
        // safety/command poll above is intentionally kept (it bounds file I/O
        // and preserves the command EMA / IMU debounce rates), but it adds up
        // to one poll period to the policy observation. Draining CAN and
        // re-reading the bridge file here removes that phase: the bridge now
        // writes on new gyro/quat frames instead of every 5 ms, so this sees
        // the freshest sample. No observation dimension, action_scale,
        // default pose or control-law change.
        if (!opt.dry_run) {
          can.drain(states, t);
        }
        if (!opt.imu_file.empty()) {
          opendoge::ImuSample fresh_imu;
          std::string imu_refresh_error;
          if (opendoge::readImuFile(opt.imu_file, fresh_imu, t, imu_refresh_error) &&
              fresh_imu.valid) {
            if (config.lin_vel_source == "zero") {
              fresh_imu.linear_velocity = {0.0, 0.0, 0.0};
            } else if (config.lin_vel_source == "command") {
              fresh_imu.linear_velocity = {command.vx, command.vy, 0.0};
            }
            imu = fresh_imu;
          }
        }
        double obs_joint_age_s = -1.0;
        for (const auto & state : states) {
          if (state.received && state.last_feedback_s > 0.0) {
            obs_joint_age_s = std::max(
              obs_joint_age_s, std::max(0.0, t - state.last_feedback_s));
          }
        }
        loop_stats.last_obs_imu_age_s =
          (imu.valid && imu.source_stamp_s > 0.0)
            ? std::max(0.0, t - imu.source_stamp_s) : -1.0;
        loop_stats.last_obs_joint_age_s = obs_joint_age_s;
        obs = opendoge::buildObservation(states, config.joints, default_pos, last_action, command, imu);
        if (!policy->infer(obs, action, error)) {
          if (runtime_state == opendoge::RuntimeState::ActiveRL) {
            runtime_state = opendoge::RuntimeState::ActivePC;
            rl_fallback_active = true;
            action.fill(0.0);
          } else {
            fault_reason = "policy infer failed: " + error;
            runtime_state = opendoge::RuntimeState::DampingFault;
          }
        }
      } else {
        action.fill(0.0);
      }
      next_infer_s = t + 1.0 / config.inference_hz;
    }

    // ── Target update block (200 Hz) ──
    if (t >= next_target_s) {
      ++loop_stats.target_ticks;
      opendoge::updateTargets(
        logical_target, limited_target, last_action, action,
        default_pos, config.joints, config, runtime_state, t,
        standup_handover_start_s);
      next_target_s = t + 1.0 / config.target_hz;
    }

    // ── Control block (control_hz) ──
    if (t >= next_control_s) {
      ++loop_stats.control_ticks;
      const double control_late_s = t - next_control_s;
      loop_stats.max_control_late_s = std::max(loop_stats.max_control_late_s, control_late_s);
      if (control_late_s > 0.0005) {
        ++loop_stats.missed_control_deadlines;
      }
      opendoge::computeMotorCommands(
        commands, states, config.joints, config,
        runtime_state, limited_target, default_pos, t, pc_startup_start_s, standup_start_s);
      if (!opt.dry_run) {
        // Keep sending to every joint even if one bus write fails. The old
        // code broke out on the first failure and also gated the whole block
        // on the sticky can.ok(), so a single bad bus stopped damping commands
        // to all other legs; those motors then held the last RL target until
        // their 0x7028 canTimeout expired (looked like "the other legs keep
        // walking on IMU tilt").
        bool any_write_failed = false;
        std::string first_write_error;
        for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
          if (!can.sendMotion(joints[i], commands[i])) {
            if (!any_write_failed) {
              first_write_error = can.lastError();
            }
            any_write_failed = true;
          }
        }
        if (any_write_failed) {
          fault_reason = first_write_error;
          runtime_state = opendoge::RuntimeState::DampingFault;
        } else {
          // All 12 writes (hence all buses) succeeded this tick: clear a
          // previous transient error so clear_fault can work after recovery.
          can.clearError();
        }
      }
      next_control_s = t + 1.0 / config.control_hz;
    }

    // ── Status output (1 Hz) ──
    if (t >= next_status_s) {
      opendoge::emitStatus(
        runtime_state, command, opt.status_file,
        loop_stats, last_can_sent, last_can_received,
        can.stats(), fault_reason, rl_fallback_active,
        states, limited_target, config.joints, joints, imu,
        pc_startup_start_s, config.pc_startup_ramp_s, t);
      next_status_s = t + 1.0;
    }

    const double next_deadline_s = std::min(
      std::min(next_control_s, next_infer_s),
      std::min(std::min(next_target_s, next_fault_poll_s), std::min(next_status_s, next_input_s)));
    opendoge::waitUntilNextDeadline(next_deadline_s);
  }

  if (!opt.dry_run) {
    opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
    if (opt.disable_on_exit) {
      // COMM_STOP (0x04) disables the motor output. Without it the motors keep
      // applying the last damping/position command after this process exits, so
      // the robot still resists being moved by hand.
      //
      // A SocketCAN write can succeed even when the USB-CAN adapter is dead, so "the
      // write returned true" does NOT prove the motor received it. Check the
      // last feedback age first: if a joint's link was already stale, report
      // that disable is unconfirmed instead of printing a false success.
      std::string stale_joints;
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        const double age_s = states[i].received
          ? (nowSeconds() - states[i].last_feedback_s) : 1.0e9;
        if (age_s > 3.0 * config.state_timeout_s) {
          if (!stale_joints.empty()) {
            stale_joints += ", ";
          }
          stale_joints += joints[i].name;
        }
      }
      // Longer window (200 ms) to ride out short USB-CAN bursts. A truly dead
      // adapter still cannot receive these; the motor-side 0x7028 canTimeout is
      // the final fallback.
      bool all_sent = true;
      for (int repeat = 0; repeat < 20; ++repeat) {
        for (const auto & joint : joints) {
          if (!can.sendStop(joint, false)) {
            all_sent = false;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (all_sent && stale_joints.empty()) {
        std::cout << "Motors disabled (COMM_STOP) on exit.\n";
      } else if (!stale_joints.empty()) {
        std::cerr << "Warning: cannot confirm disable on: " << stale_joints
                  << " (no fresh feedback before exit). If they stay stiff, "
                     "cut motor power; 0x7028 canTimeout will release them if set.\n";
      } else {
        std::cerr << "Warning: some COMM_STOP frames failed; motors may stay enabled. "
                     "Cut motor power if the robot is still stiff.\n";
      }
    }
    can.close();
  }

  std::cout << "OpenDoge deploy stopped";
  if (runtime_state == opendoge::RuntimeState::DampingFault) {
    std::cout << " with fault: " << fault_reason;
  }
  std::cout << "\n";
  return runtime_state == opendoge::RuntimeState::DampingFault ? 2 : 0;
}
