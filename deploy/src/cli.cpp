#include "opendoge_deploy/cli.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <sched.h>
#include <sys/mman.h>

namespace opendoge
{

void printUsage()
{
  std::cout
    << "Usage: opendoge_deploy [options]\n"
    << "  --policy-backend none|linear_csv|onnx   default: none\n"
    << "  --policy-path PATH                      ONNX or linear CSV path\n"
    << "  --config PATH                           deploy key=value config\n"
    << "  --command-file PATH                     command.state input (vx/vy/yaw_rate/active/estop/position_control/rl_inference)\n"
    << "  --imu-file PATH                         wx/wy/wz/gx/gy/gz input\n"
    << "  --status-file PATH                      write JSON status snapshot each second\n"
    << "  --cmd VX VY YAW                         static command\n"
    << "  --real                                  open can0..can3 and send frames\n"
    << "  --enable                                set motion mode and enable motors on startup\n"
    << "  --clear-fault                           send stop(clear_fault=1) before enable\n"
    << "  --start-active                          enter active after readiness checks\n"
    << "  --low-gain                              force low_gain_mode (joystick-proof)\n"
    << "  --allow-missing-imu                     permit active with default IMU sample\n"
    << "  --realtime                              try mlockall + SCHED_FIFO\n"
    << "  --rt-priority N                         SCHED_FIFO priority (default 60)\n"
    << "  --cpu N                                 pin process to CPU N\n"
    << "  --duration-sec SEC                      stop after SEC, 0 means until Ctrl+C\n"
    << "  --lin-vel-source zero|command|imu        base_lin_vel source for the 48-dim obs\n"
    << "  --kp K / --kd K                         override PD gains for bring-up\n"
    << "  --dump-config                           print effective calibration and exit\n"
    << "  --disable-on-exit                       send COMM_STOP to all motors on exit (default)\n"
    << "  --keep-enabled                          leave motors enabled in damping on exit\n"
    << "  --allow-latched-activation              accept active=true at startup (default: require a fresh A)\n"
    << "  --skip-self-check                       skip the startup motor mode/fault self-check\n";
}

bool parseArgs(int argc, char ** argv, Options & opt)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto need_value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + name);
      }
      return argv[++i];
    };
    try {
      if (arg == "--help" || arg == "-h") {
        printUsage();
        return false;
      } else if (arg == "--policy-backend") {
        opt.policy_backend = need_value(arg);
      } else if (arg == "--policy-path") {
        opt.policy_path = need_value(arg);
      } else if (arg == "--config") {
        opt.config_path = need_value(arg);
      } else if (arg == "--command-file") {
        opt.command_file = need_value(arg);
      } else if (arg == "--imu-file") {
        opt.imu_file = need_value(arg);
      } else if (arg == "--cmd") {
        opt.static_command.vx = std::stod(need_value(arg));
        opt.static_command.vy = std::stod(need_value(arg));
        opt.static_command.yaw_rate = std::stod(need_value(arg));
      } else if (arg == "--real") {
        opt.dry_run = false;
      } else if (arg == "--enable") {
        opt.enable = true;
      } else if (arg == "--clear-fault") {
        opt.clear_fault = true;
      } else if (arg == "--start-active") {
        opt.start_active = true;
        opt.static_command.active = true;
      } else if (arg == "--low-gain") {
        opt.low_gain = true;
      } else if (arg == "--allow-missing-imu") {
        opt.allow_missing_imu = true;
      } else if (arg == "--realtime") {
        opt.realtime = true;
      } else if (arg == "--rt-priority") {
        opt.rt_priority = std::stoi(need_value(arg));
      } else if (arg == "--cpu") {
        opt.cpu = std::stoi(need_value(arg));
      } else if (arg == "--status-file") {
        opt.status_file = need_value(arg);
      } else if (arg == "--duration-sec") {
        opt.duration_s = std::stod(need_value(arg));
      } else if (arg == "--lin-vel-source") {
        opt.lin_vel_source = need_value(arg);
      } else if (arg == "--kp") {
        opt.kp_override = std::stod(need_value(arg));
      } else if (arg == "--kd") {
        opt.kd_override = std::stod(need_value(arg));
      } else if (arg == "--dump-config") {
        opt.dump_config = true;
      } else if (arg == "--disable-on-exit") {
        opt.disable_on_exit = true;
      } else if (arg == "--keep-enabled") {
        opt.disable_on_exit = false;
      } else if (arg == "--allow-latched-activation") {
        opt.require_fresh_activation = false;
      } else if (arg == "--skip-self-check") {
        opt.skip_self_check = true;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    } catch (const std::exception & exc) {
      std::cerr << exc.what() << "\n";
      return false;
    }
  }
  return true;
}

void applyRuntimeTuning(const Options & opt)
{
  if (opt.cpu >= 0) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(opt.cpu, &mask);
    if (::sched_setaffinity(0, sizeof(mask), &mask) != 0) {
      std::cerr << "Warning: sched_setaffinity failed\n";
    }
  }
  if (opt.realtime) {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      std::cerr << "Warning: mlockall failed\n";
    }
    sched_param param{};
    param.sched_priority = opt.rt_priority;
    if (::sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      std::cerr << "Warning: sched_setscheduler(SCHED_FIFO) failed\n";
    }
  }
}

}  // namespace opendoge
