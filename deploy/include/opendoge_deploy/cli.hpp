#pragma once

#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

struct Options
{
  std::string policy_backend{"none"};
  std::string policy_path;
  std::string config_path;
  std::string command_file;
  std::string imu_file;
  bool dry_run{true};
  bool enable{false};
  bool clear_fault{false};
  bool start_active{false};
  // Force low_gain_mode regardless of the command file (the joystick bridge
  // rewrites that file at ~100 Hz and would otherwise clear it).
  bool low_gain{false};
  bool allow_missing_imu{false};
  bool realtime{false};
  int cpu{-1};
  int rt_priority{60};
  double duration_s{0.0};
  std::string status_file;
  // Overrides DeployConfig::lin_vel_source when non-empty.
  std::string lin_vel_source;
  // Optional bring-up overrides (<0 = keep config value).
  double kp_override{-1.0};
  double kd_override{-1.0};
  // Print the effective calibration and exit without touching CAN.
  bool dump_config{false};
  // On exit, send COMM_STOP so the motors do not keep applying the last
  // damping/position command after this process is gone.
  bool disable_on_exit{true};
  // Ignore a latched active=true from the joystick file at startup until it
  // has been false at least once. Prevents the robot from standing up by
  // itself when the deploy is restarted while the gamepad is still "active".
  bool require_fresh_activation{true};
  // Skip the startup motor self-check (run_mode / faultSta / running mode).
  bool skip_self_check{false};
  OperatorCommand static_command;
};

void printUsage();

bool parseArgs(int argc, char ** argv, Options & opt);

void applyRuntimeTuning(const Options & opt);

}  // namespace opendoge
