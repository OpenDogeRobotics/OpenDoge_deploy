#pragma once

#include <array>
#include <map>
#include <string>
#include <unordered_map>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

class El05SocketCan
{
public:
  bool open(const std::array<JointMap, kNumJoints> & joints);
  void close();

  bool sendMotionMode(const JointMap & joint);
  bool sendEnable(const JointMap & joint);
  bool sendStop(const JointMap & joint, bool clear_fault);
  bool sendReadFaultStatus(const JointMap & joint);
  /// Read one EL05 parameter (type 17). Used by the feedback watchdog to
  /// refresh mechPos (0x7019) / mechVel (0x701B) when a joint's reply is lost.
  bool sendReadParam(const JointMap & joint, std::uint16_t index);
  bool sendMotion(const JointMap & joint, const MotorCommand & command);
  /// Write the EL05 hardware torque limit (0x700B, 0-6 Nm). RAM-only; the
  /// motor reverts to its stored limit after a power cycle.
  bool sendTorqueLimit(const JointMap & joint, double nm);
  /// Write a uint32 parameter (e.g. 0x7028 canTimeout). RAM-only (no save).
  /// Read back with sendReadParam(joint, index) and check
  /// MotorState::can_timeout_received / can_timeout_units.
  bool sendWriteParamUint32(const JointMap & joint, std::uint16_t index, std::uint32_t value);
  void drain(std::array<MotorState, kNumJoints> & states, double now_s);

  bool ok() const { return ok_; }
  /// Recover from a transient write/read error after a fully successful
  /// control tick. Without this, one ENOBUFS/ENETDOWN latched ok_=false
  /// forever and blocked fault-clear even after the bus recovered.
  void clearError() { ok_ = true; last_error_.clear(); }
  const std::string & lastError() const { return last_error_; }
  const CanStats & stats() const { return stats_; }

private:
  struct Bus
  {
    std::string name;
    int fd{-1};
    std::size_t index{0};  // 0=can0 ... 3=can3 for CanStats::buses
  };

  static double clamp(double value, double lower, double upper);
  static std::uint16_t floatToUint(double value, double lower, double upper);
  static double uintToFloat(std::uint16_t value, double lower, double upper);
  static std::uint32_t buildExtId(std::uint8_t comm_type, std::uint16_t data2, std::uint8_t target_id);

  bool openBus(const std::string & name);
  bool sendFrame(
    const std::string & can, std::uint8_t comm_type, std::uint8_t motor_id,
    const std::array<std::uint8_t, 8> & data, std::uint16_t data2);
  void parseFrame(
    std::uint32_t can_id, const std::array<std::uint8_t, 8> & data,
    std::array<MotorState, kNumJoints> & states, double now_s);
  void fail(const std::string & error);

  std::map<std::string, Bus> buses_;
  std::unordered_map<int, std::size_t> motor_to_index_;
  CanStats stats_;
  bool ok_{true};
  std::string last_error_;

  int master_id_{0xfd};
  double p_min_{-12.57};
  double p_max_{12.57};
  double v_min_{-50.0};
  double v_max_{50.0};
  double t_min_{-6.0};
  double t_max_{6.0};
  double kp_min_{0.0};
  double kp_max_{500.0};
  double kd_min_{0.0};
  double kd_max_{5.0};
};

}  // namespace opendoge
