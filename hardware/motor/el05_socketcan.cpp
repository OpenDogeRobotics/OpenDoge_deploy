#include "el05_socketcan.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace opendoge
{
namespace
{
constexpr std::uint8_t kCommControl = 0x01;
constexpr std::uint8_t kCommStatus = 0x02;
constexpr std::uint8_t kCommEnable = 0x03;
constexpr std::uint8_t kCommStop = 0x04;
constexpr std::uint8_t kCommReadParam = 0x11;
constexpr std::uint8_t kCommWriteParam = 0x12;
constexpr std::uint32_t kCanEffMask = 0x1FFFFFFF;
constexpr std::uint32_t kStatusFaultMask = 0x3F;
constexpr std::uint16_t kIndexRunMode = 0x7005;
constexpr std::uint16_t kIndexTorqueLimit = 0x700B;
constexpr std::uint16_t kIndexMechPos = 0x7019;
constexpr std::uint16_t kIndexMechVel = 0x701B;
constexpr std::uint16_t kIndexFaultStatus = 0x3022;
// CAN watchdog, uint32, units of 50 us (20000 = 1 s). RAM-only when written
// without COMM_SAVE_PARAM (0x16).
constexpr std::uint16_t kIndexCanTimeout = 0x7028;
constexpr std::uint8_t kRunModeMotion = 0x00;

std::size_t busIndex(const std::string & name)
{
  if (name.size() == 4 && name.rfind("can", 0) == 0 &&
      name[3] >= '0' && name[3] <= '3') {
    return static_cast<std::size_t>(name[3] - '0');
  }
  return 0;
}

void parseCanError(std::uint32_t can_id, const can_frame & frame, BusCanStats & bus)
{
  ++bus.error_frames;
  const auto crtl = frame.data[1];
  if (can_id & CAN_ERR_BUSOFF) {
    ++bus.bus_off;
    bus.state = "BUS-OFF";
  }
  if (can_id & CAN_ERR_RESTARTED) {
    ++bus.restarts;
    bus.state = "ERROR-ACTIVE";
  }
  if (can_id & CAN_ERR_CRTL) {
    if (crtl & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) {
      bus.state = "ERROR-PASSIVE";
    } else if (crtl & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) {
      bus.state = "ERROR-WARNING";
    } else if (crtl & CAN_ERR_CRTL_ACTIVE) {
      bus.state = "ERROR-ACTIVE";
    }
  }
}

// EL05 type-17 read replies carry the value as a little-endian IEEE-754 float
// in data[4:8].
float floatFromLittleEndian(const std::array<std::uint8_t, 8> & data, std::size_t offset)
{
  const std::uint32_t bits =
    static_cast<std::uint32_t>(data[offset]) |
    (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
    (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
    (static_cast<std::uint32_t>(data[offset + 3]) << 24);
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

double El05SocketCan::clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

std::uint16_t El05SocketCan::floatToUint(double value, double lower, double upper)
{
  const double bounded = clamp(value, lower, upper);
  return static_cast<std::uint16_t>(
    (bounded - lower) * static_cast<double>(0xFFFF) / (upper - lower));
}

double El05SocketCan::uintToFloat(std::uint16_t value, double lower, double upper)
{
  return static_cast<double>(value) * (upper - lower) / static_cast<double>(0xFFFF) + lower;
}

std::uint32_t El05SocketCan::buildExtId(
  std::uint8_t comm_type, std::uint16_t data2, std::uint8_t target_id)
{
  return ((static_cast<std::uint32_t>(comm_type) & 0x1F) << 24) |
         ((static_cast<std::uint32_t>(data2) & 0xFFFF) << 8) |
         static_cast<std::uint32_t>(target_id);
}

bool El05SocketCan::open(const std::array<JointMap, kNumJoints> & joints)
{
  ok_ = true;
  last_error_.clear();
  stats_ = {};
  motor_to_index_.clear();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    auto [it, inserted] = motor_to_index_.try_emplace(joints[i].motor_id, i);
    if (!inserted) {
      fail("duplicate motor_id " + std::to_string(joints[i].motor_id)
           + " for joints " + joints[it->second].name + " and " + joints[i].name);
      return false;
    }
    if (!openBus(joints[i].can)) {
      return false;
    }
  }
  return true;
}

void El05SocketCan::close()
{
  for (auto & [_, bus] : buses_) {
    if (bus.fd >= 0) {
      ::close(bus.fd);
      bus.fd = -1;
    }
  }
  buses_.clear();
}

bool El05SocketCan::openBus(const std::string & name)
{
  if (buses_.count(name) != 0) {
    return true;
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) {
    fail("socket(" + name + "): " + std::strerror(errno));
    return false;
  }

  ifreq ifr{};
  std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", name.c_str());
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    fail("ioctl(" + name + "): " + std::strerror(errno));
    ::close(fd);
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    fail("bind(" + name + "): " + std::strerror(errno));
    ::close(fd);
    return false;
  }

  // Ask the kernel to deliver CAN error frames (the default mask is not
  // guaranteed to include them). Failure is non-fatal: hw without error
  // reporting still works, just without CAN_ERR telemetry.
  const can_err_mask_t err_mask = CAN_ERR_MASK;
  (void)::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

  const auto index = busIndex(name);
  buses_[name] = Bus{name, fd, index};
  // Assume ERROR-ACTIVE on a successfully opened bus; CAN_ERR_* frames update
  // this if the controller goes warning/passive/bus-off.
  stats_.buses[index].state = "ERROR-ACTIVE";
  return true;
}

bool El05SocketCan::sendMotionMode(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(kIndexRunMode & 0xFF),
    static_cast<std::uint8_t>((kIndexRunMode >> 8) & 0xFF),
    0,
    0,
    kRunModeMotion,
    0,
    0,
    0};
  return sendFrame(joint.can, kCommWriteParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendEnable(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{};
  return sendFrame(joint.can, kCommEnable, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendStop(const JointMap & joint, bool clear_fault)
{
  std::array<std::uint8_t, 8> data{};
  data[0] = clear_fault ? 1 : 0;
  return sendFrame(joint.can, kCommStop, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendReadFaultStatus(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(kIndexFaultStatus & 0xFF),
    static_cast<std::uint8_t>((kIndexFaultStatus >> 8) & 0xFF),
    0,
    0,
    0,
    0,
    0,
    0};
  return sendFrame(joint.can, kCommReadParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendReadParam(const JointMap & joint, std::uint16_t index)
{
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(index & 0xFF),
    static_cast<std::uint8_t>((index >> 8) & 0xFF),
    0, 0, 0, 0, 0, 0};
  return sendFrame(
    joint.can, kCommReadParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendTorqueLimit(const JointMap & joint, double nm)
{
  // 通信类型 18: Byte0~1 index, Byte2~3 = 0, Byte4~7 = float (low byte first)
  const float value = static_cast<float>(nm);
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(kIndexTorqueLimit & 0xFF),
    static_cast<std::uint8_t>((kIndexTorqueLimit >> 8) & 0xFF),
    0,
    0,
    static_cast<std::uint8_t>(bits & 0xFF),
    static_cast<std::uint8_t>((bits >> 8) & 0xFF),
    static_cast<std::uint8_t>((bits >> 16) & 0xFF),
    static_cast<std::uint8_t>((bits >> 24) & 0xFF)};
  return sendFrame(
    joint.can, kCommWriteParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendWriteParamUint32(
  const JointMap & joint, std::uint16_t index, std::uint32_t value)
{
  // 通信类型 18: Byte0~1 index, Byte2~3 = 0, Byte4~7 uint32 (low byte first).
  // No COMM_SAVE_PARAM: the value is RAM-only and reverts on power cycle.
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(index & 0xFF),
    static_cast<std::uint8_t>((index >> 8) & 0xFF),
    0,
    0,
    static_cast<std::uint8_t>(value & 0xFF),
    static_cast<std::uint8_t>((value >> 8) & 0xFF),
    static_cast<std::uint8_t>((value >> 16) & 0xFF),
    static_cast<std::uint8_t>((value >> 24) & 0xFF)};
  return sendFrame(
    joint.can, kCommWriteParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendMotion(const JointMap & joint, const MotorCommand & command)
{
  const auto tau_u = floatToUint(command.tau, t_min_, t_max_);
  const auto q_u = floatToUint(command.q, p_min_, p_max_);
  const auto dq_u = floatToUint(command.dq, v_min_, v_max_);
  const auto kp_u = floatToUint(command.kp, kp_min_, kp_max_);
  const auto kd_u = floatToUint(command.kd, kd_min_, kd_max_);

  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>((q_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(q_u & 0xFF),
    static_cast<std::uint8_t>((dq_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(dq_u & 0xFF),
    static_cast<std::uint8_t>((kp_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(kp_u & 0xFF),
    static_cast<std::uint8_t>((kd_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(kd_u & 0xFF)};

  const bool sent = sendFrame(
    joint.can, kCommControl, static_cast<std::uint8_t>(joint.motor_id), data, tau_u);
  if (sent) {
    const auto it = motor_to_index_.find(joint.motor_id);
    if (it != motor_to_index_.end()) {
      ++stats_.expected_status[it->second];
    }
  }
  return sent;
}

bool El05SocketCan::sendFrame(
  const std::string & can, std::uint8_t comm_type, std::uint8_t motor_id,
  const std::array<std::uint8_t, 8> & data, std::uint16_t data2)
{
  auto it = buses_.find(can);
  if (it == buses_.end() || it->second.fd < 0) {
    fail("CAN bus unavailable: " + can);
    return false;
  }

  can_frame frame{};
  frame.can_id = buildExtId(comm_type, data2, motor_id) | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::copy(data.begin(), data.end(), frame.data);
  auto & bus_stats = stats_.buses[it->second.index];
  const auto n = ::write(it->second.fd, &frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    const int write_errno = errno;
    if (write_errno == EAGAIN || write_errno == EWOULDBLOCK || write_errno == ENOBUFS) {
      // Transient TX-queue full: drop this frame; the next control tick resends
      // the same target. Do not latch ok_ or fault the whole robot.
      ++stats_.tx_queue_full;
      ++bus_stats.tx_queue_full;
      return true;
    }
    ++stats_.write_errors;
    ++bus_stats.write_errors;
    fail("write(" + can + "): " + std::strerror(write_errno));
    return false;
  }
  ++stats_.frames_sent;
  ++bus_stats.tx;
  return true;
}

void El05SocketCan::drain(std::array<MotorState, kNumJoints> & states, double now_s)
{
  for (auto & [_, bus] : buses_) {
    auto & bus_stats = stats_.buses[bus.index];
    while (true) {
      can_frame frame{};
      const auto n = ::read(bus.fd, &frame, sizeof(frame));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        ++stats_.read_errors;
        ++bus_stats.read_errors;
        fail("read(" + bus.name + "): " + std::strerror(errno));
        break;
      }
      if (n != static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }
      if (frame.can_id & CAN_ERR_FLAG) {
        parseCanError(frame.can_id, frame, bus_stats);
        continue;
      }
      if (!(frame.can_id & CAN_EFF_FLAG)) {
        continue;
      }
      std::array<std::uint8_t, 8> data{};
      std::copy(frame.data, frame.data + std::min<std::size_t>(frame.can_dlc, 8), data.begin());
      ++stats_.frames_received;
      ++bus_stats.rx;
      parseFrame(frame.can_id & kCanEffMask, data, states, now_s);
    }
  }
}

void El05SocketCan::parseFrame(
  std::uint32_t can_id, const std::array<std::uint8_t, 8> & data,
  std::array<MotorState, kNumJoints> & states, double now_s)
{
  const auto comm_type = static_cast<std::uint8_t>((can_id >> 24) & 0x1F);
  const auto data2 = static_cast<std::uint16_t>((can_id >> 8) & 0xFFFF);
  const int motor_id = data2 & 0xFF;
  const auto it = motor_to_index_.find(motor_id);
  if (it == motor_to_index_.end()) {
    return;
  }

  auto & state = states[it->second];
  if (comm_type == kCommReadParam) {
    ++state.param_frames;
    const auto index = static_cast<std::uint16_t>(data[0] | (data[1] << 8));
    if (index == kIndexRunMode) {
      state.run_mode = static_cast<int>(std::lround(
        static_cast<double>(floatFromLittleEndian(data, 4))));
      return;
    }
    if (index == kIndexMechPos) {
      state.position = static_cast<double>(floatFromLittleEndian(data, 4));
      state.received = true;
      state.last_feedback_s = now_s;
      state.last_position_s = now_s;
      return;
    }
    if (index == kIndexMechVel) {
      state.velocity = static_cast<double>(floatFromLittleEndian(data, 4));
      state.received = true;
      state.last_feedback_s = now_s;
      state.last_velocity_s = now_s;
      return;
    }
    if (index == kIndexFaultStatus) {
      state.param_fault =
        static_cast<std::uint32_t>(data[4]) |
        (static_cast<std::uint32_t>(data[5]) << 8) |
        (static_cast<std::uint32_t>(data[6]) << 16) |
        (static_cast<std::uint32_t>(data[7]) << 24);
      state.fault_status_received = true;
    }
    if (index == kIndexCanTimeout) {
      state.can_timeout_units =
        static_cast<std::uint32_t>(data[4]) |
        (static_cast<std::uint32_t>(data[5]) << 8) |
        (static_cast<std::uint32_t>(data[6]) << 16) |
        (static_cast<std::uint32_t>(data[7]) << 24);
      state.can_timeout_received = true;
    }
    return;
  }

  if (comm_type != kCommStatus) {
    return;
  }

  const auto pos_u = static_cast<std::uint16_t>((data[0] << 8) | data[1]);
  const auto vel_u = static_cast<std::uint16_t>((data[2] << 8) | data[3]);
  const auto trq_u = static_cast<std::uint16_t>((data[4] << 8) | data[5]);
  const auto tmp_u = static_cast<std::uint16_t>((data[6] << 8) | data[7]);

  state.position = uintToFloat(pos_u, p_min_, p_max_);
  state.velocity = uintToFloat(vel_u, v_min_, v_max_);
  state.torque = uintToFloat(trq_u, t_min_, t_max_);
  state.temperature = static_cast<double>(tmp_u) * 0.1;
  state.fault = static_cast<std::uint16_t>((data2 >> 8) & kStatusFaultMask);
  state.mode = static_cast<std::uint8_t>((data2 >> 14) & 0x03);
  if (state.last_status_s > 0.0) {
    const double gap = now_s - state.last_status_s;
    state.last_gap_s = gap;
    state.max_gap_s = std::max(state.max_gap_s, gap);
  }
  state.last_position_s = now_s;
  state.last_velocity_s = now_s;
  state.last_status_s = now_s;
  ++state.status_frames;
  state.received = true;
  state.last_feedback_s = now_s;
}

void El05SocketCan::fail(const std::string & error)
{
  ok_ = false;
  last_error_ = error;
}

}  // namespace opendoge
