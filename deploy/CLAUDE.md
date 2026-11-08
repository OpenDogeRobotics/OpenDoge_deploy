# opendoge_deploy — C++ 实机部署模块

## 架构概述

`opendoge_deploy` 是非 ROS 的 C++ 实机 RL 部署运行时。完整链路：

```
hardware/imu/dm_imu_bridge.py             → /tmp/opendoge_imu.state      → opendoge_deploy 文件轮询
hardware/gamepad/xbox_command_bridge.py  → /tmp/opendoge_command.state  → opendoge_deploy 文件轮询
opendoge_deploy (C++, SCHED_FIFO)
  → ONNX policy 推理 @ 100 Hz (CPU)；推理前重新 drain CAN + 重读 IMU 文件
  → 目标位置计算 @ 200 Hz (PD + rate limit)
  → EL05 运控帧发送 @ control_hz (q/dq/tau/kp/kd)
  → SocketCAN can0/can1/can2/can3
  → USB-CAN 适配器 (CHIWU 4×CAN, 原生 can0-3) → EL05 电机
```

## 文件职责

| 文件 | 职责 | 改动约束 |
|------|------|----------|
| `types.hpp` | 数据类型 (JointMap, MotorState, DeployConfig, ImuSample 等) + 工具函数 | 全局共享，新增字段需同步 `deploy_mujoco.py` |
| `policy.hpp/cpp` | 策略抽象层 (Policy 基类 + makePolicy 工厂) | 只加 backend，不改接口 |
| `onnx_policy.cpp` | ONNX Runtime 策略后端 | 无特殊限制 |
| `el05_socketcan.hpp/cpp` | EL05 私有 CAN 协议层 | 协议常量 (kComm*, kIndex*) 需与 `hardware/motor/el05_motor_menu.py` 保持一致 |
| `runtime_io.hpp/cpp` | 文件 I/O (配置解析、command.state、imu.state) | 格式见下方 |
| `main.cpp` | **主循环 + CLI + 状态机 + 观测构建 + 安全监控 + PD 控制 + JSON 状态输出** (god object) | **重构目标** |

## 当前问题

### main.cpp god object (1047行)

`main.cpp` 包含以下应独立模块化的逻辑：

1. **状态机** (L45-75 枚举定义, L766-842 转换逻辑) → 应拆为 `controller.cpp`
2. **观测构建** (L274-332, `buildObservation()`) → 应拆为 `observer.cpp`
3. **安全监控** (L350-465, `safetyFault()` + `JointSafetyState`) → 应拆为 `safety.cpp`
4. **步态相位** (L261-269, `advancePhase()`) → 应拆为 `gait.cpp`
5. **CLI 解析** (L96-177, `parseArgs()`) → 应拆为 `cli.cpp`
6. **JSON 状态输出** (L970-1028) → 应拆为 `status.cpp`
7. **PD 控制逻辑** (L887-943) → 应拆为 `controller.cpp`

### C++ 与 Python 控制逻辑重复

`sim2sim/deploy_mujoco.py` 中的 `DeployController` 类完整复现了 `main.cpp` 的控制回路：
- 状态机转换逻辑 — 两份实现
- 观测构建 (`buildObservation` vs `build_observation`) — 两份实现
- 步态相位 (`advancePhase` vs `advance_phase`) — 两份实现
- rate limiter — 两份实现

任何对控制逻辑的修改必须同步两份代码。当前通过 `sim2sim/CLAUDE.md` 记录 Gap 来手动追踪一致性。

### EL05 协议两份实现

| 位置 | 语言 | 用途 |
|------|------|------|
| `hardware/motor/el05_socketcan.cpp` | C++ | 生产部署 |
| `hardware/motor/el05_motor_menu.py` | Python | 硬件 bringup/调测 |
| `hardware/motor/protocol_selftest.py` | Python | 协议自检 |

CAN ID 构造、float→uint 映射范围 (P_MIN, V_MIN, T_MIN 等) 必须三处一致。

### 电机单独调参接口缺失

`opendoge_deploy` 运行时**没有**单电机调参能力。所有电机单独测试必须通过 `hardware/motor/el05_motor_menu.py` 的交互菜单完成。如果需要"在真实控制回路下测试单个电机响应"的功能，需要在 C++ 侧新增模式。

## 配置与数据格式

### command.state 格式

```
vx=0.0
vy=0.0
yaw_rate=0.0
active=false
estop=false
position_control=false
rl_inference=false
clear_fault=false
low_gain_mode=false
```

### imu.state 格式

```
wx=0.0
wy=0.0
wz=0.0
gx=0.0
gy=0.0
gz=-1.0
# 可选: 机体线速度 (m/s)。缺省时按 lin_vel_source 处理
vx=0.0
vy=0.0
vz=0.0
```

`wx/wy/wz` = 角速度 rad/s, `gx/gy/gz` = projected gravity (已取反 = -upvector)。
`vx/vy/vz` 可选；`dm_imu_bridge.py` 目前**不输出**这三个字段。

### 部署配置 (opendoge_deploy.conf)

```
# 控制频率
inference_hz=100
target_hz=200
control_hz=200   # 当前 CHIWU 原生 CAN；旧 USB-CAN bridge 时代是 100
# 修改 64：观测在推理 tick 前刷新，不再等待 200 Hz 输入轮询相位；
# 实际观测年龄看 status 的 obs_imu_age_ms / obs_joint_age_ms。

# PD 增益 (LainLab BuiltinPositionActuatorCfg: stiffness=12.0, damping=0.5)
kp=12.0
kd=0.65          # 2026-11-07 现场调参；训练值 0.5（修改 64 前记录 kd=0.8 的效果）
safe_kd=2.0
action_scale=0.25

# 48 维观测中 base_lin_vel 的来源:
#   zero    — 恒为 0 (最保守)
#   command — 用速度指令近似 (默认)
#   imu     — 读取 imu.state 的可选 vx/vy/vz
lin_vel_source=command

# EL05 电机侧失联保护 (0x7028 canTimeout, RAM-only)
# 0=关闭；非 0 时电机在这么久收不到 CAN 指令会自动 reset/释放。
# deploy 启动时逐电机写入并读回校验；必须为 0 或 >=0.5。
can_timeout_s=1.0

# 安全阈值
state_timeout_s=0.1   # 当前 live config；旧文档的 0.02 是更早的保守值
# 新增可观测/策略（默认 warn/hold/warn，不改变控制行为）
imu_timeout_s=0.25
imu_stale_action=warn       # warn|fault
feedback_hold_max_s=0.03
stale_action=hold           # hold|freeze|fault
can_error_action=warn       # warn|fault; BUS-OFF always fault
over_temperature_c=80.0
temp_warn_c=65.0
torque_threshold=3.0
torque_timeout_s=0.5
tracking_error_threshold=0.5
tracking_error_timeout_s=0.3
command_timeout_s=0.5
fall_gravity_z_threshold=0.3
fall_timeout_s=0.3
# 起立稳定段 + 交接淡入 + 倒地检测时间窗（覆盖 4 s 斜坡 + 1 s 稳定 + 1 s 交接）
standup_settle_s=1.0
standup_handover_s=1.0
standup_grace_s=6.5
feedback_wait_timeout_s=5.0
command_smoothing_alpha=0.0

# 位置控制斜坡
pc_startup_ramp_s=2.0
pc_startup_max_deviation=0.25

# 故障轮询
fault_poll_hz=10.0

# 每关节校准 (实际格式为逐字段 key=value)
joint.FL_hip_joint.direction=-1
joint.FL_hip_joint.offset=0.0000
joint.FL_hip_joint.lower=-1.0000
joint.FL_hip_joint.upper=0.5218
joint.FL_hip_joint.max_position_step=0.015
# 可选: reduction (小腿减速比 1.5), max_velocity/max_torque/max_kp/max_kd
# 注意: 四条 thigh 的 direction 为 L=-1 / R=+1、offset = -direction*0.8
# (URDF 正方向是膝盖朝尾部；见修改 54)
# ... 其余 11 关节，完整值见 deploy/configs/opendoge_deploy.conf
```

### 诊断字段 (status JSON, 修改 57/59)

- IMU：`imu_age_ms/imu_gyro_age_ms/imu_quat_age_ms/imu_stamp/imu_gyro_seq/imu_quat_seq`
  （bridge 写带内 `t=/gyro_t=/quat_t=/seq=`；冻结的文件也能看出来）。
  `obs_imu_age_ms/obs_joint_age_ms` 是**最近一次策略观测**实际使用的 IMU/关节
  样本年龄（推理前刷新后），用于验证观测延迟是否下降（修改 64）。
- 每关节：`age_ms`（joint-wide）、`pos_age_ms/vel_age_ms/status_age_ms`、
  `status_frames/expected/missed/max_gap_ms/last_gap_ms/rx_ratio`。
- CAN：`can_tx_queue_full`、`can_buses[]`（tx/rx/err/qfull/error_frames/bus_off/state）。
- `EAGAIN/ENOBUFS` 只计数并在下一 tick 重发，不再直接 DampingFault；
  `BUS-OFF` 始终 fault，`ERROR-PASSIVE` 按 `can_error_action`。
- 默认 `imu_stale_action=warn`、`stale_action=hold`、`can_error_action=warn`；
  需要更激进的行为时显式改成 `fault`/`freeze`。

### Observation 格式 (48 维, LainLab mjlab actor obs)

```
base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
+ joint_pos_rel(12) + joint_vel_rel(12) + last_action(12) + command(3)
```

- 关节位置/速度需经 `kJointSign` 从固件 URDF 约定映射到训练约定
- `last_action` 为原始策略输出，**不裁剪、不翻转符号**
- 无步态相位 (新策略端到端生成步态)
- `base_lin_vel` 由 `lin_vel_source` 配置 (真机 IMU 无此测量)
- 旧 49 维布局保留在 `buildLegacyObservation()`，仅供调试

## 与外部的关系

### 与 sim2sim/deploy_mujoco.py 的关系

`sim2sim/deploy_mujoco.py` 是 `main.cpp` 控制回路的 Python 级验证仿真器。修改 `main.cpp` 的控制逻辑后，必须同步更新 `deploy_mujoco.py`，反之亦然。关键同步点：

- 状态机转换规则
- `buildObservation()` / `build_observation()` 观测维度、顺序、归一化
- `advancePhase()` / `advance_phase()` 步态公式
- `rateLimit()` / `rate_limit()` 限速语义
- PD 增益逻辑 (阻尼/斜坡/Active/LowGain)
- `last_action` 的 clamp 语义 (RL 模式不 clamp)

### 与 LainLab 训练侧的关系

- `DeployConfig.kp/kd/action_scale` 必须匹配训练侧
  `BuiltinPositionActuatorCfg(stiffness=12.0, damping=0.5)` 与
  `JointPositionActionCfg(scale=0.25)`
- `defaultJointPosition()` 必须匹配训练侧 `default_joint_pos`，当前每腿
  `(0, 0.8, -1.6)` (固件约定)
- 观测结构必须与训练 actor 观测一致 (当前 **48 维**，含 `base_lin_vel`)
- 关节符号必须经 `kJointSign` 映射，详见 `sim2sim/CLAUDE.md` Round 4
- ONNX 模型中的 obs_normalizer 使用训练期 running mean/std
- XML 物理参数 (joint damping/frictionloss/armature, foot friction/solimp/condim,
  cone/impratio) 已对齐 → 见 `sim2sim/CLAUDE.md`

### 与 daemons / bringup 的关系

`hardware/` 是**实机硬件 I/O 层**，负责将物理设备桥接为文件 IPC：

```
hardware/imu/dm_imu_bridge.py             → 串口读数 → /tmp/opendoge_imu.state
hardware/gamepad/xbox_command_bridge.py  → HID 输入 → /tmp/opendoge_command.state
```

`opendoge_deploy` 仅通过文件轮询读取这些数据，不直接操作硬件。这是**有意设计**：
- 让 C++ 实时循环免于处理串口协议、HID 设备
- 守护进程可以独立重启而不影响主控制回路
- 文件格式简单，方便 dry-run 测试时手动注入数据

`hardware/motor/` 是 bringup/标定工具，在部署前使用，运行时不需要。

## 安全状态机

```
WaitFeedback → Ready ↔ EnteringPosition → ActivePC ↔ ActiveRL
                   ↕          ↑              ↓ (fault)
              LowGainTest     │         DampingFault
                              │
                   Ready + active + 离默认站姿 > pc_startup_max_deviation
                              → StandingUp（斜坡到站姿、不跑策略）
                              → 保持默认姿态 standup_settle_s
                              → ActivePC / ActiveRL（总时长 standup_duration_s + standup_settle_s）
                              → 交接后 standup_handover_s 内目标从默认姿态淡入策略动作
```

**起立/交接注意**：`standup_grace_s` 是从进入 `StandingUp`/`LowGainTest` 开始计的
**时间窗**，不随状态切换结束，所以覆盖“斜坡 + 稳定段 + 交接淡入”；旧版绑定
状态，会在切到 ActiveRL 的瞬间恢复倒地检测，末尾 calf 弹一下就可能误报。
交接后的 `standup_handover_s` 内仍保留 rate limiter（正常 ActiveRL 不 rate limit），
避免第一帧策略动作把目标瞬跳。

故障触发条件 (进入 `DampingFault`):
- 电机反馈丢失/超时
- 电机故障位 (`fault` / `faultSta`) 非 0
- 电机过温 (> `over_temperature_c`)
- 持续力矩超限 (> `torque_timeout_s`)
- 持续跟踪误差 (> `tracking_error_timeout_s`)
- CAN 读写异常
- command 文件 `estop=true`
- IMU 倒地检测 (`projected_gravity.z > -fall_gravity_z_threshold` 持续 > `fall_timeout_s`；
  竖直时 `gz=-1`，倾斜/倒置时趋近 0 或 +1)
- IMU 持续无效 (> `imu_debounce_count` 帧)
- IMU stale（`imu_timeout_s` + `imu_stale_action=fault`；默认 warn）
- 电机 type-2 反馈 stale（`feedback_hold_max_s` + `stale_action=fault`；默认 hold）
- CAN `BUS-OFF`（始终 fault）；`ERROR-PASSIVE`（`can_error_action=fault` 时）
- EnteringPosition 阶段关节偏差超限

故障恢复: `clear_fault=true` → 重新发送 stop(clear_fault=1) + motion_mode + enable → WaitFeedback

## 修改指南

1. **新增配置参数**: 在 `DeployConfig` (`types.hpp`) 中加字段 → `runtime_io.cpp` 的 `loadDeployConfig()` 增加解析 → `main.cpp` 按需使用
2. **新增状态**: 在 `RuntimeState` 枚举添加 → 所有 switch(state) 处增加处理
3. **修改观测格式**: `buildObservation()` + `deploy_mujoco.py` 的 `build_observation()` 同步修改，确保维度为 `kObsDim`
4. **修改 CAN 协议**: `el05_socketcan.cpp` + `hardware/motor/el05_motor_menu.py` 同步修改
5. **修改控制回路**: `main.cpp` 的控制块 (L887-943) + `deploy_mujoco.py` 的 `DeployController.step()` 同步修改
6. **新增 policy backend**: 实现 `Policy` 接口 → 在 `makePolicy()` 注册

## 电机映射 (不可变)

```
can0: FL_hip/motor 1, FL_thigh/motor 2, FL_calf/motor 3
can1: FR_hip/motor 4, FR_thigh/motor 5, FR_calf/motor 6
can2: RL_hip/motor 7, RL_thigh/motor 8, RL_calf/motor 9
can3: RR_hip/motor 10, RR_thigh/motor 11, RR_calf/motor 12
```

## 默认站立姿态 (必须匹配训练 default_joint_pos)

```
全部四条腿 (固件 URDF 约定): hip=0, thigh=0.8, calf=-1.6
```

> 该姿态是固件约定下的值，已包含与训练侧的 `kJointSign` 映射。训练侧
> `default_joint_pos` 的大小腿约为 `0.8 / -1.6`。
