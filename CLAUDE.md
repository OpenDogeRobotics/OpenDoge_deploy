# OpenDoge Firmware — 四足机器人实机部署仓库

## 目录结构与职责

```
OpenDoge_firmware/
├── hardware/                 ← 硬件 I/O 层
│   ├── motor/                ← EL05 CAN 协议 + 全部电机调测工具
│   ├── imu/                  ← DM-IMU-L1 串口桥接
│   └── gamepad/              ← Xbox 手柄桥接
│
├── daemons/                  ← systemd 后台持久化服务单元
│
├── deploy/                   ← C++ 实机部署运行时 (生产代码)
│   ├── main.cpp              ← 主循环编排
│   ├── cli.cpp/hpp           ← CLI 参数解析
│   ├── controller.cpp/hpp    ← 状态机 + PD 控制回路
│   ├── safety.cpp/hpp        ← 安全监控 (力矩/跟踪/倒地/温度)
│   ├── observer.cpp/hpp      ← 观测构建 + 步态相位
│   ├── status.cpp/hpp        ← JSON 状态输出
│   ├── policy.cpp/hpp        ← 策略抽象层
│   ├── onnx_policy.cpp       ← ONNX Runtime backend
│   └── runtime_io.cpp/hpp    ← 文件 I/O
│
├── sim2sim/                  ← sim2sim 部署管线验证仿真
│   ├── deploy_mujoco.py      ← 复现 C++ 控制回路 (必须与 main.cpp 同步)
│   ├── calc_zero_offset.py   ← URDF 零位 → 趴伏补偿角
│   └── CLAUDE.md             ← 训练↔部署 Gap 追踪记录
│
├── test/                     ← 其他测试 (回归测试等)
├── web_tools/                ← Web 控制台 (独立工具)
├── scripts/                  ← Shell 编排 (CAN 启动、整机启动)
├── docs/URDF/                ← 机器人描述 (URDF + MuJoCo XML)
└── policy/                   ← ONNX 策略模型
```

## 核心不变量

### 1. 电机类型

**全部 12 个关节使用灵足 EL05 (RobStride) 准直驱电机模组**，6 N·m 版本。

| 属性 | 值 |
|------|-----|
| 制造商 | 北京灵足时代科技有限公司 (robstride.com) |
| 手册 | `docs/EL05使用说明书2600428.pdf` |
| CAN 协议 | 29-bit 扩展帧, 私有 RobStride 协议 (非 CANopen) |
| 默认波特率 | 1 Mbps |
| 出厂默认 CAN ID | **127** (0x7F) — 接入前必须改为 1-12 |
| 上位机软件 | EDULITE-TOOL (Windows, 用于改 ID/升级固件) |

> ⚠️ `docs/usb2can/mi_motor_demo_TB.py` 是**小米电机**的供应商参考脚本，**不是 EL05 协议**。
> 它仅作为 `python-can` + SocketCAN 的链路参考保留。EL05 协议实现见下方。

### 2. EL05 协议一致性

EL05/RobStride CAN 协议在以下三处实现，参数必须一致：

| 位置 | 语言 | 用途 |
|------|------|------|
| `hardware/motor/el05_socketcan.cpp` | C++ | 生产部署 |
| `hardware/motor/el05_motor_menu.py` | Python | 硬件调测 |
| `hardware/motor/protocol_selftest.py` | Python | 协议自检 |

同步项：CAN ID 构造 (`buildExtId`)、float↔uint 映射范围 (P_MIN=-12.57, P_MAX=12.57, V_MIN=-50, V_MAX=50, T_MIN=-6, T_MAX=6, KP_MAX=500, KD_MAX=5)、comm_type 常量 (0x00-0x19)、参数索引。

参数索引：

| 索引 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 0x7005 | run_mode | uint32 | 运行模式 (0=运控, 1=位置, 2=速度, 3=电流) |
| 0x7019 | mechPos | float | 机械位置 (rad) |
| 0x701B | mechVel | float | 机械速度 (rad/s) |
| 0x7029 | zero_sta | **uint8** | **零点标志位: 0=0~2π, 1=-π~π (必须设为1!)** |
| 0x3022 | faultSta | uint32 | 故障状态寄存器 |

完整 comm_type 列表：

| 类型 | 值 | 用途 | 实现 |
|------|-----|------|------|
| COMM_GET_DEVICE_ID | 0x00 | 获取设备 ID (广播扫描) | ✅ scan 工具 |
| COMM_CONTROL | 0x01 | 运控模式控制指令 | ✅ |
| COMM_STATUS | 0x02 | 电机反馈数据 | ✅ |
| COMM_ENABLE | 0x03 | 电机使能运行 | ✅ |
| COMM_STOP | 0x04 | 电机停止运行 | ✅ |
| COMM_SET_ZERO | 0x06 | 设置机械零位 (需运控模式, 不先stop) | ✅ |
| COMM_SET_CAN_ID | 0x07 | 设置电机 CAN ID | ✅ |
| COMM_READ_PARAM | 0x11 | 单个参数读取 | ✅ |
| COMM_WRITE_PARAM | 0x12 | 单个参数写入 | ✅ |
| COMM_FAULT_FEEDBACK | 0x15 | 故障反馈帧 | - |
| COMM_SAVE_PARAM | 0x16 | **保存参数到 Flash (标零/改参后必须调用)** | ✅ |

### 3. CAN 硬件 (USB-CAN 适配器)

生产链路：

```
Orange Pi 5 → USB 2.0 Hub (带外部供电) → 4× CANable (gs_usb, candleLight)
                                           → can0/can1/can2/can3 → EL05 电机
```

| 属性 | 值 |
|------|-----|
| 适配器芯片 | candleLight / CANable (gs_usb 驱动) |
| USB VID/PID | `1d50:606f` |
| 内核模块 | `gs_usb` |
| CAN 接口数 | 4 (can0-can3) |
| 供电 | **USB Hub 必须带外部供电** (Orange Pi 5 供电不足) |

> **当前实际适配器**: 赤乌（CHIWU）4×CAN + 3×RS485 控制板，单 USB-A，
> 原生 SocketCAN `can0-can3`，见 `docs/usb2can/chiwu_usb2can.md`。
> 不创建 vcan、不需要 userspace bridge；EL05 仍使用经典 CAN 1 Mbps，不要开 FD。
>
> 旧的 DINGLAB/SOULDE USB2CAN V2.0（`1483:1110`, CDC-ACM + vcan 桥接）
> 已归档到 `~/DingLab usb转can代码`，见 `docs/legacy/README.md`。
> 达妙 DM-USB2CANFD Dual（`gs_usb` 固件）保留为备选，当前不使用。

CAN 接口由 `opendoge-can.service` (system oneshot) 在开机时自动配置。手动管理：
```bash
sudo systemctl status opendoge-can              # 查看状态
sudo systemctl restart opendoge-can             # 重新配置 4 个 CAN 口
sudo journalctl -u opendoge-can -n 20           # 查看最近日志
sudo scripts/reset_can_usb.sh                   # 电机断电导致 CHIWU 板复位后 (err=-110)
```

底层手动命令 (通常不需要，用 systemd 即可)：
```bash
sudo modprobe gs_usb can can_raw
sudo ./scripts/setup_can.sh can0 1000000
# ... can1-3 同理
```

只读扫描全部电机 (不使能、不写参数)：
```bash
python3 hardware/motor/scan_motors_readonly.py
```

> ⚠️ 新电机出厂 ID 为 **127** (0x7F)，不是 1-12。需要用 Windows 上位机 EDULITE-TOOL
> 逐个改为 OpenDoge 标准 ID。未改 ID 的电机 `scan_motors_readonly.py` 仍能发现
> 但 `opendoge_deploy` 不会识别。

### 4. 控制回路同步 (C++ ↔ Python)

`deploy/src/main.cpp` 和 `sim2sim/deploy_mujoco.py` 实现相同的控制回路。任何修改必须双向同步：

- 状态机转换规则 (`RuntimeState` enum, `updateStateMachine` / `DeployController.step`)
- 观测构建 (`buildObservation` / `build_observation`) — 49 维格式不可变
- 步态相位 (`advancePhase` / `advance_phase`) — 公式不可变
- rate limiter 语义 (RL 模式跳过, PC 模式 3 rad/s)
- PD 增益逻辑 (阻尼/斜坡/Active/LowGain)
- `last_action` 的 clamp 语义

### 5. 电机 ID 映射 (不可变)

12 个 EL05 电机分布在 4 条 CAN 总线上，每条总线挂载一条腿的 3 个关节。
电机在腿上从近端到远端依次为 **hip → thigh → calf**。

| 数组索引 | 关节名 | CAN 总线 | 电机 ID | 腿上位置 | 默认站立角 | 轴 |
|---------|--------|---------|--------|---------|-----------|-----|
| 0 | `FL_hip_joint` | can0 | 1 | FL Hip (髋) | 0.0 | X (roll) |
| 1 | `FL_thigh_joint` | can0 | 2 | FL Thigh (大腿) | 0.8 | Y (pitch) |
| 2 | `FL_calf_joint` | can0 | 3 | FL Calf (小腿) | -1.6 | Y (pitch) |
| 3 | `FR_hip_joint` | can1 | 4 | FR Hip (髋) | 0.0 | X (roll) |
| 4 | `FR_thigh_joint` | can1 | 5 | FR Thigh (大腿) | 0.8 | Y (pitch) |
| 5 | `FR_calf_joint` | can1 | 6 | FR Calf (小腿) | -1.6 | Y (pitch) |
| 6 | `RL_hip_joint` | can2 | 7 | RL Hip (髋) | 0.0 | X (roll) |
| 7 | `RL_thigh_joint` | can2 | 8 | RL Thigh (大腿) | 0.8 | Y (pitch) |
| 8 | `RL_calf_joint` | can2 | 9 | RL Calf (小腿) | -1.6 | Y (pitch) |
| 9 | `RR_hip_joint` | can3 | 10 | RR Hip (髋) | 0.0 | X (roll) |
| 10 | `RR_thigh_joint` | can3 | 11 | RR Thigh (大腿) | 0.8 | Y (pitch) |
| 11 | `RR_calf_joint` | can3 | 12 | RR Calf (小腿) | -1.6 | Y (pitch) |

**物理接线对应：**
- CAN 口 0 → 左前腿 (FL) → 电机 1/2/3
- CAN 口 1 → 右前腿 (FR) → 电机 4/5/6
- CAN 口 2 → 左后腿 (RL) → 电机 7/8/9
- CAN 口 3 → 右后腿 (RR) → 电机 10/11/12

**ONNX 策略输出 → 电机 ID 映射：**
ONNX 推理输出是 12 维数组，**无任何重排序**，直接按索引 1:1 对应：

```
ONNX output[i] → action[i] → target[i] → motor_id = i+1
```

即 ONNX 输出的第 0 维控制电机 1 (FL_hip)，第 1 维控制电机 2 (FL_thigh)，以此类推。

**设置电机 ID 时需要将出厂默认 127 逐个改为表中 Motor ID。** 每条 CAN 总线上的 3 个电机按物理位置 (hip→thigh→calf) 分别设为 1-3/4-6/7-9/10-12。

### 6. 默认站立姿态 (必须匹配训练 default_joint_pos)

```
全部四条腿 (固件 URDF 约定): hip=0, thigh=0.8, calf=-1.6
```

### 7. 观测格式 (48 维，LainLab mjlab，无 privileged info)

```
base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
+ joint_pos_rel(12) + joint_vel_rel(12) + last_action(12) + command(3)
```

关节量与动作需经 `kJointSign = {+1,+1,+1, +1,-1,-1, -1,+1,+1, -1,-1,-1}`
在固件 URDF 约定与训练约定之间映射。`last_action` 为原始策略输出（不裁剪）。
`base_lin_vel` 在真机上由 `lin_vel_source=zero|command|imu` 决定。
详见 `sim2sim/CLAUDE.md` Round 4。

## systemd 服务架构

所有硬件 I/O 守护进程和 CAN 接口均由 systemd 管理，开机自启。`opendoge_deploy` 不直接操作硬件，通过文件 IPC 读取数据。

```
系统服务 (root, 开机自启):
  opendoge-can.service       → 配置 can0-3 @ 1 Mbps (oneshot)
  opendoge-xboxdrv.service   → xboxdrv → /dev/input/js0

用户服务 (linger, 开机自启):
  opendoge-imu.service       → dm_imu_bridge.py → /tmp/opendoge_imu.state
  opendoge-joystick.service  → xbox_command_bridge.py → /tmp/opendoge_command.state
```

服务依赖链：
```
opendoge-can (can0-3 UP)
opendoge-xboxdrv → /dev/input/js0 → opendoge-joystick → /tmp/opendoge_command.state
opendoge-imu (auto-detect /dev/ttyUSBx by USB ID 6877:4d55) → /tmp/opendoge_imu.state
```

> 当前硬件是赤乌（CHIWU）4×CAN 原生 SocketCAN：`opendoge-can.service`
> 直接配置 `can0-can3`。旧的 DINGLAB vcan + bridge 方案已归档到
> `~/DingLab usb转can代码`（见 `docs/legacy/README.md`），不再安装/运行。

C++ deploy 通过 `runtime_io.cpp` 的 `readImuFile()` / `readCommandFile()` 轮询这些文件。
安全/命令轮询仍是 200 Hz；**策略观测在推理 tick（100 Hz）前会重新 `can.drain` 并
重读 IMU 文件**，避免观测等待 200 Hz 轮询相位（修改 64）。这是有意设计：
- 实时循环 (`SCHED_FIFO`) 免于处理串口/HID I/O
- 守护进程可独立重启
- dry-run 测试可直接写入文件注入假数据

**一键安装所有服务：**
```bash
bash scripts/install_services.sh
```

### DM-IMU-L1 状态

| 属性 | 值 |
|------|-----|
| 硬件 | DM-Tech DM-IMU-L1 |
| USB ID | `6877:4d55` |
| 串口 | `/dev/ttyUSBx` (自动检测, 服务启动时根据 USB ID 查找) |
| 桥接脚本 | `hardware/imu/dm_imu_bridge.py` |
| systemd 服务 | `opendoge-imu.service` (用户服务, Restart=always) |
| 输出文件 | `/tmp/opendoge_imu.state` |
| 数据格式 | `wx wy wz gx gy gz` (角速度 rad/s, projected gravity 已取反) |
| 当前状态 | ✅ 正常运行, `gz ≈ -1.0` (竖直), gyro 噪声 < 0.02 rad/s |

验证命令：
```bash
cat /tmp/opendoge_imu.state                        # 查看实时 IMU 数据
systemctl --user status opendoge-imu               # IMU bridge 状态
journalctl --user -u opendoge-imu -f               # IMU bridge 实时日志
lsusb | grep 6877:4d55                              # 确认 IMU 已连接
```

### Xbox 2.4G 手柄状态

| 属性 | 值 |
|------|-----|
| 硬件 | 2.4G Xbox 兼容接收器（`413d:2104` 或 Zikway `3537:1041`） |
| USB ID | `413d:2104`（xboxdrv）/ `3537:1041`（内核原生 js0） |
| 驱动 | `413d:2104` → xboxdrv；`3537:1041` → 内核 `hid-generic`/`joydev` |
| 设备节点 | `/dev/input/js0`（名称取决于驱动） |
| 桥接脚本 | `hardware/gamepad/xbox_command_bridge.py` |
| 输出文件 | `/tmp/opendoge_command.state` |
| 当前状态 | ⚠️ 接收器为 `3537:1041`，原生 js0；`opendoge-xboxdrv` 已改为包装脚本，按钮映射待 `jstest` 确认 |

**轴/按键映射** (xboxdrv 默认, 已验证):

| 物理输入 | js 事件 | 功能 |
|----------|---------|------|
| 左摇杆 Y | axis1 | vx (前后速度) |
| 左摇杆 X | axis0 | vy (侧向速度) |
| 右摇杆 X | axis2 | yaw_rate (转向角速度) |
| A | btn0 | 激活 + 位置控制模式 |
| B | btn1 | 退出位置控制/RL，回到阻尼（不是电机失能） |
| X | btn2 | 进入 RL 推理 |
| Y | btn3 | 退出 RL 推理 |
| BACK | btn6 | 切换 low_gain_mode |
| START | btn7 | 切换使能 |
| RB | btn5 | 可配置为急停 (`--btn-estop`)；当前实机 Zikway 用 btn11 |

> Zikway `3537:1041` 原生 js0 的按钮编号：A=0、B=1、X=btn3、Y=btn4、
> START=btn6(LB)、BACK=btn7(RB)、**急停=btn11**（Home 右边、三个横杠平行排列的键）。
> 急停是**锁存**：按一次置 `estop=true`、active=false、速度归零；再按 **A**
> 清除 estop 并发送 `clear_fault` 恢复。**不需要一直按住**。
> 与 sim2sim `--joystick` 默认值一致（sim 不含急停键）；
> `xbox_command_bridge.py --profile auto` 会自动套用，仍建议用
> `jstest /dev/input/js0` 确认。

**systemd 服务** (开机自启):

```
opendoge-xboxdrv.service   (system) → start_gamepad.sh (xboxdrv or native js0), Type=simple, Restart=always
opendoge-joystick.service  (user)   → bridge, Type=simple, Restart=always
opendoge-imu.service       (user)   → DM-IMU-L1 bridge, Type=simple, Restart=always
opendoge-can.service       (system) → CAN interface setup, Type=oneshot
```

服务链：`opendoge-xboxdrv` → `/dev/input/js0` → `opendoge-joystick` → `/tmp/opendoge_command.state`

**稳定性机制**:
- xboxdrv 崩溃 → systemd `Restart=always`, 5s 后自动拉起
- js0 消失 → bridge 捕获 `DeviceLostError`, 写安全中性命令, 自动重连
- IMU 串口断开 → bridge 内建重连逻辑 + systemd `Restart=always`, 3s 后自动拉起
- USB 热插拔 → `ExecStartPre` 等待硬件出现 (最多 15-30s)
- 无限重启次数 (`StartLimitIntervalSec=0`)

日常管理：
```bash
# CAN 接口
systemctl status opendoge-can                     # CAN 状态
sudo journalctl -u opendoge-can -n 20             # CAN 最近日志

# Xbox 手柄
systemctl status opendoge-xboxdrv                 # xboxdrv 状态
systemctl --user status opendoge-joystick         # bridge 状态
journalctl --user -u opendoge-joystick -f         # bridge 实时日志
watch -n 0.2 cat /tmp/opendoge_command.state      # 监控手柄命令
sudo journalctl -u opendoge-xboxdrv -f            # xboxdrv 实时日志

# IMU
systemctl --user status opendoge-imu              # IMU bridge 状态
journalctl --user -u opendoge-imu -f              # IMU bridge 实时日志
watch -n 0.2 cat /tmp/opendoge_imu.state          # 监控 IMU 数据

# 一键检查所有服务
bash scripts/start_robot.sh verify
```

安装/重装：
```bash
bash scripts/install_services.sh
```

## 构建与测试

```bash
# 构建
export ONNXRUNTIME_ROOT=$(realpath build/deps/onnxruntime)
colcon build --symlink-install --packages-select opendoge_deploy
source install/setup.bash

# dry-run (无硬件)
./install/opendoge_deploy/bin/opendoge_deploy --policy-backend none --duration-sec 2

# MuJoCo 仿真验证
cd test && python3 deploy_mujoco.py --mode idle --no-render --duration 2

# vcan 无硬件 CAN 测试
sudo ./scripts/setup_vcan.sh can0 can1 can2 can3
./install/opendoge_deploy/bin/opendoge_deploy --real --enable --allow-missing-imu --policy-backend none --duration-sec 1
```

## Bringup 进度

> 记录实机 bringup 关键里程碑。每次验证通过后更新日期和状态。

| 日期 | 里程碑 | 状态 |
|------|--------|------|
| 2026-06-26 | CAN 接口: 4× CANable (gs_usb), can0-3 全部 UP @ 1 Mbps, LOWER_UP ✅ | ✅ |
| 2026-06-26 | 电机 ID 验证: 12/12 EL05 全部在线, ID 1-12 正确, 通道分配正确, 运控模式, 无故障 | ✅ |
| 2026-06-26 | 电机零位标定: 趴伏姿态标零 + offset 补偿 (MuJoCo 仿真趴伏角→校准偏移) | ✅ |
| 2026-06-26 | 电机方向检测: MuJoCo 仿真验证 URDF 正方向 + discover_direction.py 12/12 实测 | ✅ |
| - | IMU 数据流验证: `/tmp/opendoge_imu.state` 正常输出 | ⬜ |
| - | Xbox 手柄数据流验证: `/tmp/opendoge_command.state` 正常输出 | ⬜ |
| - | C++ deploy 空策略 dry-run: `--policy-backend none --real` 无 CAN 错误 | ⬜ |
| - | C++ deploy PD 站立: 位置控制模式, 12 电机稳定站立 | ⬜ |
| - | C++ deploy RL 行走: ONNX 推理闭环, 稳定行走 | ⬜ |

### 电机方向检测 (2026-06-26)

通过 MuJoCo 仿真 + 实机推杆验证，确定每个关节的 `direction` 参数 (±1)。
**原理**: 推关节朝 URDF 正方向, mechPos 增大 → +1, 减小 → -1。

**URDF 正方向物理含义** (经 MuJoCo scene.xml 验证):

| 关节类型 | 轴 | URDF +angle | 推杆操作 |
|----------|-----|------------|----------|
| FL/RL Hip | X (1,0,0) | 外展 (远离中线) | 向外推 |
| FR/RR Hip | X (1,0,0) | 内收 (靠近中线) | **向内推** |
| 全部 Thigh | Y (0,1,0) | 伸髋 (膝盖向尾部) | **向后推 (膝盖朝尾部)** |
| 全部 Calf | Y (0,1,0) | 伸膝 (脚远离) | 伸直小腿 |

⚠️ FR/RR hip 与 FL/RL hip 的 URDF 正方向物理含义相反 (都绕 +X 轴, 右手定则下左右侧对称)。

**实测结果** (`discover_direction.py`, 2026-06-26):

| 关节 | dir | 关节 | dir |
|------|-----|------|-----|
| FL_hip | **-1** | FR_hip | **-1** |
| FL_thigh | **-1** | FR_thigh | **+1** |
| FL_calf | +1 | FR_calf | **-1** |
| RL_hip | +1 | RR_hip | +1 |
| RL_thigh | **-1** | RR_thigh | **+1** |
| RL_calf | +1 | RR_calf | **-1** |

规律: Calf 左右镜像 (L=+1, R=-1); Thigh 因 URDF 正方向朝尾部，映射为
L=-1, R=+1（与 Calf 相反）; Hip 前后分 (F=-1, R=+1)。

> ⚠️ 2026-11-06 修正（修改 54）：Thigh 的 URDF 正方向是“膝盖向尾部”，
> 旧表写的“屈髋（膝向前）”是错的，导致四条大腿的 direction 反了。
> `discover_direction.py` 的 thigh 提示已改为“向后推（膝盖往尾部方向）”，
> 上表已按修正后的约定更新。髋和小腿不受影响。

### 电机零位标定方案 (2026-06-26)

采用 **趴伏标零 + 补偿角** 方案，避免抬起机器人的风险。

**原理**: 机器人在平地趴伏时对所有电机执行 COMM_SET_ZERO (mechPos=0)。
此时 URDF 关节角 = 趴伏角 (MuJoCo 仿真计算)。
通过 `calibration.offset = -direction * prone_angle` 使 `logicalPosition(0) = prone_angle`，
从而 `motorPosition(default_pos) = default_pos - prone_angle` 正确映射站立姿态。
(direction=+1 时 offset = -prone_angle; direction=-1 时 offset = +prone_angle)

**关键发现**: EL05 手册 4.2.5 节 — "csp 和运控模式下可以标零，pp 模式标零会屏蔽"。
`el05_motor_menu.py` 选项 9 原先 `stop()` → `set_zero()` 的流程会破坏运控模式导致标零失效。
**修复**: 移除 stop(), 直接发 COMM_SET_ZERO + COMM_SAVE_PARAM (0x16) 持久化。

**zero_sta 配置 (2026-06-26)**: 参数索引 **0x7029** (uint8), 出厂默认 0 (位置范围 0~2π)。
设为 **1** (范围 -π~π) 可解决编码器在零点附近的 π 叠加故障。
修改后需重启电机生效。CAN 即可写入，不需要 Windows 上位机。

**MuJoCo 趴伏仿真角度 & deployed 校准参数** (calc_zero_offset.py + discover_direction.py):

| 关节 | 趴伏角 (rad) | **dir** | **offset** | 关节 | 趴伏角 (rad) | **dir** | **offset** |
|------|-------------|---------|-----------|------|-------------|---------|-----------|
| FL_hip | +0.0600 | -1 | **+0.0600** | FR_hip | -0.0600 | -1 | **-0.0600** |
| FL_thigh | +0.7615 | +1 | **-0.7615** | FR_thigh | +0.7615 | -1 | **+0.7615** |
| FL_calf | -2.2623 | +1 | **+2.2623** | FR_calf | -2.2624 | -1 | **-2.2624** |
| RL_hip | +0.2610 | +1 | **-0.2610** | RR_hip | -0.2610 | +1 | **+0.2610** |
| RL_thigh | +1.1358 | +1 | **-1.1358** | RR_thigh | +1.1358 | -1 | **+1.1358** |
| RL_calf | -2.6275 | +1 | **+2.6275** | RR_calf | -2.6275 | -1 | **-2.6275** |

> offset = -direction × prone_angle。与 discover_direction.py 实测结果一致，已写入 deploy/configs/opendoge_deploy.conf。

⚠️ 仿真中 RL/RR hip 和 thigh 触及关节限位，实机趴伏姿态可能有偏差。
首次使能 (PD 站立) 前需将机器人抬离地面，LowGain 模式验证。

### 上次扫描结果 (2026-06-26, 标零后)

```
can0 (FL): ID=1(hip +0.0000) 2(thigh +0.0000) 3(calf +0.0000) — 全部运控/无故障 ✅
can1 (FR): ID=4(hip +0.0000) 5(thigh +0.0000) 6(calf +0.0000) — 全部运控/无故障 ✅
can2 (RL): ID=7(hip +0.0000) 8(thigh +0.0000) 9(calf +0.0000) — 全部运控/无故障 ✅
can3 (RR): ID=10(hip +0.0000) 11(thigh -0.0000) 12(calf -0.0017) — 全部运控/无故障 ✅
```

> 电机零位标定 + 方向检测完成，配置已更新。下一步: 将机器人抬至站立姿态, LowGain 模式 PD 站立验证。

## 修改指南

1. **新增配置参数**: `types.hpp` → `runtime_io.cpp` → `main.cpp`
2. **修改控制回路**: `controller.cpp` + `deploy_mujoco.py` 同步
3. **修改 CAN 协议**: `el05_socketcan.cpp` + `hardware/motor/el05_motor_menu.py` + `protocol_selftest.py` 同步
4. **新增 policy backend**: 实现 `Policy` 接口，在 `makePolicy()` 注册
5. **XML 物理参数对齐**: 固件 `Opendoge.xml` 的物理参数必须对齐 UniLab 训练 `opendoge.xml`+`scene_flat.xml`，追踪记录见 `sim2sim/CLAUDE.md`

## 相关仓库

- UniLab 训练框架: `../UniLab/`
- OpenDoge 硬件设计: `../OpenDoge_hardware/`
- 模块详细文档: `deploy/CLAUDE.md`
