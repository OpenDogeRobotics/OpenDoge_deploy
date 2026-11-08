# OpenDoge Firmware

适用：Linux 实机部署环境，四足机器人 **OpenDoge**。

当前仓库的主线是非 ROS 单进程强化学习部署：`opendoge_deploy` 直接管理 CPU ONNX 推理、4 路 SocketCAN、EL05 电机运控帧和安全阻尼。ROS / ros2_control 控制链已经从本仓库移除，避免实机路径被 controller manager、topic queue 或 ROS executor 额外耦合。

详细状态记录见 [Changelog/Codex.md](Changelog/Codex.md)。

## 目录结构

```text
OpenDoge_firmware/
  README.md
  docs/                            # EL05 手册、IMU 文档等参考
  requirements.txt                 # Python 参考工具依赖
  hardware/                        # 硬件 I/O 层
    motor/                         # EL05 CAN 协议 + 全部电机调测工具
    imu/                           # DM-IMU-L1 串口桥接
    gamepad/                       # Xbox 手柄桥接
  daemons/                         # systemd 后台持久化服务单元
  deploy/                          # C++ 实机部署运行时
  sim2sim/                         # sim2sim 管线验证仿真
  docs/                            # EL05 手册、原理图、vendor 参考
  scripts/
    setup_can.sh                   # USB-CAN (CHIWU) 原生 SocketCAN 配置
    setup_vcan.sh                  # 虚拟 CAN 无硬件测试（手动，仅离线）
    setup_onnx.sh                  # ONNX Runtime 下载安装
    start_robot.sh                 # 整机启动聚合脚本
    install_services.sh            # 一键安装 systemd 服务
  test/                            # 其他测试
  web_tools/                       # Web 控制台
  policy/                          # ONNX 强化学习策略模型
  build/ install/ log/             # colcon 生成，git 忽略
```

## 主部署链路

```text
opendoge_deploy
  -> CPU ONNX policy 100 Hz
  -> position target hold 200 Hz
  -> EL05 q/dq/tau/kp/kd control 1000 Hz
  -> SocketCAN can0/can1/can2/can3
  -> USB 转 4 路 CAN2.0 模块
  -> EL05 motors
```

`opendoge_deploy` 不依赖 ROS、rclcpp、ros2_control、hardware_interface、controller_interface 或 controller_manager。

## 电机映射

每条 CAN 总线带一条腿 3 个电机：

```text
can0: 左前 FL, motor 1/2/3  = hip/thigh/calf
can1: 右前 FR, motor 4/5/6  = hip/thigh/calf
can2: 左后 RL, motor 7/8/9  = hip/thigh/calf
can3: 右后 RR, motor 10/11/12 = hip/thigh/calf
```

部署侧、真实 URDF 和训练侧统一使用关节名：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

## 策略模型

当前部署使用 **UniLab Round 26** 模型（`policy/opendoge_r26.onnx`）。

| 属性 | 值 |
|------|-----|
| 训练 | UniLab PPO, MuJoCo, OpenDogeJoystickFlat |
| Best/Final reward | 143.08 / 113.03 |
| 输入 | 49 维单帧 observation（全部可部署，无 privileged info） |
| 输出 | 12 维 joint action |
| 网络结构 | MLP 49→512→256→128→12 (ELU) |
| 归一化 | 嵌入 ONNX (Sub+Div, 训练期 running mean/std) |
| PD 增益 | Kp=20.0, Kd=0.3 |
| Action scale | 0.50 |
| 默认站姿 | FL/FR: [0, 0.5, -1.3], RL/RR: [0, 0.7, -1.3] (对齐训练 keyframe) |

### Observation 格式（49 维）

```
gyro(3) + neg_gravity(3) + dof_pos_delta(12) + dof_vel(12)
+ last_action(12) + commands(3) + feet_phase(4)
```

| 分量 | 索引 | 说明 |
|------|------|------|
| gyro | 0:3 | IMU 角速度 (rad/s) |
| neg_gravity | 3:6 | IMU 投影重力方向 (已取反 = -upvector) |
| dof_pos_delta | 6:18 | 关节位置 − default_angles |
| dof_vel | 18:30 | 关节速度 (rad/s) |
| last_action | 30:42 | 上一帧策略输出 |
| commands | 42:45 | vx, vy, v (原始值) |
| feet_phase | 45:49 | 自适应步态相位 (FL, FR, RL, RR) |

> **关键设计**：linvel（局部线速度）从 actor 观测中移除。训练采用非对称 actor-critic — critic 保留 privileged linvel 做值估计，actor 只依赖实机可获取的 49 维观测。这消除了 sim2real 的根本性 gap。

### 命令输入

命令文件 (`--command-file`) 提供 `vx/vy/yaw_rate/active/estop`，与训练侧 `vel_limit` 对应：

| 轴 | 训练范围 |
|----|---------|
| vx | [-0.8, 0.8] m/s |
| vy | [-0.6, 0.6] m/s |
| vyaw | [-1.5, 1.5] rad/s |

## 环境准备

### ONNX Runtime（运行策略推理必须）

```bash
# 自动下载并安装到 build/deps/onnxruntime/（不进 git）
./scripts/setup_onnx.sh

# 之后每次构建前设置环境变量：
export ONNXRUNTIME_ROOT=$(realpath build/deps/onnxruntime)
```

ONNX Runtime 安装在仓库本地 `build/deps/` 下，不会被 git 追踪。`setup_onnx.sh` 支持以下选项：

```bash
ONNX_VERSION=1.20.1 ./scripts/setup_onnx.sh   # 指定版本
DEPS_DIR=~/onnx ./scripts/setup_onnx.sh       # 指定安装目录
```

### 串口权限（IMU 读取）

DM-IMU-L1 通过 USB 虚拟串口连接，设备节点属于 `dialout` 组：

```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

## 构建

> **执行根目录**：以下所有命令均在仓库根目录 `OpenDoge_firmware/` 下执行。

```bash
# 启用 ONNX 后端（必需）：
export ONNXRUNTIME_ROOT=$(realpath build/deps/onnxruntime)
colcon build --symlink-install --packages-select opendoge_deploy
source install/setup.bash
```

如果未找到 ONNX Runtime，`opendoge_deploy` 仍会构建，但只支持 `none` / `linear_csv` 后端（无法加载 ONNX 模型）。

## 验证 IMU

```bash
# 测试 IMU 数据读取（输出到终端和文件）
python3 hardware/imu/dm_imu_bridge.py \
  --source serial --device /dev/ttyACM0 --baud 921600 \
  --output /tmp/opendoge_imu_test.state
# 新开终端查看输出：
cat /tmp/opendoge_imu_test.state
# wx=0.006391  wy=0.005326  wz=0.005326
# gx=-0.016831 gy=0.023595  gz=-0.999580
```

`gz ≈ -1.0` 且陀螺仪接近零说明 IMU 水平放置且数据正常。

如果 IMU 在另一设备（如 `/dev/ttyUSB0`），使用：

```bash
python3 hardware/imu/dm_imu_bridge.py \
  --source serial --device /dev/ttyUSB0 --baud 921600 \
  --output /tmp/opendoge_imu.state &
```

## 运行

### 快速启动整机

非 ROS 部署下，`scripts/start_robot.sh` 承担类似 ROS launch 的聚合启动职责：初始化四路 SocketCAN，启动 IMU bridge、手柄 command bridge，并运行 `opendoge_deploy` 主程序。

先做 dry-run，确认二进制和配置文件可用：

```bash
./scripts/start_robot.sh dry
```

上机后的第一步建议只进入实机阻尼/安全保持模式，不加载策略：

```bash
./scripts/start_robot.sh damping
```

运行 ONNX 策略时显式传入模型路径：

```bash
POLICY_PATH=policy/opendoge_r26.onnx \
  ./scripts/start_robot.sh policy
```

常用环境变量：

```bash
IMU_DEVICE=/dev/ttyACM0
JOYSTICK_DEVICE=/dev/input/js0
COMMAND_FILE=/tmp/opendoge_command.state
IMU_FILE=/tmp/opendoge_imu.state
REALTIME_ARGS="--realtime --cpu 0"
```

脚本默认写入 `active=false` 和 `estop=false` 的初始 command 文件；使用手柄时 `--require-rb` 会要求按住 RB 才输出 active，避免上电后直接进入主动运动。

### 无硬件 dry-run 测试

```bash
# 不使用任何硬件，仅验证二进制和模型加载
./install/opendoge_deploy/bin/opendoge_deploy --policy-backend none --duration-sec 2
```

主动 dry-run，验证状态机可以进入 active：

```bash
./install/opendoge_deploy/bin/opendoge_deploy \
  --policy-backend none --start-active --cmd 0.1 0.0 0.0 --duration-sec 2
```

### ONNX 策略 dry-run（IMU 输入）

启动 IMU bridge 后，用真实 IMU 数据验证 ONNX 推理全链路：

```bash
# 终端 1: 启动 IMU bridge
python3 hardware/imu/dm_imu_bridge.py \
  --source serial --device /dev/ttyACM0 --baud 921600 \
  --output /tmp/opendoge_imu_test.state &

# 终端 2: 运行 ONNX 推理 dry-run
./install/opendoge_deploy/bin/opendoge_deploy \
  --policy-backend onnx \
  --policy-path policy/opendoge_r26.onnx \
  --imu-file /tmp/opendoge_imu_test.state \
  --start-active --cmd 0.1 0.0 0.0 \
  --duration-sec 2
```

期望输出：

```text
OpenDoge deploy running: dry-run, policy=onnx, control=1000Hz
state=active active_cmd=1 imu=1 ctrl_ticks=1000 infer_ticks=100 target_ticks=200 ...
```

运行时状态每秒输出一次，包含控制 tick、推理 tick、target tick、最大控制延迟、missed deadline、CAN 收发和错误计数。

### 可选实时性参数

```bash
./install/opendoge_deploy/bin/opendoge_deploy \
  --policy-backend onnx --policy-path policy/gen52_model4800.onnx \
  --realtime --cpu 0 --duration-sec 2
```

`--realtime` 会尝试 `mlockall` 和 `SCHED_FIFO`；权限不足时只打印 warning，不会阻止运行。

### 实机 CAN 测试

实机运行前启动四路 CAN：

```bash
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000
sudo ./scripts/setup_can.sh can2 1000000
sudo ./scripts/setup_can.sh can3 1000000
```

实机阻尼/站姿安全测试：

```bash
./install/opendoge_deploy/bin/opendoge_deploy --real --enable --policy-backend none
```

ONNX 策略实机运行：

```bash
./install/opendoge_deploy/bin/opendoge_deploy \
  --real --enable \
  --policy-backend onnx \
  --policy-path policy/opendoge_r26.onnx \
  --imu-file /tmp/opendoge_imu.state \
  --command-file /tmp/opendoge_command.state
```

`--enable` 会参考 `mi_motor_demo_TB.py` 的流程发送运控模式设置和电机使能。部署程序不会自动执行机械置零。

默认配置文件：

```text
deploy/configs/opendoge_deploy.conf
```

上机前必须把每个关节的 `direction`、`offset`、`lower`、`upper` 和 `max_position_step` 改成实测值。

### 非 ROS 输入

非 ROS 输入文件格式参考：

```text
deploy/configs/command.example
deploy/configs/imu.example
```

命令输入包含 `vx/vy/yaw_rate/active/estop`。IMU 输入包含 `wx/wy/wz/gx/gy/gz`，其中 `g*` 是投影重力方向。

Xbox 兼容手柄可以通过 joystick bridge 写入同一个命令文件：

```bash
./hardware/gamepad/xbox_command_bridge.py \
  --output /tmp/opendoge_command.state --require-rb

./install/opendoge_deploy/bin/opendoge_deploy \
  --policy-backend onnx \
  --policy-path policy/opendoge_r26.onnx \
  --command-file /tmp/opendoge_command.state \
  --imu-file /tmp/opendoge_imu.state
```

IMU bridge：

```bash
./hardware/imu/dm_imu_bridge.py \
  --device /dev/ttyACM0 --baud 921600 \
  --output /tmp/opendoge_imu.state
```

如果 IMU 安装方向与训练坐标系不一致，通过轴映射修正：

```bash
./hardware/imu/dm_imu_bridge.py \
  --device /dev/ttyACM0 --baud 921600 \
  --axis-map xzy --axis-signs "1,-1,1" \
  --output /tmp/opendoge_imu.state
```

## vcan 无硬件 CAN 测试（仅离线）

> ⚠️ 生产硬件是 CHIWU 原生 can0-can3；vcan 与原生接口同名，测试前先
> `sudo systemctl stop opendoge-can` 并拔掉 USB-CAN 板，测完删除 vcan 再恢复。

EL05 协议打包自检：

```bash
./hardware/motor/protocol_selftest.py
```

vcan 测试 SocketCAN 打开和发送路径：

```bash
sudo ./scripts/setup_vcan.sh can0
sudo ./scripts/setup_vcan.sh can1
sudo ./scripts/setup_vcan.sh can2
sudo ./scripts/setup_vcan.sh can3
./install/opendoge_deploy/bin/opendoge_deploy \
  --real --enable --allow-missing-imu --policy-backend none --duration-sec 1
```

vcan 没有电机反馈，所以程序应保持在 `wait_feedback` 或进入安全阻尼，不应进入真实 active。

## 安全策略

`opendoge_deploy` 在以下情况进入阻尼模式：

- 电机反馈故障位非 0。
- `faultSta` 参数非 0。
- 电机温度超过阈值。
- 电机反馈超时。
- CAN 打开、发送或接收异常。
- 命令文件触发 `estop=true`。
- IMU/命令输入解析失败。

阻尼输出语义：

```text
q = 当前反馈位置
dq = 0
tau = 0
kp = 0
kd = safe_kd
```

## 上机前检查

- 电机 ID、CAN 通道、腿部布线一致。
- 关节正方向和机械零点 offset 已标定。
- 真实 URDF 使用 `../OpenDoge_description/URDF`，不要使用 firmware 内的旧占位描述。
- 软件限位、position rate limit、velocity limit、torque limit 已设置为保守值。
- EL05 单电机、单腿测试通过后再启用 12 电机。
- ONNX observation/action 数值已和训练侧 replay 对齐。
- IMU 坐标系和重力投影方向确认后再启用完整行走策略。
