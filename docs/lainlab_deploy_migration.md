# LainLab 策略部署迁移修改文档

> 日期: 2026-10-19
> 范围: `OpenDoge_firmware` 的 C++ 部署运行时、MuJoCo sim2sim、固件 XML、配置与回归测试
> 训练侧: `/home/hhl/RC/LainLab` (mjlab 1.6.0 + RSL-RL PPO)

---

## 1. 概要

本次修改把部署侧从旧 UniLab 训练契约（49 维观测、kp=20/kd=0.3、默认姿态前后腿不对称）
迁移到 LainLab/mjlab 训练契约（**48 维观测、kp=12/kd=0.5、四腿统一默认姿态**），
并修复了若干会导致策略无法正常行走的仿真/Gap。

**最重要的不兼容变更**：

| 项目 | 旧 | 新 |
|------|-----|-----|
| 观测维度 | 49 | **48** |
| 可用模型 | `policy/opendoge_r5.onnx` (49 维) | `LainLab/src/baseline/opendoge/flat/policy.onnx` (48 维) |
| 默认姿态 | 前腿 `(0,0.5,-1.3)` / 后腿 `(0,0.7,-1.3)` | 四腿统一 `(0,0.8,-1.6)` (固件 URDF 约定) |
| PD 增益 | kp=20, kd=0.3 | kp=12, kd=0.5 |
| 动作裁剪 | `clamp(-1,1)` | 无裁剪 |

> ⚠️ 旧 49 维 ONNX 在新运行时下会因输入维度不匹配加载失败，**不能继续使用**。

---

## 2. 变更文件清单

| 文件 | 变更类型 | 主要内容 |
|------|----------|----------|
| `deploy/include/opendoge_deploy/types.hpp` | 契约 | `kObsDim=48`、`kJointSign`、统一默认姿态、`kp/kd/action_scale`、`lin_vel_source`、`ImuSample.linear_velocity` |
| `deploy/include/opendoge_deploy/observer.hpp` | 契约 | 新 48 维 `buildObservation()`；旧布局改为 `buildLegacyObservation()` |
| `deploy/src/observer.cpp` | 契约 | 实现 48 维 LainLab 观测 |
| `deploy/src/controller.cpp` | 控制 | 动作→目标加入符号映射；移除裁剪；LowGain 分支 |
| `deploy/src/onnx_policy.cpp` | 控制 | 移除输出 `clamp(-1,1)` |
| `deploy/src/main.cpp` | 控制 | 线速度来源解析；移除步态相位；使用新观测 |
| `deploy/src/runtime_io.cpp` | I/O | 解析 `lin_vel_source`；IMU 文件可选 `vx/vy/vz` |
| `deploy/src/cli.cpp` + `cli.hpp` | CLI | 新增 `--lin-vel-source zero\|command\|imu` |
| `deploy/configs/opendoge_deploy.conf` | 配置 | 新增益、`action_scale`、`lin_vel_source`、注释 |
| `docs/URDF/xml/Opendoge.xml` | 物理参数 | joint damping/frictionloss/armature、脚部 friction/solimp/condim |
| `sim2sim/deploy_mujoco.py` | 仿真 | 48 维观测、符号约定、消融开关、传感器/执行器修复 |
| `test/deploy_gap_regression.py` | 测试 | 同步新契约（51 项） |
| `hardware/motor/el05_socketcan.cpp` | 构建 | 修正 include 路径 |
| `CLAUDE.md`、`deploy/CLAUDE.md`、`sim2sim/CLAUDE.md` | 文档 | 更新不变量、配置说明、Round 4 Gap 记录 |

---

## 3. 详细变更

### 3.1 观测契约：49 维 → 48 维

**旧布局（已废弃，仅供调试）**

```
gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
+ last_action(12) + commands(3) + feet_phase(4) = 49
```

**新布局（LainLab mjlab actor obs）**

```
base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
+ joint_pos_rel(12) + joint_vel_rel(12) + last_action(12) + command(3) = 48
```

要点：

- 新策略是端到端步态，**没有 feet_phase**。`advancePhase()` 保留但不再参与推理。
- 关节位置/速度必须乘 `kJointSign` 转换到训练约定。
- `last_action` 保存原始策略输出，不裁剪、不翻转符号。
- `default_pos` 以固件 URDF 约定存储，观测中同样乘符号。

实现位置：`deploy/src/observer.cpp`、`sim2sim/deploy_mujoco.py::build_observation()`。

### 3.2 关节符号约定

```cpp
constexpr std::array<double, kNumJoints> kJointSign = {
  +1, +1, +1,   // FL hip/thigh/calf
  +1, -1, -1,   // FR
  -1, +1, +1,   // RL
  -1, -1, -1,   // RR
};
```

公式：

```
q_train[i]          = sign[i] * q_firmware[i]
obs_joint_pos[i]    = sign[i] * (q_firmware[i] - default_firmware[i])
obs_joint_vel[i]    = sign[i] * qd_firmware[i]
target_firmware[i]  = default_firmware[i] + sign[i] * 0.25 * action_train[i]
```

规律：FR thigh/calf、RL/RR hip、RR thigh/calf 与固件 URDF 方向相反。

### 3.3 默认姿态

| 关节 | 旧值 | 新值 |
|------|------|------|
| hip | 0.0 | 0.0 |
| thigh (FL/FR) | 0.5 | **0.8** |
| thigh (RL/RR) | 0.7 | **0.8** |
| calf | -1.3 | **-1.6** |

`defaultJointPosition()` 与 `sim2sim` 的 `DEFAULT_POS` 已统一。
注意这是**固件约定下的值**，已包含与训练侧的符号映射。

### 3.4 动作与目标计算

- ONNX 输出不再 `clamp(-1,1)`；实测策略输出范围约 `[-3.8, +3.3]`。
- `last_action` 直接保存原始输出。
- 目标计算：

  ```cpp
  last_action[i] = action[i];
  logical_target[i] = default_pos[i] + kJointSign[i] * last_action[i] * action_scale;
  logical_target[i] = clamp(logical_target[i], joint_cfg.lower, joint_cfg.upper);
  ```

- RL 模式仍跳过 rate limiter；PC / EnteringPosition 保留 3 rad/s 限速。
- LowGainTest 仍强制目标为默认姿态。

### 3.5 PD 增益与执行器模型

| 参数 | 旧 | 新 |
|------|-----|-----|
| `kp` | 20.0 | **12.0** |
| `kd` | 0.3 | **0.5** |
| `safe_kd` | 2.0 | 2.0 |
| `action_scale` | 0.25 | 0.25 (配置曾为 0.50，已修正) |

对应训练侧 `BuiltinPositionActuatorCfg(stiffness=12.0, damping=0.5)`。

### 3.6 base_lin_vel 来源（真机无此测量）

训练时 `base_lin_vel` 加入 `U(-0.5, +0.5) m/s` 噪声，因此部署侧缺失该量影响很小。

**C++ 配置 / CLI**

```ini
lin_vel_source=command   # zero | command | imu
```

```bash
--lin-vel-source zero|command|imu
```

| 取值 | 行为 |
|------|------|
| `zero` | 恒为 0（最保守） |
| `command` | `[vx, vy, 0]`，当前默认 |
| `imu` | 读取 `imu.state` 的可选 `vx/vy/vz`；当前桥不输出，等价于 `zero` |

**Python 消融开关**

```bash
--lin-vel-mode true|zero|command|noisy
```

#### 消融结果（LainLab 模型，10 s，固定 seed）

| 场景 / 指标 | true | zero | command | noisy |
|---|---|---|---|---|
| vx=0.4 → 实际 vx (m/s) | 0.353 | **0.377** | 0.350 | 0.353 |
| vy=0.3 → 实际 vy (m/s) | 0.262 | 0.268 | 0.261 | 0.262 |
| vx=0.3, yaw=0.5 → 实际 wz | 0.494 | 0.497 | **0.499** | 0.498 |
| 摔倒 | 无 | 无 | 无 | 无 |
| 机身高度 z (m) | 0.157 | 0.157 | 0.157 | 0.157 |
| 峰值关节力矩 (N·m) | 3.0–3.5 | 3.0–3.5 | 3.0–3.5 | 3.0–3.5 |

**结论**：四种模式差异均在训练噪声范围内。默认 `command`；`zero` 可作为安全回退。

复现：

```bash
cd sim2sim
python3 deploy_mujoco.py \
  --model ../../LainLab/src/assets/robots/opendoge/xmls/scene_opendoge.xml \
  --joint-convention training --start-pose default --mode rl \
  --policy ../../LainLab/src/baseline/opendoge/flat/policy.onnx \
  --no-render --duration 10 --cmd 0.4 0 0 --lin-vel-mode zero
```

### 3.7 固件 XML 物理参数对齐

`docs/URDF/xml/Opendoge.xml`：

| 属性 | 旧 | 新 |
|------|-----|-----|
| joint `damping` | 0.5 | **0** |
| joint `frictionloss` | 0.2 | **0** |
| joint `armature` | 0.01 | **0.005** |
| foot `friction` | `0.4 0.02 0.01` | **`0.8 0.005 0.0001`** |
| foot `solimp` | `0.015 1 0.023` | **`0.9 0.95 0.023`** |
| foot `condim` | 6 | **3** |

原因：固件 XML 的 joint damping 会与 PD kd 叠加；脚部接触参数与训练 `FULL_COLLISION` 不同。
实测同一策略在固件模型上的平均速度：

| 配置 | 平均速度 |
|------|----------|
| 旧参数 | 0.130 m/s |
| 只改 joint damping/frictionloss/armature | 0.290 m/s |
| 只改脚部接触参数 | 0.188 m/s |
| **两项都改（当前）** | **0.359 m/s** |
| LainLab 原始模型（参照） | 0.353 m/s |

### 3.8 sim2sim 修复

`sim2sim/deploy_mujoco.py`：

1. **传感器地址 bug**：`sensordata` 偏移必须用 `model.sensor_adr[id]`，不能直接使用
   `sensor.id`。此前固件模型把 `position` 传感器当成 gyro，LainLab 模型把
   `imu_lin_acc` 当成线速度。
2. **训练模型无 actuator**：LainLab MJCF `nu=0`，PD 力矩写入
   `data.qfrc_applied[dof]`；有 actuator 时才写 `data.ctrl`。
3. **armature 注入**：mjlab 在运行时注入 `armature=0.005`，裸 MuJoCo 加载时需手动
   设置 `model.dof_armature`，否则小腿显式积分发散 (QACC NaN)。
4. **新增参数**：
   - `--joint-convention {firmware,training}`
   - `--start-pose {rest,default}`、`--base-height`
   - `--lin-vel-mode {true,zero,command,noisy}`、`--seed`
5. 日志新增 `base_x / base_y / base_yaw / true_vx / true_vy / true_wz` 列。

### 3.9 构建修复

`el05_socketcan.hpp` 实际位于 `hardware/motor/`，但三处代码使用
`#include "opendoge_deploy/el05_socketcan.hpp"`，导致编译失败。已统一为：

```cpp
#include "el05_socketcan.hpp"
```

涉及 `deploy/src/main.cpp`、`deploy/include/opendoge_deploy/controller.hpp`、
`hardware/motor/el05_socketcan.cpp`。

### 3.10 仿真手柄支持与键盘修复 (2026-10-20)

**问题 1：方向键 ↓ 无效**

`deploy_mujoco.py` 使用键盘码 266 处理 ↓，但 GLFW 的
`GLFW_KEY_DOWN = 264`，266 实际是 `PAGE_UP`，因此按 ↓ 不会产生负 vx。
修复为 264，并加入回归测试 `test_keyboard_mapping()`。

**问题 2：仿真只能用键盘，不能用手柄**

新增 `--joystick [DEVICE]`：

```bash
# 自动选择 /dev/input/js*
python3 deploy_mujoco.py --joystick --policy ... --model ...
# 指定设备
python3 deploy_mujoco.py --joystick /dev/input/js0 ...
```

实现复用 `hardware/gamepad/opendoge_joystick.py` 的 `LinuxJoystick` +
`XboxCommandMapper`，与实机 bridge **完全同一套按键/摇杆映射**：

| 输入 | 功能 |
|------|------|
| 左摇杆 | vx / vy |
| 右摇杆 X | yaw_rate |
| A | 激活 + 位置控制 |
| B | 停用 |
| X | RL 推理 |
| Y | 位置控制 |
| START | 切换使能 |
| BACK | 切换 low_gain |

- 手柄模式下键盘只保留 `Esc / Q / Ctrl+C` 退出。
- 手柄与实机一样**没有急停键**，急停请用终端 `Ctrl+C`。
- 未检测到设备时给出明确错误，不会抛 traceback。

**摇杆符号修正 (2026-10-20)**

`XboxCommandMapper.command()` 原先只对 vx 取反，vy/yaw 保持原始轴值。
Linux joystick 原始轴是 `X+ = 右、Y+ = 下`，而机器人 body frame
(REP-103) 是 `X+ = 前、Y+ = 左、yaw+ = 逆时针`，导致左摇杆右推时机器人左移、
右摇杆右推时左转。已改为三轴全部取反：

```python
vx_axis  = apply_deadzone(-axes[axis_vx],  deadzone)
vy_axis  = apply_deadzone(-axes[axis_vy],  deadzone)
yaw_axis = apply_deadzone(-axes[axis_yaw], deadzone)
```

该修改在共享的 `hardware/gamepad/opendoge_joystick.py` 中，**实机 bridge 同样生效**。

**按键编号重映射**

不同手柄的 A/B/X/Y 编号可能不同（例如物理 X 上报为 btn3 而不是 btn2，
会导致按 X 无反应或行为与 Y 相同）。用以下命令确认：

```bash
jstest /dev/input/js0          # 或运行仿真时加 --joy-debug
```

当前 `--joystick` 的默认值已匹配 Zikway HID gamepad (`3537:1041`)：

| 功能 | 键号 |
|------|------|
| A / B | 0 / 1 |
| X / Y | 3 / 4 |
| START（使能） | 6（LB） |
| BACK（low_gain） | 7（RB） |
| 急停（仅实机，锁存） | 11（Home 右边、三个横杠平行排列的键；按 A 复位） |

> **该手柄没有独立 START 键**：只有 Home 键，对应 **btn12**，但 Home 长按会
> 关机，因此默认用 **LB（btn6）** 作为 START。代码中已注释保留 Home 映射
> （`# btn_start: int = 12,  # Home`），需要时可把 `btn_start` 改为 12，或
> 运行时传 `--joy-btn-start 12`；其余代码无需改动。

其他手柄可通过参数覆盖，无需改代码：

```bash
python3 deploy_mujoco.py --joystick \
  --joy-btn-a 0 --joy-btn-b 1 --joy-btn-x 2 --joy-btn-y 3 \
  --joy-btn-start 7 --joy-btn-back 6 ...
```

> 共享的 `XboxCommandMapper` 默认值仍是标准 Xbox 布局 (X=2/Y=3,
> START=7/BACK=6)，因此实机 bridge 不受影响；只有仿真 `--joystick`
> 的默认值改成了上述手柄布局。

#### 手柄没有 `/dev/input/js*` 时的排查

**先排查是不是接收器问题，换一个接收器试一试**
2.4G 克隆 dongle（`413d:2104` / `413d:2106`, `Zikway HID zk`）**不会被内核
`xpad` 驱动识别**，只会以 vendor HID 设备出现（`hidraw*`），因此不会生成
`/dev/input/js*`。实机上是通过 xboxdrv 剥离 `hid-generic` 后创建虚拟手柄的。

PC 上需要同样安装并运行 xboxdrv：

```bash
sudo apt install xboxdrv
sudo xboxdrv --device-by-id 413d:2106 --type xbox360 \
  --detach-kernel-driver --dbus disabled
# 另开终端
ls /dev/input/js*
```

官方 Xbox 接收器或内核原生支持的手柄则不需要 xboxdrv，插上即有 `js0`。

> 注意：本机实测该 dongle 还会出现 `device descriptor read/64, error -32` →
> `unable to enumerate USB device`，属于 USB 供电/链路不稳定，优先换直连 USB 2.0
> 口；软件层无法修复反复掉线。

---

## 4. 验证记录

| 验证项 | 命令 | 结果 |
|--------|------|------|
| C++ 编译 | `colcon build --packages-select opendoge_deploy` | 通过 |
| C++ dry-run（空策略） | `opendoge_deploy --policy-backend none --duration-sec 1 --lin-vel-source zero` | 退出码 0 |
| C++ + 48 维 ONNX | `--policy-backend onnx --policy-path .../flat/policy.onnx --cmd 0.3 0 0 --start-active` | 退出码 0 |
| 回归测试 | `python3 test/deploy_gap_regression.py` | **51 passed, 0 failed** |
| MuJoCo 行走 | vx=0.4，10 s | 0.35 m/s，无摔倒，z≈0.157 |
| MuJoCo 侧移 | vy=0.3，10 s | 0.26 m/s，无摔倒 |
| MuJoCo 转向 | vx=0.3, wz=0.5，10 s | wz≈0.50, 无摔倒 |

---

## 5. 部署操作变化

### 5.1 模型路径

旧：

```bash
POLICY_PATH=policy/opendoge_r5.onnx   # ❌ 49 维，不兼容
```

新：

```bash
POLICY_PATH=../LainLab/src/baseline/opendoge/flat/policy.onnx
# 或复制到仓库内
cp ../LainLab/src/baseline/opendoge/flat/policy.onnx policy/opendoge_flat.onnx
```

### 5.2 启动流程不变

```bash
bash scripts/start_robot.sh verify
bash scripts/start_robot.sh damping
bash scripts/start_robot.sh low-gain
POLICY_PATH=policy/opendoge_flat.onnx bash scripts/start_robot.sh policy
```

手柄：`A`（进入位置控制，等 2 s 斜坡）→ `X`（RL）→ 摇杆；
`Y` 回位置控制，`B` 退出位置控制回到阻尼（不是电机失能），`BACK` 切换 low_gain。
注意机器人按 A 前必须已在默认姿态 ±0.25 rad 内。

### 5.3 配置

`deploy/configs/opendoge_deploy.conf` 已包含新值：

```ini
kp=12.0
kd=0.5
action_scale=0.25
lin_vel_source=command
```

运行时覆盖：

```bash
opendoge_deploy --lin-vel-source zero ...
```

---

## 6. 已知风险与待办

| 风险 | 说明 | 建议 |
|------|------|------|
| 小腿力矩上限 | 训练 effort limit 9 N·m，EL05 仅 6 N·m | 实机注意落地冲击与持续饱和，必要时降速 |
| 关节限位 | 配置保留硬件 URDF 的窄限位，训练 XML 更宽 | 超限会被 clamp，实机行为可能与仿真有差异 |
| 质量/惯量分布 | 固件 XML 与训练 XML 的 base 惯量、质心未逐项核对 | 后续逐项对比 |
| IMU 线速度 | `dm_imu_bridge.py` 不输出 `vx/vy/vz` | `imu` 模式当前等价 `zero` |
| 手柄急停/清故障 | 桥未把 `estop`/`clear_fault` 绑定到按键；BACK 提示文案有误 | 后续补按键绑定 |
| 旧模型兼容 | 49 维 `opendoge_r5.onnx` 不可用 | 保留为历史文件，运行时仅支持 48 维 |

---

## 7. 回滚方法

本次修改未提交为多个原子提交，回滚需按文件处理：

```bash
# 查看全部改动
git diff

# 仅回滚 C++/配置（保留仿真与文档）
git checkout -- deploy/ docs/URDF/xml/Opendoge.xml

# 或整体回滚（会丢失本轮全部修改）
git checkout -- .
```

注意：回滚 XML 会影响 sim2sim 行走速度；回滚 `types.hpp` / `observer.cpp`
后必须同时回退 `sim2sim/deploy_mujoco.py` 与回归测试，否则两条管线不一致。
