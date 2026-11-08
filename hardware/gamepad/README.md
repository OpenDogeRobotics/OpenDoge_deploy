# OpenDoge Joystick Bridge

Xbox 手柄 → `/tmp/opendoge_command.state` 桥接工具，供 `opendoge_deploy` 运行时读取。

## 支持的接收器

| 接收器 | USB ID | 驱动方式 | js0 |
|--------|--------|----------|-----|
| Xbox 360 2.4G 克隆 dongle | `413d:2104` | xboxdrv | xboxdrv 创建 |
| Zikway 2.4G 接收器 | `3537:1041` | 内核 `hid-generic`/`joydev` | 内核自动创建，**不需要 xboxdrv** |
| Zikway 另一模式 | `3537:2106` | 仅 hidraw | **不创建 js0**，需拔插/切回 1041 |

`scripts/start_gamepad.sh` 会自动识别以上情况：

- `413d:2104` → 启动 xboxdrv；
- `3537:1041` → 等待内核创建 `/dev/input/js0`，保持服务 active；
- `3537:2106` → 打印提示并继续等待，直到切回 1041。

`opendoge-xboxdrv.service` 现在调用这个包装脚本，不再硬编码只等 `413d:2104`。

## 文件清单

| 文件 | 用途 |
|------|------|
| `xbox_command_bridge.py` | **主桥接程序** — 读取 `/dev/input/js0`，写入命令文件 |
| `opendoge_joystick.py` | 共享库 — `LinuxJoystick` / `XboxCommandMapper` / `atomic_write` |
| `start_joystick_bridge.sh` | 启动器 — 先拉起 xboxdrv，再启动桥接 |
| `diag_joystick.py` | 诊断工具 — 检测 xboxdrv/js0 是否正常工作 |
| `gamepad_tester.html` | 网页手柄测试页 — 浏览器中查看手柄状态 |

## 快速开始

### 1. 确保 js0 出现

推荐直接使用服务（自动处理 413d/3537）：

```bash
sudo systemctl restart opendoge-xboxdrv
systemctl status opendoge-xboxdrv
ls -la /dev/input/js0
cat /sys/class/input/js0/device/name
```

手动方式：

```bash
# 413d:2104 Xbox dongle：xboxdrv 创建 js0
sudo xboxdrv --device-by-id 413d:2104 --type xbox360 --detach-kernel-driver --silent &

# 3537:1041 Zikway：内核 hid-generic/joydev 自动创建 js0，不要启动 xboxdrv
ls -la /dev/input/js0
```

> 如果 USB ID 不同，用 `lsusb` 查找手柄的 `xxxx:xxxx`。
> `3537:2106` 模式不会创建 js0，需要拔插/切换模式。

### Zikway `3537:1041` 原生映射

`xbox_command_bridge.py --profile auto` 会根据 js0 设备名（包含 `Zikway`）自动套用：

| 功能 | 键号 |
|------|------|
| A / B | 0 / 1 |
| X / Y | 3 / 4 |
| START（使能） | 6（LB） |
| BACK（low_gain） | 7（RB） |
| 急停（实机专用，锁存） | 11（Home 右边、三个横杠平行排列的键） |

> 该映射与 sim2sim `--joystick` 默认值一致；btn11 是**锁存急停**：
> 按一次置 `estop=true`、active=false、速度归零；再按 **A** 清除 estop
> 并发送 `clear_fault` 恢复。**不需要一直按住**。
> 建议先用 `jstest /dev/input/js0` 确认；如果不同，用
> `--btn-back` / `--btn-estop` 覆盖。

### 2. 运行桥接

```bash
./hardware/gamepad/xbox_command_bridge.py --profile auto
```

或使用启动器脚本（自动处理 xboxdrv）：

```bash
./hardware/gamepad/start_joystick_bridge.sh --profile auto
```

> `--require-rb` 是旧的“按住死手”模式，默认服务已不再使用；现在用 btn11 锁存急停。

### 3. 验证

```bash
# 观察命令文件实时更新
watch -n 0.2 cat /tmp/opendoge_command.state
```

按下手柄 A 键后应看到 `active=true`、`position_control=true`，推动摇杆应看到 vx/vy/yaw 数值变化。

## 手柄映射

```
  Left stick   -> vx (前后) / vy (左右)
  Right stick  -> yaw_rate (转向)

  A            -> 启动机器人 + 位置控制模式 (position_control)
  B            -> 退出位置控制/RL，回到阻尼 (active=false；不切断电机输出)
  X            -> 进入 RL 推理状态 (rl_inference)
  Y            -> 退出 RL 推理状态
  BACK         -> 切换 low_gain
  START        -> 切换使能
  急停 (btn11) -> 锁存 estop；按 A 清除并恢复
```

状态组合：

| 手柄操作 | active | position_control | rl_inference | estop |
|----------|--------|------------------|--------------|-------|
| 空闲 | false | false | false | false |
| 按 A | true | true | false | false |
| 按 A 后按 X | true | false | true | false |
| RL 中按 Y | true | true | false | false |
| 按 B | false | false | false | false |
| 按 BACK | false | false | false | false（切 low_gain） |
| 按 btn11 急停 | false | false | false | **true**（锁存，A 复位） |

## 轴映射（Zikway `3537:1041` 原生 js0，实测）

| 物理输入 | js 轴号 | 功能 |
|----------|---------|------|
| 左摇杆 X | `axis0` | vy（侧向） |
| 左摇杆 Y | `axis1` | vx（前后） |
| 右摇杆 X | `axis2` | yaw_rate（转向） |
| 右摇杆 Y | `axis3` | 未使用 |
| LT / RT | `axis4` / `axis5` | 未使用（初始值 -32767） |
| 十字键 | `axis6` / `axis7` | 未使用 |

桥接内部会做 `-axis`（摇杆向上/向右为负）和 `deadzone=0.08` 处理。
如果换了手柄或驱动方式，先用 `jstest /dev/input/js0` 确认轴号，再用
`--axis-vx/--axis-vy/--axis-yaw` 覆盖。

## 速度映射（满杆对应的指令）

`--max-vx / --max-vy / --max-yaw-rate` 决定摇杆推满时的指令上限。当前默认：

| 参数 | 默认值 | 训练 flat 初始范围 |
|------|--------|--------------------|
| `--max-vx` | **0.60 m/s** | ±0.8 |
| `--max-vy` | **0.40 m/s** | ±0.5 |
| `--max-yaw-rate` | **1.00 rad/s** | ±0.8 |

历史：bring-up 期间曾降到 75%（0.45/0.30/0.75）和 50%（0.30/0.20/0.50），
2026-11-04 恢复到 100%。vx/vy 在训练初始范围内；**yaw 1.0 略高于 ±0.8**，
满杆转向不稳时优先降 `--max-yaw-rate`。

> **注意**：systemd 服务用的是 unit 里 `ExecStart` 的**显式参数**。
> 只改 `xbox_command_bridge.py` 的 default 不会影响已安装的服务，必须同步改
> `daemons/opendoge-joystick.service` 并重装（见下一节）。

## 修改映射后如何生效（重装服务）

1. 改仓库里的 `daemons/opendoge-joystick.service`（速度参数）或
   `xbox_command_bridge.py`（轴/按键默认值）；
2. 重新安装 **user** unit（不需要 sudo）：
   ```bash
   sed "s|/home/orangepi/Opendoge/OpenDoge_firmware|$HOME/RC/Lain_Code|g" \
     daemons/opendoge-joystick.service \
     > ~/.config/systemd/user/opendoge-joystick.service
   systemctl --user daemon-reload
   systemctl --user restart opendoge-joystick
   ```
   （把 `$HOME/RC/Lain_Code` 换成实际仓库路径。）
3. 验证：
   ```bash
   systemctl --user cat opendoge-joystick | grep -E "max-|CPUAffinity"
   pgrep -af xbox_command_bridge.py
   ```
   进程参数里应出现新的 `--max-*`。
4. 或一键重装全部服务（会重启 CAN/IMU/手柄，机器人空闲时做）：
   ```bash
   bash scripts/install_services.sh
   ```

## 命令行参数

```
xbox_command_bridge.py [选项]

  --device PATH         手柄设备路径 (默认: /dev/input/js* 中第一个)
  --output PATH         命令文件输出路径 (默认: /tmp/opendoge_command.state)
  --rate-hz HZ          更新频率 (默认: 100)
  --deadzone VAL        摇杆死区 (默认: 0.08)
  --max-vx VAL          前进最大速度 m/s (默认: 0.60)
  --max-vy VAL          侧向最大速度 m/s (默认: 0.40)
  --max-yaw-rate VAL    转向最大角速度 rad/s (默认: 1.00)
  --profile auto|default|zikway  按键布局 (auto 自动识别 Zikway 3537:1041)
  --btn-estop N         锁存急停按键 (Zikway 默认 11；按 A 复位)
  --require-rb          旧模式：要求按住 RB 才输出有效命令 (默认服务已不用)
  --quiet               不打印周期性状态
  --axis-vx N / --axis-vy N / --axis-yaw N   自定义轴号
  --btn-a N / --btn-b N / --btn-x N / --btn-y N ...  自定义按键号
```

## systemd 服务（开机自启）

桥接已固化为 systemd 服务，开机自动运行，无需手动启动。

### 安装

```bash
# 一键安装（推荐）
./scripts/install_services.sh

# 或手动安装用户级桥接服务（无需 sudo）
# 注意 unit 在 daemons/，并且里面的路径是 /home/orangepi/... 占位，需要替换
mkdir -p ~/.config/systemd/user
sed "s|/home/orangepi/Opendoge/OpenDoge_firmware|$HOME/RC/Lain_Code|g" \
  daemons/opendoge-joystick.service \
  > ~/.config/systemd/user/opendoge-joystick.service
systemctl --user daemon-reload
systemctl --user enable --now opendoge-joystick
loginctl enable-linger   # 确保用户服务在开机时启动
```

### 日常管理

```bash
# 状态
systemctl --user status opendoge-joystick

# 实时日志
journalctl --user -u opendoge-joystick -f

# 重启 / 停止
systemctl --user restart opendoge-joystick
systemctl --user stop opendoge-joystick
```

### 服务架构

```
opendoge-xboxdrv.service (系统级, root)     opendoge-joystick.service (用户级, orangepi)
         │                                              │
         │ 提供 /dev/input/js0                            │ 读取 js0
         │                                              │ 写入 /tmp/opendoge_command.state
         ▼                                              ▼
     [xboxdrv] ──── /dev/input/js0 ──── [xbox_command_bridge.py]
                                                │
                                                ▼
                                    /tmp/opendoge_command.state
                                                │
                                                ▼
                                    [opendoge_deploy]
```

> **注意**: `opendoge-xboxdrv.service` 是系统级 service（需 sudo 安装一次），
> 现在调用 `scripts/start_gamepad.sh`：
> - `413d:2104` 时启动 xboxdrv；
> - `3537:1041` 时只等待内核 js0，不再启动 xboxdrv。
>
> 如果 js0 已由其他方式提供，也可以只安装 `opendoge-joystick.service`。

### 与 start_robot.sh 的集成

`start_robot.sh` 的 `start_joystick_bridge()` 函数会优先检测 systemd 服务：

- 如果 `opendoge-joystick.service` 已在运行 → **跳过**，打印提示
- 如果未运行 → 回退到直接 spawn 桥接进程（兼容未安装 systemd 服务的场景）

## 诊断

如果手柄不响应：

```bash
# 1. 检查 xboxdrv
pgrep -la xboxdrv

# 2. 检查 js0
ls -la /dev/input/js0
cat /sys/class/input/js0/device/name

# 3. 运行诊断工具
./hardware/gamepad/diag_joystick.py

# 4. 检查桥接日志
journalctl --user -u opendoge-joystick -n 50

# 5. 手动测试 — 直接读取 js0 事件
cat /dev/input/js0 | xxd    # 按手柄按键应看到数据
```

### 常见问题

| 症状 | 原因 | 解决 |
|------|------|------|
| `No joystick found` | js0 不存在 | `413d:2104` 启动 xboxdrv；`3537:1041` 用内核 js0；检查 USB 连接 |
| 接收器显示 `3537:2106` | Zikway 进入了不创建 js0 的模式 | 拔插接收器/切换模式，回到 `3537:1041` |
| 按键无反应 | 驱动方式或映射不对 | 先确认 js0 存在；`jstest /dev/input/js0` 查轴/按键号 |
| USB ID 不匹配 | 手柄/dongle 不是 `413d:2104` | 用 `lsusb` 查实际 ID；`3537:1041` 走原生 js0 |
| 命令文件不更新 | 桥接进程挂了 | `systemctl --user restart opendoge-joystick` |
| 按 btn11 后机器人停止 | 正常的锁存急停 | 按 A 清除 estop 并恢复；不需要一直按住 |
