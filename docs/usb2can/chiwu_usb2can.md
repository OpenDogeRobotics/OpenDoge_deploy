# 赤乌（CHIWU）4×CAN + 3×485 USB 控制板接入

> 当前 OpenDoge 使用的 USB-CAN 适配器。旧 DingLab 方案已归档到
> `~/DingLab usb转can代码`，见 [../legacy/README.md](../legacy/README.md)。

## 1. 硬件

- 板子：赤乌科技（CHIWU）机器人关节电机控制板
- 接口：4×CAN（板载 CAN1-CAN4）、3×RS485、USB-A、4-pin UART、电源端子
- 连接：一条 USB-A 转 USB-A 线接 Orange Pi / PC
- 本项目只使用 **4 路 CAN**；RS485、UART 不接线、不配置
- CAN：经典 CAN 1 Mbps，**不要开 CAN FD**

## 2. Linux 接口

厂商例程 `test/chiwu_code.py` 的做法：

```bash
sudo ip link set down can0
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set up can0
# can1/can2/can3 同理
```

然后：

```python
import can
bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=1000000)
```

这说明板子在 Linux 下应**原生暴露 `can0-can3`**（由内核驱动创建），
不需要 vcan 或 userspace bridge。实际驱动用以下命令确认：

```bash
lsusb
readlink -f /sys/class/net/can0/device/driver
ip -details link show can0
dmesg -T | tail -60
```

常见驱动是 `gs_usb`；如果是厂商模块，把模块名写进
`/etc/default/opendoge-can` 的 `CAN_MODULE=`，并把 `CAN_EXPECT_DRIVER`
设为实际驱动名（或 `auto`）。

## 2b. 实测枚举（2026-11-05，当前机器）

板子内部不是一个复合设备，而是 **4 个独立的 gs_usb USB 设备** + 3 个
CH340（RS485）：

```text
1d50:606f  Geschwister Schneider CAN adapter  ×4   → /sys/.../1-3.6.1..4
1a86:7523  CH340 serial converter             ×3   → RS485，本项目不用
```

| USB 路径 | serial | 用途 |
|----------|--------|------|
| `1-3.6.1` | `002F00455641571820373433` | 板子 CAN1 |
| `1-3.6.2` | `002A004A4E56511220363434` | 板子 CAN2 |
| `1-3.6.3` | `002C004A4E56511420333634` | 板子 CAN3 |
| `1-3.6.4` | `002200424E56510E20353334` | 板子 CAN4 |

驱动是 **`gs_usb`**，内核会创建 4 个 SocketCAN 接口。接口名不一定正好是
can0-can3：如果本次启动里 can0-3 已经被 vcan 用过，会变成 **can4-can7**。
`scripts/prepare_can_interfaces.sh` 会按 USB 路径排序，把前 4 个
gs_usb（默认 VID/PID `1d50:606f`）改名为 can0-can3；已经是 can0-3 时
no-op。它由 `opendoge-can.service` 在配置前调用。

> 如果以后板子的 VID/PID 不同，给服务/脚本设置
> `CAN_USB_VID` / `CAN_USB_PID`（默认 1d50/606f）。
> 想先预览映射：`bash scripts/prepare_can_interfaces.sh --dry-run`。

## 3. 通道映射

```text
板子 CAN1 → can0 → 左前 FL  (motor 1/2/3)
板子 CAN2 → can1 → 右前 FR  (motor 4/5/6)
板子 CAN3 → can2 → 左后 RL  (motor 7/8/9)
板子 CAN4 → can3 → 右后 RR  (motor 10/11/12)
```

接 4 条腿的 CANH/CANL 前先确认板子丝印；如果枚举顺序不是
can0=CAN1 …，优先交换接线，必要时用 udev/systemd 按 USB 路径改名。
最终必须让 `verify_can_mapping.py` 显示 can0={1,2,3} … can3={10,11,12}。

## 4. 配置与启动

```bash
# systemd
sudo systemctl restart opendoge-can
systemctl status opendoge-can
ip -details link show can0

# 手动
sudo modprobe gs_usb can can_raw
sudo bash scripts/setup_can.sh can0 1000000
sudo bash scripts/setup_can.sh can1 1000000
sudo bash scripts/setup_can.sh can2 1000000
sudo bash scripts/setup_can.sh can3 1000000
```

`opendoge-can.service` 会：

- 加载 `gs_usb`/`can`/`can_raw`（以及可选的 `CAN_MODULE`）；
- 等待 `can0-can3` 出现（最多 60 s）；
- 校验驱动（`CAN_EXPECT_DRIVER`，默认 `gs_usb`；`auto` 只警告）；
- 设置 1 Mbps、fd off、loopback off、UP。

`ExecStart` 由 `scripts/setup_can_interfaces.sh` 执行：如果 `ip link set up`
失败（例如 CHIWU 被电机断电一起复位，内核报
`gs_usb ... Couldn't start device (err=-110)`），它会先对 4 个 `1d50:606f`
做 USB `authorized 0/1` 复位、重新 `prepare_can_interfaces.sh`，再重试
（`CAN_SETUP_ATTEMPTS`，默认 3）。如果整块板子没 24V，重试也救不了，会明确失败。

## 5. 验证

```bash
ip -details link show can0                    # UP, 1000000, 无 FD
sudo python3 scripts/verify_can_mapping.py    # 12/12
sudo python3 hardware/motor/scan_motors_readonly.py
sudo python3 hardware/motor/watch_angles.py --once
bash scripts/start_robot.sh verify
bash scripts/start_robot.sh damping           # 先悬挂/支撑
```

## 6. 厂商例程说明

`test/chiwu_code.py` 是卖家提供的**小米/CyberGear 类电机**驱动例程：

- 只把它当 **SocketCAN 配置和 `python-can` 用法**参考；
- 它的 `main()` 会对 motor 6 执行 `set_motor_angle_zero()`，并让
  motor 6/7 来回转动。**不要原样在整机上运行**；
- EL05 的协议实现仍以 `hardware/motor/el05_socketcan.cpp` 和
  `el05_motor_menu.py` 为准。

## 7. 排障

| 现象 | 检查 |
|------|------|
| 没有 can0-can3 | `lsusb`、`dmesg`、驱动模块；`CAN_MODULE`/`CAN_EXPECT_DRIVER` |
| 接口顺序不对 | 板子 CAN1-4 与 can0-3 的对应；优先重接线，或 udev 改名 |
| `bitrate` 设置失败 | 需 root；`ip link set ... type can bitrate 1000000 fd off loopback off` |
| 出现 CAN FD | 必须 `fd off`；EL05 只支持经典 CAN |
| 某个通道无回包 | CANH/CANL、120 Ω 终端（只在两端）、共地、电机供电 |
| USB 反复 disconnect | USB 线/供电；Hub 必须外部供电；看 `journalctl -k` |
| `ip link set canX up` 报 `RTNETLINK ... Connection timed out`，内核 `gs_usb ... Couldn't start device (err=-110)` | CHIWU 板被复位/24V 未上：恢复板子供电，`sudo scripts/reset_can_usb.sh`；仍失败就彻底拔 USB/断电 5–10 s |
| 给电机断电后 can0-3 掉线 | CHIWU 的 24V 很可能和电机共开关；让板子单独常供，或断电后 `sudo scripts/reset_can_usb.sh` 再启动 |
| 旧 vcan 还在 | `sudo systemctl disable --now opendoge-vcan opendoge-usb2can0 opendoge-usb2can1`，删除 vcan 接口 |
