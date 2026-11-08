# USB-CAN 适配器

本目录存放 OpenDoge 的 USB-CAN 硬件、链路验证与参考脚本。
上位机（Orange Pi / PC）只依赖 **原生 SocketCAN `can0-can3`**。

> 完整调试流程见 [../调试与上机操作手册.md](../调试与上机操作手册.md)。

## 当前方案：赤乌（CHIWU）4×CAN + 3×485

```text
Orange Pi 5 / PC
  → 一条 USB-A 线
  → 赤乌（CHIWU）4×CAN + 3×RS485 控制板
  → Linux 原生 SocketCAN can0-can3（经典 CAN, 1 Mbps）
  → opendoge_deploy / scan_motors_readonly.py / el05_motor_menu.py / watch_angles.py
```

| 属性 | 值 |
|------|-----|
| 板子 | CHIWU 赤乌科技 4×CAN + 3×RS485 |
| USB | 单 USB-A（A 转 A 线） |
| CAN 通道 | 4（板载标号 CAN1-CAN4） |
| CAN 速率 | 经典 CAN 1 Mbps，**不要开 FD** |
| RS485 | 3 路，本项目不使用 |
| 驱动 | 插上后由 Linux 内核创建 can0-can3；实际驱动用 `readlink /sys/class/net/can0/device/driver` 确认（常见 gs_usb） |
| 详细文档 | [chiwu_usb2can.md](chiwu_usb2can.md) |
| 旧 DingLab 方案 | 已归档：`~/DingLab usb转can代码`，见 [../legacy/README.md](../legacy/README.md) |

### 通道映射（不可变）

```text
can0 = 左前 FL: motor 1/2/3  (hip/thigh/calf)  ← 板子 CAN1
can1 = 右前 FR: motor 4/5/6                    ← 板子 CAN2
can2 = 左后 RL: motor 7/8/9                    ← 板子 CAN3
can3 = 右后 RR: motor 10/11/12                 ← 板子 CAN4
```

如果板子 CAN1-CAN4 的枚举顺序与上表不一致，优先**换接线**；必要时用
udev/systemd 按 USB 路径改名。接口名必须最终是 can0-can3。

### 配置与启动

```bash
# systemd（root，开机自启）
sudo systemctl restart opendoge-can
systemctl status opendoge-can

# 手动等价命令
sudo modprobe gs_usb can can_raw
for i in 0 1 2 3; do
    sudo ip link set down can$i
    sudo ip link set can$i type can bitrate 1000000 fd off loopback off
    sudo ip link set up can$i
done
```

`opendoge-can.service` 支持 `/etc/default/opendoge-can`：

```bash
CAN_EXPECT_DRIVER=gs_usb   # 或实际 vendor 驱动名；auto = 只警告
CAN_MODULE=                # 如需 vendor 内核模块，填模块名
```

### 只读验证

```bash
ip -details link show can0      # UP / bitrate 1000000 / no FD
sudo python3 scripts/verify_can_mapping.py
sudo python3 hardware/motor/scan_motors_readonly.py
sudo python3 hardware/motor/watch_angles.py --once
```

### 文件

| 文件 | 用途 |
|------|------|
| `scripts/setup_can.sh` | 配置单个经典 CAN 接口 |
| `scripts/setup_vcan.sh` | **仅手动 dry-run**：无硬件时创建 vcan，不用于生产 |
| `scripts/verify_can_mapping.py` | 只读校验 can0-can3 与电机 ID 映射 |
| `test/chiwu_code.py` | 厂商例程参考（**不要直接运行**，它会置零/驱动电机 6/7） |
| `../hardware/motor/scan_motors_readonly.py` | EL05 只读扫描 |
| `../hardware/motor/el05_motor_menu.py` | EL05 交互式调测 |
| `../hardware/motor/watch_angles.py` | 实时角度查看 |
| `../hardware/motor/zero_all_motors.py` | 12 电机站立位一键置零 |

## EL05 电机协议

详见 `docs/EL05使用说明书2600428.pdf`。关键参数：

```text
torque:   -6..6 Nm
position: -12.57..12.57 rad
velocity: -50..50 rad/s
kp:       0..500
kd:       0..5
```

### 出厂默认值

**新电机 CAN ID 为 127 (0x7F)**，不是 OpenDoge 标准 ID 1-12。
需要用 Windows 上位机 **EDULITE-TOOL**（灵足官网下载）逐个改 ID。

### 只读扫描（安全）

```bash
python3 hardware/motor/scan_motors_readonly.py
```

- 使用 `COMM_GET_DEVICE_ID` (0x00) 广播发现电机（包括出厂 ID 127）
- 使用 `COMM_READ_PARAM` (0x11) 读取位置/速度/模式/故障
- **不发送任何控制命令**

### 交互式调测（谨慎）

```bash
./hardware/motor/el05_motor_menu.py --channel can0 --master-id 0xfd
```

该工具可执行使能、停止、置零等写操作；使用前确保机器人已支撑。

## 安装依赖

```bash
python3 -m pip install python-can
```

## 观察总线

```bash
candump can0                 # 实时 CAN 流量
ip -details link show can0   # ERROR-ACTIVE = 正常
```
