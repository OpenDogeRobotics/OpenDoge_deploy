# 达妙 DM-USB2CANFD Dual 接入指南 (Linux / RK3588 / aarch64)

> ⚠️ **当前硬件为赤乌（CHIWU）4×CAN 原生 SocketCAN；本文件仅作达妙
> gs_usb 备选参考。** 当前接入请见 [chiwu_usb2can.md](chiwu_usb2can.md)；
> 旧 DingLab 方案见 [../legacy/README.md](../legacy/README.md)。

本文说明如何用 **2× 达妙 USB转CAN2FD Dual** 替代 CANable，给 OpenDoge 提供
`can0`–`can3` 四个标准 SocketCAN 接口。

> 结论：**不需要 CANable，也不需要厂商上位机/SDK**。达妙模块刷成
> `socketcan/gs_usb` 固件后，就是一个标准 gs_usb 设备；RK3588/aarch64 安装达妙的
> gs_usb 驱动包即可。项目代码零改动。

## 0. 硬件与总线映射

```
Orange Pi 5 (aarch64, Ubuntu 24.04, kernel 6.1)
        │  USB (建议带外部供电的 Hub)
  2× DM-USB2CANFD Dual (gs_usb 固件)
        ├── 模块 A: CAN1 → can0 (FL, 电机 1/2/3)
        │           CAN2 → can1 (FR, 电机 4/5/6)
        └── 模块 B: CAN1 → can2 (RL, 电机 7/8/9)
                    CAN2 → can3 (RR, 电机 10/11/12)
```

| 接口 | 腿 | 电机 ID | 默认站立角 (hip/thigh/calf) |
|------|----|---------|-----------------------------|
| can0 | FL 左前 | 1/2/3 | 0 / 0.8 / -1.6 |
| can1 | FR 右前 | 4/5/6 | 0 / 0.8 / -1.6 |
| can2 | RL 左后 | 7/8/9 | 0 / 0.8 / -1.6 |
| can3 | RR 右后 | 10/11/12 | 0 / 0.8 / -1.6 |

**EL05 使用经典 CAN：1 Mbps、8 字节、29-bit 扩展帧。不要开 CAN FD。**

## 1. 刷 SocketCAN 固件（Windows）

资料在达妙 `dm-tools` 仓库：

- 升级工具：`USB2CANFD通用升级工具/USB2CANFD系列升级工具.zip`
- 双路 gs_usb 固件：`USB2CANFD_Dual/固件/socketcan/dm_usb2canfd_dual_gsusb_1004.enc`
- 出厂固件（回滚用）：`USB2CANFD_Dual/固件/出厂固件/dm_usb2canfd_dual_app_1007.enc`

步骤：

1. Windows 解压升级工具。
2. **一次只插一个模块**，选择 `..._dual_gsusb_1004.enc` 刷入。
3. 刷完记录模块序列号/贴标签，并记录 CAN1/CAN2 两个端子接哪条腿。
4. 第二个模块重复。
5. 刷完后插到 x86 笔记本验证（见下一节）。

> ⚠️ 刷成 SocketCAN 固件后，达妙上位机无法连接；需要恢复时刷回出厂固件。
> 固件和工具版本会更新，刷写前以 `dm-tools` 仓库里的最新文件为准。

## 2. x86 笔记本先验证（不要直接上 Orange Pi）

x86 Ubuntu 内核 6.8 自带 `gs_usb`，标准 VID/PID 免驱，适合先验证固件和电机协议。

```bash
# 插一个模块
lsusb | grep -E "1d50:606f|1209:2323|1cd2:606f|16d0:10b8|16d0:0f30"
ip -br link
ip -details link show type can
dmesg | tail -30
```

预期出现 `canX`。然后按**经典 CAN** 配置：

```bash
sudo ip link set canX down
sudo ip link set canX type can bitrate 1000000 loopback off
sudo ip link set canX up
ip -details link show canX      # can state ERROR-ACTIVE, bitrate 1000000
```

接一个 EL05 电机，用现有只读脚本确认：

```bash
python3 hardware/motor/scan_motors_readonly.py --channel canX --all-ids
```

**不要**照搬达妙 CANFD 示例的 `dbitrate 5000000 fd on`，EL05 是经典帧。

## 3. Orange Pi 5：Ubuntu 24.04 + 达妙 RK3588 驱动

### 3.1 系统与依赖

推荐 ubuntu-rockchip 的 Ubuntu 24.04（kernel 6.1）或 Orange Pi 官方 Ubuntu。
安装基础工具：

```bash
sudo apt update
sudo apt install -y build-essential cmake git can-utils \
                    linux-headers-$(uname -r) python3-pip python3-numpy
python3 -m pip install python-can pyserial
```

### 3.2 安装达妙 gs_usb 驱动

驱动包：`dm-tools/gs_usb_drives/gs_usb_sppe_RK_board/`

| 内核 | 目录 | 预编译 .ko |
|------|------|-----------|
| 5.1x | `gs_usb_5.1/` | 无，需编译 |
| 6.1  | `gs_usb_6.1/` | 有 |
| 6.8  | `gs_usb_6.8/` | 有 |

```bash
git clone https://gitee.com/kit-miao/dm-tools.git
cd dm-tools/gs_usb_drives/gs_usb_sppe_RK_board/gs_usb_6.1

# 1) 核对预编译模块是否匹配当前内核（vermagic 第一段必须等于 uname -r）
modinfo -F vermagic gs_usb.ko
uname -r
grep -E '^CONFIG_(CAN_GS_USB|MODVERSIONS)=' /boot/config-$(uname -r) 2>/dev/null || true

# 2) 不匹配则在板子上重新编译
make -j4

# 3) 停止服务、放下接口，临时加载验证
sudo systemctl stop opendoge-can 2>/dev/null || true
for i in 0 1 2 3; do sudo ip link set can$i down 2>/dev/null || true; done
sudo modprobe -r gs_usb
sudo modprobe can can-dev usbcore
sudo insmod ./gs_usb.ko

# 4) 验证 OK 后永久安装到 updates/（不要覆盖 in-tree 模块）
sudo install -D -m 0644 ./gs_usb.ko \
    /lib/modules/$(uname -r)/updates/gs_usb.ko
sudo depmod -a
sudo modprobe gs_usb

# 5) 确认加载的是 updates/ 里的模块
modinfo -F filename gs_usb
lsmod | grep gs_usb
ip -br link
```

> `CONFIG_MODVERSIONS=y` 时，预编译模块除了 vermagic 还要符号 CRC 匹配；
> 不匹配就只能在板子上用**同版本内核头文件**编译。若内核启用了模块签名，
> 还要检查 `dmesg` 里的 signature/MOK 报错。

仓库里也提供了助手脚本（默认只检查、不修改系统）：

```bash
bash scripts/setup_dm_usb2canfd.sh --check
sudo bash scripts/setup_dm_usb2canfd.sh --apply --driver-dir /path/to/gs_usb_6.1
```

### 3.3 配置四个经典 CAN 接口

```bash
for i in 0 1 2 3; do
  sudo ip link set can$i down 2>/dev/null || true
  sudo ip link set can$i type can bitrate 1000000 fd off loopback off
  sudo ip link set can$i up
done

ip -details -statistics link show can0
# 预期: bitrate 1000000, 无 dbitrate / CAN_CTRLMODE_FD, 有节点时 ERROR-ACTIVE
```

生产上由 `opendoge-can.service` 自动做同样的事。

## 4. 接口顺序与稳定命名（重要安全项）

两个相同的双路模块，USB 枚举顺序可能变化，导致 `can0-can3` 互换；一旦互换，
控制指令会发到错误的腿。**每次使能前必须先做只读映射校验。**

### 4.1 采集接口属性

```bash
for i in 0 1 2 3; do
  echo "== can$i"
  udevadm info -q property -n can$i | \
    grep -E "ID_PATH|ID_SERIAL|ID_NET_NAME|DEVPATH"
  ip -details link show can$i | grep -E "dev_port|phys_port|can state"
done
```

### 4.2 固定命名

双路模块的两个通道对应两个 USB interface，例如 `1-1.2:1.0` / `1-1.2:1.1`。
用 udev `KERNELS` 按 USB interface 路径命名，比按 VID/PID 或枚举顺序可靠：

```udev
ACTION=="add", SUBSYSTEM=="net", KERNELS=="1-1.2:1.0", NAME="can0"
ACTION=="add", SUBSYSTEM=="net", KERNELS=="1-1.2:1.1", NAME="can1"
ACTION=="add", SUBSYSTEM=="net", KERNELS=="1-1.3:1.0", NAME="can2"
ACTION=="add", SUBSYSTEM=="net", KERNELS=="1-1.3:1.1", NAME="can3"
```

仓库提供了生成器：

```bash
# 先确认当前 can0-can3 映射正确，再冻结当前 USB 路径
sudo bash scripts/setup_can_names.sh --freeze-current

# 或显式指定
sudo bash scripts/setup_can_names.sh \
  can0=1-1.2:1.0 can1=1-1.2:1.1 \
  can2=1-1.3:1.0 can3=1-1.3:1.1
```

注意：

- 规则写到 `/etc/udev/rules.d/70-opendoge-can.rules`，会先备份旧文件。
- 执行后重新插拔模块或重启，**冷启动两次**确认名称稳定。
- 如果 Orange Pi 板载原生 CAN 占用了 `can0`，udev 无法覆盖；先在
  `/boot/orangepiEnv.txt` 或 `/boot/armbianEnv.txt` 禁用对应 overlay，或
  blacklist 原生 CAN 驱动。
- 上机前用 `sudo python3 scripts/verify_can_mapping.py` 校验（见 4.3）。

### 4.3 只读映射校验（必须）

```bash
sudo python3 scripts/verify_can_mapping.py
# 允许部分电机暂未接入:
sudo python3 scripts/verify_can_mapping.py --allow-missing
```

脚本会断言：

- can0 只出现 ID 1/2/3
- can1 只出现 ID 4/5/6
- can2 只出现 ID 7/8/9
- can3 只出现 ID 10/11/12

出现意外 ID、缺失接口或空总线都会返回非 0，**不要在这些情况下使能电机**。

## 5. 项目集成

- `daemons/opendoge-can.service` 现在会：
  - 等待 can0-can3 最多 60 s；
  - 检查每个接口的驱动确实是 `gs_usb`；
  - 用 `bitrate 1000000 fd off loopback off` 配置经典 CAN；
  - 任一接口配置失败就退出非 0（不再静默跳过）。
- `daemons/opendoge-imu.service` / `opendoge-joystick.service` 里硬编码了
  `/home/orangepi/Opendoge/OpenDoge_firmware`；`scripts/install_services.sh`
  安装时会用当前仓库路径替换。
- aarch64 构建建议使用独立的 build/install/log 目录，避免复用 x86 的缓存：
  ```bash
  DEPS_DIR="$PWD/build_aarch64/deps" ./scripts/setup_onnx.sh
  export ONNXRUNTIME_ROOT="$(realpath build_aarch64/deps/onnxruntime)"
  colcon build --symlink-install --packages-select opendoge_deploy \
    --build-base build_aarch64 --install-base install_aarch64 --log-base log_aarch64
  source install_aarch64/setup.bash
  ```
- 如果 `opendoge_deploy` 需要非 root 运行 raw CAN socket，可以：
  ```bash
  sudo setcap cap_net_raw+ep install_aarch64/opendoge_deploy/bin/opendoge_deploy
  ```
  （只给 deploy 二进制授权，不要给整个 Python 解释器授权。）

## 6. 排障

| 现象 | 检查 |
|------|------|
| `lsusb` 仍是 `34b7:*` | 固件没刷成功，重新用升级工具刷 gs_usb 固件 |
| 有 gs_usb 设备但没有 canX | 驱动没加载/vermagic 不匹配；`dmesg`、`modinfo gs_usb.ko` |
| canX 有但 `candump` 无响应 | 1 Mbps、CAN_H/CAN_L、120 Ω 终端电阻、电机供电 |
| `insmod` 报 invalid module format | 在板子上 `make` 重新编译 |
| can0-can3 顺序不对 | 用 4.2 的 `.link` + 4.3 的校验脚本 |
| 接口 BUS-OFF / ERROR-PASSIVE | 终端电阻、线序、波特率；先 `ip link set canX down/up` |
| 误开 CANFD | 只用 `bitrate 1000000`，不要加 `fd on`/`dbitrate` |

## 7. 回滚

- 刷回出厂固件：`USB2CANFD_Dual/固件/出厂固件/dm_usb2canfd_dual_app_1007.enc`，
  用同一个升级工具。
- 恢复原厂 gs_usb 模块：把 `gs_usb.ko.stock` 复制回原路径，`depmod -a`，
  重新插拔模块。
- 项目侧：切回 CANable 时只需把 4 个适配器插上，`can0-can3` 会由内核自动创建。

## 参考

- SocketCAN C++ 例程 WORKFLOW：
  https://gitee.com/kit-miao/motor-control-routine/blob/fa8f511cab00ac424a10fb94d4ad2c11e270f5b2/SocketCan%E6%8E%A7%E5%88%B6%E4%BE%8B%E7%A8%8B/C%2B%2B%E4%BE%8B%E7%A8%8B/WORKFLOW.md
- 双路 SocketCAN 固件：
  https://gitee.com/kit-miao/dm-tools/tree/master/USB2CANFD_Dual/%E5%9B%BA%E4%BB%B6/socketcan
- RK3588 gs_usb 驱动包：
  https://gitee.com/kit-miao/dm-tools/tree/master/gs_usb_drives/gs_usb_sppe_RK_board
- 升级工具：
  https://gitee.com/kit-miao/dm-tools/tree/master/USB2CANFD%E9%80%9A%E7%94%A8%E5%8D%87%E7%BA%A7%E5%B7%A5%E5%85%B7
- 双路模块说明书：
  https://github.com/dmBots/usb2canfd-dual
