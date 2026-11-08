# DM-IMU Bridge

`dm_imu_bridge.py` converts DM-IMU-L1 data to the OpenDoge `imu.state` file consumed by `opendoge_deploy`.

Output format:

```text
wx=...
wy=...
wz=...
gx=...
gy=...
gz=...
t=...          # 最近一帧的单调收包时间 (CLOCK_MONOTONIC)
gyro_t=...     # gyro 帧的收包时间
gyro_seq=...   # gyro 帧计数（只在真实新帧时 +1）
quat_t=...     # quaternion 帧的收包时间
quat_seq=...   # quaternion 帧计数
```

The bridge uses gyroscope data for `w*` and quaternion attitude for projected gravity `g*`.

`t/gyro_t/quat_t/seq` 是给 deploy 做**样本年龄/冻结检测**用的：bridge 现在
在收到新的 gyro/quat 帧时立即更新文件，但**写文件上限 2 ms（500 Hz）**；
DM-IMU-L1 实测 gyro/quat 各约 1 kHz，逐帧写（约 2 kHz 的
`atomic_write`+rename）会让 bridge 自身产生 40–130 ms 的年龄尖峰。deploy
用 `now - t` 得到年龄，用 `seq` 是否变化判断对应分量是否还在更新。旧的
消费脚本可以忽略这些行。

## Permissions (udev)

DM-IMU-L1 enumerates as `/dev/ttyACM*` with `root:dialout 0660`.
`opendoge-imu` runs as a **user service**, so the login user must be able to
open the tty. Install the udev rule:

```bash
sudo cp hardware/imu/99-opendoge-imu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=tty
ls -l /dev/ttyACM*          # IMU 应为 crw-rw-rw-
```

If the mode is still `0660`, unplug/replug the IMU or reboot.
Alternatively add the user to `dialout` and re-login:

```bash
sudo usermod -aG dialout "$USER"
```

`PermissionError: [Errno 13]` in `journalctl --user -u opendoge-imu` means the
rule/group is missing.

## USB / RS485 Active Frames

The deployment path uses the USB virtual serial active frame protocol:

```text
55 AA ID 02 gyro_x gyro_y gyro_z CRC16 0A
55 AA ID 04 quat_w quat_x quat_y quat_z CRC16 0A
```

Configure the IMU to output gyro and quaternion frames, then run:

```bash
./hardware/imu/dm_imu_bridge.py --device /dev/ttyUSB0 --baud 921600 --output /tmp/opendoge_imu.state
```

To configure the module through USB quick commands before reading:

```bash
./hardware/imu/dm_imu_bridge.py --device /dev/ttyUSB0 --baud 921600 --configure-usb --output /tmp/opendoge_imu.state
```

Add `--check-crc` to validate USB serial CRC16 before accepting frames.

## CAN Frames

Use a SocketCAN interface dedicated to the IMU or a bus where ID collision is impossible:

```bash
./hardware/imu/dm_imu_bridge.py --source can --can can4 --can-id 0x01 --output /tmp/opendoge_imu.state
```

## Axis Alignment

The default assumes the IMU coordinate frame is mounted the same as the robot base frame:

```text
robot x = imu x
robot y = imu y
robot z = imu z
```

If the physical mounting differs, use:

```bash
--axis-map xyz --axis-signs 1,1,1
```

`--axis-map` selects which IMU axes become robot x/y/z. `--axis-signs` flips the mapped axes.

## 重装/换向后如何确认与策略一致

策略（LainLab/mjlab）用 **x 前 / y 左 / z 上（右手系）** 的机体陀螺仪和
`projected_gravity_b = R^T·(0,0,-1)`。正确安装时：

| 动作 | 期望 g | 期望角速度 |
|------|--------|-----------|
| 竖直静止 | `gz≈-1`，`gx≈gy≈0` | ≈0 |
| 低头 | **`gx>0`** | `wy>0` |
| 抬头 | `gx<0` | `wy<0` |
| 右侧向下 | `gy<0` | `wx>0` |
| 左侧向下 | `gy>0` | `wx<0` |
| 从上方看逆时针偏航 | `g` 不变 | `wz>0` |

验证：

```bash
systemctl --user status opendoge-imu
journalctl --user -u opendoge-imu -n 20 --no-pager   # ready=1, age_ms 小
watch -n 0.1 cat /tmp/opendoge_imu.state
```

按上表缓慢倾斜并保持 1–2 s（等姿态滤波收敛），检查 6 个量。

判定：

- 全部匹配 → 保持 `--axis-map xyz --axis-signs 1,1,1`，不要加参数。
- **绕 z 的纯偏航安装**（`gz` 正确，`gx/gy` 成对旋转/反号）→ 可用
  `--axis-map/--axis-signs`，但必须是 proper rotation（det=+1）。
- **侧装/倒装**（竖直 `gz≈+1`，或只有单轴反号 = 反射）→ 当前 bridge 的
  quaternion 重映射不可靠：优先物理重装；否则改为“先用原始 quat 算 gravity，
  再对 gravity 向量和 gyro 应用 `remap_vec`”。
- `gz≈-1` 但低头 `gx<0` 说明 x 轴反了（反射），不要只给 x 加负号凑；
  先检查安装方向。

## 角速度低通（可选，真机振动大时）

训练策略期望的是**原始机体角速度 rad/s**。如果真机脚掌冲击/振动让
`wx/wy/wz` 有明显高频噪声，策略会过度反应，可以给 bridge 加低通：

```bash
--gyro-lowpass-hz 50     # 0=关闭（默认）；40-60 Hz 常用
```

- 一阶 IIR，按**实际写文件间隔**（≤2 ms 上限）计算，不再假设固定 200 Hz；
  相位滞后约 `1/(2πf)`：50 Hz ≈ 3 ms，40 Hz ≈ 4 ms；
- 只影响 `wx/wy/wz`，`projected_gravity` 仍来自姿态四元数（本身已平滑）；
- **不要**用 `--gyro-scale` 之类把角速度整体缩小来“降低敏感度”：
  那会改变策略的速度反馈，通常会让振荡更糟；
- 先确认角速度符号/量程正确，再考虑低通。量程验证：把机器人绕 z 轴
  匀速转一整圈（约 2 s），`∫wz dt` 应约 `2π`；偏差 ~57 倍说明单位是
  deg/s 而不是 rad/s。
