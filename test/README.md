# test

| 文件 | 用途 |
|------|------|
| `deploy_gap_regression.py` | 控制回路纯函数回归测试（无硬件） |
| `monitor_rotation_test.py` | `monitor_status.py` 自动 CSV / 5 次轮转 / `--csv` 旧行为测试（无硬件） |
| `highrate_recorder_test.py` | `record_highrate.py` 帧解码/标定转换/CSV 轮转/flag + socketpair 端到端测试（无 CAN 硬件） |
| `imu_bridge_write_test.py` | `dm_imu_bridge.py` 收到新 gyro/quat 帧立即写、空转不重写、低通按实际帧间隔（无 IMU 硬件） |
| `chiwu_code.py` | 赤乌（CHIWU）USB-CAN 板厂商例程 |

> ⚠️ `chiwu_code.py` 是**厂商小米/CyberGear 电机例程**，不要原样在整机上运行：
> 它的 `main()` 会对 motor 6 执行 `set_motor_angle_zero()`，并让 motor 6/7
> 来回转动。只参考其中的 `ip link set canX type can ...` 和 `python-can`
> `socketcan` 用法。EL05 的协议实现见 `hardware/motor/el05_socketcan.cpp`。
