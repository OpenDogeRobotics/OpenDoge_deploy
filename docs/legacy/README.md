# Legacy：已移除的 DingLab / SOULDE USB2CAN 方案

2026-11-05 起，项目从 **DINGLAB/SOULDE USB2CAN V2.0 双模块 + vcan 桥接**
迁移到 **赤乌（CHIWU）4×CAN 原生 SocketCAN**（`can0-can3`）。

旧代码、服务、udev 规则和文档已完整归档到：

```text
~/DingLab usb转can代码/
```

（路径带空格；归档目录里的 `README.md` 包含旧拓扑、恢复步骤和警告。）

## 从当前仓库移除的内容

| 文件 | 说明 |
|------|------|
| `hardware/motor/usb2can_driver.py` | DingLab 17 字节串口协议 native/vendor 后端 |
| `hardware/motor/usb2can_vcan_bridge.py` | USB2CAN ↔ vcan 桥接 |
| `hardware/motor/el05_usb2can_scan.py` | 经 DingLab 设备的 EL05 扫描 |
| `hardware/motor/dinglab_protocol_selftest.py` | 17 字节协议自检 |
| `hardware/motor/can_udp_gateway.py` | 已废弃的 UDP 网关 |
| `daemons/opendoge-vcan.service` | 创建 vcan can0-can3 |
| `daemons/opendoge-usb2can0.service` | USB2CAN0 → can0/can1 |
| `daemons/opendoge-usb2can1.service` | USB2CAN1 → can2/can3 |
| `docs/usb2can/dinglab_usb2can.md` | 旧方案详细文档 |

同时：

- `scripts/install_services.sh` 删除了 `--vcan` / `--udev-rule` 模式；
- `scripts/start_robot.sh` 删除了 vcan bridge 检查；
- `scripts/setup_vcan.sh` 仅保留为**无硬件 dry-run 手动测试**工具，
  不再由 systemd 安装；
- `/etc/udev/rules.d/usb_can.rules` 不再需要（归档目录有副本）。

## 为什么要移除

- 旧方案两模块分别挂在 USB Hub 上，USB2CAN1（1483:1111）曾反复
  disconnect/re-enumerate，vcan 仍保持 UP，导致 bridge 死了但上层以为
  CAN 可用；`COMM_STOP` 静默丢失，后腿电机保持阻尼无法失能。
- 新 CHIWU 板原生暴露 `can0-can3`（Linux SocketCAN），不需要 vcan 和
  userspace bridge；部署/扫描/菜单/监控的接口契约完全不变。

## 恢复旧方案

按归档目录 `~/DingLab usb转can代码/README.md` 操作。要点：

1. `sudo systemctl disable --now opendoge-can`；
2. 删除当前原生 can0-3 接口（如果驱动已创建）；
3. 把归档文件复制回仓库对应位置；
4. 安装 `usb_can.rules`，重新启用 `opendoge-vcan` + `opendoge-usb2can0/1`；
5. 确认 `can0-can3` 是 vcan，再启动 deploy。

**不要同时运行两套方案**（同名 can0-3 会冲突）。
