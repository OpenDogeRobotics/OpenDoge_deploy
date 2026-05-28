# OpenDoge Deploy

OpenDoge 强化学习策略的 MuJoCo Sim2Sim 部署与验证。

## 环境

```bash
conda activate himloco
cd /home/lain/OpenDoge/OpenDoge_deploy
```

## 启动方式

### 键盘控制（方向键 + A/D）

```bash
python deploy/deploy_mujoco/deploy_opendoge.py
# 或指定 ONNX 模型
python deploy/deploy_mujoco/deploy_opendoge.py --onnx onnx/flat_opendoge_9000_omni.onnx
```

| ↑ 前进 | ↓ 后退 | ← 左转 | → 右转 | Ctrl+← 左移 | Ctrl+→ 右移 | 空格 暂停 |
|--------|--------|--------|--------|--------|--------|-----------|

### XBOX 手柄控制

```bash
python deploy/deploy_mujoco/deploy_opendoge_xbox.py
# 或指定 ONNX 模型
python deploy/deploy_mujoco/deploy_opendoge_xbox.py --onnx onnx/flat_opendoge_9000_omni.onnx
```

| 操作 | 映射 |
|------|------|
| 左摇杆 ↑↓ | 前进 / 后退 |
| 左摇杆 ←→ | 左 / 右平移 |
| 右摇杆 ←→ | 左转 / 右转 |
| START | 暂停 / 恢复 |
| BACK | 退出仿真 |

未检测到手柄时脚本仍可启动（零指令，仅供调试观察）。

## 可用 ONNX 模型

| 文件 | 说明 |
|------|------|
| `onnx/flat_opendoge_5700.onnx` | Gen4 风格策略，5700 轮 |
| `onnx/flat_opendoge_9000_omni.onnx` | 全向策略，9000 轮（推荐） |
| `onnx/flat_opendoge_gen52_4800.onnx` | Gen52 策略，4800 轮 |

## PD 参数对齐

配置文件 `deploy/deploy_mujoco/configs/opendoge.yaml` 已与训练配置对齐：

| 参数 | 部署值 | 训练值 |
|------|--------|--------|
| kp | 12.0 | 12.0 |
| kd | 0.5 | 0.5 |
| action_scale | 0.30 | 0.30 |
| control_decimation | 2 (100Hz) | 2 (100Hz) |
| simulation_dt | 0.005 (200Hz) | 0.005 (200Hz) |
| init_base_height | 0.15m | 0.15m |

## 目录结构

```
OpenDoge_deploy/
├── deploy/
│   └── deploy_mujoco/
│       ├── configs/
│       │   └── opendoge.yaml           # 部署配置（PD、观测缩放、默认角度等）
│       ├── deploy_opendoge.py          # 键盘控制 Sim2Sim
│       ├── deploy_opendoge_xbox.py     # XBOX 手柄控制 Sim2Sim
│       └── onnx_path_utils.py          # ONNX 路径解析
├── onnx/                               # 训练好的 ONNX 策略
├── resources/
│   └── robots/Opendoge/
│       ├── xml/Opendoge.xml            # MJCF 模型（已对齐 URDF 碰撞/惯性/关节限位）
│       ├── xml/scene.xml
│       └── meshes/                     # STL 网格
└── mujoco/                             # 传统控制相关（IK、位置控制等）
```
