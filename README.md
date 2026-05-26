# OpenDoge_deploy

OpenDoge 强化学习策略部署与 Sim2Sim / Sim2Real 迁移仓库，包含 Mujoco 仿真验证、真实机器人部署与预训练模型。

## 目录结构

```
OpenDoge_deploy/
├── deploy/
│   ├── deploy_mujoco/          # Mujoco 仿真部署
│   │   ├── configs/            # 各机器人配置文件 (yaml)
│   │   │   ├── galileo.yaml    # Galileo 平地配置
│   │   │   ├── galileo_stairs.yaml
│   │   │   ├── galileo_terrain.yaml
│   │   │   ├── go2.yaml        # Go2 配置
│   │   │   ├── g1.yaml, h1.yaml, h1_2.yaml
│   │   │   └── zsl1.yaml, pi_plus.yaml 等
│   │   ├── deploy_go2.py          # Go2 部署脚本
│   │   ├── deploy_galileo*.py     # Galileo 部署脚本
│   │   ├── deploy_mujoco.py       # Mujoco 通用部署基类
│   │   └── F710GamePad.py         # 手柄驱动
│   ├── deploy_real/            # 真实机器人部署
│   │   ├── configs/            # 实机部署配置
│   │   ├── deploy_real.py      # 实机策略推理
│   │   ├── deploy_go2.py       # Go2 实机部署示例
│   │   ├── pt2onnx.py          # PyTorch → ONNX 模型导出
│   │   └── common/             # 工具模块
│   │       ├── command_helper.py    # 指令辅助
│   │       ├── remote_controller.py # 遥控器接口
│   │       └── rotation_helper.py   # 旋转转换工具
│   └── pre_train/              # 预训练模型权重
│       ├── galileo/motion.pt   # Galileo
│       ├── g1/motion.pt        # G1
│       ├── h1/motion.pt        # H1
│       ├── h1_2/motion.pt      # H1_2
│       └── zsl1/               # ZSL1 (含 CMORL 变体)
```

## 功能说明

### Sim2Sim (Mujoco 仿真验证)

在 Mujoco 环境中验证策略，支持键盘/手柄遥控：

```bash
# 平地行走
python deploy/deploy_mujoco/deploy_galileo.py galileo_plane.yaml
python deploy/deploy_mujoco/deploy_go2.py go2.yaml

# 复杂地形
python deploy/deploy_mujoco/deploy_galileo.py galileo_stairs.yaml
python deploy/deploy_mujoco/deploy_galileo.py galileo_terrain.yaml

# 键盘控制（无需手柄）
python deploy/deploy_mujoco/deploy_galileo_keyboard.py galileo_stairs.yaml

# 无遥控（持续给定前进速度）
python deploy/deploy_mujoco/deploy_galileo_no_teleop.py galileo_stairs.yaml
```

### Sim2Real (实机部署)

将 ONNX 策略部署到真实机器人：

```bash
# PyTorch → ONNX 导出
python deploy/deploy_real/pt2onnx.py

# 实机部署
python deploy/deploy_real/deploy_real.py --config configs/go2.yaml
python deploy/deploy_real/deploy_go2.py
```

### 手柄操作

| 前进 | 后退 | 左移 | 右移 | 左转 | 右转 |
|------|------|------|------|------|------|
| ↑ | ↓ | ← | → | Ctrl+← | Ctrl+→ |

Logitech F710：X 模式，左摇杆控制平移，右摇杆控制转向。

## 依赖

- PyTorch (模型加载与导出)
- ONNX / ONNX Runtime (实机推理)
- Mujoco (仿真验证)
- LCM (实机通信)
- NumPy

## 许可证

参见仓库 LICENSE 文件。

## MuJoCo 位置控制仿真

MuJoCo 控制逻辑位于 `mujoco/`，默认加载 `OpenDoge_description/URDF/xml/scene.xml`。

```bash
cd /home/lain/OpenDoge/OpenDoge_deploy/mujoco
python3 -m pip install -r requirements.txt
python3 scripts/run_position_control.py --mode stand
```

无图形界面验证：

```bash
python3 scripts/run_position_control.py --mode sine --no-render --duration 5
```

键盘动作逆解 demo：

```bash
python3 scripts/run_keyboard_ik_control.py
```

该 demo 使用 trot 步态、周期相位、支撑/摆动段和摆线摆腿曲线，并在关节目标上加入电机转速限幅。方向键为按住运动、松开回 stand，支持斜向全向解算；`Ctrl + 左/右` 为差速转向。

# C 风格
python3 scripts/run_keyboard_ik_control.py --c-style

# 原有 planner（默认）
python3 scripts/run_keyboard_ik_control.py

# 无渲染测试
python3 scripts/run_keyboard_ik_control.py --c-style --no-render --duration 5 --cmd-vx 1

cd /home/lain/OpenDoge/OpenDoge_deploy/mujoco
python3 scripts/run_keyboard_ik_control.py --c-style
