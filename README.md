# OpenDoge_deploy
部署与迁移仓库

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
