# OpenDoge MuJoCo Position Control

This folder contains a MuJoCo torque-level PD position-control simulation for OpenDoge.

## Layout

```text
mujoco/
├── configs/
│   └── position_control.json
├── opendoge_mujoco/
│   └── position_controller.py
├── scripts/
│   └── run_position_control.py
└── requirements.txt
```

The default config loads the canonical model from this path, resolved relative to `configs/position_control.json`:

```text
../../../OpenDoge_description/URDF/xml/scene.xml
```

That keeps model assets in `OpenDoge_description` and control logic in `OpenDoge_deploy`.

## Install

```bash
cd /home/lain/OpenDoge/OpenDoge_deploy/mujoco
python3 -m pip install -r requirements.txt
```

## Run

Hold the default standing joint targets:

```bash
cd /home/lain/OpenDoge/OpenDoge_deploy/mujoco
python3 scripts/run_position_control.py --mode stand
```

Run a simple sinusoidal joint-position target:

```bash
python3 scripts/run_position_control.py --mode sine
```

Run without viewer:

```bash
python3 scripts/run_position_control.py --mode stand --no-render --duration 5
```

## Keyboard Action IK Demo

Run the trot action-IK demo:

```bash
python3 scripts/run_keyboard_ik_control.py
```

The demo uses:

- Trot gait: `FL + RR` and `FR + RL` move as diagonal pairs
- Time cycle: `action_ik.cycle_time`
- Stance ratio: `action_ik.duty_factor`
- Swing curve: cycloid horizontal progress with sinusoidal foot lift
- Stance traction: after swing touchdown, support feet pull backward by the same distance as `action_ik.step_x`, `action_ik.step_y`, and `action_ik.step_yaw`
- Stand point: the nominal foot position is the middle of each leg's stance segment
- Startup blend: `action_ik.startup_blend_time` blends from stand feet to the trot phase targets
- Larger step commands: `action_ik.step_x`, `action_ik.step_y`, `action_ik.step_yaw`
- Rear-leg stance height offset: `action_ik.rear_stance_height_offset`
- Motor speed limit: `action_ik.max_joint_speed_rad_s`
- Key release state machine: real X11 key polling when available; otherwise `action_ik.key_hold_timeout` fallback
- IMU feedback: `imu_feedback` reads MuJoCo `orientation` and `angular-velocity` sensors for heading hold and roll/pitch foot-height correction

Controls:

- hold `Up` / `Down`: forward / backward
- hold `Left` / `Right`: move left / right
- hold two direction keys: diagonal omnidirectional motion
- hold `Ctrl + Left` / `Ctrl + Right`: turn left / right
- hold `Up/Down + Ctrl + Left/Right`: move forward/backward while turning
- release movement keys: return to stand
- `Space`: stop
- `R`: reset pose
- `Esc` or closing the viewer window: exit the process

Headless smoke test:

```bash
python3 scripts/run_keyboard_ik_control.py --no-render --duration 5 --cmd-vx 1
```

Other headless commands:

```bash
python3 scripts/run_keyboard_ik_control.py --no-render --duration 5 --cmd-vy 1
python3 scripts/run_keyboard_ik_control.py --no-render --duration 5 --cmd-yaw 1
```

## Notes

- The MuJoCo model exposes torque motors named the same as the 12 actuated joints.
- The controller computes `tau = kp * (q_des - q) + kd * (dq_des - dq)`.
- Target positions are clamped to joint limits before control.
- Torques are clipped to each actuator `ctrlrange` from the MJCF.
