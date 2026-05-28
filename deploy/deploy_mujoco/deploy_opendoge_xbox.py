"""
OpenDoge MuJoCo Sim2Sim — XBOX 手柄控制版

左摇杆: 前进/后退 + 左/右平移
右摇杆: 左转/右转
START 键: 暂停/恢复
BACK 键: 退出

用法:
    python deploy/deploy_mujoco/deploy_opendoge_xbox.py
    python deploy/deploy_mujoco/deploy_opendoge_xbox.py --onnx onnx/flat_opendoge_9000_omni.onnx
"""

import time
import os
import argparse
import numpy as np
import mujoco
import mujoco.viewer
import onnxruntime as ort
import yaml
from collections import deque

# ================= 1. 路径配置 =================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
YAML_PATH = os.path.join(SCRIPT_DIR, "configs", "opendoge.yaml")
DEFAULT_XML_PATH = os.path.join(PROJECT_ROOT, "resources", "robots", "Opendoge", "xml", "scene.xml")

from onnx_path_utils import resolve_onnx_path


def parse_args():
    parser = argparse.ArgumentParser(description="Deploy OpenDoge policy in MuJoCo with XBOX controller.")
    parser.add_argument("--onnx", type=str, default=None)
    return parser.parse_args()


ARGS = parse_args()
ONNX_PATH = resolve_onnx_path(
    project_root=PROJECT_ROOT,
    cli_onnx=ARGS.onnx,
    env_vars=["OPENDOGE_ONNX_PATH"],
    onnx_glob="flat_opendoge*.onnx",
    robot_name="opendoge",
)

print(f"YAML: {YAML_PATH}")
print(f"ONNX: {ONNX_PATH}")

# ================= 2. 全局变量 =================
cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # [vx, vy, omega]
paused = False
default_dof_pos = None
running = True


# ================= 3. 辅助函数 =================
def quat_rotate_inverse(q, v):
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def build_policy_input(obs_raw, history_buffer, input_dim, num_obs):
    if input_dim == num_obs:
        history_buffer.appendleft(obs_raw.copy())
        return np.concatenate(list(history_buffer), axis=0).reshape(1, -1)
    if input_dim == 64:
        policy_input = np.zeros((1, 64), dtype=np.float32)
        policy_input[0, :45] = obs_raw
        return policy_input
    if input_dim == 45:
        return obs_raw.reshape(1, -1)
    raise ValueError(f"Unsupported ONNX input dim: {input_dim}")


def apply_deadzone(value, deadzone=0.08):
    """对摇杆轴值施加死区"""
    if abs(value) < deadzone:
        return 0.0
    # 将 [deadzone, 1.0] 重新映射到 [0, 1.0]
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


# ================= 4. XBOX 手柄输入 =================
try:
    import pygame

    pygame.init()
    pygame.joystick.init()

    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"检测到手柄: {joystick.get_name()} (轴:{joystick.get_numaxes()} 按钮:{joystick.get_numbuttons()})")
    else:
        print("警告: 未检测到手柄，请连接 XBOX 控制器后重新启动。")
        print("将使用零指令运行，仅供调试观察。")
except ImportError:
    print("警告: pygame 未安装，无法使用手柄。 pip install pygame")
    print("将使用零指令运行，仅供调试观察。")
    joystick = None
    pygame = None

# 速度指令缩放
CMD_VX_SCALE = 1.5   # 最大前进速度 (m/s)
CMD_VY_SCALE = 1.0   # 最大侧移速度 (m/s)
CMD_OMEGA_SCALE = 2.0  # 最大转向速度 (rad/s)


def poll_joystick():
    """读取手柄状态，更新全局 cmd。返回 False 表示需要退出。"""
    global cmd, paused, running

    if joystick is None or pygame is None:
        return True

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 7:  # START
                paused = not paused
                print(f"Paused: {paused}")
            elif event.button == 6:  # BACK
                print("BACK 键按下，退出仿真。")
                running = False
                return False

    # 读取摇杆轴
    # 左摇杆: 轴0=X(左/右), 轴1=Y(上/下, 前推为负)
    # 右摇杆: 轴3=X(左/右), 轴4=Y(上/下)
    lx = apply_deadzone(joystick.get_axis(0))   # 左摇杆 X  -> vy (侧移)
    ly = apply_deadzone(-joystick.get_axis(1))  # 左摇杆 Y  -> vx (前进, 取反因为前推为负)
    rx = apply_deadzone(joystick.get_axis(3))   # 右摇杆 X  -> omega (转向)

    cmd[0] = ly * CMD_VX_SCALE
    cmd[1] = lx * CMD_VY_SCALE
    cmd[2] = rx * CMD_OMEGA_SCALE

    return True


# ================= 5. MuJoCo 回调 =================
def key_callback(keycode):
    """MuJoCo viewer 键盘回调 — 空格暂停作为手柄外的备用控制"""
    global paused
    if chr(keycode) == " ":
        paused = not paused
        print(f"Paused: {paused}")


# ================= 6. 主程序 =================
def run_simulation():
    global cmd, default_dof_pos, running

    if not os.path.exists(YAML_PATH):
        print(f"错误: 找不到配置文件 {YAML_PATH}")
        return

    with open(YAML_PATH, "r", encoding="utf-8") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    sim_dt = float(config.get("simulation_dt", 0.005))
    control_decimation = int(config.get("control_decimation", 2))
    num_actions = int(config.get("num_actions", 12))
    num_obs = int(config.get("num_obs", 270))
    num_one_step_obs = int(config.get("num_one_step_obs", 45))
    init_base_height = float(config.get("init_base_height", 0.15))

    kps = np.array(config["kps"], dtype=np.float32)
    kds = np.array(config["kds"], dtype=np.float32)
    default_dof_pos = np.array(config["default_angles"], dtype=np.float32)

    ang_vel_scale = config["ang_vel_scale"]
    dof_pos_scale = config["dof_pos_scale"]
    dof_vel_scale = config["dof_vel_scale"]
    action_scale = config["action_scale"]
    cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

    if len(default_dof_pos) != num_actions or len(kps) != num_actions or len(kds) != num_actions:
        print("错误: YAML 中 num_actions 与 kps/kds/default_angles 维度不一致")
        return

    xml_path_cfg = config.get("xml_path", "")
    if xml_path_cfg:
        xml_path = xml_path_cfg.replace("{LEGGED_GYM_ROOT_DIR}", PROJECT_ROOT)
    else:
        xml_path = DEFAULT_XML_PATH

    print(f"XML : {xml_path}")

    if not os.path.exists(xml_path):
        print(f"错误: 找不到模型文件 {xml_path}")
        return

    print("正在加载 MuJoCo 模型...")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    model.opt.timestep = sim_dt

    use_gyro_sensor = True
    try:
        _ = data.sensor("angular-velocity").data
    except KeyError:
        use_gyro_sensor = False
        print("警告: 未找到传感器 'angular-velocity'，将回退到 data.qvel[3:6]。")

    print(f"正在加载 ONNX: {ONNX_PATH}")
    ort_session = ort.InferenceSession(ONNX_PATH)
    input_name = ort_session.get_inputs()[0].name
    input_shape = ort_session.get_inputs()[0].shape
    input_dim = int(input_shape[-1]) if isinstance(input_shape[-1], int) else num_obs
    print(f"ONNX Input Shape: {input_shape}")

    # --- 初始化状态 ---
    data.qpos[7:7 + num_actions] = default_dof_pos
    data.qpos[2] = init_base_height
    mujoco.mj_forward(model, data)

    target_dof_pos = default_dof_pos.copy()
    action = np.zeros(num_actions, dtype=np.float32)

    print("仿真开始！XBOX 手柄控制模式。")
    print("  左摇杆: 前进/后退 + 左/右平移")
    print("  右摇杆: 左转/右转")
    print("  START: 暂停  BACK: 退出  空格: 暂停(备用)")

    history_len = max(1, num_obs // num_one_step_obs)
    obs_dim = num_one_step_obs
    obs_history_buffer = deque(
        [np.zeros(obs_dim, dtype=np.float32) for _ in range(history_len)],
        maxlen=history_len,
    )

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        step_counter = 0
        while viewer.is_running() and running:
            step_start = time.time()

            # 读取手柄
            if not poll_joystick():
                break

            if not paused:
                if step_counter % control_decimation == 0:
                    qj = data.qpos[7:7 + num_actions]
                    dqj = data.qvel[6:6 + num_actions]
                    quat = data.qpos[3:7]

                    if use_gyro_sensor:
                        omega = data.sensor("angular-velocity").data.astype(np.float32)
                    else:
                        omega = data.qvel[3:6].astype(np.float32)

                    gravity_vec = np.array([0.0, 0.0, -1.0], dtype=np.float32)
                    proj_gravity = quat_rotate_inverse(quat, gravity_vec)

                    qj_norm = (qj - default_dof_pos) * dof_pos_scale
                    dqj_norm = dqj * dof_vel_scale
                    omega_norm = omega * ang_vel_scale
                    cmd_norm = cmd * cmd_scale

                    obs_raw = np.concatenate(
                        [cmd_norm, omega_norm, proj_gravity, qj_norm, dqj_norm, action],
                        axis=0,
                    ).astype(np.float32)

                    policy_input = build_policy_input(
                        obs_raw=obs_raw,
                        history_buffer=obs_history_buffer,
                        input_dim=input_dim,
                        num_obs=num_obs,
                    )

                    outputs = ort_session.run(None, {input_name: policy_input})
                    raw_action = np.clip(outputs[0][0], -10.0, 10.0)
                    action = raw_action
                    target_dof_pos = raw_action * action_scale + default_dof_pos

                tau = pd_control(
                    target_dof_pos,
                    data.qpos[7:7 + num_actions],
                    kps,
                    np.zeros_like(kds),
                    data.qvel[6:6 + num_actions],
                    kds,
                )

                if model.nu < num_actions:
                    print(f"错误: MuJoCo actuator 数量({model.nu}) < num_actions({num_actions})")
                    return

                tau_limit = np.abs(model.actuator_ctrlrange[:num_actions, 1])
                tau = np.clip(tau, -tau_limit, tau_limit)
                data.ctrl[:num_actions] = tau

                mujoco.mj_step(model, data)
                step_counter += 1

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    if pygame:
        pygame.quit()


if __name__ == "__main__":
    run_simulation()
