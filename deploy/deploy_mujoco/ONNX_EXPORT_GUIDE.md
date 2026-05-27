# OpenDoge ONNX 导出与 Sim2Sim 部署指南

## 一、ONNX 导出

### 1.1 导出脚本

```bash
cd /home/lain/OpenDoge/OpenDoge_train
export PYTHONPATH=$PWD
conda run -n himloco python scripts/export_onnx.py \
    --checkpoint logs/flat_opendoge/<run_dir>/model_<N>.pt \
    --output onnx/flat_opendoge_<N>.onnx
```

脚本位置：[scripts/export_onnx.py](../../OpenDoge_train/scripts/export_onnx.py)

### 1.2 架构说明

HIMLoco 使用 HIMActorCritic，包含两个子模块：

```
                          ┌─────────────────────┐
obs_history (270) ───────▶│  Estimator.encoder  │──▶ vel(3) + latent(16)
                          └─────────────────────┘
                                     │
obs_history[:, :45] (45) ────────────┼──▶ [45 + 3 + 16] = 64
                                     │
                          ┌──────────▼──────────┐
                          │       Actor          │──▶ actions(12)
                          └─────────────────────┘
```

- **Estimator**: `270 → 128 → 64 → 19` （3 维预测速度 + 16 维隐变量）
- **Actor**: `64 → 512 → 256 → 128 → 12`

### 1.3 导出参数对照

| 参数 | 值 | 来源 |
|------|-----|------|
| `num_actor_obs` | 270 | `num_observations` (45 × 6 帧) |
| `num_critic_obs` | 238 | `num_privileged_obs` |
| `num_one_step_obs` | 45 | `num_one_step_observations` |
| `num_actions` | 12 | 关节数量 |
| `actor_hidden_dims` | [512, 256, 128] | `policy.actor_hidden_dims` |
| `critic_hidden_dims` | [512, 256, 128] | `policy.critic_hidden_dims` |
| `activation` | elu | `policy.activation` |

---

## 二、观测对齐

### 2.1 单步观测结构（45 维）

训练代码 `legged_robot.py:compute_observations` 与部署代码 `deploy_opendoge.py` 使用相同的观测顺序：

```
[dof_pos - default_dof_pos] × dof_pos_scale(12)  → [9:21]
[dof_vel]                  × dof_vel_scale(12)    → [21:33]
[last_action]                            (12)     → [33:45]
```

### 2.2 历史缓冲区（270 维）

6 帧 × 45 维 = 270 维，通过 `deque(maxlen=6)` 滑动窗口拼接。

### 2.3 缩放因子

| 缩放项 | 训练 | 部署 | 值 |
|--------|------|------|-----|
| 线速度指令 | `commands_scale` | `cmd_scale[0:2]` | 2.0 |
| 角速度指令 | `commands_scale[2]` | `cmd_scale[2]` | 0.25 |
| 角速度 | `obs_scales.ang_vel` | `ang_vel_scale` | 0.25 |
| 关节位置 | `obs_scales.dof_pos` | `dof_pos_scale` | 1.0 |
| 关节速度 | `obs_scales.dof_vel` | `dof_vel_scale` | 0.05 |
| 动作 | `action_scale` | `action_scale` | 0.25 |

---

## 三、PD 参数对齐

| 参数 | 训练 | 部署 | 值 |
|------|------|------|-----|
| 刚度 Kp | `control.stiffness` | `kps[0..11]` | 10.0 |
| 阻尼 Kd | `control.damping` | `kds[0..11]` | 0.5 |
| 控制频率 | `200 / decimation` | `1 / (simulation_dt × control_decimation)` | 100 Hz |
| 仿真步长 | `sim.dt` | `simulation_dt` | 0.005s |

---

## 四、Sim2Sim 运行

### 4.1 准备工作

1. 按第一章导出 ONNX 模型
2. 将 ONNX 模型复制到 `OpenDoge_deploy/onnx/` 目录
3. 确认 [configs/opendoge.yaml](configs/opendoge.yaml) 中参数已对齐

### 4.2 默认运行（自动匹配最新 ONNX）

```bash
cd /home/lain/OpenDoge/OpenDoge_deploy
conda run -n himloco python deploy/deploy_mujoco/deploy_opendoge.py
```

脚本会自动查找 `onnx/flat_opendoge*.onnx`，选择最新文件。

### 4.3 指定 ONNX 模型

```bash
conda run -n himloco python deploy/deploy_mujoco/deploy_opendoge.py \
    --onnx /path/to/model.onnx
```

或通过环境变量：

```bash
export OPENDOGE_ONNX_PATH=/path/to/model.onnx
conda run -n himloco python deploy/deploy_mujoco/deploy_opendoge.py
```

### 4.4 ONNX 路径解析优先级

1. `--onnx` CLI 参数（最高优先级）
2. `$OPENDOGE_ONNX_PATH` 环境变量
3. `onnx/flat_opendoge*.onnx` 文件匹配（取最新修改时间）

### 4.5 键盘操作

| 按键 | 功能 |
|------|------|
| ↑ | 前进 |
| ↓ | 后退 |
| ← | 左转 |
| → | 右转 |
| Space | 暂停/恢复 |

---

## 五、配置文件

[configs/opendoge.yaml](configs/opendoge.yaml) 关键字段：

```yaml
# 仿真参数
simulation_dt: 0.005        # 物理步长，与训练 sim.dt 一致
control_decimation: 2       # 控制降采样 = 训练 control.decimation

# PD 参数（与训练对齐）
kps: [10, 10, 10, ...]     # 刚度，对应训练 control.stiffness
kds: [0.5, 0.5, 0.5, ...]  # 阻尼，对应训练 control.damping

# 默认关节角度（与训练 init_state.default_joint_angles 对齐）
default_angles: [0.0, 0.6, -1.5, ...]

# 缩放（与训练 normalization.obs_scales 对齐）
lin_vel_scale: 2.0
ang_vel_scale: 0.25
dof_pos_scale: 1.0
dof_vel_scale: 0.05
action_scale: 0.25
cmd_scale: [2.0, 2.0, 0.25]

# 维度
num_actions: 12
num_obs: 270                # 45 × 6 帧历史
num_one_step_obs: 45
```

---

## 六、常见问题

### 模型加载报维度不匹配

检查 `num_obs` 是否与训练时的 `num_observations` 一致（HIMLoco 标准为 270）。

### 机器人行为异常（抖动/趴倒/发疯）

逐项检查：
1. PD 的 kps/kds 是否与训练一致
2. `default_angles` 是否与训练 `default_joint_angles` 一致
3. 观测缩放因子是否匹配（第三章表格）
4. 观测顺序是否正确（第二章表格）
5. `action_scale` 是否匹配

### 找不到 ONNX 模型

确认 ONNX 文件位于 `OpenDoge_deploy/onnx/` 目录，且文件名匹配 `flat_opendoge*.onnx`。

---

## 七、依赖

- MuJoCo >= 3.0
- ONNX Runtime
- NumPy
- PyYAML
- pynput

Conda 环境 `himloco` 已包含全部依赖。
