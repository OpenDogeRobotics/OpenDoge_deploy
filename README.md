# OpenDoge_deploy

OpenDoge 强化学习策略的 MuJoCo Sim2Sim 部署与验证仓库。

## 目录结构

```
OpenDoge_deploy/
└── deploy/
    └── deploy_mujoco/
        ├── configs/
        │   └── opendoge.yaml         # OpenDoge 部署配置
        ├── deploy_opendoge.py        # OpenDoge MuJoCo 策略部署
        └── onnx_path_utils.py        # ONNX 路径解析工具
```

## 功能说明

在 MuJoCo 环境中加载训练好的 ONNX 策略进行仿真验证，支持键盘控制。

### 使用

```bash
# 自动查找 onnx/flat_opendoge*.onnx
python deploy/deploy_mujoco/deploy_opendoge.py

# 指定 ONNX 模型
python deploy/deploy_mujoco/deploy_opendoge.py --onnx onnx/flat_opendoge_xxx.onnx
```

### 键盘操作

| 前进 | 后退 | 左转 | 右转 | 暂停 |
|------|------|------|------|------|
| ↑ | ↓ | ← | → | Space |

### 配置文件

`configs/opendoge.yaml` 中的关键参数需与训练配置对齐：

- PD 刚度/阻尼、默认关节角度
- 观测/动作缩放因子
- 控制频率 (`simulation_dt` × `control_decimation`)

## 依赖

- Python >= 3.8
- MuJoCo >= 3.0
- ONNX Runtime
- NumPy
- PyYAML
- pynput

## 许可证

参见仓库 LICENSE 文件。
