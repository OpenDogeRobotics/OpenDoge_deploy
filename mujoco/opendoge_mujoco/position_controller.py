from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import mujoco
import numpy as np


@dataclass(frozen=True)
class JointCommand:
    q_des: np.ndarray
    dq_des: np.ndarray


class PDPositionController:
    """Torque-level PD position controller for MuJoCo hinge joints."""

    def __init__(
        self,
        model: mujoco.MjModel,
        joint_names: Sequence[str],
        kp: Mapping[str, float],
        kd: Mapping[str, float],
    ) -> None:
        self.model = model
        self.joint_names = list(joint_names)
        self.joint_ids = np.array(
            [self._name_to_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joint_names],
            dtype=np.int32,
        )
        self.actuator_ids = np.array(
            [self._name_to_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.joint_names],
            dtype=np.int32,
        )
        self.qpos_addr = model.jnt_qposadr[self.joint_ids]
        self.qvel_addr = model.jnt_dofadr[self.joint_ids]
        self.kp = np.array([kp[name] for name in self.joint_names], dtype=np.float64)
        self.kd = np.array([kd[name] for name in self.joint_names], dtype=np.float64)
        self.ctrl_limited = model.actuator_ctrllimited[self.actuator_ids].astype(bool)
        self.ctrl_range = model.actuator_ctrlrange[self.actuator_ids].copy()
        self.joint_limited = model.jnt_limited[self.joint_ids].astype(bool)
        self.joint_range = model.jnt_range[self.joint_ids].copy()

    def _name_to_id(self, obj_type: mujoco.mjtObj, name: str) -> int:
        obj_id = mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError(f"MuJoCo model does not contain {obj_type.name}: {name}")
        return obj_id

    def joint_positions(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self.qpos_addr].copy()

    def joint_velocities(self, data: mujoco.MjData) -> np.ndarray:
        return data.qvel[self.qvel_addr].copy()

    def clamp_to_joint_limits(self, q_des: np.ndarray) -> np.ndarray:
        q_clamped = q_des.copy()
        limited = self.joint_limited
        q_clamped[limited] = np.clip(
            q_clamped[limited],
            self.joint_range[limited, 0],
            self.joint_range[limited, 1],
        )
        return q_clamped

    def apply(self, data: mujoco.MjData, command: JointCommand) -> np.ndarray:
        q = self.joint_positions(data)
        dq = self.joint_velocities(data)
        q_des = self.clamp_to_joint_limits(command.q_des)
        tau = self.kp * (q_des - q) + self.kd * (command.dq_des - dq)

        limited = self.ctrl_limited
        tau[limited] = np.clip(tau[limited], self.ctrl_range[limited, 0], self.ctrl_range[limited, 1])
        data.ctrl[self.actuator_ids] = tau
        return tau

