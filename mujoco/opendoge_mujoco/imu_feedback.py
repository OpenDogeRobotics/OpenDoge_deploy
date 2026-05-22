from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import mujoco
import numpy as np

from opendoge_mujoco.action_gait import BodyCommand


@dataclass(frozen=True)
class IMUState:
    roll: float
    pitch: float
    yaw: float
    gyro: np.ndarray


@dataclass(frozen=True)
class IMUFeedbackConfig:
    enabled: bool
    heading_kp: float
    yaw_rate_kd: float
    max_yaw_correction: float
    roll_kp: float
    roll_kd: float
    pitch_kp: float
    pitch_kd: float
    max_foot_z_correction: float


class IMUReader:
    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.orientation_adr = self._sensor_adr("orientation", expected_dim=4)
        self.gyro_adr = self._sensor_adr("angular-velocity", expected_dim=3)

    def read(self, data: mujoco.MjData) -> IMUState:
        quat = data.sensordata[self.orientation_adr : self.orientation_adr + 4]
        gyro = data.sensordata[self.gyro_adr : self.gyro_adr + 3].copy()
        roll, pitch, yaw = quat_to_rpy(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        return IMUState(roll=roll, pitch=pitch, yaw=yaw, gyro=gyro)

    def _sensor_adr(self, name: str, expected_dim: int) -> int:
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            raise ValueError(f"MuJoCo model does not contain sensor: {name}")
        dim = int(self.model.sensor_dim[sensor_id])
        if dim != expected_dim:
            raise ValueError(f"Sensor {name} has dim {dim}, expected {expected_dim}")
        return int(self.model.sensor_adr[sensor_id])


class IMUStabilizer:
    def __init__(self, config: IMUFeedbackConfig) -> None:
        self.config = config
        self.desired_yaw = 0.0

    def reset(self, imu: IMUState) -> None:
        self.desired_yaw = imu.yaw

    def command(self, command: BodyCommand, imu: IMUState) -> BodyCommand:
        if not self.config.enabled:
            return command

        corrected = BodyCommand(vx=command.vx, vy=command.vy, yaw=command.yaw)
        if abs(command.yaw) > 1.0e-6:
            self.desired_yaw = imu.yaw
            corrected.yaw += -self.config.yaw_rate_kd * float(imu.gyro[2])
        else:
            yaw_error = wrap_to_pi(self.desired_yaw - imu.yaw)
            yaw_correction = self.config.heading_kp * yaw_error - self.config.yaw_rate_kd * float(imu.gyro[2])
            corrected.yaw += clip(yaw_correction, -self.config.max_yaw_correction, self.config.max_yaw_correction)

        corrected.yaw = clip(corrected.yaw, -1.0, 1.0)
        return corrected

    def feet(self, foot_positions: Mapping[str, np.ndarray], imu: IMUState) -> dict[str, np.ndarray]:
        if not self.config.enabled:
            return {leg: foot.copy() for leg, foot in foot_positions.items()}

        roll_term = -(self.config.roll_kp * imu.roll + self.config.roll_kd * float(imu.gyro[0]))
        pitch_term = -(self.config.pitch_kp * imu.pitch + self.config.pitch_kd * float(imu.gyro[1]))
        roll_term = clip(roll_term, -self.config.max_foot_z_correction, self.config.max_foot_z_correction)
        pitch_term = clip(pitch_term, -self.config.max_foot_z_correction, self.config.max_foot_z_correction)

        corrected: dict[str, np.ndarray] = {}
        for leg, foot in foot_positions.items():
            out = foot.copy()
            side_sign = 1.0 if leg in {"FL", "RL"} else -1.0
            fore_sign = 1.0 if leg in {"FL", "FR"} else -1.0
            out[2] += side_sign * roll_term + fore_sign * pitch_term
            corrected[leg] = out
        return corrected


def quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_arg = 2.0 * (w * y - z * x)
    pitch = math.asin(clip(pitch_arg, -1.0, 1.0))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clip(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
