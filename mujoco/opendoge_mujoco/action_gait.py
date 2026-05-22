from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import numpy as np


@dataclass
class BodyCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class GaitConfig:
    cycle_time: float
    duty_factor: float
    step_height: float
    step_x: float
    step_y: float
    step_yaw: float
    rear_stance_height_offset: float


@dataclass(frozen=True)
class FootTarget:
    position: np.ndarray
    velocity: np.ndarray


class TrotCycloidGait:
    """Trot gait planner with cycloid swing-foot trajectories."""

    def __init__(self, nominal_feet: Mapping[str, np.ndarray], config: GaitConfig) -> None:
        self.nominal_feet = {leg: foot.copy() for leg, foot in nominal_feet.items()}
        self.config = config
        if not 0.1 <= config.duty_factor <= 0.9:
            raise ValueError(f"duty_factor must be in [0.1, 0.9], got {config.duty_factor}")
        if config.cycle_time <= 0.0:
            raise ValueError(f"cycle_time must be positive, got {config.cycle_time}")

    def targets(self, t: float, command: BodyCommand) -> dict[str, FootTarget]:
        if max(abs(command.vx), abs(command.vy), abs(command.yaw)) < 1.0e-6:
            zero = np.zeros(3, dtype=np.float64)
            return {leg: FootTarget(position=foot.copy(), velocity=zero.copy()) for leg, foot in self.nominal_feet.items()}

        targets: dict[str, FootTarget] = {}

        for leg, nominal in self.nominal_feet.items():
            phase = self._leg_phase(t, leg)
            stride = self._stride_vector(leg, command)
            swing_ratio = 1.0 - self.config.duty_factor
            if phase < swing_ratio:
                swing_u = phase / swing_ratio
                position, velocity = self._swing(nominal, stride, swing_u)
            else:
                stance_u = (phase - swing_ratio) / self.config.duty_factor
                position, velocity = self._stance(nominal, stride, stance_u)
                if leg in {"RL", "RR"}:
                    position[2] += self.config.rear_stance_height_offset
            targets[leg] = FootTarget(position=position, velocity=velocity)

        return targets

    def _leg_phase(self, t: float, leg: str) -> float:
        base_phase = (self._stance_mid_phase() + t / self.config.cycle_time) % 1.0
        trot_offset = 0.0 if leg in {"FL", "RR"} else 0.5
        return (base_phase + trot_offset) % 1.0

    def _stance_mid_phase(self) -> float:
        swing_ratio = 1.0 - self.config.duty_factor
        return swing_ratio + 0.5 * self.config.duty_factor

    def _stride_vector(self, leg: str, command: BodyCommand) -> np.ndarray:
        side = 1.0 if leg in {"FL", "RL"} else -1.0
        return np.array(
            [
                self.config.step_x * self._clip_unit(command.vx) - self.config.step_yaw * self._clip_unit(command.yaw) * side,
                self.config.step_y * self._clip_unit(command.vy),
                0.0,
            ],
            dtype=np.float64,
        )

    def _stance(self, nominal: np.ndarray, stride: np.ndarray, u: float) -> tuple[np.ndarray, np.ndarray]:
        du_dt = 1.0 / (self.config.duty_factor * self.config.cycle_time)
        # Support foot starts ahead of nominal, pulls backward through nominal, then reaches swing start.
        position = nominal + (0.5 - u) * stride
        velocity = -stride * du_dt
        return position, velocity

    def _swing(self, nominal: np.ndarray, stride: np.ndarray, u: float) -> tuple[np.ndarray, np.ndarray]:
        du_dt = 1.0 / ((1.0 - self.config.duty_factor) * self.config.cycle_time)
        s = self._cycloid_progress(u)
        ds_dt = self._cycloid_progress_derivative(u) * du_dt
        lift = self.config.step_height * math.sin(math.pi * u)
        dlift_dt = self.config.step_height * math.pi * math.cos(math.pi * u) * du_dt

        # Swing foot returns from the rear point to the next support start.
        position = nominal + (-0.5 + s) * stride
        position[2] += lift
        velocity = stride * ds_dt
        velocity[2] += dlift_dt
        return position, velocity

    @staticmethod
    def _cycloid_progress(u: float) -> float:
        return u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)

    @staticmethod
    def _cycloid_progress_derivative(u: float) -> float:
        return 1.0 - math.cos(2.0 * math.pi * u)

    @staticmethod
    def _clip_unit(value: float) -> float:
        return max(-1.0, min(1.0, value))


ActionFootPlanner = TrotCycloidGait
