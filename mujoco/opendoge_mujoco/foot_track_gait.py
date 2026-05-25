"""
C-style foot trajectory planner for OpenDoge.

Ports the foot_track() state machine from motor_control.c to produce
3D hip-frame foot targets compatible with OpenDogeLegIK.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import numpy as np

from opendoge_mujoco.action_gait import BodyCommand, FootTarget

PI = math.pi


_DirMode = {"FL": 1, "FR": 2, "RL": 3, "RR": 4}


@dataclass
class FootTrackParams:
    """Gait parameters matching motor_control.c defaults, scaled to meters.

    The defaults are tuned for the MuJoCo OpenDoge model (~0.2m leg).
    """

    leg_high: float = 0.205
    leg_lift_height: float = 0.050
    start_point: float = 0.042
    start_point_turn: float = 0.040
    step_rate: float = 0.130
    fast_step_rate: float = 0.090
    slow_step_rate: float = 0.170
    medium_step_rate: float = 0.120


class FootTrackGait:
    """C-style foot trajectory planner producing 3D hip-frame targets."""

    def __init__(
        self,
        nominal_feet: Mapping[str, np.ndarray],
        params: FootTrackParams | None = None,
    ) -> None:
        self.nominal_feet: dict[str, np.ndarray] = {
            leg: foot.copy() for leg, foot in nominal_feet.items()
        }
        self.leg_names: list[str] = sorted(self.nominal_feet.keys())
        self.p = params or FootTrackParams()

        # Auto-scale height/stride to the MuJoCo model if using default C-code
        # values (which assume a ~417 mm leg).
        if params is None:
            avg_z = float(np.mean([abs(self.nominal_feet[leg][2]) for leg in self.leg_names]))
            self.p.leg_high = avg_z
            self.p.leg_lift_height = avg_z * 0.24
            self.p.start_point = avg_z * 0.30
            self.p.start_point_turn = avg_z * 0.28

        # Timing state
        self._cycle_time: float = self.p.step_rate * 2.0
        self._phase_duration: float = self.p.step_rate
        self._gait_start_time: float = 0.0
        self._gait_active: bool = False

    # ------------------------------------------------------------------
    # Phase helpers
    # ------------------------------------------------------------------

    def _leg_phase(self, t: float, leg: str) -> float:
        """Phase in [0, 1) within the full trot cycle (FL+RR opposite FR+RL)."""
        base = (t / self._cycle_time) % 1.0
        offset = 0.0 if leg in {"FL", "RR"} else 0.5
        return (base + offset) % 1.0

    def _in_stance(self, t: float, leg: str) -> bool:
        """True during the stance half of the cycle."""
        return self._leg_phase(t, leg) >= 0.5

    def _phase_u(self, t: float, leg: str) -> float:
        """Normalised time [0, 1) inside the current half-phase (swing or stance)."""
        phase = self._leg_phase(t, leg)
        half = phase if phase < 0.5 else phase - 0.5
        return min(half / 0.5, 1.0 - 1e-12)

    def _phase_dt(self) -> float:
        """Real-time duration of one half-phase in seconds."""
        return self._phase_duration

    # ------------------------------------------------------------------
    # C-code trajectory primitives
    # ------------------------------------------------------------------

    @staticmethod
    def _cycloid_progress(u: float) -> float:
        return u - math.sin(2.0 * PI * u) / (2.0 * PI)

    @staticmethod
    def _cycloid_progress_derivative(u: float) -> float:
        return 1.0 - math.cos(2.0 * PI * u)

    def _get_cycloid(self, start: float, end: float, u: float) -> float:
        """C-code Get_Cycloid: smooth interpolation from start to end."""
        length = start - end
        return start - length * self._cycloid_progress(u)

    def _get_height_sine(self, base: float, lift: float, u: float) -> float:
        """C-code Get_Height_Sine: sinusoidal foot lift."""
        return base - lift * (0.5 - 0.5 * math.cos(2.0 * PI * u))

    # ------------------------------------------------------------------
    # Locomotion  (C: Handle_Locomotion)
    # ------------------------------------------------------------------

    def _compute_locomotion(
        self, t: float, leg: str, command: BodyCommand
    ) -> tuple[float, float, float, float]:
        """Return (x, z, vx, vz) in hip frame for locomotion.

        The trajectory is centred on the nominal x position so that forward
        and backward motion share the same workspace envelope.
        """
        p = self.p
        nominal_x = self.nominal_feet[leg][0]
        in_stance = self._in_stance(t, leg)
        u = self._phase_u(t, leg)
        dt = self._phase_dt()
        du_dt = 1.0 / dt

        is_fwd = command.vx > 0.05
        is_bwd = command.vx < -0.05

        if not is_fwd and not is_bwd:
            return nominal_x, -p.leg_high, 0.0, 0.0

        start = p.start_point if is_fwd else -p.start_point
        end = -p.start_point if is_fwd else p.start_point

        if in_stance:
            x = nominal_x + start + (end - start) * u
            z = -p.leg_high
            vx = (end - start) * du_dt
            vz = 0.0
        else:
            x = nominal_x + self._get_cycloid(end, start, u)
            z = -self._get_height_sine(p.leg_high, p.leg_lift_height, u)
            vx = (end - start) * self._cycloid_progress_derivative(u) * du_dt
            dz_c = p.leg_lift_height * PI * math.sin(2.0 * PI * u) * du_dt
            vz = -dz_c

        # Mixed turning (C: ch2 mapping via command.vy)
        if abs(command.vy) > 0.05:
            mix = 0.5 * command.vy
            dm = _DirMode[leg]
            # Scale x relative to nominal for mixed turning
            x_rel = x - nominal_x
            if command.vy > 0:  # right turn
                x_rel *= 1.0 + mix if dm in (1, 4) else 1.0 - mix
            else:  # left turn
                x_rel *= 1.0 + abs(mix) if dm in (2, 3) else 1.0 - abs(mix)
            x = nominal_x + x_rel
            vx *= 1.0 + abs(mix) if dm in (2, 3) else 1.0 - abs(mix)

        return x, z, vx, vz

    # ------------------------------------------------------------------
    # Spot turn  (C: Handle_SpotTurn)
    # ------------------------------------------------------------------

    def _compute_spotturn(
        self, t: float, leg: str, command: BodyCommand
    ) -> tuple[float, float, float, float]:
        """Return (x, z, vx, vz) in hip frame for spot turn.

        Uses the same trot phase as locomotion (FL+RR opposite FR+RL).
        The diagonal pair currently in stance acts as 'drivers' (traction),
        the other pair swings forward.
        Trajectory is centred on the nominal x position per leg.
        """
        p = self.p
        nominal_x = self.nominal_feet[leg][0]
        in_stance = self._in_stance(t, leg)
        u = self._phase_u(t, leg)
        dt = self._phase_dt()
        du_dt = 1.0 / dt

        is_right = command.yaw > 0.05
        dm = _DirMode[leg]
        sp = p.start_point_turn

        is_driver = in_stance
        sign = -1.0 if dm in (2, 4) else 1.0  # FR, RR have sign=-1

        # MuJoCo hip-frame requires flipped push direction vs C code
        turn_dir = -1.0 if is_right else 1.0

        if is_driver:
            if dm == 2:  # FR
                x = nominal_x + turn_dir * (-sp + 2.0 * sp * u)
                vx = turn_dir * 2.0 * sp * du_dt
            else:
                x = nominal_x + turn_dir * sign * sp * (1.0 - 2.0 * u)
                vx = turn_dir * sign * sp * (-2.0) * du_dt
            z = -p.leg_high * 1.05
            vz = 0.0
        else:
            if dm == 3 and not is_right and in_stance:
                sign = -1.0
            x = nominal_x - turn_dir * sign * sp + 2.0 * turn_dir * sign * sp * self._cycloid_progress(u)
            z = -self._get_height_sine(p.leg_high, p.leg_lift_height, u)
            vx = 2.0 * turn_dir * sign * sp * self._cycloid_progress_derivative(u) * du_dt
            dz_c = p.leg_lift_height * PI * math.sin(2.0 * PI * u) * du_dt
            vz = -dz_c

        return x, z, vx, vz

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def targets(self, t: float, command: BodyCommand) -> dict[str, FootTarget]:
        """Return foot targets (hip-frame 3D position + velocity) for all legs."""
        command_active = (
            abs(command.vx) > 0.05
            or abs(command.vy) > 0.05
            or abs(command.yaw) > 0.05
        )

        # State machine: idle / spot-turn / locomotion
        is_spotturn = abs(command.yaw) > 0.05 and abs(command.vx) < 0.05
        is_locomotion = command_active and not is_spotturn

        # Gate the gait timer
        if command_active and not self._gait_active:
            self._gait_start_time = t
            self._gait_active = True
        elif not command_active:
            self._gait_active = False

        gait_time = t - self._gait_start_time if self._gait_active else 0.0

        targets: dict[str, FootTarget] = {}
        for leg in self.leg_names:
            nominal = self.nominal_feet[leg]

            if is_spotturn:
                x, z, vx, vz = self._compute_spotturn(gait_time, leg, command)
            elif is_locomotion:
                x, z, vx, vz = self._compute_locomotion(gait_time, leg, command)
            else:
                # Idle — hold nominal
                targets[leg] = FootTarget(
                    position=nominal.copy(),
                    velocity=np.zeros(3, dtype=np.float64),
                )
                continue

            pos = np.array([x, nominal[1], z], dtype=np.float64)
            vel = np.array([vx, 0.0, vz], dtype=np.float64)
            targets[leg] = FootTarget(position=pos, velocity=vel)

        return targets
