from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float


@dataclass(frozen=True)
class LegGeometry:
    leg: str
    thigh_offset_y: float
    thigh_length: float
    calf_length: float
    limits: tuple[JointLimit, JointLimit, JointLimit]


def _parse_xyz(value: str) -> np.ndarray:
    return np.array([float(part) for part in value.split()], dtype=np.float64)


def _joint_origin(joint: ET.Element) -> np.ndarray:
    origin = joint.find("origin")
    if origin is None:
        return np.zeros(3, dtype=np.float64)
    return _parse_xyz(origin.attrib.get("xyz", "0 0 0"))


def _joint_limit(joint: ET.Element) -> JointLimit:
    limit = joint.find("limit")
    if limit is None:
        return JointLimit(lower=-math.inf, upper=math.inf)
    return JointLimit(lower=float(limit.attrib["lower"]), upper=float(limit.attrib["upper"]))


def _rot_x(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )


def load_leg_geometries_from_urdf(urdf_path: Path, legs: Sequence[str]) -> dict[str, LegGeometry]:
    root = ET.parse(urdf_path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    geometries: dict[str, LegGeometry] = {}

    for leg in legs:
        hip_joint = joints[f"{leg}_hip_joint"]
        thigh_joint = joints[f"{leg}_thigh_joint"]
        calf_joint = joints[f"{leg}_calf_joint"]
        foot_joint = joints[f"{leg}_foot_joint"]

        thigh_origin = _joint_origin(thigh_joint)
        calf_origin = _joint_origin(calf_joint)
        foot_origin = _joint_origin(foot_joint)

        geometries[leg] = LegGeometry(
            leg=leg,
            thigh_offset_y=float(thigh_origin[1]),
            thigh_length=float(abs(calf_origin[2])),
            calf_length=float(np.linalg.norm(foot_origin)),
            limits=(
                _joint_limit(hip_joint),
                _joint_limit(thigh_joint),
                _joint_limit(calf_joint),
            ),
        )

    return geometries


class OpenDogeLegIK:
    """Analytic 3-DoF leg IK using OpenDoge URDF joint origins."""

    def __init__(self, geometries: Mapping[str, LegGeometry], joint_order: Sequence[str]) -> None:
        self.geometries = dict(geometries)
        self.joint_order = list(joint_order)

    def forward_leg(self, leg: str, q: Sequence[float]) -> np.ndarray:
        geom = self.geometries[leg]
        q_hip, q_thigh, q_calf = (float(q[0]), float(q[1]), float(q[2]))
        l1 = geom.thigh_length
        l2 = geom.calf_length

        foot_in_thigh = np.array(
            [
                -l1 * math.sin(q_thigh) - l2 * math.sin(q_thigh + q_calf),
                geom.thigh_offset_y,
                -l1 * math.cos(q_thigh) - l2 * math.cos(q_thigh + q_calf),
            ],
            dtype=np.float64,
        )
        return _rot_x(q_hip) @ foot_in_thigh

    def nominal_feet(self, joint_angles: Mapping[str, float]) -> dict[str, np.ndarray]:
        feet: dict[str, np.ndarray] = {}
        for leg in self.geometries:
            q = [
                joint_angles[f"{leg}_hip_joint"],
                joint_angles[f"{leg}_thigh_joint"],
                joint_angles[f"{leg}_calf_joint"],
            ]
            feet[leg] = self.forward_leg(leg, q)
        return feet

    def inverse_leg(self, leg: str, foot_pos_hip: np.ndarray, seed: Sequence[float]) -> np.ndarray:
        geom = self.geometries[leg]
        p = np.asarray(foot_pos_hip, dtype=np.float64)
        seed_q = np.asarray(seed, dtype=np.float64)
        l1 = geom.thigh_length
        l2 = geom.calf_length
        target_y = geom.thigh_offset_y

        radius_yz = math.hypot(float(p[1]), float(p[2]))
        if radius_yz < abs(target_y) + 1.0e-8:
            radius_yz = abs(target_y) + 1.0e-8
        hip_phase = math.atan2(float(p[2]), float(p[1]))
        hip_delta = math.acos(max(-1.0, min(1.0, target_y / radius_yz)))
        hip_candidates = (hip_phase + hip_delta, hip_phase - hip_delta)
        q_hip = min(hip_candidates, key=lambda q: abs(self._wrap_to_pi(q - seed_q[0])))

        p_thigh = _rot_x(-q_hip) @ p
        x_planar = -float(p_thigh[0])
        z_planar = -float(p_thigh[2])
        distance = math.hypot(x_planar, z_planar)
        min_reach = abs(l1 - l2) + 1.0e-6
        max_reach = l1 + l2 - 1.0e-6
        distance = max(min_reach, min(max_reach, distance))

        cos_calf = (distance * distance - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        cos_calf = max(-1.0, min(1.0, cos_calf))
        calf_candidates = (math.acos(cos_calf), -math.acos(cos_calf))

        planar_angle = math.atan2(x_planar, z_planar)
        candidates = []
        for q_calf in calf_candidates:
            q_thigh = planar_angle - math.atan2(l2 * math.sin(q_calf), l1 + l2 * math.cos(q_calf))
            q = np.array([q_hip, q_thigh, q_calf], dtype=np.float64)
            candidates.append(self._clamp_leg(geom, q))

        return min(candidates, key=lambda q: float(np.linalg.norm(q - seed_q)))

    def inverse_feet(
        self,
        foot_targets: Mapping[str, np.ndarray],
        seed_by_joint: Mapping[str, float],
    ) -> np.ndarray:
        q_des: list[float] = []
        for joint_name in self.joint_order:
            leg = joint_name.split("_", 1)[0]
            if joint_name.endswith("_hip_joint"):
                seed = [
                    seed_by_joint[f"{leg}_hip_joint"],
                    seed_by_joint[f"{leg}_thigh_joint"],
                    seed_by_joint[f"{leg}_calf_joint"],
                ]
                q_leg = self.inverse_leg(leg, foot_targets[leg], seed)
                q_des.extend(q_leg.tolist())
        return np.array(q_des, dtype=np.float64)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _clamp_leg(geom: LegGeometry, q: np.ndarray) -> np.ndarray:
        out = q.copy()
        for i, limit in enumerate(geom.limits):
            out[i] = min(limit.upper, max(limit.lower, out[i]))
        return out

