"""Robot limb inspection definitions and result evaluation.

This module intentionally has no ROS or Qt dependency so the sequence and
acceptance rules can be unit tested on a development PC.
"""

import time
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


JOINT_NAMES: Tuple[str, ...] = (
    "waist_y_joint", "waist_x_joint", "waist_z_joint",
    "l_hip_y_joint", "l_hip_x_joint", "l_hip_z_joint",
    "l_knee_y_joint", "l_ankle_y_joint", "l_ankle_x_joint",
    "r_hip_y_joint", "r_hip_x_joint", "r_hip_z_joint",
    "r_knee_y_joint", "r_ankle_y_joint", "r_ankle_x_joint",
    "l_shoulder_y_joint", "l_shoulder_x_joint", "l_shoulder_z_joint",
    "l_elbow_y_joint", "l_wrist_x_joint", "l_wrist_y_joint",
    "l_wrist_z_joint", "r_shoulder_y_joint", "r_shoulder_x_joint",
    "r_shoulder_z_joint", "r_elbow_y_joint", "r_wrist_x_joint",
    "r_wrist_y_joint", "r_wrist_z_joint",
)

JOINT_LABELS: Dict[str, str] = {
    "l_hip_y_joint": "左髋俯仰", "l_hip_x_joint": "左髋侧摆",
    "l_hip_z_joint": "左髋旋转", "l_knee_y_joint": "左膝俯仰",
    "l_ankle_y_joint": "左踝俯仰", "l_ankle_x_joint": "左踝侧摆",
    "r_hip_y_joint": "右髋俯仰", "r_hip_x_joint": "右髋侧摆",
    "r_hip_z_joint": "右髋旋转", "r_knee_y_joint": "右膝俯仰",
    "r_ankle_y_joint": "右踝俯仰", "r_ankle_x_joint": "右踝侧摆",
    "l_shoulder_y_joint": "左肩俯仰", "l_shoulder_x_joint": "左肩侧摆",
    "l_shoulder_z_joint": "左肩旋转", "l_elbow_y_joint": "左肘俯仰",
    "l_wrist_x_joint": "左腕侧摆", "l_wrist_y_joint": "左腕俯仰",
    "l_wrist_z_joint": "左腕旋转", "r_shoulder_y_joint": "右肩俯仰",
    "r_shoulder_x_joint": "右肩侧摆", "r_shoulder_z_joint": "右肩旋转",
    "r_elbow_y_joint": "右肘俯仰", "r_wrist_x_joint": "右腕侧摆",
    "r_wrist_y_joint": "右腕俯仰", "r_wrist_z_joint": "右腕旋转",
}

LIMB_JOINTS = {
    # Test from the distal end toward the high-energy proximal joints.  A
    # wrist/ankle fault is less hazardous than discovering the same fault
    # after first commanding a shoulder or hip through a large workspace.
    ("arm", "left"): tuple(reversed(JOINT_NAMES[15:22])),
    ("arm", "right"): tuple(reversed(JOINT_NAMES[22:29])),
    ("leg", "left"): tuple(reversed(JOINT_NAMES[3:9])),
    ("leg", "right"): tuple(reversed(JOINT_NAMES[9:15])),
}

# Collision-free connected ranges around the all-zero inspection pose.  These
# are the conservative intersection of the simplified collision primitives
# and direct STL visual-mesh distance scans (MuJoCo 3.10).  They describe robot
# self-collision only; an external test fixture is not present in that model.
# safe_joint_range() applies additional mechanical and collision margins.
MODEL_COLLISION_FREE_RANGE_DEG = {
    "l_hip_y_joint": (-116.9, 155.8),
    "l_hip_x_joint": (-17.0, 142.0),
    "l_hip_z_joint": (-165.0, 165.0),
    "l_knee_y_joint": (-5.0, 150.0),
    "l_ankle_y_joint": (-48.2, 45.0),
    "l_ankle_x_joint": (-20.0, 20.0),
    "r_hip_y_joint": (-116.9, 155.8),
    "r_hip_x_joint": (-142.0, 17.0),
    "r_hip_z_joint": (-165.0, 165.0),
    "r_knee_y_joint": (-5.0, 150.0),
    "r_ankle_y_joint": (-48.2, 45.0),
    "r_ankle_x_joint": (-20.0, 20.0),
    "l_shoulder_y_joint": (-165.0, 165.0),
    "l_shoulder_x_joint": (-5.9, 175.0),
    "l_shoulder_z_joint": (-30.0, 165.0),
    "l_elbow_y_joint": (-55.0, 67.9),
    "l_wrist_x_joint": (-165.0, 165.0),
    "l_wrist_y_joint": (-75.0, 75.0),
    "l_wrist_z_joint": (-45.0, 45.0),
    "r_shoulder_y_joint": (-165.0, 165.0),
    "r_shoulder_x_joint": (-175.0, 5.9),
    "r_shoulder_z_joint": (-165.0, 30.0),
    "r_elbow_y_joint": (-55.0, 67.9),
    "r_wrist_x_joint": (-165.0, 165.0),
    "r_wrist_y_joint": (-75.0, 75.0),
    "r_wrist_z_joint": (-45.0, 45.0),
}

POSITION_MIN = np.array([
    -0.5236, -0.2618, -2.8798,
    -2.8798, -0.48869, -2.8798, -0.087266, -0.87266, -0.34907,
    -2.8798, -3.0543, -2.8798, -0.087266, -0.87266, -0.34907,
    -2.8798, -0.34907, -2.8798, -0.95993, -2.8798, -1.309, -0.7854,
    -2.8798, -3.0543, -2.8798, -0.95993, -2.8798, -1.309, -0.7854,
], dtype=float)

POSITION_MAX = np.array([
    0.5236, 0.2618, 2.8798,
    2.8798, 3.0543, 2.8798, 2.618, 0.7854, 0.34907,
    2.8798, 0.48869, 2.8798, 2.618, 0.7854, 0.34907,
    2.8798, 3.0543, 2.8798, 1.6581, 2.8798, 1.309, 0.7854,
    2.8798, 0.34907, 2.8798, 1.6581, 2.8798, 1.309, 0.7854,
], dtype=float)

JOINT_KP = np.array([
    108.448, 162.672, 176.421,
    176.421, 176.421, 54.224, 176.421, 33.493, 21.771,
    176.421, 176.421, 54.224, 176.421, 33.493, 21.771,
    54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747,
    54.224, 54.224, 16.747, 54.224, 16.747, 16.747, 16.747,
], dtype=float)

JOINT_KD = np.array([
    6.904, 10.356, 11.231,
    11.231, 11.231, 3.452, 11.231, 2.132, 1.386,
    11.231, 11.231, 3.452, 11.231, 2.132, 1.386,
    3.452, 3.452, 1.066, 3.452, 1.066, 1.066, 1.066,
    3.452, 3.452, 1.066, 3.452, 1.066, 1.066, 1.066,
], dtype=float)


@dataclass(frozen=True)
class InspectionSettings:
    limb: str = "arm"
    side: str = "left"
    move_sec: float = 2.0
    hold_sec: float = 0.5
    collision_margin_deg: float = 5.0
    mechanical_margin_deg: float = 2.0
    range_speed_deg_s: float = 10.0
    full_range_confirmed: bool = False
    tracking_tolerance_deg: float = 2.0
    minimum_motion_ratio: float = 0.6
    cross_axis_limit_deg: float = 3.0
    max_velocity_deg_s: float = 30.0
    max_effort_nm: float = 80.0

    def validate(self) -> None:
        if self.limb not in ("arm", "leg"):
            raise ValueError("limb must be arm or leg")
        if self.side not in ("left", "right", "both", "both_simultaneous"):
            raise ValueError(
                "side must be left, right, both or both_simultaneous")
        numeric_values = (
            self.move_sec, self.hold_sec,
            self.collision_margin_deg, self.mechanical_margin_deg,
            self.range_speed_deg_s, self.tracking_tolerance_deg,
            self.minimum_motion_ratio, self.cross_axis_limit_deg,
            self.max_velocity_deg_s, self.max_effort_nm,
        )
        if not all(np.isfinite(value) for value in numeric_values):
            raise ValueError("检测参数必须是有限数值")
        if self.move_sec <= 0.0:
            raise ValueError("单程时间必须大于 0s")
        if self.hold_sec < 0.0:
            raise ValueError("保持时间不能小于 0s")
        if self.collision_margin_deg < 0.0:
            raise ValueError("碰撞余量不能小于 0°")
        if self.mechanical_margin_deg < 0.0:
            raise ValueError("机械限位余量不能小于 0°")
        if self.range_speed_deg_s <= 0.0:
            raise ValueError("全行程速度必须大于 0°/s")
        if min(self.tracking_tolerance_deg, self.cross_axis_limit_deg,
               self.max_velocity_deg_s, self.max_effort_nm) <= 0.0:
            raise ValueError("判定阈值必须大于 0")
        if self.minimum_motion_ratio < 0.0:
            raise ValueError("最小响应比例不能小于 0")

@dataclass
class JointResult:
    joint_name: str
    label: str
    passed: bool
    positive_motion_deg: float
    negative_motion_deg: float
    max_tracking_error_deg: float
    max_cross_axis_deg: float
    max_velocity_deg_s: float
    max_effort_nm: float
    target_min_deg: float
    target_max_deg: float
    measured_min_deg: float
    measured_max_deg: float
    sample_count: int
    reason: str

    def to_dict(self):
        return asdict(self)


def selected_joints(limb: str, side: str) -> Tuple[str, ...]:
    sides: Iterable[str] = (
        ("left", "right")
        if side in ("both", "both_simultaneous") else (side,)
    )
    return tuple(name for current in sides for name in LIMB_JOINTS[(limb, current)])


def selected_joint_groups(limb: str, side: str) -> Tuple[Tuple[str, ...], ...]:
    """Return serial singletons or simultaneous left/right joint pairs."""
    if side == "both_simultaneous":
        return tuple(zip(
            LIMB_JOINTS[(limb, "left")],
            LIMB_JOINTS[(limb, "right")],
        ))
    return tuple((name,) for name in selected_joints(limb, side))


def selected_feedback_summary(
        joint_names: Sequence[str], seen: Sequence[bool],
        feedback_at: Sequence[float], timeout_sec: float,
        now: Optional[float] = None) -> Tuple[int, int, Optional[float]]:
    """Return fresh/total and the worst age for the selected joints only."""
    current = time.monotonic() if now is None else float(now)
    fresh = 0
    ages = []
    missing = False
    for name in joint_names:
        index = JOINT_NAMES.index(name)
        if not bool(seen[index]):
            missing = True
            continue
        age = max(0.0, current - float(feedback_at[index]))
        ages.append(age)
        if age <= timeout_sec:
            fresh += 1
    worst_age = None if missing else max(ages, default=None)
    return fresh, len(joint_names), worst_age


def minimum_jerk(progress: float) -> float:
    x = float(np.clip(progress, 0.0, 1.0))
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))


def safe_joint_range(joint_name: str, settings: InspectionSettings) -> Tuple[float, float]:
    """Return collision- and limit-margined absolute targets in radians."""
    index = JOINT_NAMES.index(joint_name)
    model_low_deg, model_high_deg = MODEL_COLLISION_FREE_RANGE_DEG[joint_name]
    mechanical_low = POSITION_MIN[index] + np.deg2rad(settings.mechanical_margin_deg)
    mechanical_high = POSITION_MAX[index] - np.deg2rad(settings.mechanical_margin_deg)
    collision_low = np.deg2rad(model_low_deg + settings.collision_margin_deg)
    collision_high = np.deg2rad(model_high_deg - settings.collision_margin_deg)
    low = max(float(mechanical_low), float(collision_low))
    high = min(float(mechanical_high), float(collision_high))
    if high <= low:
        raise ValueError("%s 的安全范围为空，请检查限位余量" % JOINT_LABELS[joint_name])
    return low, high


def evaluate_joint(joint_name: str, settings: InspectionSettings,
                   baseline: np.ndarray, positions: np.ndarray,
                   commands: np.ndarray, velocities: np.ndarray,
                   efforts: np.ndarray, active_indices: Sequence[int],
                   target_range: Tuple[float, float] = None,
                   exclude_cross_indices: Sequence[int] = ()) -> JointResult:
    index = JOINT_NAMES.index(joint_name)
    if target_range is None:
        target_low = float(np.min(commands[:, index]))
        target_high = float(np.max(commands[:, index]))
    else:
        target_low, target_high = target_range
    motion_deg = np.rad2deg(positions[:, index] - baseline[index])
    errors_deg = np.abs(np.rad2deg(positions[:, index] - commands[:, index]))
    positive = max(0.0, float(np.max(motion_deg)))
    negative = max(0.0, float(-np.min(motion_deg)))
    excluded = set(exclude_cross_indices)
    other = [i for i in active_indices if i != index and i not in excluded]
    cross = 0.0
    if other:
        cross = float(np.max(np.abs(np.rad2deg(
            positions[:, other] - baseline[np.newaxis, other]
        ))))
    max_velocity = float(np.max(np.abs(np.rad2deg(velocities[:, index]))))
    max_effort = float(np.max(np.abs(efforts[:, index])))
    failures: List[str] = []
    required_positive = max(
        0.0, float(np.rad2deg(target_high - baseline[index]))
    ) * settings.minimum_motion_ratio
    required_negative = max(
        0.0, float(np.rad2deg(baseline[index] - target_low))
    ) * settings.minimum_motion_ratio
    if positive < required_positive:
        failures.append("正向运动不足")
    if negative < required_negative:
        failures.append("负向运动不足")
    if float(np.max(errors_deg)) > settings.tracking_tolerance_deg:
        failures.append("跟踪误差超限")
    if cross > settings.cross_axis_limit_deg:
        failures.append("其他关节串扰超限")
    if max_velocity > settings.max_velocity_deg_s:
        failures.append("速度超限")
    if max_effort > settings.max_effort_nm:
        failures.append("力矩超限")
    return JointResult(
        joint_name=joint_name, label=JOINT_LABELS[joint_name],
        passed=not failures, positive_motion_deg=positive,
        negative_motion_deg=negative,
        max_tracking_error_deg=float(np.max(errors_deg)),
        max_cross_axis_deg=cross, max_velocity_deg_s=max_velocity,
        max_effort_nm=max_effort,
        target_min_deg=float(np.rad2deg(target_low)),
        target_max_deg=float(np.rad2deg(target_high)),
        measured_min_deg=float(np.min(np.rad2deg(positions[:, index]))),
        measured_max_deg=float(np.max(np.rad2deg(positions[:, index]))),
        sample_count=int(positions.shape[0]),
        reason="正常" if not failures else "、".join(failures),
    )
