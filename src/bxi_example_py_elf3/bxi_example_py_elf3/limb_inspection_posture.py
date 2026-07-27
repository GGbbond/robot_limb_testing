"""Compact support poses and mirrored bilateral trajectory planning."""

import numpy as np


# Values were scanned against simplified collision geoms and STL shells over
# every configured single-side range and the bilateral mirrored trajectories.
# A 5 degree reserve is kept where the scan found a nearby collision boundary.
COMPACT_POSTURE_DEG = {
    "shoulder_y_joint": ("elbow_y_joint", -45.0),
    "shoulder_x_joint": ("elbow_y_joint", -45.0),
    "shoulder_z_joint": ("elbow_y_joint", -15.0),
    "hip_y_joint": ("knee_y_joint", 30.0),
    "hip_x_joint": ("knee_y_joint", 135.0),
    "hip_z_joint": ("knee_y_joint", 5.0),
}

# Raw bilateral collision boundaries before applying the configured angular
# collision margin.  Only the lower mirrored coordinate is collision-limited.
MIRRORED_PAIR_LOWER_BOUND_DEG = {
    "hip_x_joint": -8.0,
    "hip_z_joint": -60.5,
}


def joint_suffix(joint_name):
    return joint_name[2:] if joint_name[:2] in ("l_", "r_") else joint_name


def uses_opposite_bilateral_sign(joint_name):
    suffix = joint_suffix(joint_name)
    return "_x_joint" in suffix or "_z_joint" in suffix


def compact_posture(active_joint_names):
    """Return absolute companion-joint targets for an active group."""
    result = {}
    for name in active_joint_names:
        configuration = COMPACT_POSTURE_DEG.get(joint_suffix(name))
        if configuration is None:
            continue
        companion_suffix, angle_deg = configuration
        result[name[:2] + companion_suffix] = float(np.deg2rad(angle_deg))
    return result


def mirrored_target_ranges(groups, target_ranges, collision_margin_deg):
    """Make paired targets visually mirrored and shrink mutual-collision edges."""
    result = dict(target_ranges)
    for group in groups:
        if len(group) != 2:
            continue
        left, right = group
        left_low, left_high = result[left]
        right_low, right_high = result[right]
        if uses_opposite_bilateral_sign(left):
            low = max(left_low, -right_high)
            high = min(left_high, -right_low)
            boundary = MIRRORED_PAIR_LOWER_BOUND_DEG.get(joint_suffix(left))
            if boundary is not None:
                low = max(low, np.deg2rad(
                    boundary + float(collision_margin_deg)))
            if high <= low:
                raise ValueError("%s 的双侧镜像安全范围为空" % joint_suffix(left))
            result[left] = (float(low), float(high))
            result[right] = (float(-high), float(-low))
        else:
            low = max(left_low, right_low)
            high = min(left_high, right_high)
            if high <= low:
                raise ValueError("%s 的双侧同步安全范围为空" % joint_suffix(left))
            result[left] = result[right] = (float(low), float(high))
    return result


def safe_range_waypoints(active_joint_names, target_ranges):
    """Return low/high/center waypoints with mirrored bilateral directions."""
    names = tuple(active_joint_names)
    if len(names) == 2 and uses_opposite_bilateral_sign(names[0]):
        left, right = names
        return [
            {left: target_ranges[left][0], right: target_ranges[right][1]},
            {left: target_ranges[left][1], right: target_ranges[right][0]},
        ]
    return [
        {name: target_ranges[name][0] for name in names},
        {name: target_ranges[name][1] for name in names},
    ]
