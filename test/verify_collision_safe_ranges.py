"""Verify serial, mirrored and compact-posture trajectories in MuJoCo."""

from pathlib import Path

import numpy as np

from bxi_example_py_elf3.limb_collision_guard import CollisionGuard
from bxi_example_py_elf3.limb_inspection_core import (
    JOINT_NAMES, InspectionSettings, safe_joint_range,
    selected_joint_groups, selected_joints,
)
from bxi_example_py_elf3.limb_inspection_posture import (
    compact_posture, mirrored_target_ranges, safe_range_waypoints,
)


SAMPLES_PER_SEGMENT = 401


def verify_command(guard, command, checked_names, label):
    collisions = guard.collisions(command)
    if collisions:
        raise RuntimeError("%s 检测到碰撞：%s" % (label, collisions[:3]))
    for name in checked_names:
        visual_collisions = guard.visual_mesh_collisions(command, name)
        if visual_collisions:
            raise RuntimeError(
                "%s 检测到视觉网格侵入（%s）：%s" %
                (label, name, visual_collisions[:3]))


def planned_waypoints(group, ranges):
    posture = compact_posture(group)
    motion_names = tuple(dict.fromkeys(tuple(group) + tuple(posture)))
    center = {name: 0.0 for name in motion_names}
    folded = dict(center)
    folded.update(posture)
    waypoints = [dict(center)]
    if posture:
        waypoints.append(dict(folded))
    active_waypoints = safe_range_waypoints(group, ranges)
    active_waypoints.append({name: 0.0 for name in group})
    for active in active_waypoints:
        target = dict(folded)
        target.update(active)
        waypoints.append(target)
    if posture:
        waypoints.append(dict(center))
    return motion_names, waypoints


def verify_trajectory(guard, group, ranges, label):
    checked_names, waypoints = planned_waypoints(group, ranges)
    command = np.zeros(len(JOINT_NAMES))
    for segment, (start, target) in enumerate(zip(
            waypoints[:-1], waypoints[1:])):
        for blend in np.linspace(0.0, 1.0, SAMPLES_PER_SEGMENT):
            command[:] = 0.0
            for name in checked_names:
                command[JOINT_NAMES.index(name)] = (
                    start[name] + blend * (target[name] - start[name]))
            verify_command(
                guard, command, checked_names,
                "%s segment=%d blend=%.3f" % (label, segment, blend))


def main():
    root = Path(__file__).resolve().parents[1]
    guard = CollisionGuard(
        root / "src/bxi_example_py_elf3/data/elf3.xml")
    guard.set_base_height(1.7)
    settings = InspectionSettings(
        collision_margin_deg=5.0,
        mechanical_margin_deg=2.0)

    joints = selected_joints("arm", "both") + selected_joints("leg", "both")
    command = np.zeros(len(JOINT_NAMES))
    for name in joints:
        low, high = safe_joint_range(name, settings)
        for value in np.linspace(low, high, SAMPLES_PER_SEGMENT):
            command[:] = 0.0
            command[JOINT_NAMES.index(name)] = value
            verify_command(
                guard, command, (name,),
                "%s angle=%.2f°" % (name, np.rad2deg(value)))

    compact_serial_groups = tuple(
        group
        for limb in ("arm", "leg")
        for side in ("left", "right")
        for group in selected_joint_groups(limb, side)
        if compact_posture(group)
    )
    for group in compact_serial_groups:
        ranges = {name: safe_joint_range(name, settings) for name in group}
        verify_trajectory(
            guard, group, ranges, "serial compact " + " + ".join(group))

    pair_groups = (
        selected_joint_groups("arm", "both_simultaneous") +
        selected_joint_groups("leg", "both_simultaneous"))
    pair_ranges = {
        name: safe_joint_range(name, settings)
        for group in pair_groups for name in group
    }
    pair_ranges = mirrored_target_ranges(
        pair_groups, pair_ranges, settings.collision_margin_deg)
    for group in pair_groups:
        verify_trajectory(
            guard, group, pair_ranges,
            "bilateral mirrored compact " + " + ".join(group))

    print(
        "collision-safe trajectories: PASS "
        "(%d joints, %d compact serial groups, %d mirrored pairs)" %
        (len(joints), len(compact_serial_groups), len(pair_groups)))


if __name__ == "__main__":
    main()
