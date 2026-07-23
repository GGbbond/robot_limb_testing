import numpy as np
import pytest

from bxi_example_py_elf3.limb_inspection_core import (
    JOINT_NAMES, InspectionSettings, evaluate_joint, minimum_jerk,
    selected_joint_groups, selected_joints, validate_center,
)
from bxi_example_py_elf3.limb_inspection_posture import (
    compact_posture, mirrored_target_ranges, safe_range_waypoints,
    small_motion_waypoints,
)


def test_selection_is_bilateral_but_serial_ordered():
    arms = selected_joints("arm", "both")
    assert len(arms) == 14
    assert arms[0].startswith("l_")
    assert arms[7].startswith("r_")
    assert len(selected_joints("leg", "left")) == 6
    simultaneous = selected_joint_groups("leg", "both_simultaneous")
    assert len(simultaneous) == 6
    assert simultaneous[0] == ("l_ankle_x_joint", "r_ankle_x_joint")
    assert simultaneous[-1] == ("l_hip_y_joint", "r_hip_y_joint")


def test_minimum_jerk_endpoints():
    assert minimum_jerk(-1.0) == 0.0
    assert minimum_jerk(0.0) == 0.0
    assert minimum_jerk(1.0) == 1.0
    assert minimum_jerk(2.0) == 1.0
    assert 0.49 < minimum_jerk(0.5) < 0.51


def test_center_near_limit_is_rejected():
    center = np.zeros(len(JOINT_NAMES))
    center[JOINT_NAMES.index("l_ankle_x_joint")] = 0.34
    with pytest.raises(ValueError):
        validate_center(("l_ankle_x_joint",), center, np.deg2rad(5.0))


def test_evaluation_pass_and_fail():
    settings = InspectionSettings(amplitude_deg=5.0, max_velocity_deg_s=100.0)
    count = 40
    baseline = np.zeros(len(JOINT_NAMES))
    index = JOINT_NAMES.index("l_elbow_y_joint")
    command = np.zeros((count, len(JOINT_NAMES)))
    motion = np.deg2rad(np.r_[np.linspace(0, 5, 10), np.linspace(5, 0, 10),
                              np.linspace(0, -5, 10), np.linspace(-5, 0, 10)])
    command[:, index] = motion
    position = command.copy()
    velocity = np.zeros_like(position)
    effort = np.zeros_like(position)
    result = evaluate_joint("l_elbow_y_joint", settings, baseline, position,
                            command, velocity, effort, (index,))
    assert result.passed
    position[:, index] *= 0.1
    result = evaluate_joint("l_elbow_y_joint", settings, baseline, position,
                            command, velocity, effort, (index,))
    assert not result.passed
    assert "运动不足" in result.reason


def test_simulation_debug_accepts_effectively_unbounded_parameters():
    settings = InspectionSettings(
        amplitude_deg=1.0e8, move_sec=1.0e8, hold_sec=1.0e8,
        cycles=1_000_000, collision_margin_deg=1.0e8,
        mechanical_margin_deg=1.0e8, range_speed_deg_s=1.0e8,
        tracking_tolerance_deg=1.0e8, minimum_motion_ratio=1.0e8,
        cross_axis_limit_deg=1.0e8, max_velocity_deg_s=1.0e8,
        max_effort_nm=1.0e8,
    )
    settings.validate(simulation_debug=True)
    with pytest.raises(ValueError):
        settings.validate()


def test_simultaneous_peer_is_not_counted_as_cross_axis_motion():
    settings = InspectionSettings(
        amplitude_deg=5.0, max_velocity_deg_s=100.0,
        cross_axis_limit_deg=1.0)
    count = 40
    baseline = np.zeros(len(JOINT_NAMES))
    left = JOINT_NAMES.index("l_ankle_y_joint")
    right = JOINT_NAMES.index("r_ankle_y_joint")
    unrelated = JOINT_NAMES.index("l_knee_y_joint")
    command = np.zeros((count, len(JOINT_NAMES)))
    motion = np.deg2rad(np.r_[
        np.linspace(0, 5, 10), np.linspace(5, 0, 10),
        np.linspace(0, -5, 10), np.linspace(-5, 0, 10)])
    command[:, left] = motion
    command[:, right] = motion
    position = command.copy()
    velocity = np.zeros_like(position)
    effort = np.zeros_like(position)
    result = evaluate_joint(
        "l_ankle_y_joint", settings, baseline, position, command,
        velocity, effort, (left, right, unrelated),
        exclude_cross_indices=(left, right))
    assert result.passed
    assert result.max_cross_axis_deg == 0.0


def test_compact_postures_and_mirrored_waypoints():
    posture = compact_posture(("l_hip_x_joint", "r_hip_x_joint"))
    assert np.isclose(np.rad2deg(posture["l_knee_y_joint"]), 135.0)
    assert np.isclose(np.rad2deg(posture["r_knee_y_joint"]), 135.0)
    ranges = {
        "l_hip_x_joint": (np.deg2rad(-12.0), np.deg2rad(137.0)),
        "r_hip_x_joint": (np.deg2rad(-137.0), np.deg2rad(12.0)),
    }
    mirrored = mirrored_target_ranges(
        (("l_hip_x_joint", "r_hip_x_joint"),), ranges, 5.0)
    assert np.isclose(np.rad2deg(mirrored["l_hip_x_joint"][0]), -3.0)
    waypoints = safe_range_waypoints(
        ("l_hip_x_joint", "r_hip_x_joint"), mirrored)
    assert np.isclose(waypoints[0]["l_hip_x_joint"],
                      -waypoints[0]["r_hip_x_joint"])
    small = small_motion_waypoints(
        ("l_ankle_x_joint", "r_ankle_x_joint"),
        {"l_ankle_x_joint": 0.0, "r_ankle_x_joint": 0.0},
        np.deg2rad(5.0), 1)
    assert np.isclose(small[0]["l_ankle_x_joint"],
                      -small[0]["r_ankle_x_joint"])
    compact_small_ranges = mirrored_target_ranges(
        (("l_hip_x_joint", "r_hip_x_joint"),), {
            "l_hip_x_joint": (np.deg2rad(-5.0), np.deg2rad(5.0)),
            "r_hip_x_joint": (np.deg2rad(-5.0), np.deg2rad(5.0)),
        }, 5.0)
    compact_small = small_motion_waypoints(
        ("l_hip_x_joint", "r_hip_x_joint"),
        {"l_hip_x_joint": 0.0, "r_hip_x_joint": 0.0},
        np.deg2rad(5.0), 1, compact_small_ranges)
    assert np.isclose(np.rad2deg(compact_small[2]["l_hip_x_joint"]), -3.0)
