import numpy as np
import pytest

from bxi_example_py_elf3.limb_inspection_core import (
    JOINT_NAMES, InspectionSettings, evaluate_joint, minimum_jerk,
    selected_feedback_summary, selected_joint_groups, selected_joints,
)
from bxi_example_py_elf3.limb_inspection_posture import (
    compact_posture, mirrored_target_ranges, safe_range_waypoints,
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


def test_feedback_summary_uses_selected_fresh_joints_and_worst_age():
    seen = np.zeros(len(JOINT_NAMES), dtype=bool)
    feedback_at = np.zeros(len(JOINT_NAMES))
    selected = ("l_hip_y_joint", "l_hip_x_joint")
    first = JOINT_NAMES.index(selected[0])
    second = JOINT_NAMES.index(selected[1])
    unrelated = JOINT_NAMES.index("r_hip_y_joint")
    seen[[first, second, unrelated]] = True
    feedback_at[first] = 99.95
    feedback_at[second] = 99.70
    feedback_at[unrelated] = 100.0

    fresh, total, worst_age = selected_feedback_summary(
        selected, seen, feedback_at, timeout_sec=0.2, now=100.0)

    assert (fresh, total) == (1, 2)
    assert np.isclose(worst_age, 0.3)


def test_feedback_summary_marks_never_seen_selected_joint_missing():
    seen = np.zeros(len(JOINT_NAMES), dtype=bool)
    feedback_at = np.zeros(len(JOINT_NAMES))
    selected = ("l_hip_y_joint",)

    fresh, total, worst_age = selected_feedback_summary(
        selected, seen, feedback_at, timeout_sec=0.2, now=100.0)

    assert (fresh, total, worst_age) == (0, 1, None)


def test_minimum_jerk_endpoints():
    assert minimum_jerk(-1.0) == 0.0
    assert minimum_jerk(0.0) == 0.0
    assert minimum_jerk(1.0) == 1.0
    assert minimum_jerk(2.0) == 1.0
    assert 0.49 < minimum_jerk(0.5) < 0.51


def test_evaluation_pass_and_fail():
    settings = InspectionSettings(max_velocity_deg_s=100.0)
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


def test_simulation_and_hardware_accept_large_finite_parameters_equally():
    settings = InspectionSettings(
        move_sec=1.0e8, hold_sec=1.0e8, collision_margin_deg=1.0e8,
        mechanical_margin_deg=1.0e8, range_speed_deg_s=1.0e8,
        tracking_tolerance_deg=1.0e8, minimum_motion_ratio=1.0e8,
        cross_axis_limit_deg=1.0e8, max_velocity_deg_s=1.0e8,
        max_effort_nm=1.0e8,
    )
    settings.validate()


def test_small_positive_and_zero_margin_values_have_no_business_minimum():
    settings = InspectionSettings(
        move_sec=0.001, hold_sec=0.0,
        collision_margin_deg=0.0, mechanical_margin_deg=0.0,
        range_speed_deg_s=0.001, tracking_tolerance_deg=0.001,
        minimum_motion_ratio=0.0, cross_axis_limit_deg=0.001,
        max_velocity_deg_s=0.001, max_effort_nm=0.001,
    )
    settings.validate()


def test_invalid_lower_bounds_are_still_rejected():
    for field, value in (
            ("move_sec", 0.0),
            ("range_speed_deg_s", 0.0),
            ("tracking_tolerance_deg", 0.0),
            ("max_velocity_deg_s", 0.0),
            ("max_effort_nm", 0.0)):
        settings = InspectionSettings(**{field: value})
        with pytest.raises(ValueError):
            settings.validate()


def test_default_settings_are_shared_conservative_values():
    settings = InspectionSettings()
    settings.validate()
    assert settings.side == "left"
    assert settings.move_sec == 2.0
    assert settings.range_speed_deg_s == 10.0


def test_simultaneous_peer_is_not_counted_as_cross_axis_motion():
    settings = InspectionSettings(
        max_velocity_deg_s=100.0, cross_axis_limit_deg=1.0)
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
