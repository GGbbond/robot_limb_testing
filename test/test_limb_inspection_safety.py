import json
from queue import SimpleQueue
from threading import RLock
import time

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from bxi_example_py_elf3 import limb_inspection_config as config
from bxi_example_py_elf3.limb_inspection_controller import (
    LimbInspectionController,
)
from bxi_example_py_elf3.limb_inspection_core import (
    InspectionSettings, JOINT_NAMES, POSITION_MAX,
)


def bare_controller():
    controller = object.__new__(LimbInspectionController)
    controller.lock = RLock()
    controller.hardware_mode = True
    controller.fault_latched = False
    controller.initialized = True
    controller.test_running = False
    controller.returning = False
    controller.reset_step = 0
    controller.selected_indices = (3,)
    controller.selected_names = (JOINT_NAMES[3],)
    controller.initialized_names = controller.selected_names
    controller.seen = np.ones(len(JOINT_NAMES), dtype=bool)
    controller.velocity_seen = np.ones(len(JOINT_NAMES), dtype=bool)
    controller.effort_seen = np.ones(len(JOINT_NAMES), dtype=bool)
    controller.feedback_at = np.full(len(JOINT_NAMES), time.monotonic())
    controller.position = np.zeros(len(JOINT_NAMES))
    controller.velocity = np.zeros(len(JOINT_NAMES))
    controller.effort = np.zeros(len(JOINT_NAMES))
    controller.velocity_overrun_started_at = np.zeros(len(JOINT_NAMES))
    controller.settings = InspectionSettings()
    controller.state = "就绪"
    controller.detail = ""
    controller.progress = 0.0
    controller.command_topic = "hardware/actuators_cmds"
    controller.feedback_topic = "hardware/joint_states"
    controller.last_publish_at = time.monotonic()
    controller.max_command_gap_sec = 0.08
    controller.last_publisher_check_at = time.monotonic()
    controller.feedback_timeout_sec = 0.2
    controller.velocity_fault_duration_sec = 0.01
    controller.events = SimpleQueue()
    controller.count_publishers = lambda _topic: 1
    controller._publish_locked = lambda: None
    controller.faults = []

    def stop(reason):
        controller.faults.append(reason)
        controller.fault_latched = True
        controller.initialized = False

    controller.emergency_stop = stop
    return controller


def test_hardware_and_simulation_share_configuration(tmp_path, monkeypatch):
    defaults = tmp_path / "defaults.json"
    shared = tmp_path / "settings.json"
    legacy_hardware = tmp_path / "settings_hardware.json"
    defaults.write_text(json.dumps({"max_velocity_deg_s": 30.0}),
                        encoding="utf-8")
    shared.write_text(json.dumps({
        "max_velocity_deg_s": 40.0,
        "motion_mode": "small_motion",
        "amplitude_deg": 3.0,
        "cycles": 2,
    }),
                      encoding="utf-8")
    legacy_hardware.write_text(json.dumps({"max_velocity_deg_s": 20.0}),
                               encoding="utf-8")
    monkeypatch.setattr(config, "defaults_path", lambda: defaults)
    monkeypatch.setattr(config, "CONFIG_DIR", tmp_path)
    monkeypatch.setattr(config, "USER_SETTINGS", shared)
    monkeypatch.setattr(
        config, "LEGACY_HARDWARE_USER_SETTINGS", legacy_hardware)

    loaded = config.load_settings()
    assert loaded["max_velocity_deg_s"] == 40.0
    assert not {"motion_mode", "amplitude_deg", "cycles"} & loaded.keys()
    config.save_settings({"max_velocity_deg_s": 50.0})
    assert json.loads(shared.read_text(encoding="utf-8"))[
        "max_velocity_deg_s"] == 50.0
    assert json.loads(legacy_hardware.read_text(encoding="utf-8"))[
        "max_velocity_deg_s"] == 20.0


def test_legacy_hardware_configuration_is_migration_fallback(tmp_path,
                                                             monkeypatch):
    defaults = tmp_path / "defaults.json"
    shared = tmp_path / "settings.json"
    legacy_hardware = tmp_path / "settings_hardware.json"
    defaults.write_text(json.dumps({"max_velocity_deg_s": 30.0}),
                        encoding="utf-8")
    legacy_hardware.write_text(json.dumps({"max_velocity_deg_s": 20.0}),
                               encoding="utf-8")
    monkeypatch.setattr(config, "defaults_path", lambda: defaults)
    monkeypatch.setattr(config, "USER_SETTINGS", shared)
    monkeypatch.setattr(
        config, "LEGACY_HARDWARE_USER_SETTINGS", legacy_hardware)

    assert config.load_settings()["max_velocity_deg_s"] == 20.0


def test_feedback_timeout_is_latched_while_hardware_is_holding():
    controller = bare_controller()
    controller.feedback_at[3] = time.monotonic() - 1.0
    controller._timer_callback()
    assert controller.faults == ["带电阶段关节反馈超时"]


def test_command_gap_is_latched_during_hardware_stiffness_ramp():
    controller = bare_controller()
    controller.initialized = False
    controller.reset_step = -2
    controller.last_publish_at = time.monotonic() - 1.0
    controller._timer_callback()
    assert controller.faults == ["控制命令发布间隔超时"]


def test_second_command_publisher_is_latched_while_active():
    controller = bare_controller()
    controller.last_publisher_check_at = 0.0
    controller.count_publishers = lambda topic: (
        2 if topic == controller.command_topic else 1)
    controller._timer_callback()
    assert controller.faults == ["运行中检测到多个关节命令发布者"]


def test_second_feedback_publisher_is_latched_while_active():
    controller = bare_controller()
    controller.last_publisher_check_at = 0.0
    controller.count_publishers = lambda topic: (
        2 if topic == controller.feedback_topic else 1)
    controller._timer_callback()
    assert controller.faults == ["运行中检测到多个关节反馈发布者"]


def test_limits_are_monitored_during_initialized_hold():
    controller = bare_controller()
    message = JointState()
    message.name = [JOINT_NAMES[3]]
    message.position = [float(POSITION_MAX[3] + 0.01)]
    message.velocity = [0.0]
    message.effort = [0.0]
    controller._joint_callback(message)
    assert controller.faults == ["关节反馈越过软件限位：" + JOINT_NAMES[3]]


def test_test_recipe_must_match_initialized_joints():
    controller = bare_controller()
    with pytest.raises(RuntimeError, match="与已初始化关节不一致"):
        controller.start_test(InspectionSettings(limb="leg", side="left"))
