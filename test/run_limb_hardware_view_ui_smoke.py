"""Offscreen smoke test for the read-only hardware digital-twin viewport."""

import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("BXI_LIMB_CONFIG_DIR", "/tmp/bxi_limb_ui_smoke_config")
os.environ.setdefault(
    "BXI_LIMB_MODE_SWITCH_FILE", "/tmp/bxi_limb_ui_smoke_mode_request")
os.environ.pop("BXI_LIMB_STARTUP_WARNING", None)

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox
import rclpy

import bxi_example_py_elf3.limb_inspection_ui as limb_inspection_ui
from bxi_example_py_elf3.limb_inspection_controller import (
    LimbInspectionController,
)
from bxi_example_py_elf3.limb_inspection_ui import LimbInspectionWindow


def main():
    rclpy.init(args=[
        "--ros-args",
        "-p", "hardware_mode:=true",
        "-p", "topic_prefix:=hardware/",
        "-p", "collision_guard_enabled:=false",
    ])
    controller = LimbInspectionController()
    app = QApplication(sys.argv)
    window = LimbInspectionWindow(controller, executor=None)
    window.set_ui_mode("debug", persist=False)
    warnings = []
    original_warning = QMessageBox.warning
    QMessageBox.warning = lambda _parent, title, message: warnings.append(
        (title, message))
    os.environ["BXI_GAMEPAD_DEVICE"] = "/tmp/bxi_missing_gamepad_for_smoke"
    try:
        window._toggle_gamepad()
        window.startup_warning = "FPGA 预检失败测试"
        window._show_startup_warning()
    finally:
        QMessageBox.warning = original_warning
    if window.gamepad_enabled or len(warnings) != 2:
        raise RuntimeError("设备缺失时应提示并保持软件运行")

    class FakeWheelEvent:
        def __init__(self):
            self.ignored = False

        def ignore(self):
            self.ignored = True

    numeric_parameters = (
        window.move_sec, window.hold_sec, window.range_speed,
        window.collision_margin, window.mechanical_margin,
        window.tracking, window.response, window.cross,
        window.max_velocity, window.max_effort,
    )
    for widget in numeric_parameters:
        value_before = widget.value()
        wheel_event = FakeWheelEvent()
        widget.wheelEvent(wheel_event)
        if not wheel_event.ignored or widget.value() != value_before:
            raise RuntimeError("鼠标滚轮不应修改数值参数")

    class FakeGamepadReader:
        def __init__(self, device):
            self.device = device
            self.started = False
            self.events = []
            self.rumble_calls = []

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

        def drain_events(self):
            result, self.events = self.events, []
            return result

        def rumble(self, duration_ms=300):
            self.rumble_calls.append(duration_ms)

    original_reader = limb_inspection_ui.XboxGamepadReader
    original_question = QMessageBox.question
    actions = []
    limb_inspection_ui.XboxGamepadReader = FakeGamepadReader
    QMessageBox.question = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        RuntimeError("初始化或检测不应弹出二次确认框"))
    os.environ["BXI_GAMEPAD_DEVICE"] = "/dev/null"
    controller.request_initialize = lambda _settings: actions.append(
        "initialize")
    controller.start_test = lambda _settings: actions.append("start")
    window._save_settings = lambda: None
    try:
        window._toggle_gamepad()
        if not (window.gamepad_enabled and
                window.safety_check.isChecked() and
                window.full_range_check.isChecked()):
            raise RuntimeError("启用手柄后应自动勾选两项安全确认")
        help_text = window.gamepad_help_label.text()
        for expected in ("A 初始化", "X 开始检测", "B 平稳停止",
                         "Y 紧急停止", "← 选择手臂", "→ 选择腿"):
            if expected not in help_text:
                raise RuntimeError("手柄按键提示不完整：%s" % expected)
        reader = window.gamepad_reader
        reader.events.extend([
            ("axis", limb_inspection_ui.XBOX_DPAD_X_AXIS, 32767),
        ])
        window._poll_gamepad()
        if window.limb_combo.currentData() != "leg":
            raise RuntimeError("手柄右方向键应选择腿部测试")
        reader.events.extend([
            ("axis", limb_inspection_ui.XBOX_DPAD_X_AXIS, -32767),
        ])
        window._poll_gamepad()
        if window.limb_combo.currentData() != "arm":
            raise RuntimeError("手柄左方向键应选择手臂测试")
        with controller.lock:
            controller.test_running = True
        reader.events.extend([
            ("axis", limb_inspection_ui.XBOX_DPAD_X_AXIS, 32767),
        ])
        window._poll_gamepad()
        with controller.lock:
            controller.test_running = False
        if window.limb_combo.currentData() != "arm":
            raise RuntimeError("测试进行中不应允许手柄切换手臂/腿")
        window._initialize()
        window._start()
        if actions != ["initialize", "start"]:
            raise RuntimeError("初始化和检测应不经二次确认直接执行")
        with controller.lock:
            controller.initialized = True
        window._refresh()
        if reader.rumble_calls != [300]:
            raise RuntimeError("初始化完成后应触发一次手柄振动")
        window._disable_gamepad()
        if (window.gamepad_enabled or window.safety_check.isChecked() or
                window.full_range_check.isChecked()):
            raise RuntimeError("关闭手柄后应取消两项自动确认")
    finally:
        limb_inspection_ui.XboxGamepadReader = original_reader
        QMessageBox.question = original_question

    frames = []
    errors = []
    window.simulation_view.frame_ready.connect(
        lambda _image: frames.append(time.monotonic()))
    window.simulation_view.render_failed.connect(errors.append)
    window.show()

    def finish():
        window.request_shutdown()
        app.quit()

    QTimer.singleShot(8000, finish)
    try:
        app.exec_()
        if errors:
            raise RuntimeError("实机数字孪生视图失败：%s" % errors)
        if len(frames) < 20:
            raise RuntimeError(
                "实机数字孪生视图帧数不足：%d" % len(frames))
        if not window.mode_switch_button.isEnabled():
            raise RuntimeError("统一启动模式下切换按钮未启用")
        if window.gamepad_enabled:
            raise RuntimeError("手柄功能不应默认启用")
        print("hardware digital twin UI: PASS (%d frames)" % len(frames))
    finally:
        if window.isVisible():
            window.request_shutdown()
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
