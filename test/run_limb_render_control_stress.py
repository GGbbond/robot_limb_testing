"""Measure 100 Hz command timing while the CPU viewport is rendering."""

import os
import sys
import time
from threading import Thread

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from communication.msg import ActuatorCmds
from bxi_example_py_elf3.limb_simulation_view import SimulationViewport
from bxi_example_py_elf3.limb_inspection_controller import (
    LimbInspectionController,
)
from bxi_example_py_elf3.limb_inspection_core import (
    InspectionSettings, JOINT_NAMES, selected_joints,
)


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    rclpy.init()
    controller = LimbInspectionController()
    monitor = Node("limb_command_gap_monitor")
    command_times = []
    monitor.create_subscription(
        ActuatorCmds, "simulation/actuators_cmds",
        lambda _message: command_times.append(time.monotonic()),
        qos_profile_sensor_data)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    executor.add_node(monitor)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    view = SimulationViewport(1.7)
    frame_times = []
    render_errors = []
    view.frame_ready.connect(
        lambda _image: frame_times.append(time.monotonic()))
    view.render_failed.connect(render_errors.append)
    settings = InspectionSettings(
        limb="arm", side="left", motion_mode="small_motion",
        amplitude_deg=1.0, move_sec=0.2, hold_sec=0.0, cycles=1,
        tracking_tolerance_deg=30.0, minimum_motion_ratio=0.05,
        cross_axis_limit_deg=30.0, max_velocity_deg_s=500.0,
        max_effort_nm=1000.0)
    selected_indices = [
        JOINT_NAMES.index(name) for name in selected_joints("arm", "left")]
    state = ["feedback"]

    def advance():
        snapshot = controller.snapshot()
        if (state[0] == "feedback" and
                all(snapshot["seen"][i] for i in selected_indices)):
            controller.request_initialize(settings)
            state[0] = "initializing"
        elif state[0] == "initializing" and snapshot["initialized"]:
            controller.start_test(settings)
            state[0] = "testing"

    state_timer = QTimer()
    state_timer.timeout.connect(advance)
    state_timer.start(20)
    pose_timer = QTimer()
    pose_timer.timeout.connect(lambda: view.set_pose(
        controller.visualization_snapshot()))
    pose_timer.start(50)
    QTimer.singleShot(15000, app.quit)
    started = time.monotonic()
    app.exec_()
    elapsed = time.monotonic() - started

    view.shutdown()
    controller.emergency_stop("渲染/控制压力测试结束")
    executor.shutdown(timeout_sec=2.0)
    spin_thread.join(timeout=2.0)
    monitor.destroy_node()
    controller.destroy_node()
    rclpy.try_shutdown()
    if render_errors:
        raise RuntimeError("视图渲染失败：%s" % render_errors)
    gaps_ms = np.diff(command_times) * 1000.0
    if len(gaps_ms) < 100:
        raise RuntimeError("控制命令样本不足")
    maximum = float(np.max(gaps_ms))
    if maximum >= 80.0:
        raise RuntimeError("控制命令最大间隔 %.2f ms，超过 80 ms" % maximum)
    print(
        "render/control stress: PASS (view %.2f FPS, command mean %.2f ms, "
        "p99 %.2f ms, max %.2f ms, %.1fs)" % (
            len(frame_times) / elapsed, float(np.mean(gaps_ms)),
            float(np.percentile(gaps_ms, 99)), maximum, elapsed))


if __name__ == "__main__":
    main()
