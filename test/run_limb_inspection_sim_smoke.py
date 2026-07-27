"""Short end-to-end smoke test; run only with MuJoCo simulation active."""

from dataclasses import replace
import time
from threading import Thread

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from bxi_example_py_elf3.limb_inspection_controller import LimbInspectionController
from bxi_example_py_elf3.limb_inspection_core import (
    JOINT_NAMES, InspectionSettings, selected_joints,
)


def wait_until(predicate, timeout, message):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError(message)


def main():
    rclpy.init(args=[
        "--ros-args", "-p", "initialization_sec:=0.5",
        "-p", "feedback_timeout_sec:=0.5",
    ])
    node = LimbInspectionController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    settings = InspectionSettings(
        limb="leg", side="both_simultaneous", motion_mode="small_motion",
        amplitude_deg=1.0,
        move_sec=0.2, hold_sec=0.05, cycles=1,
        range_speed_deg_s=60.0,
        tracking_tolerance_deg=30.0, minimum_motion_ratio=0.05,
        cross_axis_limit_deg=30.0, max_velocity_deg_s=500.0,
        max_effort_nm=1000.0,
    )
    try:
        names = selected_joints(settings.limb, settings.side)
        indices = [JOINT_NAMES.index(name) for name in names]
        wait_until(
            lambda: all(node.seen[index] for index in indices),
            8.0, "没有收到双腿仿真 joint_states")
        node.request_initialize(settings)
        wait_until(lambda: node.snapshot()["initialized"], 8.0, "初始化超时")
        node.start_test(settings)
        wait_until(lambda: node.snapshot()["state"] == "检测完成",
                   45.0, "双侧同步检测超时")
        snapshot = node.snapshot(include_samples=True)
        if len(snapshot["results"]) != len(names):
            raise RuntimeError("结果数量错误")
        hip_x_samples = [
            sample for sample in snapshot["samples"]
            if sample["joint"] == "l_hip_x_joint,r_hip_x_joint"
        ]
        if not hip_x_samples:
            raise RuntimeError("没有记录双侧髋侧摆同步样本")
        left_hip = JOINT_NAMES.index("l_hip_x_joint")
        right_hip = JOINT_NAMES.index("r_hip_x_joint")
        left_knee = JOINT_NAMES.index("l_knee_y_joint")
        right_knee = JOINT_NAMES.index("r_knee_y_joint")
        if max(np.rad2deg(sample["command"][left_knee])
               for sample in hip_x_samples) < 130.0:
            raise RuntimeError("髋侧摆测试期间左膝未进入紧凑姿态")
        if max(np.rad2deg(sample["command"][right_knee])
               for sample in hip_x_samples) < 130.0:
            raise RuntimeError("髋侧摆测试期间右膝未进入紧凑姿态")
        mirror_error = max(abs(np.rad2deg(
            sample["command"][left_hip] + sample["command"][right_hip]))
            for sample in hip_x_samples)
        if mirror_error > 0.05:
            raise RuntimeError("髋侧摆命令没有保持镜像，误差 %.3f°" % mirror_error)
        if max(abs(np.rad2deg(hip_x_samples[-1]["command"][index]))
               for index in (left_knee, right_knee)) > 0.05:
            raise RuntimeError("髋侧摆测试结束前膝关节未展开")
        leg_passed = sum(result.passed for result in snapshot["results"])

        arm_settings = replace(settings, limb="arm")
        arm_names = selected_joints(arm_settings.limb, arm_settings.side)
        node.request_initialize(arm_settings)
        wait_until(lambda: node.snapshot()["initialized"],
                   8.0, "切换双臂后重新初始化超时")
        node.start_test(arm_settings)
        wait_until(lambda: node.snapshot()["state"] == "检测完成",
                   45.0, "双臂同步检测超时")
        arm_snapshot = node.snapshot(include_samples=True)
        if len(arm_snapshot["results"]) != len(arm_names):
            raise RuntimeError("双臂结果数量错误")
        shoulder_x_samples = [
            sample for sample in arm_snapshot["samples"]
            if sample["joint"] ==
            "l_shoulder_x_joint,r_shoulder_x_joint"
        ]
        left_shoulder = JOINT_NAMES.index("l_shoulder_x_joint")
        right_shoulder = JOINT_NAMES.index("r_shoulder_x_joint")
        left_elbow = JOINT_NAMES.index("l_elbow_y_joint")
        right_elbow = JOINT_NAMES.index("r_elbow_y_joint")
        if min(np.rad2deg(sample["command"][left_elbow])
               for sample in shoulder_x_samples) > -40.0:
            raise RuntimeError("肩侧摆测试期间左肘未进入紧凑姿态")
        if min(np.rad2deg(sample["command"][right_elbow])
               for sample in shoulder_x_samples) > -40.0:
            raise RuntimeError("肩侧摆测试期间右肘未进入紧凑姿态")
        shoulder_mirror_error = max(abs(np.rad2deg(
            sample["command"][left_shoulder] +
            sample["command"][right_shoulder]))
            for sample in shoulder_x_samples)
        if shoulder_mirror_error > 0.05:
            raise RuntimeError(
                "肩侧摆命令没有保持镜像，误差 %.3f°" %
                shoulder_mirror_error)
        if max(abs(np.rad2deg(
                shoulder_x_samples[-1]["command"][index]))
               for index in (left_elbow, right_elbow)) > 0.05:
            raise RuntimeError("肩侧摆测试结束前肘关节未展开")
        arm_passed = sum(
            result.passed for result in arm_snapshot["results"])
        print("simulation sequence: PASS (%d joints, %d passed)" % (
            len(names) + len(arm_names), leg_passed + arm_passed))
    finally:
        node.emergency_stop("冒烟测试结束")
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
