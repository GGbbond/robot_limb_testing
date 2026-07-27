"""ROS2 controller for safe, sequential Elf3 arm/leg bench inspection."""

import copy
import time
from queue import SimpleQueue
from threading import RLock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import communication.msg as bxi_msg
import communication.srv as bxi_srv
from ament_index_python.packages import get_package_share_path

from .limb_collision_guard import CollisionGuard
from .limb_inspection_core import (
    JOINT_KD,
    JOINT_KP,
    JOINT_NAMES,
    MODEL_COLLISION_FREE_RANGE_DEG,
    POSITION_MAX,
    POSITION_MIN,
    InspectionSettings,
    evaluate_joint,
    minimum_jerk,
    safe_joint_range,
    selected_joint_groups,
    selected_joints,
    validate_center,
)
from .limb_inspection_posture import (
    compact_posture, mirrored_target_ranges, safe_range_waypoints,
    small_motion_waypoints,
)


class LimbInspectionController(Node):
    """Owns the single actuator publisher and the inspection state machine."""

    def __init__(self):
        super().__init__("elf3_limb_inspection")
        self.topic_prefix = str(self.declare_parameter(
            "topic_prefix", "simulation/").value)
        self.hardware_mode = bool(self.declare_parameter(
            "hardware_mode", False).value)
        self.control_rate_hz = float(self.declare_parameter(
            "control_rate_hz", 100.0).value)
        self.feedback_timeout_sec = float(self.declare_parameter(
            "feedback_timeout_sec", 0.2).value)
        self.initialization_sec = float(self.declare_parameter(
            "initialization_sec", 10.0).value)
        self.simulation_bench_height_m = float(self.declare_parameter(
            "simulation_bench_height_m", 1.7).value)
        self.velocity_fault_duration_sec = float(self.declare_parameter(
            "velocity_fault_duration_sec", 0.01).value)
        self.collision_guard_enabled = bool(self.declare_parameter(
            "collision_guard_enabled", True).value)
        self.max_command_gap_sec = float(self.declare_parameter(
            "max_command_gap_sec", 0.08).value)
        self.sample_rate_hz = float(self.declare_parameter(
            "sample_rate_hz", 50.0).value)
        if not 20.0 <= self.control_rate_hz <= 500.0:
            raise ValueError("control_rate_hz must be between 20 and 500")
        if not 0.03 <= self.max_command_gap_sec <= 1.0:
            raise ValueError("max_command_gap_sec must be between 0.03 and 1.0")
        if not 1.2 <= self.simulation_bench_height_m <= 3.0:
            raise ValueError("simulation_bench_height_m must be between 1.2 and 3.0")
        if not 0.0 <= self.velocity_fault_duration_sec <= 0.5:
            raise ValueError("velocity_fault_duration_sec must be between 0 and 0.5")
        if not 10.0 <= self.sample_rate_hz <= self.control_rate_hz:
            raise ValueError(
                "sample_rate_hz must be between 10 and control_rate_hz")

        qos = QoSProfile(
            depth=1,
            durability=qos_profile_sensor_data.durability,
            reliability=qos_profile_sensor_data.reliability,
        )
        self.command_topic = self.topic_prefix + "actuators_cmds"
        self.feedback_topic = self.topic_prefix + "joint_states"
        self.publisher = self.create_publisher(
            bxi_msg.ActuatorCmds, self.command_topic, qos)
        self.subscription = self.create_subscription(
            JointState, self.feedback_topic,
            self._joint_callback, qos)
        self.reset_client = self.create_client(
            bxi_srv.RobotReset, self.topic_prefix + "robot_reset")
        self.sim_reset_client = None
        if not self.hardware_mode:
            self.sim_reset_client = self.create_client(
                bxi_srv.SimulationReset, self.topic_prefix + "sim_reset")

        self.lock = RLock()
        self.events = SimpleQueue()
        self.position = np.zeros(len(JOINT_NAMES), dtype=float)
        self.velocity = np.zeros(len(JOINT_NAMES), dtype=float)
        self.effort = np.zeros(len(JOINT_NAMES), dtype=float)
        self.seen = np.zeros(len(JOINT_NAMES), dtype=bool)
        self.velocity_seen = np.zeros(len(JOINT_NAMES), dtype=bool)
        self.effort_seen = np.zeros(len(JOINT_NAMES), dtype=bool)
        self.feedback_at = np.zeros(len(JOINT_NAMES), dtype=float)
        self.velocity_overrun_started_at = np.zeros(
            len(JOINT_NAMES), dtype=float)
        self.command = np.zeros(len(JOINT_NAMES), dtype=float)
        self.center = np.zeros(len(JOINT_NAMES), dtype=float)
        self.selected_names = tuple()
        self.selected_groups = tuple()
        self.selected_indices = tuple()
        self.controlled_indices = tuple()
        self.initialized_names = tuple()
        self.settings = InspectionSettings()
        self.state = "等待反馈"
        self.detail = "请选择测试对象并确认台架安全"
        self.progress = 0.0
        self.initialized = False
        self.fault_latched = False
        self.test_running = False
        self.returning = False
        self.return_started_at = 0.0
        self.return_start = self.command.copy()
        self.results = []
        self.samples = []
        self.current_samples = []
        self.current_joint_cursor = -1
        self.current_joint_name = ""
        self.current_joint_names = tuple()
        self.current_posture_names = tuple()
        self.current_motion_names = tuple()
        self.segment_cursor = 0
        self.segment_started_at = 0.0
        self.joint_started_at = 0.0
        self.segment_start_factor = 0.0
        self.segment_target_factor = 0.0
        self.current_waypoints = []
        self.current_waypoint_is_posture = []
        self.segment_start_positions = {}
        self.segment_target_positions = {}
        self.joint_target_ranges = {}
        self.completed_test_segments = 0
        self.total_test_segments = 0
        self.reset_future = None
        self.sim_reset_future = None
        self.reset_step = 0
        self.init_started_at = 0.0
        self.last_publish_at = 0.0
        self.last_feedback_notice = 0.0
        self.last_visual_collision_check_at = 0.0
        self.last_publisher_check_at = 0.0
        self.last_sample_at = 0.0
        self.sample_period_sec = 1.0 / self.sample_rate_hz
        self.collision_guard = None
        if self.collision_guard_enabled:
            model_path = get_package_share_path(
                "bxi_example_py_elf3") / "data" / "elf3.xml"
            self.collision_guard = CollisionGuard(model_path)
            if not self.hardware_mode:
                self.collision_guard.set_base_height(
                    self.simulation_bench_height_m)
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz, self._timer_callback)
        self._event("info", "控制器已启动，当前为%s模式" % (
            "实机" if self.hardware_mode else "仿真"))

    def _event(self, level, message):
        self.events.put((time.time(), level, message))
        if level == "error":
            self.get_logger().error(message)
        elif level == "warning":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def drain_events(self):
        result = []
        while not self.events.empty():
            result.append(self.events.get())
        return result

    def snapshot(self, include_samples=False):
        with self.lock:
            snapshot = {
                "state": self.state, "detail": self.detail,
                "progress": self.progress, "initialized": self.initialized,
                "fault_latched": self.fault_latched,
                "test_running": self.test_running,
                "returning": self.returning,
                "current_joint": self.current_joint_name,
                "current_joints": self.current_joint_names,
                "position": self.position.copy(),
                "velocity": self.velocity.copy(),
                "effort": self.effort.copy(),
                "command": self.command.copy(),
                "seen": self.seen.copy(),
                "feedback_age": time.monotonic() - float(np.max(self.feedback_at)),
                "results": copy.deepcopy(self.results),
            }
            if include_samples:
                snapshot["samples"] = list(self.samples)
            return snapshot

    def visualization_snapshot(self):
        """Return only the high-rate data needed by the 3D viewport."""
        with self.lock:
            return self.position.copy()

    def configure_selection(self, settings):
        settings.validate()
        names = selected_joints(settings.limb, settings.side)
        groups = selected_joint_groups(settings.limb, settings.side)
        indices = tuple(JOINT_NAMES.index(name) for name in names)
        with self.lock:
            if self.test_running or self.returning or self.reset_step:
                raise RuntimeError("初始化或测试运行时不能更换测试对象")
            self.settings = settings
            self.selected_names = names
            self.selected_groups = groups
            self.selected_indices = indices
            # The complete simulated robot must stay in the collision-scan
            # reference pose.  A hardware bench may contain only the selected
            # limbs, so never energize unselected hardware joints here.
            self.controlled_indices = (
                tuple(range(len(JOINT_NAMES)))
                if not self.hardware_mode else indices
            )
        return names

    def request_initialize(self, settings):
        names = self.configure_selection(settings)
        now = time.monotonic()
        with self.lock:
            if self.fault_latched:
                raise RuntimeError("急停/故障已经锁定，请重启软件后再初始化")
            missing = [name for name in names
                       if not self.seen[JOINT_NAMES.index(name)]]
            if missing:
                raise RuntimeError("以下关节没有反馈：" + ", ".join(missing))
            if any(now - self.feedback_at[i] > self.feedback_timeout_sec
                   for i in self.selected_indices):
                raise RuntimeError("所选关节反馈已超时")
            if self.hardware_mode:
                missing_velocity = [
                    JOINT_NAMES[i] for i in self.selected_indices
                    if not self.velocity_seen[i]
                ]
                missing_effort = [
                    JOINT_NAMES[i] for i in self.selected_indices
                    if not self.effort_seen[i]
                ]
                if missing_velocity:
                    raise RuntimeError(
                        "以下关节没有速度反馈：" + ", ".join(missing_velocity))
                if missing_effort:
                    raise RuntimeError(
                        "以下关节没有力矩反馈：" + ", ".join(missing_effort))
            if self.count_publishers(self.feedback_topic) > 1:
                raise RuntimeError(
                    "检测到多个关节反馈发布者，请关闭残留仿真或硬件节点")
            if self.count_publishers(self.command_topic) > 1:
                raise RuntimeError("检测到多个关节命令发布者，请关闭其他控制器")
            self.initialized = False
            self.initialized_names = tuple()
            self.last_publish_at = 0.0
            self.progress = 0.0
            self.velocity_overrun_started_at[:] = 0.0
            if self.hardware_mode:
                self.state = "初始化步骤 1/2"
                self.detail = "请求关节进入位置控制"
                self.reset_step = 1
                self._send_reset_locked(1)
            else:
                self.state = "仿真台架复位"
                self.detail = "正在把机器人抬升到悬空台架高度 %.2f m" % (
                    self.simulation_bench_height_m)
                self.reset_step = -1
                self.sim_reset_future = None
                self._send_sim_reset_locked()
        self._event("info", "开始初始化 %d 个关节" % len(names))

    def _send_reset_locked(self, step):
        if not self.reset_client.service_is_ready():
            self.reset_future = None
            return
        request = bxi_srv.RobotReset.Request()
        request.header.frame_id = "elf3"
        request.reset_step = step
        request.release = False
        self.reset_future = self.reset_client.call_async(request)

    def _send_sim_reset_locked(self):
        if (self.sim_reset_client is None or
                not self.sim_reset_client.service_is_ready()):
            self.sim_reset_future = None
            return
        request = bxi_srv.SimulationReset.Request()
        request.header.frame_id = "elf3"
        pose = Pose()
        pose.position.z = self.simulation_bench_height_m
        pose.orientation.w = 1.0
        request.base_pose = pose
        joint_state = JointState()
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = np.zeros(len(JOINT_NAMES)).tolist()
        joint_state.velocity = np.zeros(len(JOINT_NAMES)).tolist()
        joint_state.effort = np.zeros(len(JOINT_NAMES)).tolist()
        request.joint_state = joint_state
        self.sim_reset_future = self.sim_reset_client.call_async(request)

    def _sim_reset_succeeded_locked(self):
        if self.sim_reset_future is None or not self.sim_reset_future.done():
            return False
        try:
            response = self.sim_reset_future.result()
            success = bool(response and response.is_success)
        except Exception as exc:
            self._event("error", "仿真台架复位失败：%s" % exc)
            success = False
        self.sim_reset_future = None
        return success

    def start_test(self, settings):
        settings.validate()
        requested_names = selected_joints(settings.limb, settings.side)
        with self.lock:
            if not self.initialized:
                raise RuntimeError("请先完成机器人初始化")
            if requested_names != self.initialized_names:
                raise RuntimeError("检测对象与已初始化关节不一致，请重新初始化")
        self.configure_selection(settings)
        with self.lock:
            if not self.initialized:
                raise RuntimeError("请先完成机器人初始化")
            if self.fault_latched:
                raise RuntimeError("故障已经锁定，必须重启软件")
            if self.test_running:
                raise RuntimeError("测试已经在运行")
            if self.returning:
                raise RuntimeError("正在返回中心位置，请稍候")
            now = time.monotonic()
            if any(now - self.feedback_at[i] > self.feedback_timeout_sec
                   for i in self.selected_indices):
                raise RuntimeError("所选关节反馈超时，拒绝启动")
            if self.hardware_mode and self.count_publishers(
                    self.command_topic) > 1:
                raise RuntimeError("检测到多个关节命令发布者，请关闭其他控制器")
            self.center = self.position.copy()
            self.joint_target_ranges = {}
            if settings.motion_mode == "safe_range":
                if self.collision_guard is None:
                    raise RuntimeError("安全全行程需要启用 MuJoCo 碰撞预测器")
                if self.hardware_mode and not settings.full_range_confirmed:
                    raise RuntimeError(
                        "实机安全全行程尚未确认：请确认线缆、夹具和台架范围"
                    )
                reference_names = tuple(
                    JOINT_NAMES[index] for index in self.controlled_indices
                    if self.seen[index]
                )
                non_neutral = [
                    name for name in reference_names
                    if abs(np.rad2deg(self.center[JOINT_NAMES.index(name)])) > 10.0
                ]
                if non_neutral:
                    raise RuntimeError(
                        "安全全行程基于零位姿态；以下关节偏离零位超过 10°：" +
                        ", ".join(non_neutral)
                    )
                self.joint_target_ranges = {
                    name: safe_joint_range(name, settings)
                    for name in self.selected_names
                }
                if settings.side == "both_simultaneous":
                    self.joint_target_ranges = mirrored_target_ranges(
                        self.selected_groups, self.joint_target_ranges,
                        settings.collision_margin_deg)
                outside_margined_range = []
                for name, (low, high) in self.joint_target_ranges.items():
                    index = JOINT_NAMES.index(name)
                    value = self.center[JOINT_NAMES.index(name)]
                    model_low_deg, model_high_deg = \
                        MODEL_COLLISION_FREE_RANGE_DEG[name]
                    raw_low = max(
                        float(POSITION_MIN[index]),
                        float(np.deg2rad(model_low_deg)))
                    raw_high = min(
                        float(POSITION_MAX[index]),
                        float(np.deg2rad(model_high_deg)))
                    if not raw_low <= value <= raw_high:
                        raise RuntimeError(
                            "%s 的当前位置不在模型无碰撞范围内" % name
                        )
                    if not low <= value <= high:
                        outside_margined_range.append(name)
                center_collisions = self.collision_guard.collisions(self.center)
                if center_collisions:
                    geom_1, geom_2, distance = center_collisions[0]
                    raise RuntimeError(
                        "当前位置存在模型碰撞：%s ↔ %s（侵入 %.1f mm）" %
                        (geom_1, geom_2, -1000.0 * distance))
                for name in self.selected_names:
                    visual_hits = self.collision_guard.visual_mesh_collisions(
                        self.center, name)
                    if visual_hits:
                        body_1, body_2, distance = visual_hits[0]
                        raise RuntimeError(
                            "当前位置存在视觉外壳侵入：%s ↔ %s（侵入 %.1f mm）" %
                            (body_1, body_2, -1000.0 * distance))
                if outside_margined_range:
                    self._event(
                        "warning",
                        "当前位置位于带余量目标范围之外，将先平滑进入安全目标：" +
                        ", ".join(outside_margined_range))
            else:
                validate_center(self.selected_names, self.center,
                                settings.amplitude_rad)
                self.joint_target_ranges = {
                    name: (
                        float(self.center[JOINT_NAMES.index(name)] -
                              settings.amplitude_rad),
                        float(self.center[JOINT_NAMES.index(name)] +
                              settings.amplitude_rad),
                    )
                    for name in self.selected_names
                }
                if settings.side == "both_simultaneous":
                    self.joint_target_ranges = mirrored_target_ranges(
                        self.selected_groups, self.joint_target_ranges,
                        settings.collision_margin_deg)
            self.command = self.center.copy()
            self.results = []
            self.samples = []
            self.last_sample_at = 0.0
            self.test_running = True
            self.returning = False
            self.state = "自动检测中"
            self.current_joint_cursor = 0
            self.completed_test_segments = 0
            self.total_test_segments = sum(
                self._group_segment_count(group)
                for group in self.selected_groups)
            self.progress = 0.0
            self._begin_joint_locked(now)
        mode_text = (
            "开始左右对应关节同步检测："
            if settings.side == "both_simultaneous" else "开始逐关节检测：")
        self._event("info", mode_text + ", ".join(self.selected_names))

    def stop_test(self, reason="操作员停止"):
        with self.lock:
            if not self.test_running:
                return
            self.test_running = False
            self.returning = True
            self.return_started_at = time.monotonic()
            self.return_start = self.command.copy()
            self.state = "平稳停止中"
            self.detail = reason + "，正在返回中心位置"
        self._event("warning", self.detail)

    def emergency_stop(self, reason="操作员按下急停"):
        with self.lock:
            self.test_running = False
            self.returning = False
            self.initialized = False
            self.initialized_names = tuple()
            self.fault_latched = True
            self.state = "急停锁定"
            self.detail = reason + "；已停止发送控制命令"
            self.progress = 0.0
        self._event("error", self.detail)

    def _joint_callback(self, message):
        now = time.monotonic()
        names = message.name if message.name else JOINT_NAMES[:len(message.position)]
        mapping = {name: i for i, name in enumerate(names)}
        with self.lock:
            control_active = self._control_active_locked()
            if control_active and len(mapping) != len(names):
                self.emergency_stop("关节反馈包含重复名称")
                return
            seen_before = int(np.count_nonzero(self.seen))
            for index, name in enumerate(JOINT_NAMES):
                source = mapping.get(name)
                if source is None or source >= len(message.position):
                    continue
                value = float(message.position[source])
                if not np.isfinite(value):
                    if self.hardware_mode and index in self.selected_indices:
                        self.emergency_stop("收到非有限关节位置：" + name)
                    continue
                self.position[index] = value
                if source < len(message.velocity):
                    value = float(message.velocity[source])
                    if np.isfinite(value):
                        self.velocity[index] = value
                        self.velocity_seen[index] = True
                    elif control_active and index in self.selected_indices:
                        self.emergency_stop("收到非有限关节速度：" + name)
                        return
                elif control_active and index in self.selected_indices:
                    self.emergency_stop("带电阶段缺少关节速度：" + name)
                    return
                if source < len(message.effort):
                    value = float(message.effort[source])
                    if np.isfinite(value):
                        self.effort[index] = value
                        self.effort_seen[index] = True
                    elif control_active and index in self.selected_indices:
                        self.emergency_stop("收到非有限关节力矩：" + name)
                        return
                elif control_active and index in self.selected_indices:
                    self.emergency_stop("带电阶段缺少关节力矩：" + name)
                    return
                self.seen[index] = True
                self.feedback_at[index] = now
            seen_after = int(np.count_nonzero(self.seen))
            if (self.state in ("等待反馈", "待初始化") and
                    not self.initialized and self.reset_step == 0 and seen_after > 0):
                self.state = "待初始化"
                self.detail = "已收到 %d/29 个关节反馈，请选择检测对象并初始化" % seen_after
                if seen_before == 0:
                    self._event("info", "已开始接收关节反馈（%d/29）" % seen_after)
            monitor_actuators = self.test_running or (
                self.hardware_mode and self._control_active_locked())
            if monitor_actuators:
                for index in self.selected_indices:
                    if (self.position[index] < POSITION_MIN[index] or
                            self.position[index] > POSITION_MAX[index]):
                        self.emergency_stop("关节反馈越过软件限位：" + JOINT_NAMES[index])
                        break
                    speed_deg_s = abs(np.rad2deg(self.velocity[index]))
                    if speed_deg_s > self.settings.max_velocity_deg_s:
                        if self.velocity_overrun_started_at[index] <= 0.0:
                            self.velocity_overrun_started_at[index] = now
                        elif (now - self.velocity_overrun_started_at[index] >=
                              self.velocity_fault_duration_sec):
                            self.emergency_stop(
                                "关节持续速度超限：%s（%.1f°/s，持续 %.0f ms）" %
                                (JOINT_NAMES[index], speed_deg_s,
                                 1000.0 * self.velocity_fault_duration_sec))
                            break
                    else:
                        self.velocity_overrun_started_at[index] = 0.0
                    if abs(self.effort[index]) > self.settings.max_effort_nm:
                        self.emergency_stop("关节力矩超限：" + JOINT_NAMES[index])
                        break

    def _reset_succeeded_locked(self):
        if self.reset_future is None or not self.reset_future.done():
            return False
        try:
            response = self.reset_future.result()
            success = bool(response and response.is_success)
        except Exception as exc:
            self._event("error", "初始化服务调用失败：%s" % exc)
            success = False
        self.reset_future = None
        return success

    def _timer_callback(self):
        now = time.monotonic()
        with self.lock:
            if self.fault_latched:
                return
            # Every phase that can energize hardware gets the same independent
            # feedback, publisher-count and scheduling-gap protection.
            if self._control_active_locked():
                if (self.last_publish_at > 0.0 and
                        now - self.last_publish_at > self.max_command_gap_sec):
                    self.emergency_stop("控制命令发布间隔超时")
                    return
                if not self._feedback_is_fresh_locked(now):
                    self.emergency_stop("带电阶段关节反馈超时")
                    return
                if now - self.last_publisher_check_at >= 0.5:
                    self.last_publisher_check_at = now
                    if self.count_publishers(self.feedback_topic) > 1:
                        self.emergency_stop("运行中检测到多个关节反馈发布者")
                        return
                    if self.count_publishers(self.command_topic) > 1:
                        self.emergency_stop("运行中检测到多个关节命令发布者")
                        return
            if self.reset_step:
                self._advance_initialization_locked(now)
            elif self.test_running:
                if not self._feedback_is_fresh_locked(now):
                    self.emergency_stop("测试中关节反馈超时")
                    return
                self._advance_test_locked(now)
            elif self.returning:
                self._advance_return_locked(now)
            if self.initialized or self.reset_step in (-2, 2):
                self._publish_locked()

    def _advance_initialization_locked(self, now):
        if self.reset_step == -1:
            if self.sim_reset_future is None:
                self._send_sim_reset_locked()
                if now - self.last_feedback_notice > 1.0:
                    self.detail = "等待 simulation/sim_reset 服务"
                    self.last_feedback_notice = now
                return
            if not self._sim_reset_succeeded_locked():
                return
            self.reset_step = 1
            self.state = "初始化步骤 1/2"
            self.detail = "悬空台架复位完成，请求关节进入位置控制"
            self._send_reset_locked(1)
            self._event(
                "info", "仿真机器人已抬升到 %.2f m" %
                self.simulation_bench_height_m)
            return
        if self.reset_step == -2:
            scale = (now - self.init_started_at) / self.initialization_sec
            if scale >= 1.0:
                self.reset_step = 2
                self.progress = 100.0
                self.detail = "请求启用全部控制参数"
                self._send_reset_locked(2)
            return
        if self.reset_future is None:
            self._send_reset_locked(self.reset_step)
            if now - self.last_feedback_notice > 1.0:
                self.detail = "等待 robot_reset 服务"
                self.last_feedback_notice = now
            return
        if not self._reset_succeeded_locked():
            return
        if self.reset_step == 1:
            self.center = self.position.copy()
            self.command = self.center.copy()
            self.reset_step = -2
            self.init_started_at = now
            self.state = "初始化步骤 2/2"
            self.detail = "刚度缓慢加载中"
            self._event("info", "初始化步骤 1 完成")
            return
        self.initialized = True
        self.initialized_names = self.selected_names
        self.reset_step = 0
        self.progress = 0.0
        self.state = "就绪"
        self.detail = "初始化完成，可以开始关节检测"
        self._event("info", "机器人初始化完成")

    def _feedback_is_fresh_locked(self, now):
        return bool(self.selected_indices) and all(
            self.seen[i] and now - self.feedback_at[i] <= self.feedback_timeout_sec
            for i in self.selected_indices)

    def _control_active_locked(self):
        return bool(self.selected_indices) and (
            self.initialized or self.test_running or self.returning or
            self.reset_step in (-2, 2))

    def _begin_joint_locked(self, now):
        self.current_joint_names = self.selected_groups[self.current_joint_cursor]
        self.current_joint_name = self.current_joint_names[0]
        posture = compact_posture(self.current_joint_names)
        self.current_posture_names = tuple(posture)
        self.current_motion_names = tuple(dict.fromkeys(
            self.current_joint_names + self.current_posture_names))
        centers = {
            name: float(self.center[JOINT_NAMES.index(name)])
            for name in self.current_motion_names
        }
        self.current_samples = []
        self.segment_cursor = 0
        self.segment_started_at = now
        self.joint_started_at = now
        self.current_waypoints = []
        self.current_waypoint_is_posture = []
        folded = dict(centers)
        folded.update(posture)
        if posture:
            self.current_waypoints.append(dict(folded))
            self.current_waypoint_is_posture.append(True)
        if self.settings.motion_mode == "safe_range":
            active_waypoints = safe_range_waypoints(
                self.current_joint_names, self.joint_target_ranges)
            active_waypoints.append({
                name: centers[name] for name in self.current_joint_names})
            ranges = [
                "%s %.1f°→%.1f°" % (
                    name,
                    np.rad2deg(self.joint_target_ranges[name][0]),
                    np.rad2deg(self.joint_target_ranges[name][1]))
                for name in self.current_joint_names
            ]
            self.detail = "安全全行程%s检测 %s" % (
                "镜像" if len(self.current_joint_names) > 1 else "",
                "；".join(ranges))
        else:
            active_waypoints = small_motion_waypoints(
                self.current_joint_names, centers,
                self.settings.amplitude_rad, self.settings.cycles,
                self.joint_target_ranges)
            self.detail = "小幅往复%s检测 %s" % (
                "镜像" if len(self.current_joint_names) > 1 else "",
                ", ".join(self.current_joint_names))
        for active_targets in active_waypoints:
            target = dict(folded)
            target.update(active_targets)
            self.current_waypoints.append(target)
            self.current_waypoint_is_posture.append(False)
        if posture:
            self.current_waypoints.append(dict(centers))
            self.current_waypoint_is_posture.append(True)
            posture_text = ", ".join(
                "%s %.1f°" % (name, np.rad2deg(value))
                for name, value in posture.items())
            self.detail += "；先蜷缩 " + posture_text
        self.segment_start_positions = dict(centers)
        self.segment_target_positions = dict(self.current_waypoints[0])

    def _group_segment_count(self, group):
        base = 3 if self.settings.motion_mode == "safe_range" \
            else 4 * self.settings.cycles
        return base + (2 if compact_posture(group) else 0)

    def _current_move_duration(self):
        posture_transition = self.current_waypoint_is_posture[
            self.segment_cursor]
        if (self.settings.motion_mode != "safe_range" and
                not posture_transition):
            return self.settings.move_sec
        travel_deg = max(
            abs(np.rad2deg(
                self.segment_target_positions[name] -
                self.segment_start_positions[name]
            ))
            for name in self.current_motion_names
        )
        velocity_limited = 1.875 * travel_deg / self.settings.range_speed_deg_s
        return max(self.settings.move_sec, velocity_limited)

    def _advance_test_locked(self, now):
        settings = self.settings
        elapsed = now - self.segment_started_at
        move_duration = self._current_move_duration()
        if elapsed < move_duration:
            blend = minimum_jerk(elapsed / move_duration)
        self.command = self.center.copy()
        for name in self.current_motion_names:
            if elapsed < move_duration:
                joint_position = self.segment_start_positions[name] + blend * (
                    self.segment_target_positions[name] -
                    self.segment_start_positions[name])
            else:
                joint_position = self.segment_target_positions[name]
            self.command[JOINT_NAMES.index(name)] = joint_position
        segment_duration = move_duration + settings.hold_sec
        should_sample = (
            not self.current_samples or
            now - self.last_sample_at >= self.sample_period_sec or
            elapsed >= segment_duration
        )
        if should_sample:
            row = {
                "time": time.time(),
                "joint": ",".join(self.current_joint_names),
                "command": self.command.astype(np.float32, copy=True),
                "position": self.position.astype(np.float32, copy=True),
                "velocity": self.velocity.astype(np.float32, copy=True),
                "effort": self.effort.astype(np.float32, copy=True),
            }
            self.samples.append(row)
            self.current_samples.append(row)
            self.last_sample_at = now
        if elapsed < segment_duration:
            self._update_progress_locked(elapsed / segment_duration)
            return
        self.segment_cursor += 1
        if self.segment_cursor < len(self.current_waypoints):
            self.segment_start_positions = dict(self.segment_target_positions)
            self.segment_target_positions = dict(
                self.current_waypoints[self.segment_cursor])
            self.segment_started_at = now
            return
        self._finish_joint_locked(now)

    def _update_progress_locked(self, segment_fraction):
        self.progress = 100.0 * (
            self.completed_test_segments + self.segment_cursor +
            segment_fraction) / self.total_test_segments

    def _advance_return_locked(self, now):
        travel_deg = float(np.max(np.abs(np.rad2deg(
            self.return_start - self.center))))
        velocity_limited = (
            1.875 * travel_deg / self.settings.range_speed_deg_s)
        duration = max(1.0, self.settings.move_sec, velocity_limited)
        progress = (now - self.return_started_at) / duration
        blend = minimum_jerk(progress)
        self.command = self.return_start + blend * (self.center - self.return_start)
        if progress >= 1.0:
            self.command = self.center.copy()
            self.returning = False
            self.state = "已停止"
            self.detail = "已返回测试中心位置"
            self._event("info", self.detail)

    def _finish_joint_locked(self, now):
        rows = self.current_samples
        positions = np.asarray([row["position"] for row in rows])
        commands = np.asarray([row["command"] for row in rows])
        velocities = np.asarray([row["velocity"] for row in rows])
        efforts = np.asarray([row["effort"] for row in rows])
        excluded_indices = tuple(
            JOINT_NAMES.index(name)
            for name in self.current_motion_names)
        for name in self.current_joint_names:
            result = evaluate_joint(
                name, self.settings, self.center,
                positions, commands, velocities, efforts, self.selected_indices,
                self.joint_target_ranges.get(name),
                exclude_cross_indices=excluded_indices)
            self.results.append(result)
            self._event("info" if result.passed else "warning",
                        "%s：%s（%s）" % (
                            result.label, "通过" if result.passed else "不通过",
                            result.reason))
        self.completed_test_segments += len(self.current_waypoints)
        self.current_joint_cursor += 1
        if self.current_joint_cursor >= len(self.selected_groups):
            self.command = self.center.copy()
            self.test_running = False
            self.current_joint_name = ""
            self.current_joint_names = tuple()
            self.current_posture_names = tuple()
            self.current_motion_names = tuple()
            self.progress = 100.0
            passed = sum(result.passed for result in self.results)
            self.state = "检测完成"
            self.detail = "%d/%d 个关节通过" % (passed, len(self.results))
            self._event("info", self.detail)
            return
        self._begin_joint_locked(now)

    def _publish_locked(self):
        if not self.selected_indices:
            return
        kp = np.zeros(len(JOINT_NAMES), dtype=float)
        kd = np.zeros(len(JOINT_NAMES), dtype=float)
        scale = 1.0
        if self.reset_step == -2:
            scale = min((time.monotonic() - self.init_started_at) /
                        self.initialization_sec, 1.0)
            if scale < 1.0:
                # Do not send reset step 2 until the soft-start ramp completes.
                kp[list(self.controlled_indices)] = JOINT_KP[list(self.controlled_indices)] * scale
                kd[list(self.controlled_indices)] = JOINT_KD[list(self.controlled_indices)]
                self.progress = scale * 100.0
                self._publish_message_locked(kp, kd)
                return
            if self.reset_future is None:
                self._send_reset_locked(2)
            return
        kp[list(self.controlled_indices)] = JOINT_KP[list(self.controlled_indices)]
        kd[list(self.controlled_indices)] = JOINT_KD[list(self.controlled_indices)]
        self._publish_message_locked(kp, kd)

    def _publish_message_locked(self, kp, kd):
        if self.collision_guard is not None:
            collisions = self.collision_guard.collisions(self.command)
            if collisions:
                geom_1, geom_2, distance = collisions[0]
                self.emergency_stop(
                    "预测到模型碰撞：%s ↔ %s（侵入 %.1f mm）" %
                    (geom_1, geom_2, -1000.0 * distance)
                )
                return
            if not self.hardware_mode:
                floor_hits = self.collision_guard.foot_floor_interference(
                    self.command)
                if floor_hits:
                    side, distance = floor_hits[0]
                    self.emergency_stop(
                        "预测到%s脚与仿真地面干涉（侵入 %.1f mm）" %
                        ("左" if side == "l" else "右", -1000.0 * distance)
                    )
                    return
        if (self.collision_guard is not None and
                (self.test_running or self.returning) and
                self.current_motion_names):
            # Check often enough that the maximum travel between mesh checks
            # stays below half of the configured collision-angle margin.
            visual_period = min(
                0.05,
                self.settings.collision_margin_deg /
                (2.0 * self.settings.range_speed_deg_s),
            )
            now = time.monotonic()
            if now - self.last_visual_collision_check_at >= visual_period:
                self.last_visual_collision_check_at = now
                for active_name in self.current_motion_names:
                    visual_hits = self.collision_guard.visual_mesh_collisions(
                        self.command, active_name)
                    if visual_hits:
                        body_1, body_2, distance = visual_hits[0]
                        self.emergency_stop(
                            "预测到视觉外壳侵入：%s ↔ %s（侵入 %.1f mm）" %
                            (body_1, body_2, -1000.0 * distance)
                        )
                        return
        message = bxi_msg.ActuatorCmds()
        message.header.frame_id = "elf3"
        message.header.stamp = self.get_clock().now().to_msg()
        message.actuators_name = list(JOINT_NAMES)
        message.pos = self.command.tolist()
        message.vel = np.zeros(len(JOINT_NAMES)).tolist()
        message.torque = np.zeros(len(JOINT_NAMES)).tolist()
        message.kp = kp.tolist()
        message.kd = kd.tolist()
        self.publisher.publish(message)
        self.last_publish_at = time.monotonic()
