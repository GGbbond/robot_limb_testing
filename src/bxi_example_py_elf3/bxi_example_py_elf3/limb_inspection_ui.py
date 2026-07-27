"""PyQt5 desktop application for Elf3 limb bench inspection."""

import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from threading import Thread

# The embedded viewport is an offscreen renderer.  GLFW/GLX context creation
# is unreliable when the desktop is using an incompatible GLX visual, while
# EGL does not depend on the Qt window's X11 visual.  This must be selected
# before the controller imports MuJoCo through the collision guard.
os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pyqtgraph as pg
import rclpy
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QScrollArea, QSplitter,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from .limb_inspection_controller import LimbInspectionController
from .limb_inspection_config import (
    DEFAULT_REPORT_DIRECTORY, PARAMETER_INPUT_MAX, load_settings, save_settings,
)
from .limb_gamepad import (
    XBOX_BUTTON_ACTIONS, XBOX_DPAD_THRESHOLD, XBOX_DPAD_X_AXIS,
    XboxGamepadReader,
)
from .limb_hardware_preflight import fpga_canfd_available
from .limb_inspection_core import (
    JOINT_LABELS, JOINT_NAMES, InspectionSettings,
    selected_feedback_summary, selected_joints,
)
from .limb_inspection_report import export_report
from .limb_simulation_view import SimulationViewport


APP_TITLE = "BXI 机器人四肢检测台"


class NoWheelDoubleSpinBox(QDoubleSpinBox):
    """Keep page scrolling from accidentally changing numeric parameters."""

    def wheelEvent(self, event):
        event.ignore()


def _spin_executor(executor):
    """Stop quietly when ROS invalidates its context during SIGINT."""
    try:
        executor.spin()
    except Exception:
        if rclpy.ok():
            raise


class LimbInspectionWindow(QMainWindow):
    def __init__(self, controller, executor):
        super().__init__()
        self.controller = controller
        self.executor = executor
        self.settings_data = load_settings()
        self.last_result_count = 0
        self.last_complete_signature = None
        self.plot_time = []
        self.plot_command = []
        self.plot_position = []
        self.plot_command_peer = []
        self.plot_position_peer = []
        self.current_plot_group = tuple()
        self.simulation_view = None
        self.production_table_names = tuple()
        self.result_recipe = tuple()
        self.initialized_recipe = tuple()
        self.external_shutdown = False
        self.mode_switch_file = os.environ.get("BXI_LIMB_MODE_SWITCH_FILE", "")
        self.startup_warning = os.environ.get("BXI_LIMB_STARTUP_WARNING", "")
        self.gamepad_reader = None
        self.gamepad_enabled = False
        self.last_initialized_state = bool(
            self.controller.snapshot()["initialized"])
        self.ui_mode = self.settings_data.get("ui_mode", "production")
        if self.ui_mode not in ("production", "debug"):
            self.ui_mode = "production"
        self.plot_effort = []
        self.setWindowTitle(APP_TITLE + ("（实机）" if controller.hardware_mode else "（仿真）"))
        self.resize(1400, 820)
        self.setMinimumSize(1040, 680)
        self._build_ui()
        self._apply_theme()
        self.set_ui_mode(self.ui_mode, persist=False)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)
        self.simulation_timer = None
        if self.simulation_view is not None:
            self.simulation_timer = QTimer(self)
            self.simulation_timer.timeout.connect(self._refresh_simulation)
            self.simulation_timer.start(50)
        self.gamepad_timer = QTimer(self)
        self.gamepad_timer.timeout.connect(self._poll_gamepad)
        self.gamepad_timer.start(100)
        if self.startup_warning:
            QTimer.singleShot(0, self._show_startup_warning)

    def _apply_theme(self):
        self.setStyleSheet("""
            QWidget { background:#16181c; color:#eef1f5; font-size:10pt; }
            QGroupBox { border:1px solid #323844; border-radius:6px;
                        margin-top:16px; padding:8px 6px 6px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 5px;
                               color:#d9dee7; }
            QPushButton { background:#2d3542; border:1px solid #3d4655;
                          border-radius:5px; min-height:30px; padding:3px 9px; }
            QPushButton:hover { background:#3a4454; }
            QPushButton:disabled { color:#6f7785; background:#242932; }
            QComboBox,QDoubleSpinBox,QSpinBox { background:#101318;
                border:1px solid #3d4655; border-radius:4px; min-height:28px;
                padding:2px 6px; }
            QTableWidget { background:#0d1117; gridline-color:#303846;
                           alternate-background-color:#151b23; }
            QHeaderView::section { background:#252b35; padding:5px;
                                   border:1px solid #303846; }
            QProgressBar { background:#101318; border:1px solid #303846;
                           border-radius:4px; min-height:22px; text-align:center; }
            QProgressBar::chunk { background:#2f6fed; border-radius:3px; }
            QLabel[role='value'] { color:#75a3ff; font-size:12pt; font-weight:600; }
            QLabel[role='muted'] { color:#98a2b3; }
            QLabel[role='mode'] { color:#47d18c; font-size:12pt; font-weight:700; }
            QLabel[role='productionResult'] { font-size:26pt; font-weight:800;
                                               padding:8px 16px; }
        """)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        header = QHBoxLayout()
        title = QLabel(APP_TITLE)
        title.setStyleSheet("font-size:17pt; font-weight:800; color:#f3f6fb;")
        self.ui_mode_label = QLabel()
        self.ui_mode_label.setProperty("role", "mode")
        self.ui_mode_button = QPushButton()
        self.ui_mode_button.setMinimumWidth(130)
        self.ui_mode_button.clicked.connect(self.toggle_ui_mode)
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.ui_mode_label)
        header.addSpacing(12)
        header.addWidget(self.ui_mode_button)
        layout.addLayout(header)

        self.main_splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(self.main_splitter, 1)
        self.main_splitter.addWidget(self._control_panel())
        self.main_splitter.addWidget(self._monitor_panel())
        self.main_splitter.setSizes([390, 1010])

    def _control_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        self.status_group = QGroupBox("系统状态")
        status_layout = QGridLayout(self.status_group)
        status_layout.addWidget(QLabel("运行环境"), 0, 0)
        self.mode_label = QLabel("实机硬件" if self.controller.hardware_mode else "MuJoCo 仿真")
        self.mode_label.setProperty("role", "value")
        status_layout.addWidget(self.mode_label, 0, 1)
        status_layout.addWidget(QLabel("控制器"), 1, 0)
        self.state_label = QLabel("启动中")
        self.state_label.setProperty("role", "value")
        status_layout.addWidget(self.state_label, 1, 1)
        status_layout.addWidget(QLabel("反馈"), 2, 0)
        self.feedback_label = QLabel("等待 joint_states")
        status_layout.addWidget(self.feedback_label, 2, 1)
        status_layout.addWidget(QLabel("当前关节"), 3, 0)
        self.current_joint_label = QLabel("-")
        self.current_joint_label.setProperty("role", "value")
        self.current_joint_label.setWordWrap(True)
        status_layout.addWidget(self.current_joint_label, 3, 1)
        self.detail_label = QLabel("-")
        self.detail_label.setWordWrap(True)
        self.detail_label.setProperty("role", "muted")
        status_layout.addWidget(self.detail_label, 4, 0, 1, 2)
        self.simulation_debug_label = QLabel(
            "实机反馈驱动只读数字孪生视图"
            if self.controller.hardware_mode else
            "仿真与实机使用相同参数和安全阈值")
        self.simulation_debug_label.setProperty("role", "muted")
        status_layout.addWidget(self.simulation_debug_label, 5, 0, 1, 2)
        layout.addWidget(self.status_group)

        self.debug_tools_group = QGroupBox("调试工具")
        debug_tools = QGridLayout(self.debug_tools_group)
        self.mode_switch_button = QPushButton(
            "切换到 MuJoCo 仿真" if self.controller.hardware_mode
            else "切换到实机模式")
        self.mode_switch_button.clicked.connect(self._request_mode_switch)
        if not self.mode_switch_file:
            self.mode_switch_button.setEnabled(False)
            self.mode_switch_button.setToolTip(
                "请通过 scripts/run_limb_inspection.sh 启动后使用模式切换")
        self.gamepad_button = QPushButton("启用 Xbox 手柄")
        self.gamepad_button.clicked.connect(self._toggle_gamepad)
        self.gamepad_status_label = QLabel("关闭")
        self.gamepad_status_label.setWordWrap(True)
        self.gamepad_status_label.setProperty("role", "muted")
        self.gamepad_help_label = QLabel(
            "按键：A 初始化｜X 开始检测｜B 平稳停止｜Y 紧急停止\n"
            "方向键：← 选择手臂｜→ 选择腿（初始化、检测、回中时锁定）")
        self.gamepad_help_label.setWordWrap(True)
        self.gamepad_help_label.setProperty("role", "muted")
        debug_tools.addWidget(self.mode_switch_button, 0, 0)
        debug_tools.addWidget(self.gamepad_button, 0, 1)
        debug_tools.addWidget(self.gamepad_status_label, 1, 0, 1, 2)
        debug_tools.addWidget(self.gamepad_help_label, 2, 0, 1, 2)
        layout.addWidget(self.debug_tools_group)

        self.object_group = QGroupBox("检测对象")
        form = QFormLayout(self.object_group)
        self.limb_combo = QComboBox()
        self.limb_combo.addItem("手臂", "arm")
        self.limb_combo.addItem("腿", "leg")
        self.side_combo = QComboBox()
        self.side_combo.addItem("左右两侧（对应关节同时）", "both_simultaneous")
        self.side_combo.addItem("仅左侧", "left")
        self.side_combo.addItem("仅右侧", "right")
        self._select_data(self.limb_combo, self.settings_data.get("limb", "arm"))
        configured_side = self.settings_data.get("side", "left")
        if configured_side == "both":
            configured_side = "both_simultaneous"
        self._select_data(self.side_combo, configured_side)
        form.addRow("部位", self.limb_combo)
        form.addRow("台架侧", self.side_combo)
        layout.addWidget(self.object_group)

        self.motion_group = QGroupBox("运动参数")
        motion = QFormLayout(self.motion_group)
        self.move_sec = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "move_sec", 2.0, " s")
        self.hold_sec = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "hold_sec", 0.5, " s")
        self.range_speed = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "range_speed_deg_s", 10.0, " °/s")
        self.collision_margin = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "collision_margin_deg", 5.0, " °")
        self.mechanical_margin = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "mechanical_margin_deg", 2.0, " °")
        motion.addRow("最短单程时间", self.move_sec)
        motion.addRow("全行程最高速度", self.range_speed)
        motion.addRow("模型碰撞余量", self.collision_margin)
        motion.addRow("机械限位余量", self.mechanical_margin)
        motion.addRow("端点保持", self.hold_sec)
        layout.addWidget(self.motion_group)

        self.limits_group = QGroupBox("合格判定")
        limits = QFormLayout(self.limits_group)
        self.tracking = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "tracking_tolerance_deg", 2.0, " °")
        self.response = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "minimum_motion_ratio", 0.6, "")
        self.cross = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "cross_axis_limit_deg", 3.0, " °")
        self.max_velocity = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "max_velocity_deg_s", 30.0, " °/s")
        self.max_effort = self._double(
            -PARAMETER_INPUT_MAX, PARAMETER_INPUT_MAX,
            "max_effort_nm", 80.0, " Nm")
        limits.addRow("最大跟踪误差", self.tracking)
        limits.addRow("最小运动比例", self.response)
        limits.addRow("最大关节串扰", self.cross)
        limits.addRow("最大速度", self.max_velocity)
        limits.addRow("最大力矩", self.max_effort)
        layout.addWidget(self.limits_group)

        self.safety_check = QCheckBox("台架已固定、运动区无人、急停可用")
        self.safety_check.setStyleSheet("QCheckBox { color:#ffcc66; font-weight:600; }")
        layout.addWidget(self.safety_check)
        self.full_range_check = QCheckBox("已确认线缆、夹具和全行程范围无干涉")
        self.full_range_check.setStyleSheet(
            "QCheckBox { color:#ff9f43; font-weight:600; }")
        layout.addWidget(self.full_range_check)
        buttons = QGridLayout()
        self.init_button = QPushButton("1. 初始化机器人")
        self.start_button = QPushButton("2. 一键检测")
        self.stop_button = QPushButton("平稳停止")
        self.estop_button = QPushButton("紧急停止")
        self.estop_button.setStyleSheet("QPushButton { background:#a52727; border-color:#ef6666; font-weight:700; }")
        self.export_button = QPushButton("导出报告")
        buttons.addWidget(self.init_button, 0, 0)
        buttons.addWidget(self.start_button, 0, 1)
        buttons.addWidget(self.stop_button, 1, 0)
        buttons.addWidget(self.estop_button, 1, 1)
        buttons.addWidget(self.export_button, 2, 0, 1, 2)
        layout.addLayout(buttons)
        self.progress = QProgressBar()
        layout.addWidget(self.progress)
        self.init_button.clicked.connect(self._initialize)
        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(lambda: self.controller.stop_test())
        self.estop_button.clicked.connect(self._estop)
        self.export_button.clicked.connect(self._choose_export)
        layout.addStretch(1)
        self.control_scroll = QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setWidget(panel)
        self.control_scroll.setMinimumWidth(380)
        return self.control_scroll

    def _monitor_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        self.plot = pg.PlotWidget(title="当前关节：目标位置 / 实际位置")
        self.plot.setBackground("#0d1117")
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setLabel("left", "角度", units="deg")
        self.plot.setLabel("bottom", "时间", units="s")
        self.command_curve = self.plot.plot(pen=pg.mkPen("#75a3ff", width=2), name="目标")
        self.position_curve = self.plot.plot(pen=pg.mkPen("#47d18c", width=2), name="实际")
        self.command_peer_curve = self.plot.plot(
            pen=pg.mkPen("#b388ff", width=2, style=Qt.DashLine),
            name="右侧目标")
        self.position_peer_curve = self.plot.plot(
            pen=pg.mkPen("#ffb86c", width=2, style=Qt.DashLine),
            name="右侧实际")
        self.plot.addLegend()
        self.top_splitter = QSplitter(Qt.Horizontal)
        self.top_splitter.addWidget(self.plot)
        view_title = (
            "MuJoCo 实机同步视图（只读）"
            if self.controller.hardware_mode else "MuJoCo 实时仿真")
        self.visual_group = QGroupBox(
            view_title + "｜左键旋转  右键平移  滚轮缩放  双击复位")
        simulation_layout = QVBoxLayout(self.visual_group)
        self.simulation_view = SimulationViewport(
            self.controller.simulation_bench_height_m)
        simulation_layout.addWidget(self.simulation_view)
        self.top_splitter.addWidget(self.visual_group)
        self.top_splitter.setSizes([520, 520])
        layout.addWidget(self.top_splitter, 3)

        self.production_result_group = QGroupBox("通过情况")
        production_result_layout = QVBoxLayout(self.production_result_group)
        production_header = QHBoxLayout()
        self.production_result_label = QLabel("未检测")
        self.production_result_label.setAlignment(Qt.AlignCenter)
        self.production_result_label.setProperty("role", "productionResult")
        self.production_result_detail = QLabel("等待初始化")
        self.production_result_detail.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.production_result_detail.setProperty("role", "muted")
        production_header.addWidget(self.production_result_label)
        production_header.addStretch(1)
        production_header.addWidget(self.production_result_detail)
        production_result_layout.addLayout(production_header)
        self.production_result_table = QTableWidget(0, 3)
        self.production_result_table.setHorizontalHeaderLabels([
            "关节", "判定", "说明",
        ])
        self.production_result_table.setAlternatingRowColors(True)
        self.production_result_table.verticalHeader().hide()
        self.production_result_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.production_result_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeToContents)
        self.production_result_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeToContents)
        self.production_result_table.horizontalHeader().setStretchLastSection(True)
        production_result_layout.addWidget(self.production_result_table)
        layout.addWidget(self.production_result_group, 2)

        self.result_group = QGroupBox("逐关节检测结果")
        result_layout = QVBoxLayout(self.result_group)
        self.result_table = QTableWidget(0, 8)
        self.result_table.setHorizontalHeaderLabels([
            "关节", "结果", "目标范围/°", "实测范围/°", "误差/°",
            "串扰/°", "峰值力矩/Nm", "说明",
        ])
        self.result_table.setAlternatingRowColors(True)
        self.result_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.result_table.horizontalHeader().setStretchLastSection(True)
        result_layout.addWidget(self.result_table)
        layout.addWidget(self.result_group, 2)

        self.log_group = QGroupBox("运行记录")
        log_layout = QVBoxLayout(self.log_group)
        self.log_table = QTableWidget(0, 2)
        self.log_table.setHorizontalHeaderLabels(["时间", "事件"])
        self.log_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.log_table.horizontalHeader().setStretchLastSection(True)
        self.log_table.verticalHeader().hide()
        log_layout.addWidget(self.log_table)
        layout.addWidget(self.log_group, 1)
        return panel

    def toggle_ui_mode(self):
        self.set_ui_mode("debug" if self.ui_mode == "production" else "production")

    def set_ui_mode(self, mode, persist=True):
        """Switch presentation only; controller and safety behavior stay unchanged."""
        self.ui_mode = "debug" if mode == "debug" else "production"
        production = self.ui_mode == "production"
        if production and self.gamepad_enabled:
            self._disable_gamepad()
        self.ui_mode_label.setText("生产模式" if production else "调试模式")
        self.ui_mode_label.setStyleSheet(
            "color:#47d18c;" if production else "color:#75a3ff;")
        self.ui_mode_button.setText(
            "进入调试模式" if production else "进入生产模式")
        self.motion_group.setVisible(not production)
        self.limits_group.setVisible(not production)
        self.simulation_debug_label.setVisible(not production)
        self.debug_tools_group.setVisible(not production)
        self.export_button.setVisible(not production)
        self.plot.setVisible(not production)
        self.production_result_group.setVisible(production)
        self.result_group.setVisible(not production)
        self.log_group.setVisible(not production)
        self.init_button.setMinimumHeight(48 if production else 30)
        self.start_button.setMinimumHeight(48 if production else 30)
        self.stop_button.setMinimumHeight(42 if production else 30)
        self.estop_button.setMinimumHeight(42 if production else 30)
        self.main_splitter.setSizes([360, 1040] if production else [390, 1010])
        self.top_splitter.setSizes([0, 1000] if production else [520, 520])
        if persist:
            self.settings_data["ui_mode"] = self.ui_mode
            try:
                self._save_settings()
            except Exception as exc:
                QMessageBox.warning(self, "界面模式保存失败", str(exc))

    def _double(self, low, high, key, default, suffix):
        widget = NoWheelDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(3)
        widget.setValue(float(self.settings_data.get(key, default)))
        widget.setSuffix(suffix)
        widget.setKeyboardTracking(False)
        return widget

    @staticmethod
    def _select_data(combo, value):
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _settings(self):
        settings = InspectionSettings(
            limb=self.limb_combo.currentData(), side=self.side_combo.currentData(),
            move_sec=self.move_sec.value(), hold_sec=self.hold_sec.value(),
            collision_margin_deg=self.collision_margin.value(),
            mechanical_margin_deg=self.mechanical_margin.value(),
            range_speed_deg_s=self.range_speed.value(),
            full_range_confirmed=self.full_range_check.isChecked(),
            tracking_tolerance_deg=self.tracking.value(),
            minimum_motion_ratio=self.response.value(),
            cross_axis_limit_deg=self.cross.value(),
            max_velocity_deg_s=self.max_velocity.value(),
            max_effort_nm=self.max_effort.value(),
        )
        settings.validate()
        return settings

    def _save_settings(self):
        settings = self._settings()
        data = dict(settings.__dict__)
        data.pop("full_range_confirmed", None)
        data.update({
            "ui_mode": self.ui_mode,
            "control_rate_hz": self.controller.control_rate_hz,
            "feedback_timeout_sec": self.controller.feedback_timeout_sec,
            "max_command_gap_sec": self.controller.max_command_gap_sec,
            "initialization_sec": self.controller.initialization_sec,
            "simulation_bench_height_m": self.controller.simulation_bench_height_m,
            "report_directory": self.settings_data.get(
                "report_directory", DEFAULT_REPORT_DIRECTORY),
        })
        save_settings(data)

    def _initialize(self):
        try:
            settings = self._settings()
            if not self.safety_check.isChecked():
                raise RuntimeError("请先完成并勾选台架安全确认")
            self._save_settings()
            self.controller.request_initialize(settings)
            self.initialized_recipe = selected_joints(settings.limb, settings.side)
            self.last_initialized_state = False
        except Exception as exc:
            QMessageBox.critical(self, "无法初始化", str(exc))

    def _start(self):
        try:
            if not self.safety_check.isChecked():
                raise RuntimeError("请先完成并勾选台架安全确认")
            settings = self._settings()
            if not self.full_range_check.isChecked():
                raise RuntimeError("请确认线缆、夹具和全行程范围无干涉")
            self._save_settings()
            self.plot_time.clear()
            self.plot_command.clear()
            self.plot_position.clear()
            self.last_complete_signature = None
            self.controller.start_test(settings)
            self.result_recipe = selected_joints(settings.limb, settings.side)
        except Exception as exc:
            QMessageBox.critical(self, "无法开始检测", str(exc))

    def _estop(self):
        self.controller.emergency_stop()
        QMessageBox.critical(
            self, "急停已锁定",
            "控制器已停止发送命令。请切断动力、排查机械和电气状态；本次软件必须重启后才能再次初始化。")

    def _request_mode_switch(self):
        if not self.mode_switch_file:
            QMessageBox.warning(
                self, "无法切换模式",
                "当前不是通过统一启动脚本运行，请关闭后使用 "
                "scripts/run_limb_inspection.sh 启动。")
            return
        target = "simulation" if self.controller.hardware_mode else "hardware"
        if target == "hardware" and not fpga_canfd_available():
            QMessageBox.warning(
                self, "FPGA 未连接",
                "未检测到 Xilinx PCI CAN-FD 设备 10ee:7022，"
                "已保持当前仿真模式，软件不会退出。\n\n"
                "请关闭机器人动力，检查 FPGA 板卡、PCIe 插槽和主控连接后重试。")
            return
        warning = (
            "将停止当前程序并切换到实机模式。\n"
            "实机驱动启动时会设置 motor_pwr=True，请确认机器人已固定、"
            "运动区无人且物理急停可用。"
            if target == "hardware" else
            "将停止实机驱动并切换到 MuJoCo 仿真模式。")
        if QMessageBox.question(
                self, "确认切换运行环境", warning,
                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        try:
            self._save_settings()
            Path(self.mode_switch_file).write_text(
                target + "\n", encoding="ascii")
        except Exception as exc:
            QMessageBox.critical(self, "模式切换失败", str(exc))
            return
        snap = self.controller.snapshot()
        if snap["initialized"] or snap["test_running"] or snap["returning"]:
            self.controller.emergency_stop("正在切换运行环境")
        self.external_shutdown = True
        self.close()

    def _toggle_gamepad(self):
        if self.gamepad_enabled:
            self._disable_gamepad()
            return
        device = os.environ.get("BXI_GAMEPAD_DEVICE", "/dev/input/js0")
        if not os.path.exists(device):
            QMessageBox.warning(
                self, "手柄未连接",
                "没有找到手柄设备 %s，手柄功能未启用，软件继续运行。\n\n"
                "请连接手柄后重试；可用 jstest %s 核对设备。" %
                (device, device))
            return
        if not os.access(device, os.R_OK):
            QMessageBox.warning(
                self, "无法读取手柄",
                "当前用户没有读取 %s 的权限，手柄功能未启用，"
                "软件继续运行。" % device)
            return
        self.gamepad_reader = XboxGamepadReader(device)
        self.gamepad_reader.start()
        self.gamepad_enabled = True
        self.safety_check.setChecked(True)
        self.full_range_check.setChecked(True)
        self.gamepad_button.setText("关闭 Xbox 手柄")
        self.gamepad_status_label.setText(
            "正在连接 %s…｜已自动确认台架与全行程检查" % device)

    def _disable_gamepad(self):
        if self.gamepad_reader is not None:
            self.gamepad_reader.stop()
        self.gamepad_reader = None
        self.gamepad_enabled = False
        self.safety_check.setChecked(False)
        self.full_range_check.setChecked(False)
        self.gamepad_button.setText("启用 Xbox 手柄")
        self.gamepad_status_label.setStyleSheet("")
        self.gamepad_status_label.setText("关闭")

    def _select_limb_from_gamepad(self, limb):
        snap = self.controller.snapshot()
        if (snap["test_running"] or snap["returning"] or
                bool(self.controller.reset_step)):
            self.gamepad_status_label.setText(
                "初始化、检测或回中进行中，暂不能切换手臂/腿")
            return
        index = self.limb_combo.findData(limb)
        if index >= 0:
            self.limb_combo.setCurrentIndex(index)
            name = "手臂" if limb == "arm" else "腿"
            self.gamepad_status_label.setText("已选择%s测试" % name)

    def _poll_gamepad(self):
        if self.gamepad_reader is None:
            return
        for event in self.gamepad_reader.drain_events():
            if event[0] == "status":
                if event[1]:
                    self.gamepad_status_label.setText(
                        "已连接 %s" % event[2])
                    self.gamepad_status_label.setStyleSheet("color:#47d18c;")
                else:
                    self.gamepad_status_label.setText(
                        "等待手柄：%s" % event[2])
                    self.gamepad_status_label.setStyleSheet("color:#ffcc66;")
                    QMessageBox.warning(
                        self, "手柄连接中断",
                        "手柄无法读取或已断开：%s\n\n"
                        "手柄功能已关闭，软件继续运行。" % event[2])
                    self._disable_gamepad()
                    break
                continue
            if event[0] == "rumble":
                self.gamepad_status_label.setText(
                    "初始化完成，手柄已振动提醒｜X 开始检测")
                self.gamepad_status_label.setStyleSheet("color:#47d18c;")
                continue
            _kind, number, pressed = event
            if _kind == "axis":
                if number == XBOX_DPAD_X_AXIS:
                    if pressed <= -XBOX_DPAD_THRESHOLD:
                        self._select_limb_from_gamepad("arm")
                    elif pressed >= XBOX_DPAD_THRESHOLD:
                        self._select_limb_from_gamepad("leg")
                continue
            if not pressed:
                continue
            action = XBOX_BUTTON_ACTIONS.get(number)
            if action == "initialize" and self.init_button.isEnabled():
                self.init_button.click()
            elif action == "start" and self.start_button.isEnabled():
                self.start_button.click()
            elif action == "stop" and self.stop_button.isEnabled():
                self.stop_button.click()
            elif action == "emergency_stop":
                self.estop_button.click()

    def _show_startup_warning(self):
        QMessageBox.warning(self, "设备预检未通过", self.startup_warning)

    def _refresh(self):
        snap = self.controller.snapshot()
        initialized_now = bool(snap["initialized"])
        if (self.gamepad_enabled and initialized_now and
                not self.last_initialized_state and
                self.gamepad_reader is not None):
            self.gamepad_reader.rumble(300)
        self.last_initialized_state = initialized_now
        self.state_label.setText(snap["state"])
        self.detail_label.setText(snap["detail"])
        current_names = tuple(snap.get("current_joints") or ())
        if not current_names and snap.get("current_joint"):
            current_names = (snap["current_joint"],)
        self.current_joint_label.setText(
            " + ".join(JOINT_LABELS.get(name, name) for name in current_names)
            if current_names else "-")
        self.progress.setValue(int(round(snap["progress"])))
        selected_recipe = selected_joints(
            self.limb_combo.currentData(), self.side_combo.currentData())
        fresh_count, selected_count, worst_age = selected_feedback_summary(
            selected_recipe, snap["seen"], snap["feedback_at"],
            self.controller.feedback_timeout_sec)
        worst_text = "未收到" if worst_age is None else "%.3fs" % worst_age
        self.feedback_label.setText(
            "所选反馈 %d/%d，最差 %s" % (
                fresh_count, selected_count, worst_text))
        self.feedback_label.setStyleSheet(
            "color:#47d18c" if fresh_count == selected_count
            else "color:#ff6b6b")
        locked = snap["test_running"] or snap["returning"] or bool(self.controller.reset_step)
        for widget in (
                self.limb_combo, self.side_combo,
                self.move_sec, self.hold_sec,
                self.range_speed, self.collision_margin, self.mechanical_margin,
                self.full_range_check):
            widget.setEnabled(not locked)
        self.init_button.setEnabled(not locked and not snap["fault_latched"])
        selection_initialized = self.initialized_recipe == selected_recipe
        can_start = (snap["initialized"] and selection_initialized and not locked and
                     not snap["fault_latched"])
        self.start_button.setEnabled(can_start)
        if snap["fault_latched"]:
            self.start_button.setText("2. 一键检测（故障锁定）")
            self.start_button.setToolTip("安全故障已锁定，请排查后重启软件")
        elif self.controller.reset_step:
            self.start_button.setText("2. 一键检测（初始化中）")
            self.start_button.setToolTip("等待两阶段初始化完成")
        elif snap["returning"]:
            self.start_button.setText("2. 一键检测（正在回中）")
            self.start_button.setToolTip("等待关节平稳返回检测中心")
        elif snap["test_running"]:
            self.start_button.setText("2. 检测运行中")
            self.start_button.setToolTip("")
        elif snap["initialized"] and not selection_initialized:
            self.start_button.setText("2. 一键检测（请重新初始化）")
            self.start_button.setToolTip("检测对象已改变，请先重新初始化所选关节")
        elif not snap["initialized"]:
            self.start_button.setText("2. 一键检测（请先初始化）")
            self.start_button.setToolTip("收到所选关节反馈后，先点击“初始化机器人”")
        else:
            self.start_button.setText("2. 一键检测")
            self.start_button.setToolTip("")
        self.stop_button.setEnabled(snap["test_running"])
        self.export_button.setEnabled(bool(snap["results"]))
        self._refresh_production_results(snap)
        if self.ui_mode == "debug":
            self._refresh_plot(snap)
            self._refresh_results(snap["results"])
        for timestamp, level, message in self.controller.drain_events():
            row = self.log_table.rowCount()
            self.log_table.insertRow(row)
            self.log_table.setItem(row, 0, QTableWidgetItem(
                datetime.fromtimestamp(timestamp).strftime("%H:%M:%S")))
            item = QTableWidgetItem(message)
            item.setForeground(QColor({"error": "#ff6b6b", "warning": "#ffcc66"}.get(level, "#dfe7f3")))
            self.log_table.setItem(row, 1, item)
            if self.log_table.rowCount() > 300:
                self.log_table.removeRow(0)
            self.log_table.scrollToBottom()
        signature = (snap["state"], len(snap["results"]))
        if snap["state"] == "检测完成" and signature != self.last_complete_signature:
            self.last_complete_signature = signature
            try:
                directory = Path(os.path.expanduser(self.settings_data.get(
                    "report_directory", DEFAULT_REPORT_DIRECTORY)))
                self._export(directory)
            except Exception as exc:
                QMessageBox.warning(self, "自动保存失败", str(exc))

    def _refresh_production_results(self, snap):
        """Update the production-only verdict without exposing debug metrics."""
        expected = selected_joints(
            self.limb_combo.currentData(), self.side_combo.currentData())
        if expected != self.production_table_names:
            self.production_table_names = expected
            self.production_result_table.setRowCount(len(expected))
            for row, name in enumerate(expected):
                self.production_result_table.setItem(
                    row, 0, QTableWidgetItem(JOINT_LABELS[name]))
                self.production_result_table.setItem(row, 1, QTableWidgetItem("未检测"))
                self.production_result_table.setItem(row, 2, QTableWidgetItem("-"))

        recipe_matches = not snap["results"] or self.result_recipe == expected
        result_by_name = {
            result.joint_name: result
            for result in snap["results"]
            if recipe_matches
        }
        current_names = set(snap.get("current_joints") or ())
        for row, name in enumerate(expected):
            result = result_by_name.get(name)
            if result is not None:
                verdict = "通过" if result.passed else "不通过"
                detail = result.reason
                color = "#47d18c" if result.passed else "#ff6b6b"
            elif name in current_names and snap["test_running"]:
                verdict, detail, color = "检测中", "关节正在运动", "#75a3ff"
            else:
                verdict, detail, color = "未检测", "-", "#98a2b3"
            verdict_item = self.production_result_table.item(row, 1)
            detail_item = self.production_result_table.item(row, 2)
            if verdict_item.text() != verdict:
                verdict_item.setText(verdict)
            verdict_item.setForeground(QColor(color))
            if detail_item.text() != detail:
                detail_item.setText(detail)

        completed = len(result_by_name)
        passed = sum(result.passed for result in result_by_name.values()
                     if result.passed)
        total = len(expected)
        if snap["fault_latched"]:
            verdict, color = "不通过", "#ff6b6b"
            detail = "安全故障已锁定｜通过 %d / %d" % (passed, total)
        elif snap["test_running"]:
            verdict, color = "检测中", "#75a3ff"
            detail = "已完成 %d / %d｜通过 %d" % (completed, total, passed)
        elif snap["returning"]:
            verdict, color = "正在停止", "#ffcc66"
            detail = "已完成 %d / %d｜通过 %d" % (completed, total, passed)
        elif snap["state"] == "检测完成" and recipe_matches and completed == total:
            all_passed = passed == total
            verdict = "通过" if all_passed else "不通过"
            color = "#47d18c" if all_passed else "#ff6b6b"
            detail = "通过 %d / %d" % (passed, total)
        elif snap["initialized"] and self.initialized_recipe != expected:
            verdict, color = "待初始化", "#ffcc66"
            detail = "检测对象已改变，请重新初始化"
        elif snap["initialized"]:
            verdict, color = "待检测", "#ffcc66"
            detail = "初始化完成｜共 %d 个关节" % total
        else:
            verdict, color = "未检测", "#98a2b3"
            detail = "等待初始化｜共 %d 个关节" % total
        self.production_result_label.setText(verdict)
        self.production_result_label.setStyleSheet(
            "color:%s; background:#101318; border:1px solid %s; "
            "border-radius:6px;" % (color, color))
        self.production_result_detail.setText(detail)

    def _refresh_plot(self, snap):
        names = tuple(snap.get("current_joints") or ())
        if not names and snap["current_joint"]:
            names = (snap["current_joint"],)
        if not names:
            return
        if names != self.current_plot_group:
            self.current_plot_group = names
            self.plot_time.clear()
            self.plot_command.clear()
            self.plot_position.clear()
            self.plot_command_peer.clear()
            self.plot_position_peer.clear()
        name = names[0]
        index = JOINT_NAMES.index(name)
        now = time.monotonic()
        self.plot_time.append(now)
        self.plot_command.append(float(np.rad2deg(snap["command"][index])))
        self.plot_position.append(float(np.rad2deg(snap["position"][index])))
        if len(names) > 1:
            peer_index = JOINT_NAMES.index(names[1])
            self.plot_command_peer.append(float(np.rad2deg(
                snap["command"][peer_index])))
            self.plot_position_peer.append(float(np.rad2deg(
                snap["position"][peer_index])))
        if len(self.plot_time) > 600:
            del self.plot_time[:-600]
            del self.plot_command[:-600]
            del self.plot_position[:-600]
            del self.plot_command_peer[:-600]
            del self.plot_position_peer[:-600]
        x = np.asarray(self.plot_time) - self.plot_time[-1]
        self.command_curve.setData(x, self.plot_command)
        self.position_curve.setData(x, self.plot_position)
        if len(names) > 1:
            self.command_peer_curve.setData(x, self.plot_command_peer)
            self.position_peer_curve.setData(x, self.plot_position_peer)
            self.plot.setTitle("%s + %s：同步目标 / 实际位置" % (
                JOINT_LABELS[names[0]], JOINT_LABELS[names[1]]))
        else:
            self.command_peer_curve.setData([], [])
            self.position_peer_curve.setData([], [])
            self.plot.setTitle("%s：目标位置 / 实际位置" % JOINT_LABELS[name])

    def _refresh_simulation(self):
        if self.simulation_view is not None:
            self.simulation_view.set_pose(
                self.controller.visualization_snapshot())

    def _refresh_results(self, results):
        while self.result_table.rowCount() < len(results):
            row = self.result_table.rowCount()
            result = results[row]
            self.result_table.insertRow(row)
            values = [
                result.label, "通过" if result.passed else "不通过",
                "%.1f ～ %.1f" % (result.target_min_deg, result.target_max_deg),
                "%.1f ～ %.1f" % (result.measured_min_deg, result.measured_max_deg),
                "%.2f" % result.max_tracking_error_deg,
                "%.2f" % result.max_cross_axis_deg,
                "%.2f" % result.max_effort_nm, result.reason,
            ]
            color = QColor("#47d18c" if result.passed else "#ff6b6b")
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 1:
                    item.setForeground(color)
                self.result_table.setItem(row, column, item)
        if len(results) == 0 and self.result_table.rowCount():
            self.result_table.setRowCount(0)

    def _choose_export(self):
        default_dir = os.path.expanduser(self.settings_data.get(
            "report_directory", DEFAULT_REPORT_DIRECTORY))
        directory = QFileDialog.getExistingDirectory(self, "选择报告目录", default_dir)
        if directory:
            try:
                paths = self._export(Path(directory))
                QMessageBox.information(self, "导出成功", "\n".join(map(str, paths)))
            except Exception as exc:
                QMessageBox.critical(self, "导出失败", str(exc))

    def _export(self, directory):
        paths = export_report(directory, self.controller, self._settings())
        self.settings_data["report_directory"] = str(directory)
        return paths

    def closeEvent(self, event):
        snap = self.controller.snapshot()
        if snap["test_running"] and not self.external_shutdown:
            answer = QMessageBox.question(
                self, "测试正在运行", "关闭会立即停止控制命令，确定关闭？",
                QMessageBox.Yes | QMessageBox.No)
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        if snap["initialized"]:
            self.controller.emergency_stop("软件窗口关闭")
        if self.simulation_timer is not None:
            self.simulation_timer.stop()
        if self.simulation_view is not None:
            self.simulation_view.shutdown()
        self._disable_gamepad()
        self.gamepad_timer.stop()
        self.refresh_timer.stop()
        event.accept()

    def request_shutdown(self):
        """Close without an operator prompt when the ROS launch is stopping."""
        self.external_shutdown = True
        self.close()


def main(args=None):
    ros_args = args if args is not None else sys.argv
    rclpy.init(args=ros_args)
    controller = None
    executor = None
    spin_thread = None
    try:
        controller = LimbInspectionController()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(controller)
        spin_thread = Thread(
            target=_spin_executor, args=(executor,),
            name="limb-inspection-ros", daemon=True)
        spin_thread.start()
        qt_args = remove_ros_args(args=ros_args)
        app = QApplication(qt_args)
        app.setApplicationName(APP_TITLE)
        window = LimbInspectionWindow(controller, executor)
        signal.signal(
            signal.SIGINT,
            lambda _signum, _frame: QTimer.singleShot(0, window.request_shutdown))
        window.show()
        return app.exec_()
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if controller is not None:
            controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
