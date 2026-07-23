"""Threaded, interactive MuJoCo viewport for the limb inspection UI."""

import math
from pathlib import Path
from threading import Event, Lock, Thread
import time

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_path
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel

from .limb_inspection_core import JOINT_NAMES


def model_path():
    try:
        return get_package_share_path("bxi_example_py_elf3") / \
            "data" / "elf3.xml"
    except Exception:
        return Path(__file__).resolve().parent.parent / "data" / "elf3.xml"


class SimulationViewport(QLabel):
    """Render feedback at up to 30 FPS without blocking the Qt main thread."""

    frame_ready = pyqtSignal(QImage)
    render_failed = pyqtSignal(str)

    DEFAULT_LOOKAT = np.array((0.0, 0.0, 1.35), dtype=float)
    DEFAULT_DISTANCE = 3.4
    DEFAULT_AZIMUTH = 135.0
    DEFAULT_ELEVATION = -15.0

    def __init__(self, bench_height_m, parent=None, max_fps=30.0):
        super().__init__(parent)
        self.bench_height_m = float(bench_height_m)
        self.max_fps = max(1.0, float(max_fps))
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(420, 280)
        self.setText("正在加载 MuJoCo 视图…")
        self.setToolTip(
            "左键拖动：旋转\n右键/中键拖动：平移\n"
            "滚轮：缩放\n双击：恢复默认视角")
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self._image = None
        self._state_lock = Lock()
        self._wake = Event()
        self._stop = Event()
        self._positions = np.zeros(len(JOINT_NAMES), dtype=float)
        self._lookat = self.DEFAULT_LOOKAT.copy()
        self._distance = self.DEFAULT_DISTANCE
        self._azimuth = self.DEFAULT_AZIMUTH
        self._elevation = self.DEFAULT_ELEVATION
        self._version = 1
        self._drag_mode = None
        self._last_mouse_position = None
        self.frame_ready.connect(self._accept_frame)
        self.render_failed.connect(self._show_error)
        self._thread = Thread(
            target=self._render_loop, name="limb-mujoco-render", daemon=True)
        self._thread.start()
        self._wake.set()

    def set_pose(self, positions):
        values = np.asarray(positions, dtype=float)
        if values.size != len(JOINT_NAMES) or not np.all(np.isfinite(values)):
            return
        with self._state_lock:
            self._positions = values.copy()
            self._version += 1
        self._wake.set()

    def camera_state(self):
        with self._state_lock:
            return {
                "lookat": self._lookat.copy(),
                "distance": self._distance,
                "azimuth": self._azimuth,
                "elevation": self._elevation,
            }

    def reset_camera(self):
        with self._state_lock:
            self._lookat = self.DEFAULT_LOOKAT.copy()
            self._distance = self.DEFAULT_DISTANCE
            self._azimuth = self.DEFAULT_AZIMUTH
            self._elevation = self.DEFAULT_ELEVATION
            self._version += 1
        self._wake.set()

    @staticmethod
    def _horizontal_camera_axes(azimuth_deg):
        """Return horizontal view-depth and screen-right unit vectors."""
        azimuth = math.radians(azimuth_deg)
        depth = np.array((math.cos(azimuth), math.sin(azimuth), 0.0))
        right = np.array((-math.sin(azimuth), math.cos(azimuth), 0.0))
        return depth, right

    def _change_camera(self, rotate_x=0.0, rotate_y=0.0,
                       pan_x=0.0, pan_y=0.0, zoom_steps=0.0):
        with self._state_lock:
            self._azimuth = (self._azimuth + rotate_x * 0.35) % 360.0
            self._elevation = float(np.clip(
                self._elevation + rotate_y * 0.35, -89.0, 89.0))
            if pan_x or pan_y:
                scale = self._distance * 0.0015
                _depth, right = self._horizontal_camera_axes(self._azimuth)
                self._lookat += right * pan_x * scale
                self._lookat[2] += pan_y * scale
            if zoom_steps:
                self._distance = float(np.clip(
                    self._distance * math.exp(-0.12 * zoom_steps), 0.35, 20.0))
            self._version += 1
        self._wake.set()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag_mode = (
                "pan" if event.modifiers() & Qt.ShiftModifier else "rotate")
        elif event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._drag_mode = "pan"
        else:
            super().mousePressEvent(event)
            return
        self._last_mouse_position = event.pos()
        event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_mode is None or self._last_mouse_position is None:
            super().mouseMoveEvent(event)
            return
        delta = event.pos() - self._last_mouse_position
        self._last_mouse_position = event.pos()
        if self._drag_mode == "rotate":
            self._change_camera(rotate_x=-delta.x(), rotate_y=-delta.y())
        else:
            self._change_camera(pan_x=delta.x(), pan_y=delta.y())
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.LeftButton, Qt.RightButton, Qt.MiddleButton):
            self._drag_mode = None
            self._last_mouse_position = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        self._change_camera(zoom_steps=-event.angleDelta().y() / 120.0)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        self.reset_camera()
        event.accept()

    def _snapshot_state(self):
        with self._state_lock:
            return (
                self._positions.copy(), self._lookat.copy(),
                self._distance, self._azimuth, self._elevation, self._version,
            )

    def _render_loop(self):
        renderer = None
        try:
            model = mujoco.MjModel.from_xml_path(str(model_path()))
            data = mujoco.MjData(model)
            reference_qpos = data.qpos.copy()
            reference_qpos[2] = self.bench_height_m
            addresses = {}
            for name in JOINT_NAMES:
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, name)
                addresses[name] = model.jnt_qposadr[joint_id]
            camera = mujoco.MjvCamera()
            renderer = mujoco.Renderer(model, height=360, width=640)
            last_version = -1
            last_render_at = 0.0
            minimum_period = 1.0 / self.max_fps
            while not self._stop.is_set():
                self._wake.wait(timeout=0.25)
                self._wake.clear()
                if self._stop.is_set():
                    break
                delay = minimum_period - (time.monotonic() - last_render_at)
                if delay > 0.0 and self._stop.wait(delay):
                    break
                render_started_at = time.monotonic()
                positions, lookat, distance, azimuth, elevation, version = \
                    self._snapshot_state()
                if version == last_version:
                    continue
                data.qpos[:] = reference_qpos
                for index, name in enumerate(JOINT_NAMES):
                    data.qpos[addresses[name]] = positions[index]
                mujoco.mj_forward(model, data)
                camera.lookat[:] = lookat
                camera.distance = distance
                camera.azimuth = azimuth
                camera.elevation = elevation
                renderer.update_scene(data, camera=camera)
                rgb = np.ascontiguousarray(renderer.render())
                height, width, _ = rgb.shape
                image = QImage(
                    rgb.data, width, height, 3 * width,
                    QImage.Format_RGB888).copy()
                self.frame_ready.emit(image)
                last_version = version
                # Schedule by frame start time so rendering work is part of
                # the 33 ms budget instead of being added on top of it.
                last_render_at = render_started_at
        except Exception as exc:
            self.render_failed.emit(str(exc))
        finally:
            if renderer is not None:
                renderer.close()

    def _accept_frame(self, image):
        self._image = image
        self._update_pixmap()

    def _show_error(self, message):
        self.setText("MuJoCo 内嵌视图不可用：%s" % message)

    def _update_pixmap(self):
        if self._image is None:
            return
        self.setPixmap(QPixmap.fromImage(self._image).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def shutdown(self):
        self._stop.set()
        self._wake.set()
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
