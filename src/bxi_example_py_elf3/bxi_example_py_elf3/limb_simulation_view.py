"""Process-isolated interactive MuJoCo viewport for the inspection UI."""

import math
from multiprocessing import get_context
import os
from pathlib import Path
from threading import Lock
import time

# Also select the reliable offscreen backend when this widget is imported on
# its own.  The application entry point performs the same selection before
# any indirect MuJoCo import, and an explicit user setting still takes
# precedence.
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_path
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel

from .limb_inspection_core import JOINT_NAMES


def model_path():
    filename = (
        "elf3.xml"
        if os.environ.get("BXI_MUJOCO_VIEW_DETAIL", "").strip().lower()
        in ("full", "high", "cad")
        else "elf3_view.xml"
    )
    try:
        return get_package_share_path("bxi_example_py_elf3") / \
            "data" / filename
    except Exception:
        return Path(__file__).resolve().parent.parent / "data" / filename


def _render_process(connection, frame_buffer, bench_height_m):
    """Render requested poses outside the ROS/Qt Python process."""
    renderer = None
    try:
        default_threads = min(8, max(2, (os.cpu_count() or 4) // 2))
        os.environ["LP_NUM_THREADS"] = os.environ.get(
            "BXI_VIEW_RENDER_THREADS", str(default_threads))
        os.environ.setdefault("MESA_GLTHREAD", "false")
        model = mujoco.MjModel.from_xml_path(str(model_path()))
        data = mujoco.MjData(model)
        reference_qpos = data.qpos.copy()
        reference_qpos[2] = float(bench_height_m)
        addresses = {}
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError("MuJoCo 模型缺少关节：" + name)
            addresses[name] = model.jnt_qposadr[joint_id]
        camera = mujoco.MjvCamera()
        scene_option = mujoco.MjvOption()
        scene_option.geomgroup[3] = 0
        # The viewport communicates joint pose, not photorealistic lighting.
        # Disabling shadows, reflections, fog and debug overlays cuts CPU
        # rendering time substantially while keeping the silver robot clear.
        scene_option.flags[:] = 0
        renderer = mujoco.Renderer(model, height=360, width=640)
        connection.send(("ready",))
        shared_frame = np.frombuffer(frame_buffer, dtype=np.uint8)
        while True:
            message = connection.recv()
            if not message or message[0] == "stop":
                break
            if message[0] != "render":
                continue
            _kind, positions, lookat, distance, azimuth, elevation = message
            data.qpos[:] = reference_qpos
            for index, name in enumerate(JOINT_NAMES):
                data.qpos[addresses[name]] = positions[index]
            mujoco.mj_forward(model, data)
            camera.lookat[:] = lookat
            camera.distance = distance
            camera.azimuth = azimuth
            camera.elevation = elevation
            render_started = time.perf_counter()
            renderer.update_scene(
                data, camera=camera, scene_option=scene_option)
            rgb = renderer.render()
            render_ms = 1000.0 * (time.perf_counter() - render_started)
            height, width, _channels = rgb.shape
            copy_started = time.perf_counter()
            np.copyto(shared_frame[:rgb.size], rgb.reshape(-1))
            copy_ms = 1000.0 * (time.perf_counter() - copy_started)
            connection.send((
                "frame", width, height, render_ms, copy_ms))
    except (EOFError, KeyboardInterrupt):
        pass
    except Exception as exc:
        try:
            backend = os.environ.get("MUJOCO_GL", "glfw") or "glfw"
            connection.send((
                "error", "%s（渲染后端：%s）" % (exc, backend)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if renderer is not None:
            renderer.close()
        connection.close()


class SimulationViewport(QLabel):
    """Render feedback without blocking the Qt main thread."""

    frame_ready = pyqtSignal(QImage)
    render_failed = pyqtSignal(str)

    DEFAULT_LOOKAT = np.array((0.0, 0.0, 1.35), dtype=float)
    DEFAULT_DISTANCE = 3.4
    DEFAULT_AZIMUTH = 135.0
    DEFAULT_ELEVATION = -15.0

    def __init__(self, bench_height_m, parent=None, max_fps=15.0):
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
        self._positions = np.zeros(len(JOINT_NAMES), dtype=float)
        self._lookat = self.DEFAULT_LOOKAT.copy()
        self._distance = self.DEFAULT_DISTANCE
        self._azimuth = self.DEFAULT_AZIMUTH
        self._elevation = self.DEFAULT_ELEVATION
        self._drag_mode = None
        self._last_mouse_position = None
        self._request_pending = False
        self._renderer_ready = False
        self._renderer_failed = False
        self.last_render_ms = 0.0
        self.last_copy_ms = 0.0
        self._last_request_at = 0.0
        self.render_failed.connect(self._show_error)
        context = get_context("spawn")
        self._connection, child_connection = context.Pipe()
        self._frame_buffer = context.RawArray("B", 640 * 360 * 3)
        self._process = context.Process(
            target=_render_process,
            args=(child_connection, self._frame_buffer, self.bench_height_m),
            name="limb-mujoco-render", daemon=True)
        self._process.start()
        child_connection.close()
        self._frame_timer = QTimer(self)
        self._frame_timer.timeout.connect(self._exchange_frame)
        self._frame_timer.start(10)

    def set_pose(self, positions):
        values = np.asarray(positions, dtype=float)
        if values.size != len(JOINT_NAMES) or not np.all(np.isfinite(values)):
            return
        with self._state_lock:
            if np.allclose(values, self._positions, rtol=0.0, atol=1.0e-5):
                return
            self._positions = values.copy()

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
        self._change_camera(zoom_steps=event.angleDelta().y() / 120.0)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        self.reset_camera()
        event.accept()

    def _snapshot_state(self):
        with self._state_lock:
            return (
                self._positions.copy(), self._lookat.copy(),
                self._distance, self._azimuth, self._elevation,
            )

    def _exchange_frame(self):
        if self._renderer_failed or self._process is None:
            return
        try:
            while self._connection.poll():
                message = self._connection.recv()
                if message[0] == "ready":
                    self._renderer_ready = True
                elif message[0] == "frame":
                    (_kind, width, height,
                     self.last_render_ms, self.last_copy_ms) = message
                    self._request_pending = False
                    bytes_per_line = 3 * width
                    pixels = np.frombuffer(
                        self._frame_buffer, dtype=np.uint8,
                        count=height * bytes_per_line)
                    image = QImage(
                        pixels.data, width, height, bytes_per_line,
                        QImage.Format_RGB888).copy()
                    self._accept_frame(image)
                    self.frame_ready.emit(image)
                elif message[0] == "error":
                    self._renderer_failed = True
                    self.render_failed.emit(message[1])
            if (not self._process.is_alive() and
                    not self._renderer_failed):
                self._renderer_failed = True
                self.render_failed.emit("渲染进程意外退出")
                return
            now = time.monotonic()
            if (self._renderer_ready and not self._request_pending and
                    now - self._last_request_at >= 1.0 / self.max_fps):
                state = self._snapshot_state()
                self._connection.send(("render",) + state)
                self._request_pending = True
                self._last_request_at = now
        except (BrokenPipeError, EOFError, OSError) as exc:
            self._renderer_failed = True
            self.render_failed.emit("渲染进程通信失败：%s" % exc)

    def _accept_frame(self, image):
        self._image = image
        self._update_pixmap()

    def _show_error(self, message):
        self.setText("MuJoCo 内嵌视图不可用：%s" % message)

    def _update_pixmap(self):
        if self._image is None:
            return
        self.setPixmap(QPixmap.fromImage(self._image).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.FastTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_pixmap()

    def shutdown(self):
        self._frame_timer.stop()
        if self._process is not None:
            try:
                self._connection.send(("stop",))
            except (BrokenPipeError, EOFError, OSError):
                pass
            self._process.join(timeout=3.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
            self._process = None
        self._connection.close()
