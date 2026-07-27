"""Run the vendor MuJoCo simulator on an always-unmapped nested X display."""

import argparse
import ctypes
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix


def _terminate_when_parent_exits(parent_pid):
    """Ask Linux to terminate a spawned backend if this manager disappears."""
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_PDEATHSIG = 1.  Do not leave an unmanaged simulator behind when
    # ros2 launch, the terminal, or this Python manager is killed abruptly.
    if libc.prctl(1, signal.SIGTERM) != 0:
        os._exit(127)
    # The parent may have exited between fork() and prctl().
    if os.getppid() != parent_pid:
        os.kill(os.getpid(), signal.SIGTERM)


def _spawn_owned(command, **kwargs):
    parent_pid = os.getpid()
    return subprocess.Popen(
        command, start_new_session=True,
        preexec_fn=lambda: _terminate_when_parent_exits(parent_pid),
        **kwargs)


def _acquire_instance_lock():
    runtime_dir = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp"))
    path = runtime_dir / ("bxi_limb_hidden_simulation_%d.lock" % os.getuid())
    stream = open(path, "a+", encoding="ascii")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        raise RuntimeError("已经有一套 BXI 隐藏仿真正在运行，请先关闭旧实例")
    return stream


def _residual_vendor_simulations(vendor_path):
    """Return vendor simulator PIDs left behind without their manager."""
    residual = []
    expected = str(Path(vendor_path).resolve())
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
            arguments = [part.decode(errors="replace")
                         for part in raw.split(b"\0") if part]
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if not arguments:
            continue
        try:
            executable = str(Path(arguments[0]).resolve())
        except OSError:
            continue
        if (executable == expected and
                any("__node:=simulation_mujoco" in arg for arg in arguments)):
            residual.append(int(entry.name))
    return tuple(sorted(residual))


class HiddenXDisplay:
    """Own an unmapped host window and a Xephyr server rendered into it."""

    def __init__(self, width=1280, height=720):
        self.width = int(width)
        self.height = int(height)
        self.x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        self._configure_x11()
        self.host_display = None
        self.parent_window = 0
        self.display_number = None
        self.display_name = None
        self.lock_file = None
        self.lock_path = None
        self.xephyr = None

    def _configure_x11(self):
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XCreateSimpleWindow.argtypes = [
            ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
            ctypes.c_uint, ctypes.c_uint, ctypes.c_uint,
            ctypes.c_ulong, ctypes.c_ulong,
        ]
        self.x11.XCreateSimpleWindow.restype = ctypes.c_ulong
        self.x11.XDestroyWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

    def start(self):
        if shutil.which("Xephyr") is None:
            raise RuntimeError("找不到 Xephyr，请安装 xserver-xephyr")
        self.host_display = self.x11.XOpenDisplay(None)
        if not self.host_display:
            raise RuntimeError("无法连接当前 X11 显示环境")
        root = self.x11.XDefaultRootWindow(self.host_display)
        # Never map this parent.  Xephyr and the vendor GLFW window can render
        # normally, but no host window can flash on the operator's desktop.
        self.parent_window = self.x11.XCreateSimpleWindow(
            self.host_display, root, -20000, -20000,
            self.width, self.height, 0, 0, 0)
        self.x11.XFlush(self.host_display)
        self._reserve_display_number()
        command = [
            "Xephyr", self.display_name,
            "-parent", hex(self.parent_window),
            "-screen", "%dx%d" % (self.width, self.height),
            "-noreset", "-nolisten", "tcp",
        ]
        self.xephyr = _spawn_owned(command)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if self.xephyr.poll() is not None:
                raise RuntimeError(
                    "Xephyr 启动失败，退出码 %s" % self.xephyr.returncode)
            nested = self.x11.XOpenDisplay(self.display_name.encode("ascii"))
            if nested:
                self.x11.XCloseDisplay(nested)
                return self.display_name
            time.sleep(0.05)
        raise RuntimeError("等待隐藏 Xephyr 显示环境超时")

    def _reserve_display_number(self):
        for number in range(90, 190):
            if Path("/tmp/.X11-unix/X%d" % number).exists():
                continue
            path = "/tmp/bxi_limb_xdisplay_%d.lock" % number
            stream = open(path, "a+", encoding="ascii")
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                stream.close()
                continue
            self.display_number = number
            self.display_name = ":%d" % number
            self.lock_file = stream
            self.lock_path = path
            return
        raise RuntimeError("没有可用的隐藏 X11 display 编号")

    @staticmethod
    def _stop_process(process, first_signal=signal.SIGTERM):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, first_signal)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)

    def close(self):
        self._stop_process(self.xephyr)
        self.xephyr = None
        if self.host_display and self.parent_window:
            self.x11.XDestroyWindow(self.host_display, self.parent_window)
            self.x11.XFlush(self.host_display)
            self.parent_window = 0
        if self.host_display:
            self.x11.XCloseDisplay(self.host_display)
            self.host_display = None
        if self.lock_file is not None:
            self.lock_file.close()
            self.lock_file = None
        if self.lock_path:
            try:
                os.unlink(self.lock_path)
            except FileNotFoundError:
                pass
            self.lock_path = None


def _vendor_simulation_path():
    prefix = Path(get_package_prefix("mujoco"))
    path = prefix / "lib" / "mujoco" / "simulation"
    if not path.is_file():
        raise RuntimeError("找不到供应商 MuJoCo 仿真程序：%s" % path)
    return path


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Run vendor MuJoCo on a hidden nested display")
    parser.add_argument("--model", required=True)
    options, _ros_arguments = parser.parse_known_args(args)
    model_path = Path(options.model).expanduser().resolve()
    if not model_path.is_file():
        parser.error("model file does not exist: %s" % model_path)

    instance_lock = _acquire_instance_lock()
    vendor_path = _vendor_simulation_path()
    residual = _residual_vendor_simulations(vendor_path)
    if residual:
        instance_lock.close()
        raise RuntimeError(
            "检测到未退出的 MuJoCo 后台进程 PID %s，请先关闭残留实例" %
            ", ".join(map(str, residual)))

    stopped = threading.Event()

    def request_stop(_signum, _frame):
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    hidden_display = HiddenXDisplay()
    simulation = None
    try:
        display_name = hidden_display.start()
        environment = os.environ.copy()
        environment["DISPLAY"] = display_name
        # The vendor window is never shown.  Mesa otherwise creates one
        # llvmpipe worker per CPU and lets invisible drawing starve ROS and Qt
        # on robot PCs.  Two workers leave deterministic headroom for physics
        # and control while remaining portable to machines without a GPU.
        environment["LP_NUM_THREADS"] = environment.get(
            "BXI_VENDOR_RENDER_THREADS", "2")
        environment.setdefault("MESA_GLTHREAD", "false")
        command = [
            str(vendor_path),
            str(model_path),
            "--ros-args", "-r", "__node:=simulation_mujoco",
            "-p", "simulation/model_file:=%s" % model_path,
        ]
        simulation = _spawn_owned(command, env=environment)
        while simulation.poll() is None and not stopped.wait(0.1):
            pass
        if stopped.is_set():
            HiddenXDisplay._stop_process(
                simulation, first_signal=signal.SIGINT)
            return 0
        return int(simulation.returncode or 0)
    finally:
        HiddenXDisplay._stop_process(
            simulation, first_signal=signal.SIGINT)
        hidden_display.close()
        instance_lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
