"""Short CPU-only smoke test for the process-isolated embedded viewport."""

import math
import os
import sys
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from bxi_example_py_elf3.limb_simulation_view import SimulationViewport


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QApplication(sys.argv)
    view = SimulationViewport(1.7)
    frames = []
    errors = []
    step = [0]
    view.frame_ready.connect(lambda _image: frames.append(time.monotonic()))
    view.render_failed.connect(errors.append)

    def update_pose():
        step[0] += 1
        pose = [0.0] * 29
        pose[16] = 0.1 * math.sin(step[0] * 0.1)
        view.set_pose(pose)

    timer = QTimer()
    timer.timeout.connect(update_pose)
    timer.start(30)
    started = time.monotonic()
    QTimer.singleShot(10000, app.quit)
    app.exec_()
    elapsed = time.monotonic() - started
    view.shutdown()
    if errors:
        raise RuntimeError("视图渲染失败：%s" % errors)
    if len(frames) < 40:
        raise RuntimeError("视图帧率过低：10 秒仅 %d 帧" % len(frames))
    steady_fps = (len(frames) - 1) / (frames[-1] - frames[0])
    if steady_fps < 10.0:
        raise RuntimeError(
            "视图稳定帧率过低：%.2f FPS（渲染 %.1f ms，共享复制 %.1f ms）" %
            (steady_fps, view.last_render_ms, view.last_copy_ms))
    print(
        "process viewport: PASS (%d frames, startup %.2fs, steady %.2f FPS, "
        "render %.1f ms, shared copy %.1f ms)" %
        (len(frames), frames[0] - started, steady_fps,
         view.last_render_ms, view.last_copy_ms))


if __name__ == "__main__":
    main()
