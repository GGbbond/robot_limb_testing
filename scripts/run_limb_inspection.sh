#!/usr/bin/env bash
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-simulation}"

if [ "$MODE" != "simulation" ] && [ "$MODE" != "hardware" ]; then
    echo "用法: $0 [simulation|hardware]" >&2
    exit 2
fi

if [ "$MODE" = "hardware" ] && [ "$(id -u)" -ne 0 ]; then
    echo "实机驱动需要管理员权限，即将请求 sudo 授权。"
    exec sudo -E "$0" hardware
fi

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
    source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
if [ -f "$APP_ROOT/install/setup.bash" ]; then
    source "$APP_ROOT/install/setup.bash"
elif [ -f "$APP_ROOT/install/bxi_example_py_elf3/share/bxi_example_py_elf3/local_setup.bash" ]; then
    source "$APP_ROOT/install/bxi_example_py_elf3/share/bxi_example_py_elf3/local_setup.bash"
else
    echo "找不到软件安装环境，请先在项目根目录执行 ./build.sh。" >&2
    exit 1
fi
set -u

python3 -c 'import PyQt5, pyqtgraph, rclpy, mujoco' 2>/dev/null || {
    echo "缺少运行依赖，请检查 PyQt5、pyqtgraph、rclpy 和 MuJoCo Python 模块。" >&2
    exit 1
}
if [ "$MODE" = "simulation" ] && ! command -v Xephyr >/dev/null 2>&1; then
    echo "缺少 Xephyr，无法启动无窗口 MuJoCo 后端；请安装 xserver-xephyr。" >&2
    exit 1
fi

if [ "$MODE" = "hardware" ]; then
    exec ros2 launch bxi_example_py_elf3 limb_inspection_hw.launch.py
fi
exec ros2 launch bxi_example_py_elf3 limb_inspection_sim.launch.py
