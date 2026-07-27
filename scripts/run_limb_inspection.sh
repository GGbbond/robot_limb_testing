#!/usr/bin/env bash
set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-simulation}"
HARDWARE_CHILD=0

if [ "$MODE" = "--hardware-child" ]; then
    HARDWARE_CHILD=1
    MODE="hardware"
fi

# Limb inspection, its hardware driver, and the MuJoCo backend run on the
# same robot controller.  Force DDS discovery to remain local so another
# development PC on ROS domain 0 cannot inject same-named joint feedback.
export ROS_LOCALHOST_ONLY=1

if [ "$MODE" != "simulation" ] && [ "$MODE" != "hardware" ]; then
    echo "用法: $0 [simulation|hardware]" >&2
    exit 2
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

if [ "$HARDWARE_CHILD" -eq 1 ]; then
    if [ "$(id -u)" -ne 0 ] && \
            [ "${BXI_MODE_SWITCH_TEST_CHILD:-0}" != "1" ]; then
        echo "实机驱动子进程必须以 root 运行。" >&2
        exit 1
    fi
    CHILD_ROS2_EXECUTABLE="${BXI_ROS2_EXECUTABLE:-$(command -v ros2)}"
    exec "$CHILD_ROS2_EXECUTABLE" launch \
        bxi_example_py_elf3 limb_inspection_hw.launch.py
fi

CONFIG_DIR="${BXI_LIMB_CONFIG_DIR:-$HOME/.config/bxi_limb_inspection}"
ROS2_EXECUTABLE="${BXI_ROS2_EXECUTABLE:-$(command -v ros2)}"
SUDO_EXECUTABLE="${BXI_SUDO_EXECUTABLE:-sudo}"
PKEXEC_EXECUTABLE="${BXI_PKEXEC_EXECUTABLE:-$(command -v pkexec || true)}"
XEPHYR_EXECUTABLE="${BXI_XEPHYR_EXECUTABLE:-$(command -v Xephyr || true)}"
SWITCH_DIR="$(mktemp -d /tmp/bxi_limb_mode_switch.XXXXXX)"
SWITCH_FILE="$SWITCH_DIR/request"
HARDWARE_FAILURE_FILE="$SWITCH_DIR/hardware_failure"
pending_warning=""

cleanup_mode_switch() {
    rm -rf "$SWITCH_DIR"
}
trap cleanup_mode_switch EXIT INT TERM

while true; do
    # Pre-create as the invoking user.  A root-owned hardware UI can truncate
    # this file without changing ownership, so the user supervisor can always
    # read the requested next mode after sudo exits.
    : > "$SWITCH_FILE"
    rm -f "$HARDWARE_FAILURE_FILE"
    export BXI_LIMB_MODE_SWITCH_FILE="$SWITCH_FILE"
    export BXI_LIMB_HARDWARE_FAILURE_FILE="$HARDWARE_FAILURE_FILE"
    startup_warning="$pending_warning"
    pending_warning=""
    if [ "$MODE" = "hardware" ] && ! python3 -c \
            'from bxi_example_py_elf3.limb_hardware_preflight import fpga_canfd_available; raise SystemExit(0 if fpga_canfd_available() else 1)'; then
        startup_warning="未检测到 Xilinx PCI CAN-FD 设备 10ee:7022，已保持软件运行并回退到 MuJoCo 仿真模式。请关闭机器人动力，检查 FPGA 板卡、PCIe 插槽和主控连接后重试。"
        echo "$startup_warning" >&2
        MODE="simulation"
    fi
    if [ -n "$startup_warning" ]; then
        export BXI_LIMB_STARTUP_WARNING="$startup_warning"
    else
        unset BXI_LIMB_STARTUP_WARNING
    fi
    if [ "$MODE" = "simulation" ] && [ -z "$XEPHYR_EXECUTABLE" ]; then
        echo "缺少 Xephyr，无法启动无窗口 MuJoCo 后端；请安装 xserver-xephyr。" >&2
        exit 1
    fi

    set +e
    if [ "$MODE" = "hardware" ] && [ "$(id -u)" -ne 0 ]; then
        if [ -t 0 ]; then
            echo "实机驱动需要管理员权限，即将请求 sudo 授权。"
            "$SUDO_EXECUTABLE" -E env \
                BXI_LIMB_CONFIG_DIR="$CONFIG_DIR" \
                BXI_LIMB_MODE_SWITCH_FILE="$SWITCH_FILE" \
                BXI_LIMB_HARDWARE_FAILURE_FILE="$HARDWARE_FAILURE_FILE" \
                ROS_LOCALHOST_ONLY=1 \
                "$0" --hardware-child
            status=$?
        elif [ -n "$PKEXEC_EXECUTABLE" ]; then
            echo "实机驱动需要管理员权限，即将请求图形授权。"
            "$PKEXEC_EXECUTABLE" env \
                DISPLAY="${DISPLAY:-}" \
                XAUTHORITY="${XAUTHORITY:-}" \
                XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
                DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
                QT_X11_NO_MITSHM=1 \
                MUJOCO_GL=egl \
                BXI_LIMB_CONFIG_DIR="$CONFIG_DIR" \
                BXI_LIMB_MODE_SWITCH_FILE="$SWITCH_FILE" \
                BXI_LIMB_HARDWARE_FAILURE_FILE="$HARDWARE_FAILURE_FILE" \
                BXI_GAMEPAD_DEVICE="${BXI_GAMEPAD_DEVICE:-/dev/input/js0}" \
                BXI_VIEW_RENDER_THREADS="${BXI_VIEW_RENDER_THREADS:-2}" \
                ROS_LOCALHOST_ONLY=1 \
                "$0" --hardware-child
            status=$?
        else
            echo "没有终端且找不到 pkexec，无法请求实机管理员权限。" >&2
            status=1
        fi
    elif [ "$MODE" = "hardware" ]; then
        BXI_LIMB_CONFIG_DIR="$CONFIG_DIR" \
        "$ROS2_EXECUTABLE" launch \
            bxi_example_py_elf3 limb_inspection_hw.launch.py
        status=$?
    else
        "$ROS2_EXECUTABLE" launch \
            bxi_example_py_elf3 limb_inspection_sim.launch.py
        status=$?
    fi
    set -e

    if [ -s "$SWITCH_FILE" ]; then
        read -r requested_mode < "$SWITCH_FILE"
        if [ "$requested_mode" != "simulation" ] && \
                [ "$requested_mode" != "hardware" ]; then
            echo "忽略无效的模式切换请求：$requested_mode" >&2
            exit 2
        fi
        MODE="$requested_mode"
        continue
    fi

    if [ "$MODE" = "hardware" ] && [ -s "$HARDWARE_FAILURE_FILE" ]; then
        failure_detail="$(head -n 1 "$HARDWARE_FAILURE_FILE")"
        pending_warning="实机驱动异常退出（${failure_detail}），常见原因是所需电机未接全、CAN 口连接错误或驱动初始化失败。已停止实机控制并自动回退到 MuJoCo 仿真，软件保持运行。请断开动力后检查所测手臂/腿对应电机和 CAN 口。"
        echo "$pending_warning" >&2
        MODE="simulation"
        continue
    fi

    exit "$status"
done
