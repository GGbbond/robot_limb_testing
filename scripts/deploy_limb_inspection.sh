#!/usr/bin/env bash
set -euo pipefail

APP_ID="bxi_limb_inspection"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${HOME}/.local/share/${APP_ID}"
DESKTOP_DIR="${HOME}/.local/share/applications"
DESKTOP_FILE="${DESKTOP_DIR}/${APP_ID}.desktop"

if [ "${1:-}" = "--check" ]; then
    test -f /opt/ros/humble/setup.bash || { echo "缺少 ROS2 Humble"; exit 1; }
    test -f /opt/bxi/bxi_ros2_pkg/setup.bash || { echo "缺少 BXI ROS2 软件包"; exit 1; }
    python3 -c 'import PyQt5, pyqtgraph, rclpy, mujoco' || exit 1
    command -v Xephyr >/dev/null || {
        echo "缺少 Xephyr，请安装 xserver-xephyr" >&2
        exit 1
    }
    echo "运行环境检查通过"
    exit 0
fi

if [ "${1:-}" = "--uninstall" ]; then
    rm -rf "$INSTALL_DIR"
    rm -f "$DESKTOP_FILE"
    echo "已卸载 $APP_ID；用户报告和配置未删除。"
    exit 0
fi

mkdir -p "$INSTALL_DIR" "$DESKTOP_DIR"
cp -a "$SOURCE_DIR/install" "$INSTALL_DIR/"
cp -a "$SOURCE_DIR/scripts" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/scripts/run_limb_inspection.sh"

cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=BXI 机器人四肢检测台（仿真）
Comment=Elf3 arm and leg joint inspection
Exec=$INSTALL_DIR/scripts/run_limb_inspection.sh simulation
Terminal=false
Categories=Development;Science;
EOF
chmod +x "$DESKTOP_FILE"
echo "安装完成。可从应用菜单启动，或运行："
echo "  $INSTALL_DIR/scripts/run_limb_inspection.sh simulation"
echo "实机模式："
echo "  $INSTALL_DIR/scripts/run_limb_inspection.sh hardware"
