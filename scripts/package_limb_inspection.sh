#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

set +u
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
set -u
colcon build --packages-select bxi_example_py_elf3

mkdir -p dist
STAGE_DIR="$(mktemp -d)"
trap 'rm -rf "$STAGE_DIR"' EXIT
APP_DIR="$STAGE_DIR/bxi_limb_inspection"
mkdir -p "$APP_DIR/install" "$APP_DIR/scripts"
cp -a install/bxi_example_py_elf3 "$APP_DIR/install/"
cp install/setup.bash install/setup.sh install/setup.zsh install/setup.ps1 \
   install/local_setup.bash install/local_setup.sh install/local_setup.zsh \
   install/local_setup.ps1 install/_local_setup_util_sh.py \
   install/_local_setup_util_ps1.py install/.colcon_install_layout \
   install/COLCON_IGNORE "$APP_DIR/install/"
cp scripts/run_limb_inspection.sh "$APP_DIR/scripts/"
cp scripts/deploy_limb_inspection.sh "$APP_DIR/install.sh"
chmod +x "$APP_DIR/scripts/run_limb_inspection.sh" "$APP_DIR/install.sh"

ARCHIVE="dist/bxi-limb-inspection-linux-x86_64.tar.gz"
tar -czf "$ARCHIVE" -C "$STAGE_DIR" bxi_limb_inspection

INSTALLER="dist/bxi-limb-inspection-linux-x86_64-installer.run"
sed '/^__BXI_LIMB_ARCHIVE_BELOW__$/q' scripts/self_extract_limb_inspection.sh > "$INSTALLER"
cat "$ARCHIVE" >> "$INSTALLER"
chmod +x "$INSTALLER"

echo "打包完成："
echo "  $ROOT_DIR/$ARCHIVE"
echo "  $ROOT_DIR/$INSTALLER"
