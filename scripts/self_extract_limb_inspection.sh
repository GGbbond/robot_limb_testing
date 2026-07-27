#!/usr/bin/env bash
set -euo pipefail

MARK="__BXI_LIMB_ARCHIVE_BELOW__"
LINE="$(awk "/^${MARK}\$/ {print NR + 1; exit}" "$0")"
test -n "$LINE" || { echo "安装包损坏" >&2; exit 1; }
TEMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TEMP_DIR"' EXIT
tail -n "+$LINE" "$0" | tar -xzf - -C "$TEMP_DIR"
exec bash "$TEMP_DIR/bxi_limb_inspection/install.sh" "$@"
__BXI_LIMB_ARCHIVE_BELOW__
