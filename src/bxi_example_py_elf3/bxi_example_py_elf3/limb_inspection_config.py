"""Configuration loading and persistence for limb inspection applications."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_path


CONFIG_DIR = Path(os.environ.get(
    "BXI_LIMB_CONFIG_DIR",
    str(Path.home() / ".config" / "bxi_limb_inspection"),
)).expanduser()
USER_SETTINGS = CONFIG_DIR / "settings.json"
# Releases before the unified configuration used this file only in hardware
# mode.  Keep it as a read-only migration fallback so existing installations
# do not lose their settings when settings.json has not been created yet.
LEGACY_HARDWARE_USER_SETTINGS = CONFIG_DIR / "settings_hardware.json"
DEFAULT_REPORT_DIRECTORY = "~/BXI/limb_inspection_reports"
PARAMETER_INPUT_MAX = 1_000_000_000.0


def defaults_path():
    try:
        return get_package_share_path("bxi_example_py_elf3") / \
            "config" / "limb_inspection_defaults.json"
    except Exception:
        return Path(__file__).resolve().parent.parent / \
            "config" / "limb_inspection_defaults.json"


def load_settings():
    """Load the one settings set shared by simulation and hardware."""
    result = {}
    user_path = USER_SETTINGS
    if not user_path.is_file() and LEGACY_HARDWARE_USER_SETTINGS.is_file():
        user_path = LEGACY_HARDWARE_USER_SETTINGS
    for path in (defaults_path(), user_path):
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            if isinstance(data, dict):
                result.update(data)
        except (OSError, ValueError):
            pass
    # Ignore keys written by releases that still offered small-motion mode.
    for obsolete in ("motion_mode", "amplitude_deg", "cycles"):
        result.pop(obsolete, None)
    return result


def save_settings(data):
    """Persist settings to the shared simulation/hardware file."""
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(USER_SETTINGS, "w", encoding="utf-8") as stream:
        json.dump(dict(data), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
