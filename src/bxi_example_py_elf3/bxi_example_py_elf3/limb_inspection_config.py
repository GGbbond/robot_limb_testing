"""Configuration loading and persistence for limb inspection applications."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_path


CONFIG_DIR = Path.home() / ".config" / "bxi_limb_inspection"
USER_SETTINGS = CONFIG_DIR / "settings.json"
DEFAULT_REPORT_DIRECTORY = "~/BXI/limb_inspection_reports"
SIMULATION_DEBUG_MAX = 1_000_000_000.0
SIMULATION_DEBUG_MAX_CYCLES = 2_147_483_647


def defaults_path():
    try:
        return get_package_share_path("bxi_example_py_elf3") / \
            "config" / "limb_inspection_defaults.json"
    except Exception:
        return Path(__file__).resolve().parent.parent / \
            "config" / "limb_inspection_defaults.json"


def load_settings():
    result = {}
    for path in (defaults_path(), USER_SETTINGS):
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            if isinstance(data, dict):
                result.update(data)
        except (OSError, ValueError):
            pass
    return result


def save_settings(data):
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(USER_SETTINGS, "w", encoding="utf-8") as stream:
        json.dump(dict(data), stream, ensure_ascii=False, indent=2)
        stream.write("\n")

