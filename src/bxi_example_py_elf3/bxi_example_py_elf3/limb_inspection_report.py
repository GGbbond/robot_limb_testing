"""Reusable JSON and CSV report generation for limb inspection runs."""

import csv
from datetime import datetime
import json
from pathlib import Path

import numpy as np

from .limb_inspection_core import JOINT_NAMES
from .limb_inspection_posture import (
    COMPACT_POSTURE_DEG, MIRRORED_PAIR_LOWER_BOUND_DEG,
)


def export_report(directory, controller, settings):
    directory = Path(directory)
    snapshot = controller.snapshot(include_samples=True)
    if not snapshot["results"]:
        raise RuntimeError("当前没有可导出的检测结果")
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = directory / ("elf3_limb_inspection_" + stamp)
    json_path = base.with_suffix(".json")
    csv_path = base.with_suffix(".csv")
    sample_path = directory / (base.name + "_samples.csv")
    report = {
        "generated_at": datetime.now().astimezone().isoformat(),
        "mode": "hardware" if controller.hardware_mode else "simulation",
        "topic_prefix": controller.topic_prefix,
        "simulation_bench_height_m": (
            controller.simulation_bench_height_m
            if not controller.hardware_mode else None),
        "settings": settings.__dict__,
        "motion_planning": {
            "compact_posture_deg": {
                active: {"joint": companion, "angle_deg": angle}
                for active, (companion, angle)
                in COMPACT_POSTURE_DEG.items()
            },
            "mirrored_pair_raw_lower_bound_deg": dict(
                MIRRORED_PAIR_LOWER_BOUND_DEG),
        },
        "summary": {
            "total": len(snapshot["results"]),
            "passed": sum(item.passed for item in snapshot["results"]),
        },
        "results": [item.to_dict() for item in snapshot["results"]],
    }
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    fields = list(snapshot["results"][0].to_dict())
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(item.to_dict() for item in snapshot["results"])
    names = tuple(controller.selected_names)
    with open(sample_path, "w", newline="", encoding="utf-8-sig") as stream:
        fields = [
            "time", "test_joint", "joint_name", "command_deg",
            "position_deg", "velocity_deg_s", "effort_nm",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample in snapshot["samples"]:
            for name in names:
                index = JOINT_NAMES.index(name)
                writer.writerow({
                    "time": sample["time"], "test_joint": sample["joint"],
                    "joint_name": name,
                    "command_deg": np.rad2deg(sample["command"][index]),
                    "position_deg": np.rad2deg(sample["position"][index]),
                    "velocity_deg_s": np.rad2deg(sample["velocity"][index]),
                    "effort_nm": sample["effort"][index],
                })
    return json_path, csv_path, sample_path
