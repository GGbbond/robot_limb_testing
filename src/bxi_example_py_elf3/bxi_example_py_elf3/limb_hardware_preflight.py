"""Read-only hardware presence checks used before starting motor power."""

import os
from pathlib import Path


FPGA_PCI_VENDOR = "0x10ee"
FPGA_PCI_DEVICE = "0x7022"
PCI_DEVICES_PATH = Path("/sys/bus/pci/devices")


def fpga_canfd_devices(devices_path=None):
    """Return matching Xilinx PCI CAN-FD device addresses from sysfs."""
    matches = []
    root = Path(devices_path) if devices_path is not None else Path(
        os.environ.get("BXI_FPGA_SYSFS_PATH", str(PCI_DEVICES_PATH)))
    try:
        entries = tuple(root.iterdir())
    except OSError:
        return tuple()
    for entry in entries:
        try:
            vendor = (entry / "vendor").read_text(
                encoding="ascii").strip().lower()
            device = (entry / "device").read_text(
                encoding="ascii").strip().lower()
        except OSError:
            continue
        if vendor == FPGA_PCI_VENDOR and device == FPGA_PCI_DEVICE:
            matches.append(entry.name)
    return tuple(sorted(matches))


def fpga_canfd_available(devices_path=None):
    return bool(fpga_canfd_devices(devices_path))
