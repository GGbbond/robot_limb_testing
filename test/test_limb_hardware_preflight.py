from bxi_example_py_elf3.limb_hardware_preflight import (
    fpga_canfd_available, fpga_canfd_devices,
)


def add_pci_device(root, address, vendor, device):
    path = root / address
    path.mkdir()
    (path / "vendor").write_text(vendor + "\n", encoding="ascii")
    (path / "device").write_text(device + "\n", encoding="ascii")


def test_finds_only_expected_xilinx_canfd_device(tmp_path):
    add_pci_device(tmp_path, "0000:01:00.0", "0x10ee", "0x7022")
    add_pci_device(tmp_path, "0000:02:00.0", "0x8086", "0x1234")

    assert fpga_canfd_devices(tmp_path) == ("0000:01:00.0",)
    assert fpga_canfd_available(tmp_path)


def test_missing_or_unreadable_sysfs_is_not_available(tmp_path):
    add_pci_device(tmp_path, "0000:02:00.0", "0x10ee", "0x9999")

    assert not fpga_canfd_available(tmp_path)
    assert not fpga_canfd_available(tmp_path / "missing")
