import os
from pathlib import Path
import subprocess


PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUN_SCRIPT = PROJECT_ROOT / "scripts" / "run_limb_inspection.sh"


def make_executable(path, content):
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def test_launcher_serially_switches_simulation_hardware_simulation(tmp_path):
    state = tmp_path / "state"
    trace = tmp_path / "trace"
    fake_ros2 = tmp_path / "fake_ros2"
    fake_sudo = tmp_path / "fake_sudo"
    fake_pkexec = tmp_path / "fake_pkexec"
    fake_xephyr = tmp_path / "fake_xephyr"
    fake_sysfs = tmp_path / "pci_devices"
    fpga = fake_sysfs / "0000:01:00.0"
    fpga.mkdir(parents=True)
    (fpga / "vendor").write_text("0x10ee\n", encoding="ascii")
    (fpga / "device").write_text("0x7022\n", encoding="ascii")
    make_executable(fake_ros2, """#!/bin/sh
count=0
if [ -f "$BXI_MODE_TEST_STATE" ]; then
    read -r count < "$BXI_MODE_TEST_STATE"
fi
count=$((count + 1))
printf '%s\\n' "$count" > "$BXI_MODE_TEST_STATE"
case "$*" in
  *limb_inspection_sim.launch.py*) mode=simulation ;;
  *limb_inspection_hw.launch.py*) mode=hardware ;;
  *) exit 9 ;;
esac
printf '%s\\n' "$mode" >> "$BXI_MODE_TEST_TRACE"
if [ "$count" -eq 1 ]; then
    printf 'hardware\\n' > "$BXI_LIMB_MODE_SWITCH_FILE"
elif [ "$count" -eq 2 ]; then
    printf 'simulation\\n' > "$BXI_LIMB_MODE_SWITCH_FILE"
fi
""")
    make_executable(fake_sudo, """#!/bin/sh
if [ "$1" = "-E" ]; then shift; fi
exec "$@"
""")
    make_executable(fake_pkexec, """#!/bin/sh
export BXI_MODE_SWITCH_TEST_CHILD=1
exec "$@"
""")
    make_executable(fake_xephyr, "#!/bin/sh\nexit 0\n")
    environment = os.environ.copy()
    environment.update({
        "BXI_ROS2_EXECUTABLE": str(fake_ros2),
        "BXI_SUDO_EXECUTABLE": str(fake_sudo),
        "BXI_PKEXEC_EXECUTABLE": str(fake_pkexec),
        "BXI_XEPHYR_EXECUTABLE": str(fake_xephyr),
        "BXI_FPGA_SYSFS_PATH": str(fake_sysfs),
        "BXI_MODE_TEST_STATE": str(state),
        "BXI_MODE_TEST_TRACE": str(trace),
        "BXI_LIMB_CONFIG_DIR": str(tmp_path / "config"),
    })

    result = subprocess.run(
        [str(RUN_SCRIPT), "simulation"], cwd=PROJECT_ROOT,
        env=environment, text=True, capture_output=True, timeout=15,
        check=False)

    assert result.returncode == 0, result.stdout + result.stderr
    assert trace.read_text(encoding="utf-8").splitlines() == [
        "simulation", "hardware", "simulation",
    ]


def test_hardware_start_without_fpga_falls_back_to_simulation(tmp_path):
    trace = tmp_path / "trace"
    fake_ros2 = tmp_path / "fake_ros2"
    fake_xephyr = tmp_path / "fake_xephyr"
    make_executable(fake_ros2, """#!/bin/sh
printf '%s\\n' "$*" > "$BXI_MODE_TEST_TRACE"
""")
    make_executable(fake_xephyr, "#!/bin/sh\nexit 0\n")
    environment = os.environ.copy()
    environment.update({
        "BXI_ROS2_EXECUTABLE": str(fake_ros2),
        "BXI_XEPHYR_EXECUTABLE": str(fake_xephyr),
        "BXI_FPGA_SYSFS_PATH": str(tmp_path / "missing_pci"),
        "BXI_MODE_TEST_TRACE": str(trace),
        "BXI_LIMB_CONFIG_DIR": str(tmp_path / "config"),
    })

    result = subprocess.run(
        [str(RUN_SCRIPT), "hardware"], cwd=PROJECT_ROOT,
        env=environment, text=True, capture_output=True, timeout=15,
        check=False)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "limb_inspection_sim.launch.py" in trace.read_text(
        encoding="utf-8")
    assert "10ee:7022" in result.stderr
