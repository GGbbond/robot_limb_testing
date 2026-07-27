import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _require_root(_context):
    if os.geteuid() != 0:
        raise RuntimeError("Elf3 实机检测必须以 root 运行")
    return []


def _handle_hardware_exit(event, _context):
    """Tell the outer launcher when the driver itself exits unexpectedly."""
    returncode = int(event.returncode)
    # When the UI closes or requests a mode switch, launch enters shutdown
    # before terminating the hardware process.  A driver that exits first is
    # unexpected regardless of whether it returns 0, nonzero, or a signal.
    if not _context.is_shutdown:
        marker = os.environ.get("BXI_LIMB_HARDWARE_FAILURE_FILE", "")
        if marker:
            try:
                Path(marker).write_text(
                    "hardware_elf3 returncode=%d\n" % returncode,
                    encoding="utf-8")
            except OSError:
                pass
    return [EmitEvent(event=Shutdown(reason="实机驱动已退出"))]


def generate_launch_description():
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rate = LaunchConfiguration("control_rate_hz")
    timeout = LaunchConfiguration("feedback_timeout_sec")
    initialization = LaunchConfiguration("initialization_sec")
    velocity_fault_duration = LaunchConfiguration("velocity_fault_duration_sec")
    max_command_gap = LaunchConfiguration("max_command_gap_sec")
    hardware = Node(
        package="hardware_elf3", executable="hardware_elf3",
        name="hardware_elf3", output="screen", emulate_tty=True,
        parameters=[{
            "hardware_config/imu": True,
            "hardware_config/motor_pwr": True,
            "hardware_config/motor_disable": 0x60000000,
        }])
    application = Node(
        package="bxi_example_py_elf3",
        executable="bxi_elf3_limb_inspection",
        name="elf3_limb_inspection", output="screen", emulate_tty=True,
        parameters=[{
            "topic_prefix": "hardware/", "hardware_mode": True,
            "control_rate_hz": ParameterValue(rate, value_type=float),
            "feedback_timeout_sec": ParameterValue(timeout, value_type=float),
            "initialization_sec": ParameterValue(initialization, value_type=float),
            "velocity_fault_duration_sec": ParameterValue(
                velocity_fault_duration, value_type=float),
            "max_command_gap_sec": ParameterValue(max_command_gap, value_type=float),
        }])
    return LaunchDescription([
        OpaqueFunction(function=_require_root),
        DeclareLaunchArgument("control_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("feedback_timeout_sec", default_value="0.20"),
        DeclareLaunchArgument("initialization_sec", default_value="10.0"),
        DeclareLaunchArgument("velocity_fault_duration_sec", default_value="0.01"),
        DeclareLaunchArgument("max_command_gap_sec", default_value="0.08"),
        RegisterEventHandler(OnProcessExit(
            target_action=hardware,
            on_exit=_handle_hardware_exit)),
        RegisterEventHandler(OnProcessExit(
            target_action=application,
            on_exit=[EmitEvent(event=Shutdown(reason="四肢检测软件已退出"))])),
        hardware, application,
    ])
