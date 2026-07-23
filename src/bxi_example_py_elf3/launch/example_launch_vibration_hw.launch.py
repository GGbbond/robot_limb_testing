import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _require_root(_context):
    if os.geteuid() != 0:
        raise RuntimeError(
            "Elf3 hardware vibration launch must be run as root"
        )
    return []


def generate_launch_description():
    joint_name = LaunchConfiguration("joint_name")
    amplitude_rad = LaunchConfiguration("amplitude_rad")
    start_frequency_hz = LaunchConfiguration("start_frequency_hz")
    end_frequency_hz = LaunchConfiguration("end_frequency_hz")
    duration_sec = LaunchConfiguration("duration_sec")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    stop_ramp_sec = LaunchConfiguration("stop_ramp_sec")
    motion_button_mode = LaunchConfiguration("motion_button_mode")
    joint_test_amplitude_rad = LaunchConfiguration("joint_test_amplitude_rad")
    joint_test_move_sec = LaunchConfiguration("joint_test_move_sec")
    joint_test_hold_sec = LaunchConfiguration("joint_test_hold_sec")
    joint_test_min_motion_rad = LaunchConfiguration(
        "joint_test_min_motion_rad"
    )
    log_rate_hz = LaunchConfiguration("log_rate_hz")
    max_command_gap_sec = LaunchConfiguration("max_command_gap_sec")
    log_csv_path = LaunchConfiguration("log_csv_path")

    hardware_node = Node(
        package="hardware_elf3",
        executable="hardware_elf3",
        name="hardware_elf3",
        output="screen",
        parameters=[
            {
                "hardware_config/imu": True,
                "hardware_config/motor_pwr": True,
                "hardware_config/motor_disable": 0x60000000,
            }
        ],
        emulate_tty=True,
    )

    vibration_node = Node(
        package="bxi_example_py_elf3",
        executable="bxi_example_py_elf3_vibration",
        name="bxi_example_py_elf3_vibration",
        output="screen",
        parameters=[
            {
                "topic_prefix": "hardware/",
                "hardware_mode": True,
                "shutdown_on_safety_fault": True,
                "require_joint_state": True,
                "allow_all_joints": True,
                "joint_name": joint_name,
                "amplitude_rad": ParameterValue(amplitude_rad, value_type=float),
                "start_frequency_hz": ParameterValue(
                    start_frequency_hz, value_type=float
                ),
                "end_frequency_hz": ParameterValue(
                    end_frequency_hz, value_type=float
                ),
                "duration_sec": ParameterValue(duration_sec, value_type=float),
                "control_rate_hz": ParameterValue(
                    control_rate_hz, value_type=float
                ),
                "stop_ramp_sec": ParameterValue(stop_ramp_sec, value_type=float),
                "motion_button_mode": motion_button_mode,
                "joint_test_required": True,
                "joint_test_amplitude_rad": ParameterValue(
                    joint_test_amplitude_rad, value_type=float
                ),
                "joint_test_move_sec": ParameterValue(
                    joint_test_move_sec, value_type=float
                ),
                "joint_test_hold_sec": ParameterValue(
                    joint_test_hold_sec, value_type=float
                ),
                "joint_test_min_motion_rad": ParameterValue(
                    joint_test_min_motion_rad, value_type=float
                ),
                "joint_test_verify_feedback": True,
                "log_rate_hz": ParameterValue(log_rate_hz, value_type=float),
                "initialization_sec": 10.0,
                "release_suspension": False,
                "auto_start": False,
                "joint_limit_margin_rad": 0.02,
                "joint_state_timeout_sec": 0.2,
                "max_command_gap_sec": ParameterValue(
                    max_command_gap_sec, value_type=float
                ),
                "hardware_max_amplitude_rad": 100000.0,
                "hardware_max_frequency_hz": 100000.0,
                "hardware_max_velocity_rad_s": 100000.0,
                "hardware_max_acceleration_rad_s2": 100000.0,
                "hardware_max_control_rate_hz": 500.0,
                "hardware_max_joint_test_amplitude_rad": 0.1,
                "hardware_joint_test_min_move_sec": 0.2,
                "hardware_joint_test_min_response_ratio": 0.25,
                "hardware_joint_test_max_tolerance_rad": 0.05,
                "hardware_joint_test_max_velocity_rad_s": 0.5,
                "hardware_joint_test_max_acceleration_rad_s2": 5.0,
                "log_csv_path": log_csv_path,
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_name",
                default_value="all",
                description="Hardware vibration joints; default is all 29 joints",
            ),
            DeclareLaunchArgument(
                "amplitude_rad",
                default_value="0.23",
                description="Peak hardware position amplitude in radians",
            ),
            DeclareLaunchArgument(
                "start_frequency_hz",
                default_value="10.0",
                description="Hardware sweep start frequency",
            ),
            DeclareLaunchArgument(
                "end_frequency_hz",
                default_value="20.0",
                description="Hardware sweep end frequency",
            ),
            DeclareLaunchArgument(
                "duration_sec",
                default_value="60.0",
                description="Hardware test duration",
            ),
            DeclareLaunchArgument(
                "control_rate_hz",
                default_value="200.0",
                description="Hardware actuator command publishing rate",
            ),
            DeclareLaunchArgument(
                "stop_ramp_sec",
                default_value="0.5",
                description="Smooth return-to-center time after a normal stop",
            ),
            DeclareLaunchArgument(
                "motion_button_mode",
                default_value="toggle",
                description=(
                    "btn_9 source mode: toggle for the C++ gamepad, "
                    "momentary for keyboard-style press/release sources"
                ),
            ),
            DeclareLaunchArgument(
                "joint_test_amplitude_rad",
                default_value="0.03",
                description="Per-joint positive/negative precheck amplitude",
            ),
            DeclareLaunchArgument(
                "joint_test_move_sec",
                default_value="0.4",
                description="Smooth travel time for each precheck waypoint",
            ),
            DeclareLaunchArgument(
                "joint_test_hold_sec",
                default_value="0.1",
                description="Feedback verification hold at each waypoint",
            ),
            DeclareLaunchArgument(
                "joint_test_min_motion_rad",
                default_value="0.015",
                description="Minimum measured motion in each direction",
            ),
            DeclareLaunchArgument(
                "log_rate_hz",
                default_value="100.0",
                description="Asynchronous hardware CSV sampling rate",
            ),
            DeclareLaunchArgument(
                "max_command_gap_sec",
                default_value="0.05",
                description=(
                    "Maximum time between successful hardware command publishes; "
                    "exceeding it latches a safety fault"
                ),
            ),
            DeclareLaunchArgument(
                "log_csv_path",
                default_value="/tmp/elf3_vibration_hardware.csv",
                description="Hardware command and feedback CSV output",
            ),
            OpaqueFunction(function=_require_root),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=hardware_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="hardware_elf3 exited")
                        )
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=vibration_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="vibration controller exited")
                        )
                    ],
                )
            ),
            hardware_node,
            vibration_node,
        ]
    )
