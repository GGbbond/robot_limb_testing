import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_file = os.path.join(
        get_package_share_path("bxi_example_py_elf3"),
        "data/elf3.xml",
    )

    joint_name = LaunchConfiguration("joint_name")
    amplitude_rad = LaunchConfiguration("amplitude_rad")
    start_frequency_hz = LaunchConfiguration("start_frequency_hz")
    end_frequency_hz = LaunchConfiguration("end_frequency_hz")
    duration_sec = LaunchConfiguration("duration_sec")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    stop_ramp_sec = LaunchConfiguration("stop_ramp_sec")
    motion_button_mode = LaunchConfiguration("motion_button_mode")
    joint_test_required = LaunchConfiguration("joint_test_required")
    joint_test_amplitude_rad = LaunchConfiguration("joint_test_amplitude_rad")
    joint_test_move_sec = LaunchConfiguration("joint_test_move_sec")
    joint_test_hold_sec = LaunchConfiguration("joint_test_hold_sec")
    joint_test_min_motion_rad = LaunchConfiguration(
        "joint_test_min_motion_rad"
    )
    log_rate_hz = LaunchConfiguration("log_rate_hz")
    release_suspension = LaunchConfiguration("release_suspension")
    auto_start = LaunchConfiguration("auto_start")
    log_csv_path = LaunchConfiguration("log_csv_path")

    simulation_node = Node(
        package="mujoco",
        executable="simulation",
        name="simulation_mujoco",
        output="screen",
        parameters=[{"simulation/model_file": model_file}],
        emulate_tty=True,
    )

    vibration_node = Node(
        package="bxi_example_py_elf3",
        executable="bxi_example_py_elf3_vibration",
        name="bxi_example_py_elf3_vibration",
        output="screen",
        parameters=[
            {
                "topic_prefix": "simulation/",
                "joint_name": joint_name,
                "amplitude_rad": ParameterValue(
                    amplitude_rad, value_type=float
                ),
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
                "joint_test_required": ParameterValue(
                    joint_test_required, value_type=bool
                ),
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
                "release_suspension": ParameterValue(
                    release_suspension, value_type=bool
                ),
                "auto_start": ParameterValue(auto_start, value_type=bool),
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
                description="Elf3 joint to excite; use 'all' for all 29 joints",
            ),
            DeclareLaunchArgument(
                "amplitude_rad",
                default_value="0.23",
                description="Peak sine position amplitude in radians",
            ),
            DeclareLaunchArgument(
                "start_frequency_hz",
                default_value="10.0",
                description="Sweep start frequency",
            ),
            DeclareLaunchArgument(
                "end_frequency_hz",
                default_value="20.0",
                description="Sweep end frequency",
            ),
            DeclareLaunchArgument(
                "duration_sec",
                default_value="60.0",
                description="Test duration; 0 means continuous fixed frequency",
            ),
            DeclareLaunchArgument(
                "control_rate_hz",
                default_value="200.0",
                description="Actuator command publishing rate",
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
                "joint_test_required",
                default_value="true",
                description=(
                    "Run the 29-joint rotation precheck before vibration"
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
                description="Asynchronous CSV sampling rate",
            ),
            DeclareLaunchArgument(
                "release_suspension",
                default_value="false",
                description="Release the MuJoCo virtual suspension after reset",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description=(
                    "Automatic vibration start; must remain false while the "
                    "joint precheck is required"
                ),
            ),
            DeclareLaunchArgument(
                "log_csv_path",
                default_value="/tmp/elf3_vibration_test.csv",
                description="Command and measured-position CSV output",
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=simulation_node,
                    on_start=[
                        TimerAction(period=1.0, actions=[vibration_node])
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulation_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="MuJoCo simulation exited"))
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
            simulation_node,
        ]
    )
