import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_file = os.path.join(
        get_package_share_path("bxi_example_py_elf3"), "data", "elf3.xml")
    rate = LaunchConfiguration("control_rate_hz")
    timeout = LaunchConfiguration("feedback_timeout_sec")
    initialization = LaunchConfiguration("initialization_sec")
    bench_height = LaunchConfiguration("simulation_bench_height_m")
    simulation = Node(
        package="bxi_example_py_elf3", executable="bxi_elf3_hidden_simulation",
        name="simulation_mujoco_host", output="screen", emulate_tty=True,
        arguments=["--model", model_file])
    application = Node(
        package="bxi_example_py_elf3",
        executable="bxi_elf3_limb_inspection",
        name="elf3_limb_inspection", output="screen", emulate_tty=True,
        parameters=[{
            "topic_prefix": "simulation/", "hardware_mode": False,
            "control_rate_hz": ParameterValue(rate, value_type=float),
            "feedback_timeout_sec": ParameterValue(timeout, value_type=float),
            "initialization_sec": ParameterValue(initialization, value_type=float),
            "simulation_bench_height_m": ParameterValue(
                bench_height, value_type=float),
        }])
    return LaunchDescription([
        DeclareLaunchArgument("control_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("feedback_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("initialization_sec", default_value="3.0"),
        DeclareLaunchArgument(
            "simulation_bench_height_m", default_value="1.7",
            description="Suspended test-bench base height in metres"),
        RegisterEventHandler(OnProcessExit(
            target_action=simulation,
            on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo 已退出"))])),
        RegisterEventHandler(OnProcessExit(
            target_action=application,
            on_exit=[EmitEvent(event=Shutdown(reason="四肢检测软件已退出"))])),
        application,
        TimerAction(period=0.5, actions=[simulation]),
    ])
