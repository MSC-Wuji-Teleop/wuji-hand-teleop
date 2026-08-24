"""
G1 World Output Launch File (ROS REP 103 Compliant)

Usage:
    ros2 launch g1_world_output g1_world_output.launch.py
    ros2 launch g1_world_output g1_world_output.launch.py dry_run:=true   # sim mode, no DDS/hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="90.0", description="Control loop rate (Hz)"
    )
    motion_mode_arg = DeclareLaunchArgument(
        "motion_mode", default_value="false",
        description="true -> rt/arm_sdk, false -> rt/lowcmd (overrides config/g1_robot.yaml)",
    )
    simulation_mode_arg = DeclareLaunchArgument(
        "simulation_mode", default_value="false",
        description="true -> ChannelFactoryInitialize(1) (DDS sim domain; still needs a DDS peer)",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run", default_value="false",
        description="Sim mode: solve IK, never connect to DDS/hardware. Pair with "
                     "scripts/mujoco_visualizer.py in the teleop container.",
    )

    g1_world_output_node = Node(
        package="g1_world_output",
        executable="g1_world_output_node",
        name="g1_world_output_node",
        output="screen",
        parameters=[{
            "control_rate": ParameterValue(LaunchConfiguration("control_rate"), value_type=float),
            "motion_mode": ParameterValue(LaunchConfiguration("motion_mode"), value_type=bool),
            "simulation_mode": ParameterValue(LaunchConfiguration("simulation_mode"), value_type=bool),
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
        }],
    )

    return LaunchDescription([
        control_rate_arg,
        motion_mode_arg,
        simulation_mode_arg,
        dry_run_arg,
        g1_world_output_node,
    ])
