"""
DEPRECATED -- kept for reference only, not installed by setup.py (the node
this launched, g1_joint_replay_node.py, was moved to deprecated/ alongside
this file). Use g1_world_output.launch.py with mode:=joint_replay instead --
see that file and g1_world_output_node.py's 'joint_replay' mode.

Original docstring, for context on what this used to do:

G1 Joint Replay Launch File

Alternative to g1_world_output.launch.py: drives the arms from named
joint-angle topics instead of headset wrist poses. Do not launch both
at once -- they share the same DDS arm channel.

Usage:
    ros2 launch g1_world_output g1_joint_replay.launch.py
    ros2 launch g1_world_output g1_joint_replay.launch.py dry_run:=true   # sim mode, no DDS/hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="250.0",
        description="Control loop rate (Hz). Matches G1_23_ArmController's internal DDS "
                     "write rate so interpolation between received joint-vector samples "
                     "stays smooth; do not lower this without a reason.",
    )
    motion_mode_arg = DeclareLaunchArgument(
        "motion_mode", default_value="true",
        description="true -> rt/arm_sdk (onboard controller keeps the legs; default), "
                    "false -> rt/lowcmd (robot on stand / debug). Overrides config/g1_robot.yaml.",
    )
    simulation_mode_arg = DeclareLaunchArgument(
        "simulation_mode", default_value="false",
        description="true -> ChannelFactoryInitialize(1) (DDS sim domain; still needs a DDS peer)",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run", default_value="false",
        description="Sim mode: accept joint targets, never connect to DDS/hardware.",
    )

    g1_joint_replay_node = Node(
        package="g1_world_output",
        executable="g1_joint_replay_node",
        name="g1_joint_replay_node",
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
        g1_joint_replay_node,
    ])
