"""Hand-only launch: Wuji Gloves in, two Wuji Hands out, as per-side processes.

Glove input runs in-process inside each wujihand_controller via wuji_sdk, so
there is no separate input node and no ROS topic hop on the way in.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)


def generate_launch_description() -> LaunchDescription:
    # ==================== Launch Arguments ====================
    left_serial_arg = DeclareLaunchArgument(
        "left_serial", default_value=LEFT_HAND_SERIAL,
        description="Left hand serial number",
    )
    right_serial_arg = DeclareLaunchArgument(
        "right_serial", default_value=RIGHT_HAND_SERIAL,
        description="Right hand serial number",
    )
    left_hand_name_arg = DeclareLaunchArgument(
        "left_hand_name", default_value=LEFT_HAND_NAME,
        description="Left hand wujihandros2 namespace",
    )
    right_hand_name_arg = DeclareLaunchArgument(
        "right_hand_name", default_value=RIGHT_HAND_NAME,
        description="Right hand wujihandros2 namespace",
    )
    enable_hand_driver_arg = DeclareLaunchArgument(
        "enable_hand_driver", default_value="true",
        description="Spawn wujihand_driver (the only process that touches the physical "
                     "Wuji Hand). Set false for sim mode: wujihand_controller still runs "
                     "off real glove input and publishes /left_hand|right_hand/joint_commands "
                     "unchanged -- pair with g1_world_output/scripts/mujoco_visualizer.py to "
                     "watch it in MuJoCo with no hand plugged in.",
    )
    spawn_hand_driver = IfCondition(LaunchConfiguration("enable_hand_driver"))

    left_hand_name = LaunchConfiguration("left_hand_name")
    right_hand_name = LaunchConfiguration("right_hand_name")
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str)
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str)

    return LaunchDescription([
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,
        enable_hand_driver_arg,

        # ==================== WUJIHANDROS2 DRIVERS ====================
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=left_hand_name,
            parameters=[{
                "serial_number": left_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
            condition=spawn_hand_driver,
        ),
        Node(
            package="wujihand_driver",
            executable="wujihand_driver_node",
            name="wujihand_driver",
            namespace=right_hand_name,
            parameters=[{
                "serial_number": right_serial_str,
                "publish_rate": DRIVER_PUBLISH_RATE,
                "filter_cutoff_freq": DRIVER_FILTER_CUTOFF_FREQ,
                "diagnostics_rate": DRIVER_DIAGNOSTICS_RATE,
            }],
            output="screen",
            emulate_tty=True,
            condition=spawn_hand_driver,
        ),

        # ==================== HAND CONTROLLERS (per-side, parallel processes) ====================
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_left",
            output="screen",
            emulate_tty=True,
            arguments=["--side", "left", "--hand-name", left_hand_name],
        ),
        Node(
            package="controller",
            executable="wujihand_controller",
            name="wujihand_controller_right",
            output="screen",
            emulate_tty=True,
            arguments=["--side", "right", "--hand-name", right_hand_name],
        ),
    ])
