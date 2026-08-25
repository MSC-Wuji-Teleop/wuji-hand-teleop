"""PICO teleoperation launch: PICO 4 arm tracking + dual Wuji Hand output.

Runs in the `teleop` container only. It starts the *input* and *hand* halves of
a PICO session:

    pico_input_node        -> /left_arm_target_pose, /right_arm_target_pose
                              /left_arm_elbow_direction, /right_arm_elbow_direction
                              plus TF, all in the fixed world frame
    wujihand_controller    -> /left_hand/joint_commands, /right_hand/joint_commands
      (x2, per side)          from in-process Wuji Glove input via wuji_sdk
    wujihand_driver        -> the physical Wuji Hands over USB
      (x2, per side)

It does NOT start the arm controller. `g1_world_output` lives in a separate
image and container (its Pinocchio + CasADi build needs NumPy 1.x while this
stack needs 2.x), so it cannot be a Node in this launch file. Start it in a
second terminal:

    # terminal 1 (this file, teleop container)
    ros2 launch wuji_teleop_bringup pico_teleop.launch.py

    # terminal 2 (g1 container)
    cd docker && docker compose run --rm g1_world_output \\
        ros2 launch g1_world_output g1_world_output.launch.py
    #   ... add dry_run:=true for sim: real IK, no DDS, no robot

The two containers share host networking, ROS_DOMAIN_ID, and
docker/cyclonedds.xml, so the arm target-pose topics cross between them
directly. NOTE: this end-to-end PICO -> G1 path has not been verified on
hardware yet.

Cameras are not started here. `src/camera/` is unwired pending the G1 head
cameras (RealSense D435i / D455); see docs/wuji-camera-topics.md.

Manual re-initialization of the PICO frame alignment:

    ros2 service call /pico_input/init  std_srvs/srv/Trigger
    ros2 service call /pico_input/reset std_srvs/srv/Trigger
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)
from wuji_teleop_bringup.launch_utils import resolve_config_path as _get_config


def generate_launch_description() -> LaunchDescription:
    # ==================== Module switches ====================
    enable_hand_arg = DeclareLaunchArgument(
        "enable_hand", default_value="true",
        description="Spawn the hand controllers and hand drivers. Set false to "
                    "run PICO input on its own (arm-only session).",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz", default_value="false",
        description="Enable RViz visualization of the PICO frames.",
    )
    hand_config_arg = DeclareLaunchArgument(
        "hand_config",
        default_value=_get_config("wujihand_output", "wujihand_ik.yaml"),
        description="Path to wujihand_ik.yaml (hand serials, retarget params).",
    )

    # ==================== Hand driver params ====================
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

    enable_hand = LaunchConfiguration("enable_hand")
    enable_rviz = LaunchConfiguration("enable_rviz")
    hand_config = LaunchConfiguration("hand_config")
    left_hand_name = LaunchConfiguration("left_hand_name")
    right_hand_name = LaunchConfiguration("right_hand_name")

    # Force serial_number to string (works around ROS2 param type inference).
    left_serial_str = ParameterValue(
        LaunchConfiguration("left_serial"), value_type=str)
    right_serial_str = ParameterValue(
        LaunchConfiguration("right_serial"), value_type=str)

    startup_banner = LogInfo(
        msg="""
========================================================================
  PICO Teleoperation Launch (input + hands)
========================================================================
  Starts: pico_input_node, wujihand_controller x2, wujihand_driver x2

  Arguments:
    enable_hand - hand controllers + hand drivers (default true)
    enable_rviz - RViz visualization        (default false)

  The G1 arm output is NOT started here. In a second terminal:
    cd docker && docker compose run --rm g1_world_output \\
        ros2 launch g1_world_output g1_world_output.launch.py
    (append dry_run:=true for sim: real IK, no DDS)
========================================================================
"""
    )

    # ==================== PICO INPUT (always on) ====================
    pico_input_node = Node(
        package="pico_input",
        executable="pico_input_node",
        name="pico_input_node",
        output="screen",
        parameters=[_get_config("pico_input", "pico_input.yaml")],
    )

    # ==================== HAND CONTROLLERS (per-side processes) ====================
    wujihand_controller_left = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_left",
        output="screen",
        emulate_tty=True,
        arguments=["--side", "left", "--hand-name", left_hand_name,
                   "--config", hand_config],
        condition=IfCondition(enable_hand),
    )
    wujihand_controller_right = Node(
        package="controller",
        executable="wujihand_controller",
        name="wujihand_controller_right",
        output="screen",
        emulate_tty=True,
        arguments=["--side", "right", "--hand-name", right_hand_name,
                   "--config", hand_config],
        condition=IfCondition(enable_hand),
    )

    # ==================== HAND DRIVERS (own the USB link) ====================
    wujihand_driver_left = Node(
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
        condition=IfCondition(enable_hand),
    )
    wujihand_driver_right = Node(
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
        condition=IfCondition(enable_hand),
    )

    # ==================== VISUALIZATION ====================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            str(Path(get_package_share_directory("pico_input"))
                / "rviz" / "pico_visualization.rviz"),
        ],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        enable_hand_arg,
        enable_rviz_arg,
        hand_config_arg,
        left_serial_arg,
        right_serial_arg,
        left_hand_name_arg,
        right_hand_name_arg,

        startup_banner,

        pico_input_node,

        wujihand_driver_left,
        wujihand_driver_right,
        wujihand_controller_left,
        wujihand_controller_right,

        rviz_node,
    ])
