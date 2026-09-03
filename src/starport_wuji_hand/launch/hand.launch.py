"""Bring up the Wuji hand driver(s) alone (no visualization).

One node per hand, at /{side}/wuji_hand -- the layout starport_robotiq_gripper's grippers.launch.py
already uses for its two arms. Each node takes ~/joint_command (sensor_msgs/JointState, radians)
in and publishes ~/commanded_joint_states (the post-guard-chain ghost, for RViz), ~/connected and
~/diagnostics, plus /joint_states globally so one robot_state_publisher can animate both hands and
the rest of the cell. The two hands' joint names differ by their `l_`/`r_` prefix, so a shared
/joint_states is unambiguous.

    ros2 launch starport_wuji_hand hand.launch.py                     # right hand only (default)
    ros2 launch starport_wuji_hand hand.launch.py side:=both          # both hands
    ros2 launch starport_wuji_hand hand.launch.py side:=left
    ros2 launch starport_wuji_hand hand.launch.py right_serial_number:=ABC123

`side` defaults to right because that is the hand on the bench; switch it to `both` once the
second one is plugged in. Launching a side whose hand is absent is not fatal to the other: the
node reports the missing hand and stops trying after max_connect_attempts.

Only the parameters an operator changes at the bench are exposed. Device and calibration values
are per hand; the policy values (cutoff, slew limit, homing) are shared, because they describe how
this cell drives a hand rather than which hand it is. The node validates every one of them at
construction and refuses to start on a bad value -- `hand_side` included, so a driver started with
`ros2 run` is refused the same way this file's node-per-side layout refuses it.

A sign/zero correction is twenty floats per array; the README's sign and zero note carries the
invocation and what the two findings it can absorb mean.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from starport_wuji_hand.joint_map import HAND_SIDES, NUM_JOINTS
from starport_wuji_hand.limits_io import limits_filename

PACKAGE = "starport_wuji_hand"

# The bench hand. `both` is the setting once the second one arrives.
DEFAULT_SIDE = "right"


def _per_side_args(share: str, side: str) -> list[DeclareLaunchArgument]:
    """The arguments that describe one physical hand rather than the cell's policy."""
    return [
        DeclareLaunchArgument(
            f"{side}_serial_number",
            default_value="",
            description=f"Serial number of the {side} hand; empty takes any Hand 2 the scan finds "
            "(its reported handedness is still checked).",
        ),
        DeclareLaunchArgument(
            f"{side}_limits_file",
            default_value=PathJoinSubstitution([share, "config", limits_filename(side)]),
            description=f"{side} joint limits, clamped to and cross-checked against the hardware.",
        ),
        # Write the literals as floats: an integer array is refused by the typing below, before the
        # node sees it. What the two arrays are for is the README's sign and zero note.
        DeclareLaunchArgument(
            f"{side}_joint_sign",
            default_value=str([1.0] * NUM_JOINTS),
            description=f"{NUM_JOINTS} values, each +1.0 or -1.0: which way round each joint is wired.",
        ),
        DeclareLaunchArgument(
            f"{side}_joint_offset",
            default_value=str([0.0] * NUM_JOINTS),
            description=f"{NUM_JOINTS} values in radians: where each joint's neutral really is.",
        ),
    ]


def hand_node(side: str) -> Node:
    """The driver for one hand, launched only when `side` selects it.

    `hand_side` is fixed to this node's own side rather than read from a launch argument: which
    hand a node drives is what makes it this node, and an argument there would be a second place
    to get it wrong.
    """
    # Every value is typed. A launch argument arrives as text and would otherwise be re-typed by
    # yaml rules, which disagree with the node's declared parameter types on exactly the inputs an
    # operator types by hand: rclpy does not coerce int to double, so max_joint_velocity:=2 or
    # home_on_start:=1 would be refused at bring-up, and an all-digit serial_number would arrive as
    # an int. Same reason as starport_camera_bringup/launch/realsense_camera.launch.py.
    return Node(
        package=PACKAGE,
        executable="hand_node",
        name="wuji_hand",
        namespace=f"/{side}",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("side"), f"' in ('both', '{side}')"])),
        parameters=[
            {
                "hand_side": side,
                "serial_number": ParameterValue(LaunchConfiguration(f"{side}_serial_number"), value_type=str),
                "limits_file": ParameterValue(LaunchConfiguration(f"{side}_limits_file"), value_type=str),
                "joint_sign": ParameterValue(LaunchConfiguration(f"{side}_joint_sign"), value_type=list[float]),
                "joint_offset": ParameterValue(LaunchConfiguration(f"{side}_joint_offset"), value_type=list[float]),
                "setpoint_velocity_filter_hz": ParameterValue(
                    LaunchConfiguration("setpoint_velocity_filter_hz"), value_type=float
                ),
                "max_joint_velocity": ParameterValue(LaunchConfiguration("max_joint_velocity"), value_type=float),
                "effort_limit_a": ParameterValue(LaunchConfiguration("effort_limit_a"), value_type=float),
                "home_on_start": ParameterValue(LaunchConfiguration("home_on_start"), value_type=bool),
                "max_connect_attempts": ParameterValue(LaunchConfiguration("max_connect_attempts"), value_type=int),
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE)

    args: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "side",
            default_value=DEFAULT_SIDE,
            choices=["both", *HAND_SIDES],
            description="Which hand driver(s) to launch.",
        ),
        DeclareLaunchArgument(
            "setpoint_velocity_filter_hz",
            default_value="10.0",
            description="Smoothing cutoff for the commanded setpoint velocity, in Hz.",
        ),
        DeclareLaunchArgument("max_joint_velocity", default_value="2.0", description="Slew-rate limit in rad/s."),
        DeclareLaunchArgument(
            "effort_limit_a",
            default_value="0.6",
            description="Per-joint current ceiling in amps. 1.0 buzzes on a clip replay, 0.6 does not; "
            "raise it for a task that must grip rather than track.",
        ),
        DeclareLaunchArgument("home_on_start", default_value="true", description="Sweep to the home pose on connect."),
        DeclareLaunchArgument(
            "max_connect_attempts",
            default_value="10",
            description="Give up on an absent hand after this many tries; 0 waits forever.",
        ),
    ]
    for side in HAND_SIDES:
        args.extend(_per_side_args(share, side))

    # ORDER IS LOAD-BEARING. launch visits entities in sequence, so every declaration -- and the
    # `choices` refusal on `side` with it -- is reached before any driver action. Put a Node first
    # and `side:=middle` would connect, set the effort ceiling, enable the joints and home a live
    # hand before the launch aborted. test_hand_launch.py pins the order for this file.
    return LaunchDescription([*args, *(hand_node(side) for side in HAND_SIDES)])
