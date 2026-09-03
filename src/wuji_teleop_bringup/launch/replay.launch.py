"""Replay launch: the hand drivers for the selected sides plus the clip publisher (teleop container).

What starts, per flag combination (docs/spec/spec1.md "Launch and the single terminal"):

    default              hand.launch.py side:=<hands> (starport_wuji_hand, one hand_node per side)
                         + replay_publisher --clip <clip> --arms <arms> --hands <hands> [--speed S]
                         (publisher waits for those drivers, then approaches frame 0)
    hands:=none          no hand driver; the publisher writes no hand topic
    check:=true          replay_check --arms <arms> --hands <hands> in place of the publisher
    sim:=true            no hand driver; publisher --ready-timeout 0; mujoco_visualizer.py
                         on g1_29_wuji2_fixed.xml next to the publisher (the viewer
                         mirrors the G1 node's arm commands and the publisher's hand commands)

The G1 node is never in this file: g1_world_output runs in its own container (CLAUDE.md), so
scripts/replay.sh on the host starts it before this launch and stops it after. The two-terminal
form in docs/replay.md shows the `docker compose run` line it uses. Exact operator lines:

    ros2 launch wuji_teleop_bringup replay.launch.py \\
        clip:=clips/safe/<clip> arms:=both hands:=both speed:=0.5
    ros2 launch wuji_teleop_bringup replay.launch.py clip:=clips/safe/<clip> sim:=true
    ros2 launch wuji_teleop_bringup replay.launch.py check:=true arms:=left hands:=none

Arguments: `clip` (default '', a directory under clips/safe/, resolved against the launch cwd),
`arms` and `hands` (none|left|right|both, default both), `speed` ('' means the clip's fastest safe
speed), `check` and `sim` (true|false, default false). An OpaqueFunction reads them and refuses a
bad combination -- no clip without check, an unknown side, arms none with hands none -- before any
process starts. hand.launch.py's own defaults (0.6 A effort, 2 rad/s slew, home on start, ten
connect attempts) are the driver's business; only `side` is passed to it.

The publisher and the check node carry on_exit=Shutdown(): when either ends -- a refused clip,
the check's verdict -- the launch takes the drivers down with it instead of leaving them running.

Exit code, established in the container (Humble, launch 1.0.14) with a throwaway launch file whose
only process was `bash -c 'exit 3'` under on_exit=Shutdown(): `ros2 launch` returned 0. The launch
service only returns 1 for an exception raised inside the launch itself, such as the refusals
above. The check's verdict therefore reaches scripts/replay.sh through launch's own log line for
the process ("[replay_check-N]: process has finished cleanly" or "process has died [pid P, exit
code E, ...]"), which the script captures and parses.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same set as replay.clip.SIDE_CHOICES; restated so this file imports nothing from a node package.
SIDE_CHOICES = ("none", "left", "right", "both")

# launch's own spelling of a boolean argument (what IfCondition accepts, case-insensitive).
BOOL_VALUES = {"true": True, "1": True, "false": False, "0": False}

# The hand driver launch (starport_wuji_hand); its `side` accepts left, right, both.
HAND_DRIVER_PACKAGE = "starport_wuji_hand"
HAND_DRIVER_LAUNCH = os.path.join("launch", "hand.launch.py")

# The replay package's two executables (src/input_devices/replay/setup.py console_scripts).
REPLAY_PACKAGE = "replay"
PUBLISHER_EXECUTABLE = "replay_publisher"
CHECK_EXECUTABLE = "replay_check"

# The sim viewer, installed by g1_world_output/setup.py into share/g1_world_output/scripts, and the
# composed 29-DoF model it opens (docs/replay.md "Sim").
VIEWER_PACKAGE = "g1_world_output"
VIEWER_SCRIPT = os.path.join("scripts", "mujoco_visualizer.py")
MODEL_PACKAGE = "g1_wuji2_description"
MODEL_FILE = "g1_29_wuji2_fixed.xml"


def _as_side(name: str, value: str) -> str:
    if value not in SIDE_CHOICES:
        raise RuntimeError(f"{name}:={value!r} is not one of {', '.join(SIDE_CHOICES)}")
    return value


def _as_bool(name: str, value: str) -> bool:
    try:
        return BOOL_VALUES[value.strip().lower()]
    except KeyError:
        raise RuntimeError(f"{name}:={value!r} is not a boolean; use true or false") from None


def hand_drivers(hands: str) -> IncludeLaunchDescription:
    """hand.launch.py for the selected side(s), with nothing but `side` passed."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(HAND_DRIVER_PACKAGE), HAND_DRIVER_LAUNCH)
        ),
        launch_arguments={"side": hands}.items(),
    )


def publisher(clip: str, arms: str, hands: str, speed: str, sim: bool) -> Node:
    """replay_publisher on the clip; its exit ends the launch."""
    # Absolute, so the path in the log and in the publisher's refusal message is unambiguous
    # whatever cwd the node process ends up with.
    arguments = ["--clip", os.path.abspath(clip), "--arms", arms, "--hands", hands]
    if speed:
        arguments += ["--speed", speed]
    # sim starts no hand drivers and may have no G1 state yet; waiting on
    # /{side}/wuji_hand/connected would hang until --ready-timeout.
    if sim:
        arguments += ["--ready-timeout", "0"]
    return Node(
        package=REPLAY_PACKAGE,
        executable=PUBLISHER_EXECUTABLE,
        output="screen",
        arguments=arguments,
        on_exit=Shutdown(),
    )


def connection_check(arms: str, hands: str) -> Node:
    """replay_check for the selected sources; its exit ends the launch."""
    return Node(
        package=REPLAY_PACKAGE,
        executable=CHECK_EXECUTABLE,
        output="screen",
        arguments=["--arms", arms, "--hands", hands],
        on_exit=Shutdown(),
    )


def viewer() -> ExecuteProcess:
    """mujoco_visualizer.py on the composed 29-DoF model."""
    return ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(get_package_share_directory(VIEWER_PACKAGE), VIEWER_SCRIPT),
            "--mjcf",
            os.path.join(get_package_share_directory(MODEL_PACKAGE), MODEL_FILE),
        ],
        output="screen",
    )


def replay_actions(context: LaunchContext) -> list:
    """Read the six arguments, refuse a bad combination, and return the actions to start.

    Runs when launch visits the OpaqueFunction, after the declarations and before anything else,
    so a RuntimeError here is reported by `ros2 launch` as a one-line error (exit 1) and no
    process has started.
    """
    clip = LaunchConfiguration("clip").perform(context)
    arms = _as_side("arms", LaunchConfiguration("arms").perform(context))
    hands = _as_side("hands", LaunchConfiguration("hands").perform(context))
    speed = LaunchConfiguration("speed").perform(context)
    check = _as_bool("check", LaunchConfiguration("check").perform(context))
    sim = _as_bool("sim", LaunchConfiguration("sim").perform(context))

    if arms == "none" and hands == "none":
        raise RuntimeError("arms:=none with hands:=none selects nothing to play or check")
    if not clip and not check:
        raise RuntimeError("clip:=<dir under clips/safe> is required unless check:=true")

    actions: list = []
    if hands != "none" and not sim:
        actions.append(hand_drivers(hands))
    if check:
        actions.append(connection_check(arms, hands))
    else:
        actions.append(publisher(clip, arms, hands, speed, sim))
    if sim:
        actions.append(viewer())
    return actions


def generate_launch_description() -> LaunchDescription:
    # ORDER IS LOAD-BEARING: the declarations (and the `choices` refusal on the sides) are visited
    # first, then the OpaqueFunction, which is the only thing that produces a process.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "clip",
                default_value="",
                description="Clip directory under clips/safe/ (relative to the launch cwd, or absolute). "
                "Required unless check:=true.",
            ),
            DeclareLaunchArgument(
                "arms",
                default_value="both",
                choices=list(SIDE_CHOICES),
                description="Arm topics the publisher writes; none publishes nothing to the G1 node.",
            ),
            DeclareLaunchArgument(
                "hands",
                default_value="both",
                choices=list(SIDE_CHOICES),
                description="Hand drivers to start and hand topics the publisher writes; none starts no driver.",
            ),
            DeclareLaunchArgument(
                "speed",
                default_value="",
                description="Playback speed, 0 < S <= 1; empty means the clip's fastest safe speed.",
            ),
            DeclareLaunchArgument(
                "check",
                default_value="false",
                description="Connection check: hand drivers and replay_check, no publisher.",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="No hand drivers; mujoco_visualizer.py on the composed model next to the publisher.",
            ),
            OpaqueFunction(function=replay_actions),
        ]
    )
