"""Hand drivers on their own, for the whole length of an interactive session.

    ros2 launch wuji_teleop_bringup hand_drivers.launch.py side:=both

One argument, `side` (left, right or both). It starts `hand.launch.py` for that
side and nothing else: no publisher, no connection check, no viewer.

**Why this file exists.** replay.launch.py starts the hand drivers and the
publisher together, and gives the publisher `on_exit=Shutdown()`, so the launch
takes the drivers down when the publisher ends. That is right for one clip per
invocation and wrong for a session: connecting to a hand costs 10 to 30 s (a
UDP broadcast scan, connect by serial, then the driver's blocking 3 s home), so
a session that plays several clips must outlive every publisher it starts.
Nothing here carries `on_exit=Shutdown()`, so this launch runs until it is
signalled. Design: docs/spec/spec1_2.md.

Not a mode of replay.launch.py, deliberately. Six of that file's seven
arguments (clip, arms, speed, ramp, check, sim) mean nothing without a
publisher, and a mode whose flags are mostly inert is where the rehome's
`--home --arms left` bug lived. One argument cannot have that class of bug.

**Who waits for the hands.** Not this file. It starts processes and returns.
Readiness is `replay_check --arms none --hands <side>`, run separately, which
is the same 30 s wait the publisher and scripts/replay.sh --check already use.

**What stopping this does to the hands.** Each `hand_node` disables its hand
and closes the link in teardown, so the hands de-energize and disconnect. That
is the only thing in the session that disconnects them: a side excluded from a
replay is still connected, merely not written to (docs/spec/spec1_2.md, "Three
states for a hand").
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# The driver launch this includes; its own `side` accepts left, right, both.
HAND_DRIVER_PACKAGE = "starport_wuji_hand"
HAND_DRIVER_LAUNCH = os.path.join("launch", "hand.launch.py")

# No "none" here, unlike replay.launch.py's sides: a session with no hand
# driver does not start this file at all.
SIDE_CHOICES = ("left", "right", "both")


def _configured_serials() -> tuple[str, str]:
    """Serials from wujihand_ik.yaml, or empty if that file is still placeholders / missing.

    Passing them is what stops two drivers racing for the first hand the scan
    returns. Imported lazily because hand_defaults reads the wujihand_output
    share directory at import time.

    Kept in step with the identically named helper in replay.launch.py. The
    duplication is deliberate: that file's tests monkeypatch
    ``get_package_share_directory`` in its own module namespace, so the lookup
    cannot move out of it without breaking them.
    """
    try:
        from wuji_teleop_bringup.hand_defaults import LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL
    except Exception:
        return "", ""
    left = "" if not LEFT_HAND_SERIAL or LEFT_HAND_SERIAL.startswith("YOUR_") else LEFT_HAND_SERIAL
    right = "" if not RIGHT_HAND_SERIAL or RIGHT_HAND_SERIAL.startswith("YOUR_") else RIGHT_HAND_SERIAL
    return left, right


def hand_drivers(side: str) -> IncludeLaunchDescription:
    """hand.launch.py for the selected side(s), with serials when they are configured."""
    left, right = _configured_serials()
    arguments = {"side": side}
    if left:
        arguments["left_serial_number"] = left
    if right:
        arguments["right_serial_number"] = right
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(HAND_DRIVER_PACKAGE), HAND_DRIVER_LAUNCH)
        ),
        launch_arguments=arguments.items(),
    )


def driver_actions(context: LaunchContext, *args, **kwargs) -> list:
    """Read `side`, refuse an unknown one, and return the single include."""
    side = LaunchConfiguration("side").perform(context)
    if side not in SIDE_CHOICES:
        raise RuntimeError(f"side:={side!r} is not one of {', '.join(SIDE_CHOICES)}")
    return [hand_drivers(side)]


def generate_launch_description() -> LaunchDescription:
    # Same order as replay.launch.py: the declaration and its `choices` refusal
    # are visited before the OpaqueFunction, which is the only thing that
    # produces a process.
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "side",
                default_value="both",
                choices=list(SIDE_CHOICES),
                description="Which hand driver(s) to start and keep running.",
            ),
            OpaqueFunction(function=driver_actions),
        ]
    )
