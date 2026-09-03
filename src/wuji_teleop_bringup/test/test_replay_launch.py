"""Pin replay.launch.py: its six arguments, its refusals, and the actions each flag combination produces.

The launch file is loaded by path and its package-share lookups are pointed at the source tree, so
the assertions do not depend on an installed workspace and no process is ever started. The
OpaqueFunction is called with a LaunchContext carrying the configurations, the way launch itself
would call it, and the assertions read the resolved command lines rather than the source.

Two tests do touch the installed tree, and skip when it is absent: the hand launch file the include
points at is loaded for real, and the viewer script is looked up through the ament index -- that is
the only place the g1_world_output data_files entry is observable.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
LAUNCH_FILE = PACKAGE_DIR / "launch" / "replay.launch.py"

# Where each package's share content sits in the source tree; stands in for the ament index.
SOURCE_SHARE = {
    "starport_wuji_hand": SRC_DIR / "starport_wuji_hand",
    "g1_world_output": SRC_DIR / "output_devices" / "g1_world_output",
    "g1_wuji2_description": SRC_DIR / "g1_wuji2_description",
}

EXPECTED_DEFAULTS = {"clip": "", "arms": "both", "hands": "both", "speed": "", "check": "false", "sim": "false"}
SIDES = ["none", "left", "right", "both"]


@pytest.fixture
def launch_module(monkeypatch):
    """replay.launch.py loaded by path, with the ament index replaced by the source tree."""
    spec = importlib.util.spec_from_file_location("replay_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None, "replay.launch.py is missing"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(SOURCE_SHARE[package]))
    return module


@pytest.fixture
def declared(launch_module):
    """The declared arguments by name, with their default resolved to the text an operator sees."""
    context = LaunchContext()
    return {
        action.name: (perform_substitutions(context, action.default_value), action)
        for action in launch_module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }


def _actions(launch_module, **overrides: str):
    """Call the OpaqueFunction's body the way launch would, with the defaults plus overrides."""
    context = LaunchContext()
    context.launch_configurations.update(EXPECTED_DEFAULTS)
    context.launch_configurations.update(overrides)
    return context, launch_module.replay_actions(context)


def _texts(context: LaunchContext, cmd) -> list[str]:
    """The plain-text parts of an expanded command line, in order.

    A Node's cmd also carries substitutions that only resolve inside a running launch (the
    executable lookup, the ros-args placeholders); those are skipped, the arguments are all text.
    """
    return [
        perform_substitutions(context, part)
        for part in cmd
        if all(isinstance(substitution, TextSubstitution) for substitution in part)
    ]


def _text(context: LaunchContext, value) -> str:
    """launch_ros keeps some fields as the plain str they were given and normalises others."""
    return value if isinstance(value, str) else perform_substitutions(context, value)


def _node(context: LaunchContext, actions, executable: str) -> Node | None:
    for action in actions:
        if isinstance(action, Node) and _text(context, action.node_executable) == executable:
            action._perform_substitutions(context)  # noqa: SLF001 - launch_ros exposes no public expansion
            return action
    return None


def _includes(actions) -> list[IncludeLaunchDescription]:
    return [action for action in actions if isinstance(action, IncludeLaunchDescription)]


def _include_arguments(context: LaunchContext, include: IncludeLaunchDescription) -> list[tuple[str, str]]:
    return [(_text(context, name), _text(context, value)) for name, value in include.launch_arguments]


def _processes(actions) -> list[ExecuteProcess]:
    return [action for action in actions if isinstance(action, ExecuteProcess) and not isinstance(action, Node)]


def _on_exit(action):
    return getattr(action, "_ExecuteLocal__on_exit")  # noqa: B009 - launch keeps on_exit private


def _flag(arguments: list[str], flag: str) -> str:
    return arguments[arguments.index(flag) + 1]


# ---------------------------------------------------------------- declarations


def test_declares_exactly_the_six_arguments_with_their_defaults(declared):
    assert {name: default for name, (default, _) in declared.items()} == EXPECTED_DEFAULTS


@pytest.mark.parametrize("name", ["arms", "hands"])
def test_the_sides_are_restricted_to_the_four_choices(declared, name):
    _, action = declared[name]
    assert action.choices == SIDES


def test_only_the_opaque_function_follows_the_declarations(launch_module):
    # Every process comes out of replay_actions, so the refusals in it run before anything starts.
    entities = launch_module.generate_launch_description().entities
    kinds = [type(entity) for entity in entities]
    assert kinds[:-1] == [DeclareLaunchArgument] * 6
    assert kinds[-1] is OpaqueFunction


# ---------------------------------------------------------------- refusals


def test_refuses_an_empty_clip_unless_checking(launch_module):
    with pytest.raises(RuntimeError, match="clip"):
        _actions(launch_module)
    _, actions = _actions(launch_module, check="true")
    assert actions


@pytest.mark.parametrize("name", ["arms", "hands"])
def test_refuses_an_unknown_side(launch_module, name):
    with pytest.raises(RuntimeError, match=f"{name}:='middle'"):
        _actions(launch_module, clip="clips/safe/x", **{name: "middle"})


def test_refuses_arms_none_with_hands_none(launch_module):
    with pytest.raises(RuntimeError, match="arms:=none with hands:=none"):
        _actions(launch_module, clip="clips/safe/x", arms="none", hands="none")


@pytest.mark.parametrize("name", ["check", "sim"])
def test_refuses_a_non_boolean_flag(launch_module, name):
    with pytest.raises(RuntimeError, match=f"{name}:='maybe'"):
        _actions(launch_module, clip="clips/safe/x", **{name: "maybe"})


# ---------------------------------------------------------------- what starts


def test_defaults_start_both_hand_drivers_and_the_publisher(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x")

    (include,) = _includes(actions)
    assert _include_arguments(context, include) == [("side", "both")]

    publisher = _node(context, actions, "replay_publisher")
    assert publisher is not None
    arguments = _texts(context, publisher.cmd)
    assert _flag(arguments, "--arms") == "both"
    assert _flag(arguments, "--hands") == "both"
    assert "--speed" not in arguments
    assert isinstance(_on_exit(publisher), Shutdown), "a refused clip must end the launch"

    assert _node(context, actions, "replay_check") is None
    assert _processes(actions) == []


def test_the_include_points_at_the_hand_driver_launch(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x")
    (include,) = _includes(actions)
    source = include.launch_description_source
    try:
        # Loading expands the location; it is also the one check that the include target parses.
        source.get_launch_description(context)
    except PackageNotFoundError:
        pytest.skip("starport_wuji_hand is not installed; the include target cannot be loaded here")
    assert source.location == str(SOURCE_SHARE["starport_wuji_hand"] / "launch" / "hand.launch.py")


def test_the_clip_is_passed_absolute_against_the_launch_cwd(launch_module, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    context, actions = _actions(launch_module, clip="clips/safe/x")
    publisher = _node(context, actions, "replay_publisher")
    assert _flag(_texts(context, publisher.cmd), "--clip") == str(tmp_path / "clips" / "safe" / "x")


def test_speed_is_passed_only_when_given(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x", speed="0.5")
    publisher = _node(context, actions, "replay_publisher")
    arguments = _texts(context, publisher.cmd)
    assert _flag(arguments, "--speed") == "0.5"


def test_hands_none_starts_no_driver(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x", hands="none")
    assert _includes(actions) == []
    publisher = _node(context, actions, "replay_publisher")
    assert _flag(_texts(context, publisher.cmd), "--hands") == "none"


def test_one_side_starts_that_driver_only(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x", hands="left", arms="none")
    (include,) = _includes(actions)
    assert _include_arguments(context, include) == [("side", "left")]
    assert _flag(_texts(context, _node(context, actions, "replay_publisher").cmd), "--arms") == "none"


def test_check_runs_replay_check_and_no_publisher(launch_module):
    context, actions = _actions(launch_module, check="true", arms="left", hands="none")
    check = _node(context, actions, "replay_check")
    assert check is not None
    arguments = _texts(context, check.cmd)
    assert _flag(arguments, "--arms") == "left"
    assert _flag(arguments, "--hands") == "none"
    assert "--clip" not in arguments
    assert isinstance(_on_exit(check), Shutdown), "the verdict must end the launch"
    assert _node(context, actions, "replay_publisher") is None
    assert _includes(actions) == []


def test_check_with_hands_still_starts_their_drivers(launch_module):
    context, actions = _actions(launch_module, check="true")
    (include,) = _includes(actions)
    assert _include_arguments(context, include) == [("side", "both")]
    assert _node(context, actions, "replay_check") is not None


def test_sim_starts_no_driver_and_opens_the_viewer_next_to_the_publisher(launch_module):
    context, actions = _actions(launch_module, clip="clips/safe/x", sim="true")
    assert _includes(actions) == []
    assert _node(context, actions, "replay_publisher") is not None

    (viewer,) = _processes(actions)
    cmd = _texts(context, viewer.cmd)
    assert cmd[0] == "python3"
    assert cmd[1].endswith("scripts/mujoco_visualizer.py")
    assert cmd[2:] == [
        "--mjcf",
        str(SOURCE_SHARE["g1_wuji2_description"] / "g1_29_wuji2_fixed.xml"),
    ]
    # Both paths are real files where the share directories point in the source tree.
    assert Path(cmd[1]).is_file()
    assert Path(cmd[3]).is_file()


def test_the_viewer_is_installed_where_the_launch_looks():
    # g1_world_output/setup.py has to install scripts/*.py into the share directory; the source
    # tree cannot show that, only the built workspace can.
    try:
        share = Path(get_package_share_directory("g1_world_output"))
    except PackageNotFoundError:
        pytest.skip("g1_world_output is not installed here")
    assert (share / "scripts" / "mujoco_visualizer.py").is_file()
    assert (share / "scripts" / "_mujoco_common.py").is_file(), "the viewer imports its sibling helper"
