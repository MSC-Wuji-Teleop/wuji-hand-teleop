"""Pin hand_drivers.launch.py: one argument, one include, and nothing that ends the launch.

Loaded by path with the package-share lookup pointed at the source tree, the
same way test_replay_launch.py does it, so no installed workspace is needed and
no process is ever started.

The property that matters is negative: nothing here may carry
on_exit=Shutdown(). The whole reason this file exists is that the drivers have
to outlive every publisher an interactive session starts, and a Shutdown action
anywhere in it would take them down with the first one that ended
(docs/spec/spec1_2.md).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
LAUNCH_FILE = PACKAGE_DIR / "launch" / "hand_drivers.launch.py"

SOURCE_SHARE = {"starport_wuji_hand": SRC_DIR / "starport_wuji_hand"}

SIDES = ["left", "right", "both"]


@pytest.fixture
def launch_module(monkeypatch):
    """hand_drivers.launch.py loaded by path, with the ament index replaced by the source tree."""
    spec = importlib.util.spec_from_file_location("hand_drivers_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None, "hand_drivers.launch.py is missing"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(SOURCE_SHARE[package]))
    return module


def _actions(launch_module, **overrides: str):
    context = LaunchContext()
    context.launch_configurations.update({"side": "both"})
    context.launch_configurations.update(overrides)
    return context, launch_module.driver_actions(context)


def _text(context: LaunchContext, value) -> str:
    return value if isinstance(value, str) else perform_substitutions(context, value)


def _include_arguments(context: LaunchContext, include: IncludeLaunchDescription) -> dict[str, str]:
    return {_text(context, name): _text(context, value) for name, value in include.launch_arguments}


# ---------------------------------------------------------------- declarations


def test_declares_only_side(launch_module):
    entities = launch_module.generate_launch_description().entities
    declarations = [e for e in entities if isinstance(e, DeclareLaunchArgument)]
    assert [d.name for d in declarations] == ["side"]
    assert perform_substitutions(LaunchContext(), declarations[0].default_value) == "both"
    assert declarations[0].choices == SIDES


def test_only_the_opaque_function_produces_actions(launch_module):
    kinds = [type(e) for e in launch_module.generate_launch_description().entities]
    assert kinds == [DeclareLaunchArgument, OpaqueFunction]


def test_side_has_no_none_unlike_replay_launch(launch_module):
    """A session with no hand driver does not start this file at all, so `none`
    would be a way of asking for nothing."""
    assert "none" not in launch_module.SIDE_CHOICES
    with pytest.raises(RuntimeError, match="side:='none'"):
        _actions(launch_module, side="none")


def test_refuses_an_unknown_side(launch_module):
    with pytest.raises(RuntimeError, match="side:='middle'"):
        _actions(launch_module, side="middle")


# ---------------------------------------------------------------- what starts


@pytest.mark.parametrize("side", SIDES)
def test_the_side_reaches_the_driver_launch(launch_module, side):
    context, actions = _actions(launch_module, side=side)
    (include,) = actions
    assert isinstance(include, IncludeLaunchDescription)
    assert _include_arguments(context, include)["side"] == side


def test_it_points_at_a_hand_launch_file_that_exists(launch_module):
    """Asserted on the path this file builds rather than on the include object,
    whose source attribute is a launch internal."""
    assert launch_module.HAND_DRIVER_PACKAGE == "starport_wuji_hand"
    assert launch_module.HAND_DRIVER_LAUNCH == str(Path("launch") / "hand.launch.py")
    target = SOURCE_SHARE["starport_wuji_hand"] / launch_module.HAND_DRIVER_LAUNCH
    assert target.is_file(), f"{target} is not in the source tree"


def test_nothing_else_starts(launch_module):
    """No publisher, no check, no viewer: one include and nothing more."""
    _, actions = _actions(launch_module)
    assert len(actions) == 1
    assert not [a for a in actions if isinstance(a, Node)]


# ---------------------------------------------------------------- the point of the file


def test_no_action_ends_the_launch(launch_module):
    """The drivers must outlive every publisher the session starts.

    A Shutdown anywhere here would defeat the whole file: the first publisher
    to end would take the hand drivers with it and the next clip would pay the
    10 to 30 s reconnect again.
    """
    _, actions = _actions(launch_module)
    for action in actions:
        on_exit = getattr(action, "_ExecuteLocal__on_exit", None)
        assert not isinstance(on_exit, Shutdown)
    entities = launch_module.generate_launch_description().entities
    assert not [e for e in entities if isinstance(e, Shutdown)]


def test_serials_are_passed_when_configured(launch_module, monkeypatch):
    """Passing them is what stops the two drivers racing for the first hand the
    scan returns. A placeholder or empty value must not be passed on."""
    monkeypatch.setattr(launch_module, "_configured_serials", lambda: ("LSN123", "RSN456"))
    context, actions = _actions(launch_module)
    arguments = _include_arguments(context, actions[0])
    assert arguments["left_serial_number"] == "LSN123"
    assert arguments["right_serial_number"] == "RSN456"

    monkeypatch.setattr(launch_module, "_configured_serials", lambda: ("", ""))
    context, actions = _actions(launch_module)
    arguments = _include_arguments(context, actions[0])
    assert "left_serial_number" not in arguments
    assert "right_serial_number" not in arguments


def test_placeholder_serials_are_treated_as_unset(launch_module, monkeypatch):
    """wujihand_ik.yaml ships as a template whose serials read YOUR_LEFT_HAND_SERIAL."""
    import sys
    import types

    stub = types.ModuleType("wuji_teleop_bringup.hand_defaults")
    stub.LEFT_HAND_SERIAL = "YOUR_LEFT_HAND_SERIAL"
    stub.RIGHT_HAND_SERIAL = "RSN456"
    parent = sys.modules.get("wuji_teleop_bringup") or types.ModuleType("wuji_teleop_bringup")
    monkeypatch.setitem(sys.modules, "wuji_teleop_bringup", parent)
    monkeypatch.setitem(sys.modules, "wuji_teleop_bringup.hand_defaults", stub)
    assert launch_module._configured_serials() == ("", "RSN456")


def test_missing_hand_defaults_is_not_fatal(launch_module, monkeypatch):
    """hand_defaults reads a share directory at import time, so it can fail.
    A session must still be able to start drivers that scan for any hand."""
    import builtins

    real_import = builtins.__import__

    def refuse(name, *args, **kwargs):
        if name == "wuji_teleop_bringup.hand_defaults":
            raise ImportError("no share directory here")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", refuse)
    assert launch_module._configured_serials() == ("", "")
