"""Gate hand.launch.py's argument defaults against the driver's own declared defaults.

Most of the launch arguments restate a default the node already declares -- RESTATED_DEFAULTS names
them -- so the two can drift apart silently: an operator reads the launch file, the node uses its
own value. This resolves the launch file's declared defaults and compares them to a real node's, so
a change to either side without the other fails here.

The launch file is loaded by path and its ``get_package_share_directory`` is replaced, so nothing
here depends on the package being installed, and no node or process is ever launched. This is also
hand.launch.py's only automated coverage: the coverage run measures the Python package, and the
station bring-up smoke test launches the arm bringup rather than this file.
"""

import importlib.util
import tempfile
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from starport_wuji_hand.hand_node import WujiHandNode
from starport_wuji_hand.joint_map import NUM_JOINTS

PACKAGE_DIR = Path(__file__).resolve().parents[1]
LIMITS_YAML = str(PACKAGE_DIR / "config" / "joint_limits_hand2_beta1_right.yaml")

# limits_file is deliberately absent: the node declares "" (which it refuses to start on) and the
# launch file's job is to supply the packaged table. It is checked on its own below.
# {launch argument: the node parameter it feeds}. The device and calibration arguments are named
# per hand, so the two names differ for those; the policy arguments are shared and do not.
RESTATED_DEFAULTS = {
    "setpoint_velocity_filter_hz": "setpoint_velocity_filter_hz",
    "max_joint_velocity": "max_joint_velocity",
    "home_on_start": "home_on_start",
    "max_connect_attempts": "max_connect_attempts",
    "right_serial_number": "serial_number",
    "right_joint_sign": "joint_sign",
    "right_joint_offset": "joint_offset",
}


@pytest.fixture
def launch_module(monkeypatch):
    """hand.launch.py loaded by path, with the ament index out of the way."""
    spec = importlib.util.spec_from_file_location("hand_launch", PACKAGE_DIR / "launch" / "hand.launch.py")
    assert spec is not None and spec.loader is not None, "hand.launch.py is missing"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The module imported the symbol directly, so patching it here removes the ament index -- and
    # points the default limits table at the source tree, which is what makes it a real file.
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE_DIR))
    return module


@pytest.fixture
def launch_arguments(launch_module):
    """The launch file's declared arguments, resolved to the strings an operator would see."""
    context = LaunchContext()
    return {
        action.name: (perform_substitutions(context, action.default_value), action)
        for action in launch_module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }


def _driver_parameters(description, overrides: dict[str, str], side: str = "right") -> dict:
    """The parameters the driver node would really be launched with.

    Expansion is where a launch argument becomes a node parameter, so it is the only place the
    wiring between the two is observable. launch_ros writes them to a params file and names it on
    the command line; that file is what the node would read, so it is what this reads back.
    """
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(overrides)
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    for candidate in nodes:
        candidate._perform_substitutions(context)  # noqa: SLF001 - launch_ros exposes no public expansion
    node = next(candidate for candidate in nodes if candidate.expanded_node_namespace == f"/{side}")
    # cmd carries substitutions that only resolve inside a running launch, so only the plain-text
    # parts are readable here -- which is all the params-file path is.
    parts = [
        perform_substitutions(context, part)
        for part in node.cmd
        if all(isinstance(substitution, TextSubstitution) for substitution in part)
    ]
    path = parts[parts.index("--params-file") + 1]
    with open(path) as f:
        document = yaml.safe_load(f)
    return next(iter(document.values()))["ros__parameters"]


@pytest.fixture
def node_defaults():
    """Parameter defaults exactly as the driver declares them."""
    node = None
    try:
        node = WujiHandNode(cli_args=["--ros-args", "-p", f"limits_file:={LIMITS_YAML}"])
        yield {arg: node.get_parameter(param).value for arg, param in RESTATED_DEFAULTS.items()}
    finally:
        if node is not None:
            node.destroy_node()


def test_every_restated_default_matches_the_driver(launch_arguments, node_defaults):
    for name, expected in node_defaults.items():
        assert name in launch_arguments, f"{name} is no longer a launch argument"
        raw, _ = launch_arguments[name]
        # A launch argument is text; type it the way the node's own parameter is typed before
        # comparing, so "3.0" and 3.0 are not reported as a mismatch. A uniform array has to be
        # named by its item type -- bare `list` is not a type launch will coerce to.
        value_type = list[float] if isinstance(expected, list) else type(expected)
        typed = ParameterValue([TextSubstitution(text=raw)], value_type=value_type).evaluate(LaunchContext())
        assert typed == expected, f"{name}: launch says {typed!r}, node defaults to {expected!r}"


@pytest.mark.parametrize("side", ["right", "left"])
def test_the_default_limits_file_exists(launch_arguments, side):
    raw, _ = launch_arguments[f"{side}_limits_file"]
    assert Path(raw).is_file(), f"default {side}_limits_file does not exist: {raw}"


def test_side_selects_one_hand_or_both(launch_arguments):
    # The bench default is one hand; `both` is what the second one turns on. A typo is refused
    # here rather than reaching a node -- though the node refuses an unknown side of its own
    # accord too, which is what covers `ros2 run`.
    raw, action = launch_arguments["side"]
    assert raw == "right"
    assert action.choices == ["both", "right", "left"]


def test_one_driver_is_declared_per_hand(launch_module):
    entities = launch_module.generate_launch_description().entities
    assert len([entity for entity in entities if isinstance(entity, Node)]) == 2


def test_the_side_declaration_is_visited_before_the_driver(launch_module):
    # The refusal above only refuses because of where it sits: hand.launch.py's return says what a
    # reordered description would do to a live hand, and nothing else in this package would notice.
    entities = launch_module.generate_launch_description().entities
    declaration = next(
        index
        for index, entity in enumerate(entities)
        if isinstance(entity, DeclareLaunchArgument) and entity.name == "side"
    )
    driver = next(index for index, entity in enumerate(entities) if isinstance(entity, Node))
    assert declaration < driver, "a driver action is visited before the side refusal"


def test_a_correction_passed_at_the_command_line_reaches_the_driver(launch_module, tmp_path, monkeypatch):
    # The whole point of the correction arrays is that a finger-curl finding does not mean editing
    # code, so being declared as arguments is not enough -- they have to arrive as node parameters.
    # Expanding a Node writes them to a NamedTemporaryFile(delete=False) launch_ros never removes;
    # rehoming tempfile hands those to something that rotates.
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    flipped = str([1.0] * 6 + [-1.0] + [1.0] * 13)
    shifted = str([0.0] * 3 + [0.05] + [0.0] * 16)
    parameters = _driver_parameters(
        launch_module.generate_launch_description(),
        {"right_joint_sign": flipped, "right_joint_offset": shifted},
    )
    assert parameters["joint_sign"] == [1.0] * 6 + [-1.0] + [1.0] * 13
    assert parameters["joint_offset"] == [0.0] * 3 + [0.05] + [0.0] * 16


def test_the_driver_gets_the_correction_defaults_when_nothing_is_passed(launch_module, tmp_path, monkeypatch):
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    parameters = _driver_parameters(launch_module.generate_launch_description(), {})
    assert parameters["joint_sign"] == [1.0] * NUM_JOINTS
    assert parameters["joint_offset"] == [0.0] * NUM_JOINTS
