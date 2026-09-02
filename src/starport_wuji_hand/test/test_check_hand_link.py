"""The bench link check must stay read-only.

Its whole value is that an operator can run it against a hand they have not yet decided to
energize, so the claim is enforced here rather than left to its docstring. Reading the source
rather than running it is the point: the script only reaches its SDK calls with a real device
attached, so no test could observe them.
"""

import ast
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "check_hand_link.py"

# Everything on the SDK that energizes a joint, commands one, or opens the realtime loop that
# would. A call to any of these makes the script something an operator cannot safely run first.
ENERGIZING_PREFIXES = ("write_", "set_joint_")
ENERGIZING_NAMES = frozenset({"realtime_controller"})


def _called_attributes() -> set[str]:
    tree = ast.parse(SCRIPT.read_text())
    return {
        node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }


def test_the_script_exists_and_parses():
    assert SCRIPT.is_file()
    ast.parse(SCRIPT.read_text())


def test_the_script_never_energizes_the_hand():
    called = _called_attributes()
    energizing = sorted(name for name in called if name in ENERGIZING_NAMES or name.startswith(ENERGIZING_PREFIXES))
    assert not energizing, (
        f"check_hand_link.py calls {energizing}, which can energize or command the hand; it is "
        "documented as safe to run before an operator has decided to power anything"
    )


def test_the_script_reads_the_registers_it_promises_to_report():
    # The other half of the gate: read-only is only useful if it still reads. A script that
    # quietly stopped reading the limits would pass the check above and tell the operator nothing.
    called = _called_attributes()
    for name in ("read_joint_lower_limit", "read_joint_upper_limit", "read_joint_actual_position"):
        assert name in called, f"{name} is no longer read, so the report cannot be trusted"
