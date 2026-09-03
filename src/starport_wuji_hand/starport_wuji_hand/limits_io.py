"""Read joint limits from the committed YAML, and from a standalone per-hand MJCF.

The committed YAML is what the guard chain clamps to. In this repo its source of truth is the
composed G1 + hands MJCF in ``src/g1_wuji2_description``: ``test_limits_match_mjcf.py`` holds each
hand's YAML to that model's joint ranges. ``find_mjcf`` and ``limits_from_mjcf`` read the vendor's
standalone per-hand MJCF (``assets/robots/wuji_hand2/mjcf/{side}.xml``), which this repo does not
carry; they are kept for a checkout that has it, and nothing here calls them at import.

Each hand gets its own YAML, projected from its own MJCF. The two hands' numbers are equal today,
but they are derived independently rather than mirrored, so a revision that makes them diverge
shows up as a changed file instead of a wrong clamp.

Note the attribute: the numbers come from each JOINT's ``range``, never from
``actuatorfrcrange`` (a force bound that also ends in "range" and is easy to grab by mistake).
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .joint_map import NUM_JOINTS, joint_names

_MJCF_RELDIR = Path("assets", "robots", "wuji_hand2", "mjcf")


def limits_filename(side: str) -> str:
    """Name of the committed limits YAML for ``side``."""
    return f"joint_limits_hand2_beta1_{side}.yaml"


def find_mjcf(side: str = "right") -> Path | None:
    """Locate the vendored MJCF for ``side``, or ``None`` when the asset is not fetched."""
    relpath = _MJCF_RELDIR / f"{side}.xml"
    for parent in Path(__file__).resolve().parents:
        candidate = parent / relpath
        if candidate.exists():
            return candidate
    return None


def load_limits_mapping(path: str) -> dict[str, tuple[float, float]]:
    """Load ``{joint_name: (lower, upper)}`` from a limits YAML.

    A malformed document raises ``ValueError`` naming the file and what was wrong with it. The
    driver node loads this at startup from its installed share path, where a raw ``KeyError`` --
    or an ``AttributeError`` from a ``joints:`` key that is not a mapping, whether a bare one that
    YAML loads as ``None`` or a sequence -- would reach a launch log naming neither the file nor
    the key.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    try:
        joints = doc["joints"]
        return {name: (float(v["lower"]), float(v["upper"])) for name, v in joints.items()}
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path} is not a usable limits YAML ({type(exc).__name__}: {exc})") from exc


def limits_from_mjcf(mjcf_path: str, side: str = "right") -> dict[str, tuple[float, float]]:
    """Derive ``{joint_name: (lower, upper)}`` from the MJCF's position actuators.

    Keyed by the actuator's ``joint`` attribute. The MJCF carries both naming systems -- an
    actuator named ``right_finger1_joint1_actuator`` drives a joint named ``r_thumb_cmc_flex`` --
    so reading the joint attribute takes the anatomical name from the asset itself rather than
    reconstructing it.

    Only actuated joints are returned: a joint with a range but no position actuator is not
    something the driver can command.
    """
    tree = ET.parse(mjcf_path)
    joint_range = {
        joint.get("name"): joint.get("range")
        for joint in tree.iter("joint")
        if joint.get("name") and joint.get("range")
    }
    out: dict[str, tuple[float, float]] = {}
    for actuator in tree.iter("position"):
        joint_name = actuator.get("joint")
        if not joint_name:
            continue
        rng = joint_range.get(joint_name)
        if rng is None:
            raise ValueError(f"joint {joint_name!r} has no range in {mjcf_path}")
        lower, upper = (float(v) for v in rng.split())
        out[joint_name] = (lower, upper)
    if len(out) != NUM_JOINTS:
        raise ValueError(f"expected {NUM_JOINTS} position actuators in {mjcf_path}, found {len(out)}")
    # The count alone would accept the other hand's MJCF sitting beside this one in the same
    # asset: it also has 20 position actuators. Checking the names turns that into a named
    # failure here instead of a bare KeyError in whatever consumes the result.
    expected = joint_names(side)
    unexpected = sorted(set(out) - set(expected))
    missing = [name for name in expected if name not in out]
    if unexpected or missing:
        raise ValueError(
            f"{mjcf_path} does not describe the {side} hand's {NUM_JOINTS} joints "
            f"(unexpected: {unexpected}, missing: {missing})"
        )
    return out


def load_friction(path: str, names: tuple[str, ...] | list[str]) -> list[float]:
    """Per-joint Coulomb friction in amps, ordered to match ``names``.

    Written by ``scripts/calibrate_joint_limits.py --tune``, which measures it as half the
    difference between the currents a joint draws traversing the same angle in each direction --
    the half that reverses with travel is friction, the half that does not is gravity.

    Refuses a table that does not name every joint of this hand. A missing entry silently
    defaulted to zero would leave one joint uncompensated and look like a hardware difference.
    """
    doc = json.loads(Path(path).read_text())
    table = doc.get("friction_a", doc)
    missing = [n for n in names if n not in table]
    if missing:
        raise ValueError(f"{path}: no friction for {missing}")
    unknown = [k for k in table if k not in names]
    if unknown:
        raise ValueError(f"{path}: friction for joints this hand does not have: {unknown}")
    values = [float(table[n]) for n in names]
    bad = [n for n, v in zip(names, values, strict=True) if not (v >= 0.0) or v == float("inf")]
    if bad:
        raise ValueError(f"{path}: friction must be finite and non-negative, got bad values for {bad}")
    return values
