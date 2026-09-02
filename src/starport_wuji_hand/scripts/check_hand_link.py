#!/usr/bin/env python3
"""Open a Wuji hand read-only and report what it says about itself.

The first thing a bench session should run, because it is the smallest action that exercises the
one path nothing else can: opening the device. Every test in this package stubs the SDK, so the
connect sequence has never executed outside a bench, and a failure here is a failure of exactly
one thing rather than of a whole bring-up.

DE-ENERGIZED THROUGHOUT. This constructs a ``Hand`` and issues checked register reads. It never
calls ``write_joint_enabled``, ``write_joint_effort_limit`` or ``set_joint_target_position``, and
never opens a realtime controller, so no joint is energized and no setpoint is written -- the hand
cannot move as a result of running this. ``test_check_hand_link.py`` asserts that rather than
leaving it to this docstring.

    pixi run -e starport-deploy python \\
      ros2/starport_ws/src/starport_wuji_hand/scripts/check_hand_link.py
    pixi run -e starport-deploy python \\
      ros2/starport_ws/src/starport_wuji_hand/scripts/check_hand_link.py --side left

Exit status: 0 the link is good and the hand agrees with the committed limits; 1 the device could
not be opened or read; 2 it opened but its limits disagree, which is what makes the driver refuse
to start.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from starport_wuji_hand.joint_map import HAND_SIDES, NUM_JOINTS, joint_names
from starport_wuji_hand.limits_io import limits_filename, load_limits_mapping

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

# Same tolerance hand_node's cross-check uses, so agreement here predicts agreement there.
_TOLERANCE = 1e-4


def _report_limits(side: str, hw_lower: np.ndarray, hw_upper: np.ndarray) -> bool:
    """Print the hardware envelope beside the committed one. True when they agree."""
    declared = load_limits_mapping(str(CONFIG_DIR / limits_filename(side)))
    names = joint_names(side)
    disagreements = []
    print(f"\n{'joint':<26} {'hardware':>19}   {'committed':>19}")
    for i, name in enumerate(names):
        want_lower, want_upper = declared[name]
        agrees = abs(hw_lower[i] - want_lower) <= _TOLERANCE and abs(hw_upper[i] - want_upper) <= _TOLERANCE
        mark = " " if agrees else "  <-- differs"
        print(f"{name:<26} [{hw_lower[i]:8.4f},{hw_upper[i]:8.4f}]   " f"[{want_lower:8.4f},{want_upper:8.4f}]{mark}")
        if not agrees:
            disagreements.append(name)
    if disagreements:
        print(
            f"\n{len(disagreements)} joint(s) disagree: {disagreements}\n"
            "The driver refuses to start on this, deliberately: the reference trajectories were "
            "solved against the committed envelope, so a hand enforcing a different one must not "
            "be driven. Reconcile before bring-up."
        )
        return False
    print(f"\nAll {NUM_JOINTS} joints agree with {limits_filename(side)} to {_TOLERANCE}.")
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Open a Wuji hand read-only and report on it.")
    parser.add_argument("--side", default="right", choices=list(HAND_SIDES), help="which hand to open")
    parser.add_argument(
        "--serial",
        default="",
        help="USB serial of a specific hand; selects the device, while --side still says "
        "which hand it is and so which limits table to compare against",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="per-read timeout in seconds; the SDK's own default is 0.5",
    )
    args = parser.parse_args(argv)

    try:
        import wujihandpy  # noqa: PLC0415  -- lazy, so --help works with no SDK installed
    except ImportError as exc:
        print(f"wujihandpy unavailable: {exc}", file=sys.stderr)
        return 1
    print(f"wujihandpy {getattr(wujihandpy, '__version__', 'unknown')}")

    selector = f"serial_number={args.serial}" if args.serial else f"side={args.side}"
    print(f"opening the hand ({selector}) ...")
    try:
        if args.serial:
            hand = wujihandpy.Hand(serial_number=args.serial)
        else:
            hand = wujihandpy.Hand(side=args.side)
    except Exception as exc:
        print(
            f"could not open the hand: {type(exc).__name__}: {exc}\n"
            "Check that the device enumerates (`lsusb`) and that the udev rule in this package's "
            "udev/ directory is installed, reloaded and triggered.",
            file=sys.stderr,
        )
        return 1
    print("opened.")

    try:
        hw_lower = np.asarray(hand.read_joint_lower_limit(timeout=args.timeout), dtype=np.float64)
        hw_upper = np.asarray(hand.read_joint_upper_limit(timeout=args.timeout), dtype=np.float64)
        position = np.asarray(hand.read_joint_actual_position(timeout=args.timeout), dtype=np.float64)
    except Exception as exc:
        print(f"opened, but a register read failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    for label, array in (("lower limits", hw_lower), ("upper limits", hw_upper), ("position", position)):
        if array.shape != (NUM_JOINTS,):
            print(
                f"the hand reported {label} of shape {array.shape}, not ({NUM_JOINTS},); "
                "this is not the hand this package describes",
                file=sys.stderr,
            )
            return 1

    print("\nresting pose, radians, in hardware index order:")
    print("  " + np.array2string(position, precision=4, suppress_small=True, max_line_width=100))
    if not np.isfinite(position).all():
        print("some positions are not finite; the upstream stream may not have filled yet")

    agreed = _report_limits(args.side, hw_lower, hw_upper)
    print(
        "\nThe hand was not energized: no enable, no effort ceiling, no setpoint, no controller. "
        "It is in whatever state it was in before this ran."
    )
    return 0 if agreed else 2


if __name__ == "__main__":
    raise SystemExit(main())
