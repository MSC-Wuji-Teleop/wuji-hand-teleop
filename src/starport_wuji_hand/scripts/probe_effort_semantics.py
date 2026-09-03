#!/usr/bin/env python3
"""Find out what the third field of a JointCommand actually does.

WHY THIS EXISTS. ``JointCommand`` is documented only as "position + velocity + effort", the
vendor's own example passes 0.0 for it, and nothing states the control law. If it is a FEEDFORWARD
term added to the MIT impedance torque, it is where a friction compensation belongs and the driver
already has the hook. If it is a per-command ceiling, or ignored, friction compensation needs a
different mechanism entirely. That is a design-deciding question and it is cheap to answer.

THE TEST. Hold ONE joint at the position it is already in, so the impedance loop is the only thing
acting on it, then step the effort field through small values either side of zero and watch where
the joint settles. A feedforward biases the equilibrium: the spring must stretch far enough to
cancel the injected torque, so the joint sits off-target by roughly effort/kp, PROPORTIONAL to the
value and SIGNED by it. A ceiling produces no offset at all while it stays above what the loop
needs. Anything else -- a jump, a runaway, no response either way -- is its own answer.

Deliberately small: the default probe never asks for more current than the joint draws moving
freely, and the target position never changes, so the loop is always pulling back toward where the
hand already was.

    pixi run -e starport-deploy python \\
      ros2/starport_ws/src/starport_wuji_hand/scripts/probe_effort_semantics.py
    ... --joint 4 --efforts -0.15,-0.05,0,0.05,0.15
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from starport_wuji_hand.joint_map import NUM_JOINTS, joint_names, nid_to_index
from wuji_sdk import DeviceType, JointCommand, SdkManager

_ENABLED = 2


def value_of(obj, name):
    v = getattr(obj, name)
    if callable(v):
        v = v()
    if hasattr(v, "get") and not isinstance(v, (str, int, float, bool, list, tuple, dict)):
        v = v.get()
    return v


class Latest:
    def __init__(self, stream):
        self._frame = None
        self._sub = stream.subscribe_with_callback(self._on)

    def _on(self, frame):
        self._frame = frame

    def get(self):
        return self._frame

    def wait(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._frame is not None and self._frame.joints:
                return self._frame
            time.sleep(0.005)
        raise TimeoutError("no joint_states frame")

    def close(self):
        try:
            self._sub.close()
        except Exception:
            pass


def read_joint(frame, index):
    for entry in frame.joints:
        if nid_to_index(entry.nid) == index:
            return float(entry.position), float(entry.effort)
    raise KeyError(index)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true", help="actually run; without it nothing moves")
    ap.add_argument("--joint", type=int, default=7, help="dense joint index (default 7 = index DIP)")
    ap.add_argument(
        "--efforts", default="0,0.05,0.1,0,-0.05,-0.1,0", help="comma-separated effort values to step through, in amps"
    )
    ap.add_argument("--dwell", type=float, default=2.0, help="seconds to hold each value")
    ap.add_argument("--settle", type=float, default=0.8, help="seconds to ignore before averaging")
    ap.add_argument("--rate", type=float, default=200.0, help="publish rate in Hz")
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--effort-limit", type=float, default=1.0, help="ceiling in amps")
    args = ap.parse_args(argv)

    efforts = [float(x) for x in args.efforts.split(",") if x.strip()]
    manager = SdkManager.instance()
    found = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not found:
        print("no Wuji Hand 2 on the network", file=sys.stderr)
        return 1
    hand = manager.connect(sn=found[0].sn, device_name="effort_probe")
    side = str(value_of(hand, "handedness"))
    label = joint_names(side)[args.joint]

    state = Latest(hand.joint_states())
    frame = state.wait()
    hold = [0.0] * NUM_JOINTS
    for entry in frame.joints:
        hold[nid_to_index(entry.nid)] = float(entry.position)
    start, _ = read_joint(frame, args.joint)

    print(f"hand {value_of(hand, 'serial_number')} ({side})   joint {args.joint} = {label}")
    print(f"holding it at {start:+.4f} rad and stepping effort through {efforts} A")
    if not args.execute:
        print("\nDRY RUN — nothing moved. Re-run with --execute.")
        state.close()
        hand.disconnect()
        return 0

    rows, pub = [], None
    try:
        hand.effort_limit().set(args.effort_limit)
        hand.mit_params().set((args.kp, args.kd))
        hand.enable()
        diag = Latest(hand.joint_diagnostics())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            f = diag.get()
            if f is not None and f.joints and all(e.status_word.ext_state == _ENABLED for e in f.joints):
                break
            time.sleep(0.02)
        else:
            print("enable timed out", file=sys.stderr)
            return 1
        pub = hand.joint_command().publish()

        dt = 1.0 / args.rate
        for ff in efforts:
            t0 = time.monotonic()
            positions, currents = [], []
            while time.monotonic() - t0 < args.dwell:
                pub.send([JointCommand(hold[i], 0.0, ff if i == args.joint else 0.0) for i in range(NUM_JOINTS)])
                if time.monotonic() - t0 >= args.settle:
                    f = state.get()
                    if f is not None and f.joints:
                        pos, cur = read_joint(f, args.joint)
                        positions.append(pos)
                        currents.append(cur)
                time.sleep(dt)
            offset = statistics.fmean(positions) - start if positions else float("nan")
            drawn = statistics.fmean(currents) if currents else float("nan")
            rows.append((ff, offset, drawn))
            print(f"  effort {ff:+.3f} A -> offset {offset:+.5f} rad   measured current {drawn:+.4f} A")
        return _verdict(rows, args)
    finally:
        # Zero the feedforward before releasing, so the last thing the joint saw is a plain hold.
        if pub is not None:
            try:
                neutral = [JointCommand(q, 0.0, 0.0) for q in hold]
                for _ in range(int(args.rate * 0.3)):
                    pub.send(neutral)
                    time.sleep(1.0 / args.rate)
            except Exception:
                pass
        hand.disable()
        state.close()
        hand.disconnect()
        print("de-energized and disconnected.")


def _verdict(rows, args) -> int:
    """Say what the data means, rather than leaving it to be eyeballed."""
    signed = [(ff, off) for ff, off in ((r[0], r[1]) for r in rows) if ff != 0.0]
    if not signed:
        return 0
    spread = max(off for _, off in signed) - min(off for _, off in signed)
    aligned = all((off > 0) == (ff > 0) for ff, off in signed if abs(off) > 1e-4)
    print()
    if spread < 5e-4:
        print("VERDICT: effort produced no measurable offset. It is NOT a feedforward torque here --")
        print("  most likely a per-command ceiling, or ignored. Friction compensation needs another hook.")
        return 2
    predicted = max(abs(ff) for ff, _ in signed) / args.kp
    print(f"VERDICT: effort moved the equilibrium by {spread:.4f} rad across the range,")
    print(f"  {'signed as expected' if aligned else 'NOT consistently signed -- investigate'}.")
    print(f"  A feedforward would predict about {predicted:.4f} rad at the largest value (effort/kp).")
    print("  If those agree to within a factor of a few, effort is the feedforward hook we want.")
    return 0 if aligned else 3


if __name__ == "__main__":
    raise SystemExit(main())
