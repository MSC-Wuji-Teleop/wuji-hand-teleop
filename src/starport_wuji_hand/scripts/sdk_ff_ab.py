"""Coulomb-feedforward A/B on the whole hand: identical motion, only the ff current changes.

The remaining candidate for noise at a HELD setpoint. calibrate applies ff with a HARD SIGN --
`ff = args.ff * travel`, travel = +-1 from the triangle phase -- so full compensating current is
present at zero velocity. hand_node instead ramps ff through a velocity deadzone, and says why:
"a hard sign would apply the full compensating current at a standstill and step twice the friction
across every zero crossing ... and it would make a held pose creep."

This reproduces the hard sign on all 20 joints at once, and keeps it applied through the dwells,
where velocity is zero. Phases alternate ff off / ff on so the ear gets an A/B/A/B.

A held pose under constant ff should sit ff/kp off target -- 0.174/10 = 0.017 rad. The dwell offset
is reported, so we can confirm the current is really landing rather than guess.
"""

import time

import numpy as np
from wuji_sdk import DeviceType, JointCommand, SdkManager

KP, KD, EFFORT_LIMIT_A = 10.0, 0.2, 0.6  # as the historical runs had them
RATE_HZ = 100.0  # fixed: rate was already ruled out
FF_PHASES = (0.0, 0.174, 0.0, 0.174)  # amps; 0.174 is a value those sessions used
AMPLITUDE = 0.3  # rad on every joint, well inside all 20 ranges
MOVE_S = 2.0
DWELL_S = 3.0  # longer than the rate A/B: the dwell is the condition under test
HOME_S = 3.0

mgr = SdkManager.instance()
hand = mgr.connect(sn=next(d.sn for d in mgr.scan() if d.device_type == DeviceType.WujiHand2), device_name="ff_ab")
try:
    latest = {}
    # Keep the handle: the SDK releases the subscription when the object is dropped.
    sub = hand.joint_states().subscribe_with_callback(lambda f: latest.update({"f": f}))
    for _ in range(50):
        if latest.get("f") is not None and latest["f"].joints:
            break
        time.sleep(0.05)
    else:
        raise SystemExit("no joint_states frame arrived; refusing to ramp from an unknown pose")

    def measured():
        q = np.full(20, np.nan)
        for e in latest["f"].joints:
            bus, node = divmod(e.nid - 1, 5)  # every 5th nid is a tactile slot, not a joint
            if node < 4:
                q[bus * 4 + node] = e.position
        return q

    start = measured()
    hand.effort_limit().set(EFFORT_LIMIT_A)
    hand.mit_params().set((KP, KD))
    hand.enable()
    time.sleep(0.5)  # enable() is an action, not a landed write; publishing early races it
    pub = hand.joint_command().publish()
    dt = 1.0 / RATE_HZ

    def emit(seq, label=""):
        """Publish (pose, vel, ff) frames on an absolute schedule; report error and signed offset."""
        errors, offsets = [], []
        t0 = time.monotonic()
        for i, (pose, vel, ff) in enumerate(seq, start=1):
            pub.send([JointCommand(float(p), float(v), float(a)) for p, v, a in zip(pose, vel, ff, strict=True)])
            q = measured()
            errors.append(np.abs(np.asarray(pose) - q))
            offsets.append(np.asarray(pose) - q)  # signed: ff pushes the equilibrium one way
            wait = t0 + i * dt - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        return np.array(errors), np.array(offsets)

    def minjerk(frm, to, seconds, ff_amps):
        frm, to = np.asarray(frm, float), np.asarray(to, float)
        n = max(1, int(seconds * RATE_HZ))
        sign = 1.0 if (to - frm).sum() >= 0 else -1.0
        for i in range(1, n + 1):
            t = i / n
            s = t**3 * (10.0 - 15.0 * t + 6.0 * t * t)
            sd = 30.0 * t * t * (1.0 - t) ** 2 / seconds
            # HARD sign, deliberately: full magnitude regardless of how slowly the joint is moving.
            yield frm + (to - frm) * s, (to - frm) * sd, np.full(20, ff_amps * sign)

    def dwell(pose, seconds, ff_amps, sign):
        """Hold, with ff STILL APPLIED at full magnitude -- the standstill case under test."""
        for _ in range(int(seconds * RATE_HZ)):
            yield np.asarray(pose, float), np.zeros(20), np.full(20, ff_amps * sign)

    home, out = np.zeros(20), np.full(20, AMPLITUDE)
    print("ramping to home ...")
    emit(minjerk(start, home, HOME_S, 0.0))
    for k, ff in enumerate(FF_PHASES, start=1):
        print(f"PHASE {k}/{len(FF_PHASES)}: ff = {ff:.3f} A  (hard sign, held through the dwells)", flush=True)
        seq = (
            list(minjerk(home, out, MOVE_S, ff))
            + list(dwell(out, DWELL_S, ff, +1.0))
            + list(minjerk(out, home, MOVE_S, ff))
            + list(dwell(home, DWELL_S, ff, -1.0))
        )
        err, off = emit(seq)
        d0 = int(MOVE_S * RATE_HZ)
        dwell_off = off[d0 : d0 + int(DWELL_S * RATE_HZ)]
        print(
            f"   err mean {np.nanmean(err):.4f} max {np.nanmax(err):.4f} rad   "
            f"dwell offset {np.nanmean(dwell_off):+.4f} rad (predicted {ff / KP:+.4f})",
            flush=True,
        )
finally:
    hand.disable()  # the hand goes limp: unsupported fingers will settle
    hand.disconnect()
