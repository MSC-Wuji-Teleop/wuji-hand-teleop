"""Setpoint-rate A/B on the whole hand: identical motion, only RATE_HZ changes.

Isolates the one variable. The motion is min-jerk (smooth, zero velocity and acceleration at both
ends), all 20 joints together, with the setpoint's own velocity sent -- i.e. exactly what the quiet
scripts do. The ONLY thing that differs between phases is how often a setpoint is published.

Each phase is: ease out to AMPLITUDE, DWELL, ease back to home, DWELL. The dwells matter -- the
noise being chased was reported at a HELD setpoint, which no motion-related cause explains.

Phases alternate so the ear gets an A/B/A/B rather than one comparison, and each reports the tick
period actually achieved (the historical noisy runs measured 1.0 ms mean / 1.11 ms p99 = 1 kHz) plus
tracking error, which is also how a 1 kHz freeze would show up: a joint stuck with a large, steady
error while the device still reports healthy.
"""

import time

import numpy as np
from wuji_sdk import DeviceType, JointCommand, SdkManager

KP, KD, EFFORT_LIMIT_A = 10.0, 0.2, 0.6  # kp 10 to match the historical runs, not the tuned 9
PHASES = (100.0, 1000.0, 100.0, 1000.0)  # Hz, in order
AMPLITUDE = 0.3  # rad on every joint, well inside all 20 ranges
MOVE_S = 2.0  # each ease-out / ease-back
DWELL_S = 2.5  # held at each end -- the condition the drone was reported at
HOME_S = 3.0  # initial min-jerk ramp from the resting pose to home

mgr = SdkManager.instance()
hand = mgr.connect(sn=next(d.sn for d in mgr.scan() if d.device_type == DeviceType.WujiHand2), device_name="rate_ab")
try:
    latest = {}
    # Keep the handle: the SDK releases the subscription when the object is dropped, so discarding
    # it here unsubscribes immediately and no frame ever arrives.
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

    def run_at(rate, seq):
        """Publish `seq` (pose, vel) frames at `rate`, returning tick and tracking statistics."""
        dt = 1.0 / rate
        periods, errors = [], []
        t0 = time.monotonic()
        last = None
        for i, (pose, vel) in enumerate(seq, start=1):
            pub.send([JointCommand(float(p), float(v), 0.0) for p, v in zip(pose, vel, strict=True)])
            now = time.monotonic()
            if last is not None:
                periods.append((now - last) * 1000.0)
            last = now
            errors.append(np.abs(np.asarray(pose) - measured()))
            wait = t0 + i * dt - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        ms, err = np.array(periods), np.array(errors)
        return (
            f"tick {ms.mean():.2f} ms mean / {np.percentile(ms, 99):.2f} p99 "
            f"(asked {1000 / rate:.2f})   err mean {np.nanmean(err):.4f} max {np.nanmax(err):.4f} rad"
        )

    def minjerk(frm, to, seconds, rate):
        """Frames easing frm -> to, each carrying its own velocity."""
        frm, to = np.asarray(frm, float), np.asarray(to, float)
        n = max(1, int(seconds * rate))
        for i in range(1, n + 1):
            t = i / n
            s = t**3 * (10.0 - 15.0 * t + 6.0 * t * t)
            sd = 30.0 * t * t * (1.0 - t) ** 2 / seconds
            yield frm + (to - frm) * s, (to - frm) * sd

    def dwell(pose, seconds, rate):
        """Hold. Zero velocity, because the setpoint has stopped moving."""
        for _ in range(int(seconds * rate)):
            yield np.asarray(pose, float), np.zeros(20)

    home, out = np.zeros(20), np.full(20, AMPLITUDE)
    print("ramping to home ...")
    print("   " + run_at(PHASES[0], minjerk(start, home, HOME_S, PHASES[0])))
    for k, rate in enumerate(PHASES, start=1):
        print(f"PHASE {k}/{len(PHASES)}: {rate:.0f} Hz  -- out, dwell {DWELL_S}s, back, dwell {DWELL_S}s", flush=True)
        seq = (
            list(minjerk(home, out, MOVE_S, rate))
            + list(dwell(out, DWELL_S, rate))
            + list(minjerk(out, home, MOVE_S, rate))
            + list(dwell(home, DWELL_S, rate))
        )
        print("   " + run_at(rate, seq), flush=True)
finally:
    hand.disable()  # the hand goes limp: unsupported fingers will settle
    hand.disconnect()
