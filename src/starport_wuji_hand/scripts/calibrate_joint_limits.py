#!/usr/bin/env python3
"""Discover each joint's reachable range on a real hand, one joint at a time.

WHY THIS EXISTS. The hand exposes no readable position-limit registers -- there is a
``soft_limit_enabled`` flag and a ``position_limit`` bit in each joint's status word, but no
lower/upper values to read. So the driver cannot cross-check a declared envelope against the
hardware's own at startup, the way it could over the vendor's USB SDK. This measures the envelope
instead, once, at commissioning.

WHAT IT PROVES. The MJCF envelope is what the reference trajectories were solved against. If every
MJCF bound is REACHABLE, the MJCF is a demonstrated-safe subset of what the hand allows, and the
driver can clamp to it knowing the firmware will never truncate a command it accepts. A bound that
is NOT reachable is the interesting result: it means the hand stops short of where the references
go, and that joint's clip data would be silently truncated.

HOW IT STAYS SAFE.
  * ONE JOINT MOVES AT A TIME. Every other joint holds the pose it was in when the sweep started,
    so the only travel is the one being measured and no combination of fingers closes on itself.
  * REDUCED EFFORT CEILING. The sweep runs at a fraction of the hand's 1.5 A default, so a joint
    that reaches a hard stop presses against it gently rather than at full current.
  * RAMPED TARGETS, NEVER STEPS. Under MIT impedance a stepped target is a torque impulse; every
    target here moves at a bounded rad/s from the measured pose.
  * A HARD CAP. The command never travels past the MJCF bound by more than ``--overshoot``, so a
    joint whose real limit is wider is still not explored beyond a known margin.
  * FOUR INDEPENDENT STOPS: the hand's own ``position_limit`` bit, a sustained tracking error, a
    fault-severity error code, and the cap above.
  * DE-ENERGIZED ON EVERY EXIT PATH, including Ctrl-C and any exception.

MIT GAINS: MEASURED HERE, BECAUSE THE VENDOR DOCUMENTS NONE.

Wuji's SDK reference specifies no kp/kd defaults, ranges or tuning guidance -- only that writes
reject NaN, Inf and negatives. The 3.0/0.05 in their publish example is a comment calling them
"conservative defaults", not a specification. So the numbers below are ours, measured on
WH2KA01260810003 (hw 0.2.0) against r_index_finger_mcp_flex.

  * PUBLISH RATE: NOT SHOWN TO MATTER. The default is 1 kHz because that is the hand's own loop
    rate, but no measurement here demonstrates it beats 200 Hz. The comparison that suggested it
    changed kp at the same time, and buzz returned at 1 kHz as soon as kp went back up -- so
    whatever the rate does, it does not govern the buzz. Treat this as an unvalidated default.
  * CURRENT IS NOT THE CONSTRAINT. This joint draws ~0.2-0.35 A peak whatever the ceiling is set
    to, at every speed tried from 0.08 to 0.6 rad/s. A joint that appears to saturate is being
    obstructed, not starved -- one such reading here turned out to be an operator holding the hand.
  * kp: 8 is clean, 10 is close, 15 buzzes -- and MORE DAMPING DOES NOT RESCUE kp 15 (tried 0.15
    and 0.4). That rules out simple under-damping and points at the loop interacting with backlash
    or encoder quantisation, which damping cannot fix.
  * kd HAS AN INTERIOR OPTIMUM near 0.2. Both 0.05 and 0.4 are worse, 0.4 markedly. Do not reach
    for damping as a buzz remedy here; it is as likely to amplify derivative noise.
  * SLOW IS THE WORST REGIME. Stick-slip dominates below ~0.1 rad/s, so a sweep should traverse at
    speed. That conflicts with meeting a hard stop gently, which is why an approach to a bound
    should slow down only in its final stretch.

The defaults below are provisional, chosen by ear on one joint. TUNING IS NOT FINISHED: they want
revisiting across joints, and "recommended gains for hand2 beta1" is worth asking Wuji directly.

Nothing moves without ``--execute``. The default prints the plan and exits.

    pixi run -e starport-deploy python \\
      ros2/starport_ws/src/starport_wuji_hand/scripts/calibrate_joint_limits.py
    pixi run -e starport-deploy python \\
      ros2/starport_ws/src/starport_wuji_hand/scripts/calibrate_joint_limits.py --execute
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from starport_wuji_hand.joint_map import NUM_JOINTS, joint_names, nid_to_index
from starport_wuji_hand.limits_io import limits_filename, load_limits_mapping
from wuji_sdk import DeviceType, JointCommand, SdkManager

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

# ext_state values reported in a joint's status word.
_ENABLED = 2


def frame_order(frame) -> list[int]:
    """The dense joint index of each entry in a state frame, in the frame's own order."""
    return [nid_to_index(j.nid) for j in frame.joints]


def require_canonical_order(order: list[int]) -> None:
    """Refuse a frame whose entries are not the dense order commands are built in.

    Commands are published as one positional list per frame, so an order mismatch would send one
    finger's targets to another. Loud here beats silent there.
    """
    if order != list(range(NUM_JOINTS)):
        raise RuntimeError(
            f"state frame is not in dense joint order (got {order}); commands are positional, so "
            "publishing against this frame would address the wrong joints"
        )


class StreamStalled(RuntimeError):
    """The state stream stopped delivering, so nothing measured after it can be trusted."""


def value_of(obj, name):
    """Read ``name`` off the SDK object, whatever shape it takes.

    The SDK mixes plain properties, zero-argument methods and resources returning a future, so a
    caller that guesses wrong gets a bound-method repr rather than an error. Resolving all three
    here keeps that guess out of every call site.
    """
    v = getattr(obj, name)
    if callable(v):
        v = v()
    if hasattr(v, "get") and not isinstance(v, (str, int, float, bool, list, tuple, dict)):
        v = v.get()
    return v


class Latest:
    """Holds the newest frame from a stream, updated on the SDK's callback thread.

    The publish loop must not drain the stream itself: at 999 Hz the number of queued frames --
    and so the cost of draining -- varies with how long the previous tick took, which feeds the
    loop's own jitter back into the publish period.
    """

    def __init__(self, stream):
        self._frame = None
        self._stamp = None
        self._sub = stream.subscribe_with_callback(self._on)

    def _on(self, frame):
        self._frame = frame
        self._stamp = time.monotonic()

    def get(self):
        return self._frame

    def age(self):
        """Seconds since the last frame, or None if none has arrived."""
        return None if self._stamp is None else time.monotonic() - self._stamp

    def wait(self, timeout):
        """Block until a frame with joints has arrived, or raise."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._frame
            if frame is not None and frame.joints:
                return frame
            time.sleep(0.005)
        raise TimeoutError("no frame arrived; is the hand streaming?")

    def close(self):
        try:
            self._sub.close()
        except Exception:
            pass


def sweep(hand, publisher, state_sub, diag_sub, index, target_bound, hold, args):
    """Ramp one joint toward ``target_bound`` and report where it actually stopped.

    ``hold`` is the full pose every other joint keeps. Returns a dict describing the stop.
    """
    names = joint_names(str(value_of(hand, "handedness")))
    label = names[index]
    start = hold[index]
    direction = 1.0 if target_bound > start else -1.0
    cap = target_bound + direction * args.overshoot
    leashed = False
    if args.max_travel > 0:
        leash = start + direction * args.max_travel
        if (direction > 0 and leash < cap) or (direction < 0 and leash > cap):
            cap, leashed = leash, True
    dt = 1.0 / args.rate
    span = abs(cap - start)

    # The budget follows the distance: a fixed timeout is shorter than the longest legitimate
    # sweep (index_pip travels 2.07 rad, 26 s at the default speed) and would report a good sweep
    # as a timeout. --joint-timeout is the floor, not the value.
    fast_part = max(0.0, abs(cap - start) - args.crawl_zone)
    slow_part = min(abs(cap - start), args.crawl_zone)
    budget = max(
        args.joint_timeout,
        (fast_part / args.speed + slow_part / args.crawl_speed) * 1.5 + 5.0,
    )

    periods = []
    last_tick = None
    # Full per-tick series. Effort against position along a joint's range IS a friction-versus-angle
    # curve, and summary statistics throw exactly that away -- so the sweep we already run can
    # answer whether friction is flat or structured, without a dedicated survey.
    series = {"t": [], "command": [], "measured": [], "effort": [], "speed": []}
    # Peaks and means over the whole sweep. A single end-of-sweep sample is not a summary of the
    # run: the target has stopped by then, so the damping term reads ~0 whatever kd was set to.
    peak_effort = 0.0
    errors = []
    reason, reached, measured, effort = "cap", start, start, 0.0
    stalled_since = None
    command = start
    t0 = time.monotonic()
    ticks = 0

    travelled = 0.0
    prev = time.monotonic()
    while True:
        now_t = time.monotonic()
        # Traverse at speed, crawl the last stretch. Stick-slip is worst at low speed, so creeping
        # the whole way is both slow and rough -- but arriving at a hard stop at full speed is how
        # a joint gets hurt, and precision is only needed at the end. Integrated rather than
        # computed from elapsed time, because the speed changes partway.
        remaining = span - travelled
        speed_now = args.crawl_speed if remaining <= args.crawl_zone else args.speed
        travelled += speed_now * (now_t - prev)
        prev = now_t
        if travelled >= span:
            command, reason = cap, ("leash" if leashed else "cap")
            _publish(publisher, hold, index, command)
            break
        command = start + direction * travelled
        _publish(publisher, hold, index, command)
        now = time.monotonic()
        if last_tick is not None:
            periods.append(now - last_tick)
        last_tick = now
        ticks += 1
        series["t"].append(now - t0)
        series["command"].append(command)
        series["measured"].append(measured)
        series["effort"].append(effort)
        series["speed"].append(speed_now)

        diag = diag_sub.get()
        if diag is not None and diag.joints:
            for entry in diag.joints:
                if nid_to_index(entry.nid) != index:
                    continue
                if getattr(entry.status_word, "position_limit", False):
                    reason = "position_limit"
                if getattr(entry.status_word, "ext_state", _ENABLED) != _ENABLED:
                    reason = f"ext_state={entry.status_word.ext_state}"
        if reason != "cap":
            break

        state = state_sub.get()
        if state is not None and state.joints:
            for entry in state.joints:
                if nid_to_index(entry.nid) != index:
                    continue
                measured = float(entry.position)
                effort = float(entry.effort)
            error = abs(command - measured)
            errors.append(error)
            peak_effort = max(peak_effort, abs(effort))
            # The crawl deliberately runs slowly, which is the regime where stick-slip makes
            # tracking error noisiest. Holding it to the traverse threshold reports friction as a
            # limit; scale the allowance with the speed actually being commanded.
            allowance = args.stall_error * (1.0 if speed_now >= args.speed else args.stall_crawl_factor)
            if error > allowance:
                stalled_since = stalled_since or time.monotonic()
                if time.monotonic() - stalled_since > args.stall_time:
                    reason = "stalled"
                    break
            else:
                stalled_since = None
            reached = measured

        stale = state_sub.age()
        if stale is None or stale > args.stream_timeout:
            raise StreamStalled(
                f"no joint_states for {stale if stale is None else round(stale, 2)}s while sweeping "
                f"{label}. Every reading after a stream stops is the frozen frame, which presents "
                "as every joint stalling in place -- so the run is abandoned rather than recorded."
            )
        if time.monotonic() - t0 > budget:
            reason = "timeout"
            break
        # Absolute schedule: sleeping a fixed dt would let every tick's own duration accumulate
        # into the publish period, so the rate would sag as the loop does more work.
        wait = (t0 + dt * ticks) - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    return {
        "joint": label,
        "index": index,
        "direction": "upper" if direction > 0 else "lower",
        "mjcf_bound": round(target_bound, 4),
        "commanded_stop": round(command, 4),
        "measured_stop": round(reached, 4),
        "effort_at_stop": round(effort, 4),
        "stop_reason": reason,
        "bound_reached": bool(abs(reached - target_bound) <= args.reach_tol),
        # Evidence about the ENVELOPE comes from one place only: the hand raising position_limit,
        # or the command reaching the MJCF bound unobstructed. A sweep cut short by our own
        # --max-travel says nothing, and a stall says only that this current ceiling was not
        # enough to keep moving -- which at a reduced ceiling is expected against gravity.
        "conclusive": reason in ("cap", "position_limit"),
        "ticks": ticks,
        "peak_effort": round(peak_effort, 4),
        "mean_error": round(sum(errors) / len(errors), 4) if errors else None,
        "max_error": round(max(errors), 4) if errors else None,
        "tick_ms": _stats(periods),
        "series": series,
    }


def _stats(periods):
    """Publish-period statistics in milliseconds. Jitter here is jitter the joint feels."""
    if not periods:
        return {}
    ms = sorted(p * 1000.0 for p in periods)
    return {
        "mean": round(sum(ms) / len(ms), 3),
        "min": round(ms[0], 3),
        "p50": round(ms[len(ms) // 2], 3),
        "p99": round(ms[min(len(ms) - 1, int(len(ms) * 0.99))], 3),
        "max": round(ms[-1], 3),
    }


def _publish(publisher, hold, index, value, ff=0.0, qd=0.0):
    """Command ``index`` to ``value`` and hold every other joint where it is.

    ``ff`` is a feedforward current in amps, added to the moving joint only. The device adds it
    to the impedance torque, so it shifts the equilibrium by roughly ff/kp -- which is how a
    Coulomb friction term cancels the stiction the position loop would otherwise have to wind
    up against.

    ``qd`` is the commanded velocity. Leaving it at zero -- as every path here did until now --
    does not merely forgo a feedforward: it asks the joint to be STATIONARY while also asking it
    to be somewhere else, so the damping term fights the motion with kd*qd of current the position
    error then has to overcome. At kd=0.2 and 0.3 rad/s that is 0.06 A of entirely self-inflicted
    drag, which is the same size as the friction being measured.
    """
    pose = list(hold)
    pose[index] = value
    commands = [JointCommand(p, 0.0, 0.0) for p in pose]
    if ff or qd:
        commands[index] = JointCommand(value, qd, ff)
    publisher.send(commands)


def ramp_to(publisher, frm, to, args):
    """Move the whole pose from ``frm`` to ``to`` at the configured speed."""
    span = max(abs(a - b) for a, b in zip(frm, to, strict=True)) if frm else 0.0
    steps = max(1, int((span / args.speed) * args.rate))
    dt = 1.0 / args.rate
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        a = i / steps
        pose = [f + (t - f) * a for f, t in zip(frm, to, strict=True)]
        publisher.send([JointCommand(p, 0.0, 0.0) for p in pose])
        wait = (t0 + dt * i) - time.monotonic()
        if wait > 0:
            time.sleep(wait)


def _frozen(series, args, window_s=1.0, still=2e-3, displaced=0.2, slack=0.5):
    """True once the joint has stopped following while the command left it far behind.

    Seen on this hardware: a joint stops dead mid-run and holds one position while the device
    still reports Enabled, 100% comm response, no limit flags and no error code -- and while its
    current sits near zero against an error that should saturate the ceiling. None of the
    device's own fields distinguish it, so the trace is the only evidence there is.

    The cause is the SETPOINT RATE, not the hardware. At --rate 1000 the right hand froze all four
    pinky joints at ~4.0 s, reproducibly across runs; at --rate 100, with everything else equal,
    the same four joints tracked cleanly. The driver publishes at 100 Hz and has never produced
    one. This guard stays because the failure is silent and a stale trace reads as a stiffer,
    lower-friction joint than the hardware has -- but a freeze now means the rate is too high.

    Everything after the freeze is a held position rather than a measurement, and it does not
    merely add noise: a stationary joint contributes no friction and no tracking error, so
    averaging it in invents a stiffer, lower-friction joint than the hardware has.

    Three conditions, because each alone has an innocent explanation. The joint must be still;
    the command must have been far away for the WHOLE window, not merely have jumped at the end
    of it, which is what a settled dwell followed by a step looks like; and the current must be
    slack, since a joint stuck against static friction is pushing as hard as it was told to while
    a frozen one is not being driven at all. Together they separate a freeze from both ordinary
    stiction and a normal step response.

    Slack is judged against what the loop is ASKING for -- kp times the error, capped by the
    ceiling -- not against the ceiling alone. At a low enough kp the demand never approaches the
    ceiling, so a fixed fraction of it would condemn every healthy low-gain run.
    """
    n = int(window_s * args.rate)
    if len(series["measured"]) < n:
        return False
    recent = series["measured"][-n:]
    if (max(recent) - min(recent)) >= still:
        return False
    errors = [abs(c - q) for c, q in zip(series["command"][-n:], recent, strict=True)]
    if min(errors) <= displaced:
        return False
    demanded = min(args.kp * min(errors) + abs(args.ff), args.effort_limit)
    return max(abs(e) for e in series["effort"][-n:]) < slack * demanded


def _record_diag(series, diag_sub, index):
    """Append the device's own status for ``index`` to the trace, or NaN if it has nothing to say."""
    frame = diag_sub.get()
    entry = None
    if frame is not None and frame.joints:
        for candidate in frame.joints:
            try:
                if nid_to_index(candidate.nid) == index:
                    entry = candidate
                    break
            except ValueError:
                continue
    if entry is None:
        for field in ("comm_pct", "error_code", "state", "limits", "temp_c"):
            series[field].append(float("nan"))
        return
    status = entry.status_word
    series["comm_pct"].append(float(entry.comm_response_rate_pct))
    series["error_code"].append(float(entry.error_code_current))
    series["state"].append(float(status.ext_state))
    # The three limit flags packed as bits: 1=current, 2=position, 4=velocity.
    series["limits"].append(
        float(bool(status.current_limit_active))
        + 2.0 * float(bool(status.position_limit_active))
        + 4.0 * float(bool(status.velocity_limit_active))
    )
    series["temp_c"].append(float(entry.mcu_temp_c_fb))


def _default_amplitude(args):
    return args.max_travel if args.max_travel > 0 else 0.3


def _room_for(pose, index, bounds, args, margin=0.05):
    """The largest symmetric triangle ``index`` can swing from ``pose`` without reaching a stop.

    A survey run from one pose has a different amount of room at every joint -- an abduction joint
    near its neutral has a fraction of a flexion joint's travel -- and a triangle that reaches the
    stop measures the stop, not the friction.
    """
    low, high = bounds
    room = min(pose[index] - (low + margin), (high - margin) - pose[index])
    return min(_default_amplitude(args), max(0.0, room))


def step_cycles(publisher, state_sub, diag_sub, index, hold, target, args):
    """Swing one joint between two held poses, dwelling at each end.

    A triangle never stops, which hides what stiction actually does: at a low kp the joint stops
    SHORT of a held target by roughly friction/kp and then simply sits there. Dwelling at each end
    is what makes that visible, and repeating the swing shows whether the shortfall is consistent
    or whether the joint sticks and breaks free at a different place each time.

    Unlike the triangle this does not abort on a freeze -- the point is to be watched, so it
    reports the freeze and keeps going rather than cutting the demonstration short.
    """
    start = hold[index]
    dt = 1.0 / args.rate
    move_ticks = max(1, int(abs(target - start) / args.speed * args.rate))
    dwell_ticks = max(1, int(args.hold * args.rate))
    series = {"t": [], "command": [], "measured": [], "effort": [], "direction": []}
    if diag_sub is not None:
        series.update({"comm_pct": [], "error_code": [], "state": [], "limits": [], "temp_c": []})
    t0 = time.monotonic()
    ticks = 0
    peak = 0.0
    frozen_at: float | None = None

    def emit(value, ff, direction):
        nonlocal ticks, peak, frozen_at
        _publish(publisher, hold, index, value, ff=ff)
        frame = state_sub.get()
        if frame is not None and frame.joints:
            for entry in frame.joints:
                if nid_to_index(entry.nid) != index:
                    continue
                series["t"].append(time.monotonic() - t0)
                series["command"].append(value)
                series["measured"].append(float(entry.position))
                series["effort"].append(float(entry.effort))
                series["direction"].append(direction)
                peak = max(peak, abs(float(entry.effort)))
                if diag_sub is not None:
                    _record_diag(series, diag_sub, index)
                if frozen_at is None and _frozen(series, args):
                    frozen_at = round(series["t"][-1], 3)
        ticks += 1
        wait = (t0 + dt * ticks) - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    def settled():
        """Mean measured position over the last third of a second, i.e. the end of a dwell."""
        n = min(len(series["measured"]), max(1, int(0.33 * args.rate)))
        return sum(series["measured"][-n:]) / n if n else float("nan")

    legs = []
    for cycle in range(1, args.cycles + 1):
        for frm, to in ((start, target), (target, start)):
            direction = 1.0 if to > frm else -1.0
            before = peak
            peak = 0.0
            for i in range(1, move_ticks + 1):
                emit(frm + (to - frm) * (i / move_ticks), args.ff * direction, direction)
            for _ in range(dwell_ticks):
                emit(to, 0.0, 0.0)
            rest = settled()
            legs.append(
                {
                    "cycle": cycle,
                    "target": round(to, 4),
                    "settled": round(rest, 4),
                    "shortfall": round(to - rest, 4),
                    "peak_effort": round(peak, 4),
                }
            )
            print(
                f"  cycle {cycle} -> {to:+.3f}: settled {rest:+.3f}  "
                f"short by {to - rest:+.4f} rad  peak {peak:.3f} A",
                flush=True,
            )
            peak = max(peak, before)

    if frozen_at is not None:
        print(f"  NOTE: froze at {frozen_at}s -- legs after that are not measurements.")
    return {
        "mode": "step",
        "joint": index,
        "start": round(start, 4),
        "target": round(target, 4),
        "legs": legs,
        "frozen_at": frozen_at,
        "series": series,
    }


def oscillate(hand, publisher, state_sub, index, hold, seconds, args, diag_sub=None, amplitude=None):
    """Rock one joint back and forth so gains can be judged on sustained motion.

    A single one-second pass is a poor instrument for hearing buzz or feeling stick-slip. This
    holds the same conditions for as long as asked, so the ear has something steady to judge, and
    reports the tracking and current the motion actually cost.
    """
    amplitude = _default_amplitude(args) if amplitude is None else amplitude
    start = hold[index]
    dt = 1.0 / args.rate
    period = 4.0 * amplitude / args.speed  # a full there-and-back at the configured speed
    t0 = time.monotonic()
    peak_effort, errors, ticks = 0.0, [], 0
    frozen_at = None
    # The triangle crosses every angle in BOTH directions, which is what makes friction separable:
    # at a given angle, half the difference between the two directions is friction and half the sum
    # is gravity. A one-way sweep cannot do this -- its two passes cover disjoint ranges.
    series = {"t": [], "command": [], "measured": [], "effort": [], "direction": []}
    # A joint that stops following looks identical whether its commands stopped landing, the
    # device clamped it at its own limit, or it faulted. Recording the device's own view along
    # with the motion is what tells those apart after the fact.
    if diag_sub is not None:
        series.update({"comm_pct": [], "error_code": [], "state": [], "limits": [], "temp_c": []})

    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= seconds or frozen_at is not None:
            break
        # Triangle wave: constant speed, direction reversing at the amplitude -- the same motion
        # profile a sweep uses, just repeated.
        # Quarter-period offset so the wave STARTS at the joint's current position. Starting at
        # phase 0 puts the first command a full amplitude away -- a step input, and the one thing
        # every other motion path here is careful not to produce.
        phase = ((elapsed / period) + 0.25) % 1.0
        offset = 4.0 * phase - 1.0 if phase < 0.5 else 3.0 - 4.0 * phase
        target = start + amplitude * offset
        # Coulomb compensation opposes the direction of travel, so it flips with the triangle.
        travel = 1.0 if phase < 0.5 else -1.0
        _publish(
            publisher,
            hold,
            index,
            target,
            ff=args.ff * travel,
            qd=args.speed * travel if args.qd_ref else 0.0,
        )

        frame = state_sub.get()
        if frame is not None and frame.joints:
            for entry in frame.joints:
                if nid_to_index(entry.nid) == index:
                    errors.append(abs(target - float(entry.position)))
                    peak_effort = max(peak_effort, abs(float(entry.effort)))
                    series["t"].append(elapsed)
                    series["command"].append(target)
                    series["measured"].append(float(entry.position))
                    series["effort"].append(float(entry.effort))
                    # +1 while the triangle is rising, -1 while falling.
                    series["direction"].append(1.0 if phase < 0.5 else -1.0)
                    if diag_sub is not None:
                        _record_diag(series, diag_sub, index)
                    if frozen_at is None and _frozen(series, args):
                        frozen_at = elapsed
        ticks += 1
        wait = (t0 + dt * ticks) - time.monotonic()
        if wait > 0:
            time.sleep(wait)

    return {
        "mode": "oscillate",
        "joint": index,
        "seconds": seconds,
        "amplitude": amplitude,
        "mean_error": round(sum(errors) / len(errors), 4) if errors else None,
        "max_error": round(max(errors), 4) if errors else None,
        "peak_effort": round(peak_effort, 4),
        "frozen_at": round(frozen_at, 3) if frozen_at is not None else None,
        "series": series,
    }


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--execute", action="store_true", help="actually move the hand; without it nothing moves")
    p.add_argument("--joints", default="", help="comma-separated dense indices to sweep (default: all 20)")
    p.add_argument(
        "--effort-limit",
        type=float,
        default=0.6,
        help="current ceiling in Amps during the sweep; the device default is 1.5",
    )
    p.add_argument("--kp", type=float, default=10.0, help="MIT position gain")
    p.add_argument("--kd", type=float, default=0.2, help="MIT damping gain")
    p.add_argument("--speed", type=float, default=0.4, help="target travel in rad/s")
    p.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="command publish rate in Hz. Matching the driver's command_rate is not only for "
        "comparability: at 1000 Hz this hand stops driving joints mid-run while still reporting "
        "them healthy, reproducibly and a whole finger at a time.",
    )
    p.add_argument("--crawl-speed", type=float, default=0.05, help="rad/s for the final approach into a bound")
    p.add_argument(
        "--crawl-zone", type=float, default=0.15, help="radians before the bound at which to drop to --crawl-speed"
    )
    p.add_argument(
        "--overshoot",
        type=float,
        default=0.05,
        help="radians the command may travel past the MJCF bound before stopping",
    )
    p.add_argument(
        "--stall-error", type=float, default=0.08, help="tracking error in radians that counts as pressing a stop"
    )
    p.add_argument("--stall-time", type=float, default=0.30, help="seconds that error must persist")
    p.add_argument(
        "--stall-crawl-factor",
        type=float,
        default=2.5,
        help="multiply --stall-error by this during the crawl, where stick-slip makes tracking "
        "error noisy for reasons unrelated to meeting a stop",
    )
    p.add_argument(
        "--reach-tol",
        type=float,
        default=0.05,
        help="radians within which the MJCF bound counts as reached. Must exceed the loop's "
        "steady-state tracking lag (~0.02-0.03 rad at the default gains) or a joint that arrived "
        "will be reported as falling short",
    )
    p.add_argument("--joint-timeout", type=float, default=25.0, help="seconds before abandoning one direction")
    p.add_argument(
        "--max-travel",
        type=float,
        default=0.0,
        help="cap every sweep this many radians from the start pose (0 = no cap). Use a "
        "small value for the first trial on a new hand: it bounds the motion while "
        "still showing whether the joint tracks its target",
    )
    p.add_argument(
        "--tune",
        type=float,
        default=0.0,
        help="instead of sweeping, rock the joint back and forth for this many seconds "
        "so gains can be judged on sustained motion",
    )
    p.add_argument(
        "--stream-timeout",
        type=float,
        default=0.5,
        help="abandon the run if joint_states goes quiet this long; the hand streams at "
        "~1 kHz, so half a second is unambiguous",
    )
    p.add_argument(
        "--step",
        type=float,
        default=0.0,
        help="swing one joint this many radians from where it is and dwell there, repeatedly, "
        "instead of sweeping; clamped to stay inside the joint's bounds",
    )
    p.add_argument("--cycles", type=int, default=4, help="how many there-and-back swings (--step)")
    p.add_argument("--hold", type=float, default=1.0, help="seconds to dwell at each end (--step)")
    p.add_argument(
        "--home",
        action="store_true",
        help="ramp to the home pose (a logical zero, clipped inside the limits) and survey from "
        "there, so a run does not depend on wherever the hand happened to be resting",
    )
    p.add_argument(
        "--qd-ref",
        action="store_true",
        help="send the commanded velocity in the reference instead of zero, so the damping term "
        "tracks the motion rather than opposing it",
    )
    p.add_argument(
        "--ff",
        type=float,
        default=0.0,
        help="Coulomb feedforward in amps, applied against the direction of travel (0 = off). "
        "Use the per-joint friction measured by a --tune triangle.",
    )
    p.add_argument("--out", default="", help="where to write the calibration JSON")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    manager = SdkManager.instance()
    found = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not found:
        print(
            "no Wuji Hand 2 on the network. Discovery is a UDP broadcast, so the host NIC has to "
            "be on the hand's own subnet -- see scripts/set_hand_ip.py --list",
            file=sys.stderr,
        )
        return 1
    hand = manager.connect(sn=found[0].sn, device_name="calibrate")

    side = str(value_of(hand, "handedness"))
    declared = load_limits_mapping(str(CONFIG_DIR / limits_filename(side)))
    names = joint_names(side)
    indices = [int(x) for x in args.joints.split(",") if x.strip()] or list(range(NUM_JOINTS))

    online = int(value_of(hand, "online_joints_count"))
    serial = str(value_of(hand, "serial_number"))
    print(f"hand {serial}  side={side}  online={online}/{NUM_JOINTS}")
    if online != NUM_JOINTS:
        print("refusing: not every joint is online, so a held pose cannot be guaranteed", file=sys.stderr)
        return 1

    state_sub = Latest(hand.joint_states())
    first = state_sub.wait(5.0)
    require_canonical_order(frame_order(first))
    start_pose = [float(j.position) for j in first.joints]

    print(f"start pose: {[f'{p:+.3f}' for p in start_pose]}")
    print(
        f"sweeping {len(indices)} joint(s) at {args.speed} rad/s, effort ceiling {args.effort_limit} A, "
        f"overshoot cap {args.overshoot} rad"
    )
    if not args.execute:
        print("\nDRY RUN — nothing moved. Re-run with --execute to sweep.")
        for i in indices:
            lo, hi = declared[names[i]]
            shown = []
            for bound in (lo, hi):
                d = 1.0 if bound > start_pose[i] else -1.0
                cap = bound + d * args.overshoot
                if args.max_travel > 0:
                    leash = start_pose[i] + d * args.max_travel
                    cap = min(cap, leash) if d > 0 else max(cap, leash)
                shown.append(f"{cap:+.3f}")
            print(
                f"  {i:>2} {names[i]:<26} from {start_pose[i]:+.3f} -> {shown[0]} and -> {shown[1]}"
                + ("   (leashed by --max-travel)" if args.max_travel > 0 else "")
            )
        state_sub.close()
        hand.disconnect()
        return 0

    results, publisher = [], None
    try:
        hand.effort_limit().set(args.effort_limit)
        hand.mit_params().set((args.kp, args.kd))
        hand.enable()
        diag_sub = Latest(hand.joint_diagnostics())
        if not _wait_enabled(diag_sub, timeout=5.0):
            print("enable timed out; not every motor reached Enabled", file=sys.stderr)
            return 1
        print("enabled.")
        publisher = hand.joint_command().publish()

        if args.home:
            margin = 0.05
            home = [
                min(max(0.0, declared[names[i]][0] + margin), declared[names[i]][1] - margin) for i in range(NUM_JOINTS)
            ]
            print(f"ramping to home: {[f'{p:+.3f}' for p in home]}")
            ramp_to(publisher, start_pose, home, args)
            start_pose = home

        if args.step:
            if len(indices) != 1:
                print("--step drives one joint; pass a single index to --joints", file=sys.stderr)
                return 1
            index = indices[0]
            low, high = declared[names[index]]
            start = start_pose[index]
            # Stay clear of the bound itself: the point is a clean swing, not a stall against a stop.
            margin = 0.05
            target = max(low + margin, min(high - margin, start + args.step))
            if abs(target - (start + args.step)) > 1e-6:
                print(f"clamped target to {target:+.3f}, {margin} rad inside [{low:+.3f}, {high:+.3f}]")
            print(
                f"swinging {names[index]} between {start:+.3f} and {target:+.3f} "
                f"({abs(target - start):.3f} rad) x{args.cycles} at kp={args.kp} kd={args.kd} "
                f"ff={args.ff} A, dwelling {args.hold}s at each end"
            )
            results.append(step_cycles(publisher, state_sub, diag_sub, index, start_pose, target, args))
            ramp_to(publisher, _current(state_sub, start_pose), start_pose, args)
            return _write(results, hand, side, args)

        if args.tune > 0:
            for i in indices:
                print(
                    f"  rocking {names[i]} for {args.tune:.0f}s at kp={args.kp} kd={args.kd} "
                    f"speed={args.speed} rate={args.rate:.0f}Hz ...",
                    flush=True,
                )
                amplitude = _room_for(start_pose, i, declared[names[i]], args)
                if amplitude < 0.05:
                    print(f"  SKIPPING {names[i]}: only {amplitude:.3f} rad of room from this pose")
                    continue
                r = oscillate(hand, publisher, state_sub, i, start_pose, args.tune, args, diag_sub, amplitude)
                print(f"    mean_err={r['mean_error']} max_err={r['max_error']} " f"peak_effort={r['peak_effort']}A")
                if r["frozen_at"] is not None:
                    print(
                        f"    FROZE at {r['frozen_at']}s: the joint stopped following while the "
                        "device still reported Enabled."
                    )
                    print("    Discard this run -- its error and friction figures are not measurements.")
                dropped = _disabled_joints(diag_sub)
                if dropped:
                    r["faulted_joints"] = [names[j] for j in dropped]
                    print(f"    WARNING: no longer Enabled: {r['faulted_joints']}")
                    print("    That trace is not trustworthy -- re-run before using it.")
                results.append(r)
                ramp_to(publisher, _current(state_sub, start_pose), start_pose, args)
            # Write, don't discard: the oscillation's series is the bidirectional friction data,
            # which is the whole reason to run one.
            return _write(results, hand, side, args)

        for i in indices:
            lo, hi = declared[names[i]]
            for bound in (lo, hi):
                print(f"  sweeping {names[i]} toward {bound:+.3f} ...", flush=True)
                r = sweep(hand, publisher, state_sub, diag_sub, i, bound, start_pose, args)
                results.append(r)
                print(
                    f"    stopped at {r['measured_stop']:+.3f} ({r['stop_reason']}), "
                    f"bound_reached={r['bound_reached']}"
                )
                ramp_to(publisher, _pose_with(start_pose, i, r["commanded_stop"]), start_pose, args)
        return _write(results, hand, side, args)
    finally:
        try:
            if publisher is not None:
                ramp_to(publisher, _current(state_sub, start_pose), start_pose, args)
        finally:
            hand.disable()
            state_sub.close()
            hand.disconnect()
            print("de-energized and disconnected.")


def _pose_with(pose, index, value):
    out = list(pose)
    out[index] = value
    return out


def _current(state_sub, fallback):
    frame = state_sub.get()
    return [float(j.position) for j in frame.joints] if frame is not None and frame.joints else list(fallback)


def _disabled_joints(diag_sub):
    """Indices whose motor is no longer Enabled.

    A joint that faults mid-run keeps streaming position while it stops being driven, so its
    trace is indistinguishable from a joint that simply refused to move -- low current, no
    motion, no error. Naming it is the difference between a hardware finding and a fabricated one.
    """
    frame = diag_sub.get()
    if frame is None or not frame.joints:
        return []
    dropped = []
    for entry in frame.joints:
        if entry.status_word.ext_state != _ENABLED:
            try:
                dropped.append(nid_to_index(entry.nid))
            except ValueError:
                pass
    return sorted(dropped)


def _wait_enabled(diag_sub, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = diag_sub.get()
        if frame is not None and frame.joints and all(e.status_word.ext_state == _ENABLED for e in frame.joints):
            return True
        time.sleep(0.05)
    return False


def _write(results, hand, side, args):
    serial = str(value_of(hand, "serial_number"))
    out = Path(args.out) if args.out else CONFIG_DIR / f"measured_limits_{serial}.json"
    doc = {
        "serial_number": serial,
        "handedness": side,
        "hw_version": str(value_of(hand, "hw_version")),
        "sweep": {
            k: getattr(args, k)
            for k in ("effort_limit", "kp", "kd", "speed", "overshoot", "stall_error", "stall_time", "reach_tol")
        },
        "joints": results,
    }
    # The per-tick series goes to a companion NPZ: thousands of points per sweep would bury the
    # summary this file exists to be readable as.
    arrays, trimmed = {}, []
    for r in doc["joints"]:
        r = dict(r)
        series = r.pop("series", None)
        if series and series.get("t"):
            # A one-way sweep is keyed by the bound it chased; a triangle covers both directions
            # at once and has no single direction.
            key = f"{r['joint']}__{r.get('direction', 'triangle')}"
            for field, values in series.items():
                arrays[f"{key}__{field}"] = np.asarray(values, dtype=np.float64)
        trimmed.append(r)
    doc["joints"] = trimmed
    if arrays:
        series_path = out.with_suffix(".series.npz")
        np.savez_compressed(series_path, **arrays)
        doc["series_file"] = series_path.name
        print(f"wrote {series_path} ({len(arrays) // 5} sweep series)")
    out.write_text(json.dumps(doc, indent=2) + "\n")
    # A triangle chases no bound, so the envelope verdict below covers only the sweeps.
    sweeps = [r for r in results if "conclusive" in r]
    inconclusive = [r for r in sweeps if not r["conclusive"]]
    unreached = [r["joint"] for r in sweeps if r["conclusive"] and not r["bound_reached"]]
    print(f"\nwrote {out}")
    if inconclusive:
        leashed = [r for r in inconclusive if r["stop_reason"] == "leash"]
        stalled = [r for r in inconclusive if r["stop_reason"] == "stalled"]
        if leashed:
            print(f"{len(leashed)} sweep(s) stopped at --max-travel; re-run without it to measure.")
        if stalled:
            print(f"{len(stalled)} sweep(s) STALLED: {sorted({r['joint'] for r in stalled})}")
            print(
                "  A stall is not an envelope finding. It means the joint stopped following at "
                f"{args.effort_limit} A and {args.speed} rad/s -- raise the current ceiling, lower "
                "the speed, or both, and re-run before concluding anything about those joints."
            )
    if unreached:
        print(f"{len(unreached)} bound(s) NOT reached: {sorted(set(unreached))}")
        print("Those joints stop short of the reference envelope; their clip data would be truncated.")
    elif sweeps and not inconclusive:
        print("every MJCF bound was reachable: the committed envelope is a safe subset.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
