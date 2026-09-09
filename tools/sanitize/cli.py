"""The command line: read a clip, sanitize it, write the same layout back.

    python3 tools/sanitize_clip.py IN.npz -o OUT.npz
    python3 tools/sanitize_clip.py clips/safe/<clip> -o clips/candidate/<clip>
    python3 tools/sanitize_clip.py IN.npz --check          # measure, write nothing

Order of work, per docs/spec/RoboSTAR_dropin_fix.md:

    1. read either layout (clipio), clamp the hand angles to the URDF limits
       and leave them otherwise untouched;
    2. remove 2pi wraps from the arm joints and cap the accumulated wrist
       drift (unwrap);
    3. extract the wrist placement and elbow position every source frame
       implies, by forward kinematics on the model (model.targets);
    4. find the branch flips: a joint jump the wrist pose did not follow
       (unwrap.detect_flips), and gate the elbow task for their duration;
    5. re-solve every frame, shoulder to elbow then elbow to wrist, seeded
       with the previous frame and bounded by the joint limits and a
       per-frame step (ik);
    6. sweep the full collision mesh set and hand every pair it finds to that
       frame's own solve as a separation constraint, carrying the active set
       to the next frame; report what the arms cannot part (collision,
       report);
    7. write the same layout out, plus sanitize.json.

Exit codes: 0 clean; 3 written, but N frames could not be cleared; 2 a
refused run (bad arguments, or a model that is not collision-free at rest);
1 an unexpected error.

What this tool does NOT do. It does not smooth -- prepare_clip.py owns the
Butterworth pass and the trim -- it does not touch legs or waist, it never
regenerates hand joints, and it does not judge the result: the dynamic
verdict still comes from tools/clip_audit.py, which has to be re-run on the
output before the clip is filed as safe.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from clip_audit import ARM_JOINT_NAMES, SIDES, sha256_file

from . import clipio, collision as coll, ik as ik_mod, unwrap as unwrap_mod
from .model import NUM_ARM_JOINTS, SanitizeModel
from .report import EXIT_REFUSED, Failure, Report

# Constants. Each one says where its value comes from.

# Source-to-solved elbow gap that ends a flipped span; a flip is centimetres.
DEFAULT_ELBOW_REJOIN_MM = 50.0

# Re-solves for a frame that moved into an unknown pair; each costs a sweep.
DEFAULT_AVOID_PASSES = 3

# Consecutive attempts clearing nothing before the frame is given up on.
AVOID_STALL_PASSES = 2

# Frames a pair stays active after last seen close; contact lasts tens of them.
ACTIVE_PAIR_MEMORY_FRAMES = 15

# Progress line every this many frames, so a long clip shows movement.
PROGRESS_EVERY = 50


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="sanitize_clip",
        description="Offline drop-in sanitizer for RoboSTAR retargeting output: "
                    "re-solves a joint-space clip against the G1 + Wuji Hand 2 URDF "
                    "with continuity, joint limits and full-mesh collision checking.")
    p.add_argument("input", type=Path,
                   help="flat npz (arm_q + left/right_hand_q20) or a replay clip directory")
    p.add_argument("-o", "--out", type=Path, default=None,
                   help="output path, same layout as the input; required unless --check")
    p.add_argument("--check", action="store_true",
                   help="measure and report only, write nothing")
    p.add_argument("--urdf", type=Path, default=None,
                   help="override the model (default: src/g1_wuji2_description/g1_29_wuji2.urdf)")

    g = p.add_argument_group("collision")
    g.add_argument("--clearance-mm", type=float, default=1e3 * coll.DEFAULT_CLEARANCE_M,
                   help="how far apart geometries must stay (default: %(default).1f)")
    g.add_argument("--no-collision", action="store_true",
                   help="skip collision checking entirely (kinematics only, much faster)")
    g.add_argument("--avoid-passes", type=int, default=DEFAULT_AVOID_PASSES,
                   help="re-solve attempts for a frame that moved into a pair the "
                        "solve did not know about (default: %(default)s)")
    g.add_argument("--avoid-iters", type=int, default=ik_mod.DEFAULT_AVOID_ITERS,
                   help="stage-3 iterations while separation rows are active "
                        "(default: %(default)s)")
    g.add_argument("--separation-weight", type=float,
                   default=ik_mod.DEFAULT_SEPARATION_WEIGHT,
                   help="weight of a separation row against the pose tasks, which are "
                        "1: higher gives up more pose to clear a contact "
                        "(default: %(default).1f)")
    g.add_argument("--avoid-budget-deg", type=float,
                   default=ik_mod.DEFAULT_AVOID_BUDGET_DEG,
                   help="how far the clearance solve may sit from the frame's "
                        "pose-faithful solution (default: %(default).1f)")
    g.add_argument("--fail-on", choices=("touch", "clearance"), default="touch",
                   help="what counts as a defect, to constrain the solve and then to "
                        "fail on: two meshes actually touching, or anything inside the "
                        "clearance (default: %(default)s)")

    g = p.add_argument_group("continuity")
    g.add_argument("--max-step-deg", type=float, default=ik_mod.DEFAULT_MAX_STEP_DEG,
                   help="extra cap on the per-frame joint step, on top of the URDF "
                        "velocity limit, which always applies (default: none; pass 15 "
                        "to match prepare_clip.py's clamp)")
    g.add_argument("--flip-step-deg", type=float, default=unwrap_mod.DEFAULT_FLIP_STEP_DEG,
                   help="joint jump that may be a branch flip (default: %(default).1f)")
    g.add_argument("--flip-pose-pos-mm", type=float,
                   default=1e3 * unwrap_mod.DEFAULT_FLIP_POSE_POS_M,
                   help="wrist travel below which such a jump IS a flip (default: %(default).1f)")
    g.add_argument("--flip-pose-ori-deg", type=float,
                   default=unwrap_mod.DEFAULT_FLIP_POSE_ORI_DEG,
                   help="wrist rotation below which such a jump IS a flip (default: %(default).1f)")
    g.add_argument("--elbow-rejoin-mm", type=float, default=DEFAULT_ELBOW_REJOIN_MM,
                   help="source-to-solved elbow gap that ends a flipped span "
                        "(default: %(default).1f)")
    g.add_argument("--ik-iters", type=int, default=ik_mod.DEFAULT_MAX_ITERS,
                   help="least-squares iterations per stage (default: %(default)s)")

    g = p.add_argument_group("drift")
    g.add_argument("--max-drift-deg", type=float, default=unwrap_mod.DEFAULT_MAX_DRIFT_DEG,
                   help="end-to-start wrist rotation a clip may keep; inf to only "
                        "measure it (default: %(default).1f)")
    g.add_argument("--drift-window", type=int, default=unwrap_mod.DEFAULT_DRIFT_WINDOW_FRAMES,
                   help="frames averaged at each end to measure drift (default: %(default)s)")
    g.add_argument("--drift-warn-deg", type=float, default=unwrap_mod.DEFAULT_DRIFT_WARN_DEG,
                   help="measured drift that earns a line on stderr (default: %(default).1f)")

    p.add_argument("-v", "--verbose", action="store_true",
                   help="print every failing contact rather than one line per "
                        "failing frame, and a line per constrained frame")
    return p


def _options_dict(args: argparse.Namespace) -> dict:
    """Every knob the run used, for the report."""
    return {"clearance_mm": args.clearance_mm, "collision": not args.no_collision,
            "fail_on": args.fail_on,
            "avoid_passes": args.avoid_passes, "avoid_iters": args.avoid_iters,
            "separation_weight": args.separation_weight,
            "avoid_budget_deg": args.avoid_budget_deg,
            "active_pair_memory_frames": ACTIVE_PAIR_MEMORY_FRAMES,
            "max_step_deg": args.max_step_deg, "flip_step_deg": args.flip_step_deg,
            "flip_pose_pos_mm": args.flip_pose_pos_mm,
            "flip_pose_ori_deg": args.flip_pose_ori_deg,
            "elbow_rejoin_mm": args.elbow_rejoin_mm, "ik_iters": args.ik_iters,
            "max_drift_deg": args.max_drift_deg, "drift_window": args.drift_window,
            "drift_warn_deg": args.drift_warn_deg}


def sanitize(clip: clipio.Clip, args: argparse.Namespace,
             stream: Optional[object] = None) -> tuple:
    """Sanitize one clip. Returns (new clip, Report).

    stream is resolved here, not in the signature: a default argument would
    bind sys.stderr at import time and escape any later redirection.
    """
    stream = sys.stderr if stream is None else stream
    sm = SanitizeModel(urdf=args.urdf, build_geometry=not args.no_collision)

    checker: Optional[coll.CollisionChecker] = None
    if not args.no_collision:
        checker = coll.CollisionChecker(sm, clearance_m=args.clearance_mm / 1e3)
        resting = checker.verify_model_at_rest()
        if resting:
            pairs = ", ".join(f"{c.pair.name_a}<->{c.pair.name_b}" for c in resting[:5])
            raise ValueError(
                f"the model already touches itself at its own neutral pose ({len(resting)} "
                f"pairs: {pairs}). The collision exclusion set no longer matches "
                f"{sm.urdf.name}; refusing rather than reporting every frame.")

    report = Report(options=_options_dict(args),
                    source={"path": str(clip.path), "layout": clip.layout,
                            "urdf": str(sm.urdf), "urdf_sha256": sha256_file(sm.urdf)},
                    frames=clip.frames, rate_hz=clip.rate_hz,
                    clearance_m=(checker.clearance_m if checker else 0.0),
                    drift_warn_deg=args.drift_warn_deg, drift_cap_deg=args.max_drift_deg)
    if checker is not None:
        report.collision = checker.as_dict()

    # -- hands: clamped to the URDF limits, otherwise passed through -------
    hand: Dict[str, np.ndarray] = {}
    for side in SIDES:
        raw = clip.hand[side]
        clamped = np.clip(raw, sm.hand_lower[side], sm.hand_upper[side])
        report.hand_clamped[side] = int(np.count_nonzero(np.abs(clamped - raw) > 1e-12))
        hand[side] = clamped

    # -- arms: wraps and drift, before anything is solved ------------------
    source: Dict[str, np.ndarray] = {}
    max_drift_rad = (math.inf if not math.isfinite(args.max_drift_deg)
                     else math.radians(args.max_drift_deg))
    for side in SIDES:
        lower = sm.arm_lower[ik_mod.side_columns(side)]
        upper = sm.arm_upper[ik_mod.side_columns(side)]
        wrapped = unwrap_mod.unwrap_trajectory(clip.arm[side], ARM_JOINT_NAMES[side],
                                               lower, upper)
        if wrapped.removed or wrapped.recentred or wrapped.kept:
            report.wraps[side] = wrapped.as_dict()
        capped, drift = unwrap_mod.cap_wrist_drift(
            wrapped.q, ARM_JOINT_NAMES[side], side, max_drift_rad, args.drift_window)
        report.drift[side] = [{"joint": d.joint,
                               "measured_deg": round(d.measured_deg, 3),
                               "removed_deg": round(d.removed_deg, 3),
                               "residual_deg": round(d.residual_deg, 3)} for d in drift]
        source[side] = capped

    # An out-of-range source angle is unreachable; counted so the report says why.
    out_of_limits = 0
    for side in SIDES:
        cols = ik_mod.side_columns(side)
        out_of_limits += int(np.count_nonzero(
            (source[side] < sm.arm_lower[cols] - 1e-9) | (source[side] > sm.arm_upper[cols] + 1e-9)))
    report.source["arm_values_outside_joint_limits"] = out_of_limits

    # -- the poses the source implies --------------------------------------
    targets: List[Dict[str, object]] = []
    wrist_poses: Dict[str, List[object]] = {s: [] for s in SIDES}
    for k in range(clip.frames):
        q = sm.configuration(arm={s: source[s][k] for s in SIDES},
                             hand={s: hand[s][k] for s in SIDES})
        frame_targets = sm.targets(q)
        targets.append(frame_targets)
        for side in SIDES:
            wrist_poses[side].append(frame_targets[side].wrist)

    # -- the branch flips ---------------------------------------------------
    flip_frames: Dict[str, set] = {}
    for side in SIDES:
        pos_delta, ori_delta = unwrap_mod.pose_deltas(wrist_poses[side])
        flips = unwrap_mod.detect_flips(
            source[side], ARM_JOINT_NAMES[side], side, clip.rate_hz,
            pos_delta, ori_delta,
            step_threshold_deg=args.flip_step_deg,
            pose_pos_tol_m=args.flip_pose_pos_mm / 1e3,
            pose_ori_tol_deg=args.flip_pose_ori_deg)
        report.flips[side] = [{"frame": f.frame, "t_s": round(f.t_s, 4), "joint": f.joint,
                               "step_deg": round(f.step_deg, 2),
                               "wrist_travel_mm": round(f.pose_pos_mm, 3),
                               "wrist_rotation_deg": round(f.pose_ori_deg, 3)} for f in flips]
        flip_frames[side] = {f.frame for f in flips}

    # -- the frame loop -----------------------------------------------------
    solver = ik_mod.ArmIK(sm, max_iters=args.ik_iters,
                          max_step_rad=ik_mod.step_limit(sm, clip.rate_hz, args.max_step_deg))
    solver.weights = ik_mod.IKWeights(separation=args.separation_weight)
    avoid_budget = np.full(NUM_ARM_JOINTS, math.radians(args.avoid_budget_deg))
    gates = {s: unwrap_mod.ElbowGate(args.elbow_rejoin_mm / 1e3) for s in SIDES}
    out_arm = np.zeros((clip.frames, NUM_ARM_JOINTS))
    previous: Optional[np.ndarray] = None
    solved_elbow: Dict[str, Optional[np.ndarray]] = {s: None for s in SIDES}
    started = time.time()

    # Pairs stage 3 solves against -> frame last seen close, carried across frames.
    active: Dict[coll.Pair, int] = {}

    for k in range(clip.frames):
        frame_hand = {s: hand[s][k] for s in SIDES}
        seed = (sm.join_arm({s: source[s][0] for s in SIDES}) if previous is None else previous)

        elbow_enabled: Dict[str, bool] = {}
        for side in SIDES:
            if k in flip_frames[side]:
                gates[side].close(k)
            elbow_enabled[side] = gates[side].update(k, targets[k][side].elbow,
                                                     solved_elbow[side])
            if not elbow_enabled[side]:
                report.gated_frames[side] += 1

        if checker is None:
            result = solver.solve(targets[k], seed=seed, hand=frame_hand,
                                  elbow_enabled=elbow_enabled, previous=previous)
        else:
            # --fail-on decides both what is constrained and what fails.
            def needs_clearing(contacts):
                return [c for c in contacts
                        if c.touching or args.fail_on == "clearance"]

            # Drop pairs clear long enough to stop being worth a row.
            active = {pair: seen for pair, seen in active.items()
                      if k - seen <= ACTIVE_PAIR_MEMORY_FRAMES}

            passes = 0
            stalled = 0
            last_pending: Optional[int] = None
            while True:
                def rows(q, _active=tuple(active)):
                    return checker.separation_rows(checker.recheck(q, _active))

                result = solver.solve(targets[k], seed=seed, hand=frame_hand,
                                      elbow_enabled=elbow_enabled, previous=previous,
                                      separation=(rows if active else None),
                                      separation_iters=args.avoid_iters,
                                      separation_budget_rad=avoid_budget)
                contacts = checker.check(
                    sm.configuration(arm_all=result.arm, hand=frame_hand),
                    classes=coll.ALL_CLASSES)
                fixable = [c for c in contacts if c.pair.fixable]
                pending = needs_clearing(fixable)

                # A pair the frame moved into joins the set; near misses stay too.
                learned = any(c.pair not in active for c in pending)
                for contact in fixable:
                    active[contact.pair] = k

                if not pending or passes >= args.avoid_passes:
                    break
                if last_pending is not None and len(pending) >= last_pending:
                    stalled += 1
                else:
                    stalled = 0
                if stalled >= AVOID_STALL_PASSES and not learned:
                    break
                last_pending = len(pending)
                passes += 1

            for contact in contacts:
                if not contact.pair.fixable:
                    report.observe_unfixable(contact, clip.time_of(k))
            if contacts:
                report.frames_with_contact += 1
            if passes and args.verbose:
                state = "cleared" if not pending else "FAILED"
                print(f"      frame {k:5d}  t={clip.time_of(k):8.3f}s  {state} "
                      f"after {passes} extra solve(s)", file=stream)
            # A defect left over is a failure; anything else close is a near miss.
            for contact in fixable:
                if contact.touching or args.fail_on == "clearance":
                    report.observe_failure(Failure(
                        frame=k, t_s=clip.time_of(k), klass=contact.pair.klass,
                        a=contact.pair.name_a, b=contact.pair.name_b,
                        gap_m=contact.gap_m, touching=contact.touching, passes=passes,
                        pos_err_m=result.max_pos_err_m, ori_err_rad=result.max_ori_err_rad))
                else:
                    report.observe_near_miss(contact, clip.time_of(k))

        report.observe_frame(k, result, previous)
        out_arm[k] = result.arm
        previous = result.arm.copy()
        sm.update(sm.configuration(arm_all=result.arm, hand=frame_hand))
        for side in SIDES:
            solved_elbow[side] = sm.data.oMf[sm.elbow_frame[side]].translation.copy()

        if PROGRESS_EVERY and (k + 1) % PROGRESS_EVERY == 0:
            elapsed = time.time() - started
            print(f"      {k + 1}/{clip.frames} frames  {elapsed:.1f}s  "
                  f"failures {len(report.failures)}", file=stream)

    out = clip.with_arm_all(out_arm)
    for side in SIDES:
        out.hand[side] = hand[side]
    return out, report


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.out is None and not args.check:
        print("sanitize_clip: give -o/--out, or --check to only measure", file=sys.stderr)
        return EXIT_REFUSED

    try:
        clip = clipio.read_clip(args.input)
        out, report = sanitize(clip, args)
    except (FileNotFoundError, ValueError, KeyError, RuntimeError) as error:
        print(f"sanitize_clip: {error}", file=sys.stderr)
        return EXIT_REFUSED

    report.print_failures(verbose=args.verbose)
    report.print_summary()

    if not args.check:
        written = clipio.write_clip(out, args.out, report=report.as_dict())
        for path in written:
            print(f"wrote {path}", file=sys.stderr)
        print("re-run tools/clip_audit.py on the output before filing it as safe",
              file=sys.stderr)
    return report.exit_code


if __name__ == "__main__":
    sys.exit(main())
