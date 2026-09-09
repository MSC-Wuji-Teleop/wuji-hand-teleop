"""Pins for tools/sanitize/ik.py: the staging, the bounds, and branch recovery.

Needs Pinocchio and the real URDF, so everything here skips without them (the
sanitize_model fixture does the importorskip).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from clip_audit import HAND_JOINT_NAMES, SIDES
from sanitize import ik as ik_mod
from sanitize.model import NUM_ARM_JOINTS

pytestmark = pytest.mark.usefixtures("sanitize_model")

OPEN_HAND = {side: np.zeros(len(HAND_JOINT_NAMES[side])) for side in SIDES}


def smooth_arms(sm, frames=30):
    """(frames, 14) of smooth, in-limit arm motion around the stand pose.

    Built around the MJCF stand keyframe's arm values (shoulder pitch 0.2,
    roll +-0.2, elbow 1.28), which is a pose with the arms clear of the body.
    """
    t = np.linspace(0.0, 1.0, frames)
    out = np.zeros((frames, NUM_ARM_JOINTS))
    for i, side in enumerate(SIDES):
        sign = 1.0 if side == "left" else -1.0
        base = np.array([0.2, sign * 0.2, 0.0, 1.28, 0.0, 0.0, 0.0])
        for j in range(7):
            out[:, i * 7 + j] = base[j] + 0.25 * np.sin(2 * math.pi * t + 0.7 * j)
    return np.clip(out, sm.arm_lower + 1e-3, sm.arm_upper - 1e-3)


def targets_for(sm, arm):
    return sm.targets(sm.configuration(arm_all=arm, hand=OPEN_HAND))


def track(solver, sm, source, seed=None):
    """Solve a whole trajectory the way the CLI does, and return the result."""
    out = np.zeros_like(source)
    previous = None
    results = []
    for k in range(source.shape[0]):
        current_seed = source[0] if previous is None else previous
        result = solver.solve(targets_for(sm, source[k]), seed=current_seed,
                              hand=OPEN_HAND, previous=previous)
        out[k] = result.arm
        previous = result.arm.copy()
        results.append(result)
    return out, results


# -- plumbing ---------------------------------------------------------------

def test_side_columns_split_the_fourteen_dofs():
    assert list(ik_mod.side_columns("left")) == list(range(7))
    assert list(ik_mod.side_columns("right")) == list(range(7, 14))
    # Stage 1 owns shoulder pitch and roll; stage 2 owns the other five.
    assert list(ik_mod.side_columns("left", ik_mod.SHOULDER_ELBOW_LOCAL)) == [0, 1]
    assert list(ik_mod.side_columns("right", ik_mod.WRIST_CHAIN_LOCAL)) == [9, 10, 11, 12, 13]
    # Between them they cover every joint exactly once.
    assert sorted(ik_mod.SHOULDER_ELBOW_LOCAL + ik_mod.WRIST_CHAIN_LOCAL) == list(range(7))


def test_step_limit_is_the_velocity_limit_by_default(sanitize_model):
    limit = ik_mod.step_limit(sanitize_model, rate_hz=50.0)
    assert np.allclose(limit, sanitize_model.arm_velocity / 50.0)
    # 37 rad/s shoulder and elbow, 22 rad/s wrists -> 42 and 25 deg at 50 Hz.
    assert math.degrees(limit[0]) == pytest.approx(42.4, abs=0.2)
    assert math.degrees(limit[5]) == pytest.approx(25.2, abs=0.2)


def test_step_limit_intersects_a_configured_cap(sanitize_model):
    limit = ik_mod.step_limit(sanitize_model, rate_hz=50.0, max_step_deg=15.0)
    assert np.allclose(limit, math.radians(15.0))
    # A cap looser than the joint itself cannot loosen the joint.
    loose = ik_mod.step_limit(sanitize_model, rate_hz=50.0, max_step_deg=180.0)
    assert np.allclose(loose, sanitize_model.arm_velocity / 50.0)


# -- fidelity ---------------------------------------------------------------

def test_a_clean_trajectory_comes_back_unchanged(sanitize_model):
    sm = sanitize_model
    source = smooth_arms(sm)
    solver = ik_mod.ArmIK(sm, max_step_rad=ik_mod.step_limit(sm, 50.0))
    out, results = track(solver, sm, source)

    # What came from this model comes back: 0.2 deg here, 0.00013 on a real clip.
    assert np.degrees(np.abs(out - source)).max() < 0.2
    assert max(r.max_pos_err_m for r in results) < 1e-4
    assert max(r.max_ori_err_rad for r in results) < 1e-4
    assert max(r.max_elbow_err_m for r in results) < 1e-4


def test_every_stage_runs_and_is_counted(sanitize_model):
    sm = sanitize_model
    source = smooth_arms(sm, frames=3)
    solver = ik_mod.ArmIK(sm)
    result = solver.solve(targets_for(sm, source[1]), seed=source[0], hand=OPEN_HAND)
    assert set(result.iterations) == {"shoulder_to_elbow", "elbow_to_wrist", "refine"}
    assert all(v >= 1 for v in result.iterations.values())


def test_the_refine_stage_is_what_makes_the_pose_exact(sanitize_model):
    """Stage 2 has five joints for a six-DoF pose; stage 3 closes the gap."""
    sm = sanitize_model
    source = smooth_arms(sm, frames=12)
    solver = ik_mod.ArmIK(sm)

    staged_only = np.zeros_like(source)
    previous = None
    for k in range(source.shape[0]):
        seed = source[0] if previous is None else previous
        targets = targets_for(sm, source[k])
        lower, upper = solver._box(seed if previous is None else previous)
        arm = np.clip(seed, lower, upper)
        free = np.concatenate([ik_mod.side_columns(s, ik_mod.SHOULDER_ELBOW_LOCAL)
                               for s in SIDES])
        arm, _ = solver._stage(arm, hand=OPEN_HAND, targets=targets, free=free, seed=seed,
                               lower=lower, upper=upper,
                               use_wrist={s: False for s in SIDES},
                               use_elbow={s: True for s in SIDES})
        free = np.concatenate([ik_mod.side_columns(s, ik_mod.WRIST_CHAIN_LOCAL)
                               for s in SIDES])
        arm, _ = solver._stage(arm, hand=OPEN_HAND, targets=targets, free=free, seed=seed,
                               lower=lower, upper=upper,
                               use_wrist={s: True for s in SIDES},
                               use_elbow={s: False for s in SIDES})
        staged_only[k] = arm
        previous = arm.copy()

    full, _ = track(solver, sm, source)
    staged_error = np.degrees(np.abs(staged_only - source)).max()
    full_error = np.degrees(np.abs(full - source)).max()
    assert full_error < 0.05
    # Five joints cannot close a six-DoF pose: 13.5 deg against 0.0001 on a clip.
    assert staged_error > 10 * full_error


# -- bounds -----------------------------------------------------------------

def test_the_step_box_is_never_exceeded(sanitize_model):
    sm = sanitize_model
    source = smooth_arms(sm, frames=20)
    # Ask for a step far tighter than the motion needs, so the box binds.
    cap = ik_mod.step_limit(sm, rate_hz=50.0, max_step_deg=1.0)
    solver = ik_mod.ArmIK(sm, max_step_rad=cap)
    out, results = track(solver, sm, source)
    steps = np.abs(np.diff(out, axis=0))
    assert steps.max() <= cap.max() + 1e-9
    assert any(r.at_step_bound.any() for r in results)     # and it did bind


def test_joint_limits_are_never_exceeded(sanitize_model):
    sm = sanitize_model
    # Outside the reachable set: clamp, report a residual, stay commandable.
    source = smooth_arms(sm, frames=2)
    targets = targets_for(sm, source[0])
    beyond = sm.arm_upper + 1.0
    solver = ik_mod.ArmIK(sm)
    result = solver.solve(targets, seed=np.clip(beyond, sm.arm_lower, sm.arm_upper),
                          hand=OPEN_HAND)
    assert np.all(result.arm >= sm.arm_lower - 1e-9)
    assert np.all(result.arm <= sm.arm_upper + 1e-9)


# -- the branch flip --------------------------------------------------------

def alternative_branch(sm, solver, source_arm, seed):
    """Another exact solution for the same wrist poses, on a different branch.

    Found the way flipping happens: the wrist pose with no elbow task, from a
    far seed. 32 distinct exact solutions were found for one pose that way,
    which is what makes the flip test real rather than a guessed offset.
    """
    result = solver.solve(targets_for(sm, source_arm), seed=seed, hand=OPEN_HAND,
                          elbow_enabled={s: False for s in SIDES})
    return result


def test_an_alternative_branch_exists_and_holds_the_same_pose(sanitize_model):
    sm = sanitize_model
    solver = ik_mod.ArmIK(sm)
    source = smooth_arms(sm, frames=1)[0]
    seed = np.clip(source + np.array([0.6, -0.5, 1.4, 0.5, 1.2, 0.3, -0.4] * 2),
                   sm.arm_lower, sm.arm_upper)
    other = alternative_branch(sm, solver, source, seed)
    # A far seed converges less tightly; sub-millimetre is the same pose here.
    assert other.max_pos_err_m < 1e-3 and other.max_ori_err_rad < 1e-3
    # Same wrist pose, materially different joint angles: a branch, not noise.
    assert np.degrees(np.abs(other.arm - source)).max() > 20.0


def test_a_flipped_span_is_pulled_back_onto_the_previous_branch(sanitize_model):
    sm = sanitize_model
    solver = ik_mod.ArmIK(sm, max_step_rad=ik_mod.step_limit(sm, 50.0))
    source = smooth_arms(sm, frames=24)

    # A source that jumps to another exact branch for frames 8..15.
    seed = np.clip(source[8] + np.array([0.6, -0.5, 1.4, 0.5, 1.2, 0.3, -0.4] * 2),
                   sm.arm_lower, sm.arm_upper)
    flipped = source.copy()
    for k in range(8, 16):
        flipped[k] = alternative_branch(sm, solver, source[k], seed).arm
    jump = np.degrees(np.abs(np.diff(flipped, axis=0))).max()
    assert jump > 20.0, "the injected flip has to be a real jump"

    # Elbow task gated off across the span, as the CLI does after detect_flips.
    out = np.zeros_like(source)
    previous = None
    for k in range(source.shape[0]):
        gated = 8 <= k < 16
        result = solver.solve(targets_for(sm, flipped[k]),
                              seed=source[0] if previous is None else previous,
                              hand=OPEN_HAND,
                              elbow_enabled={s: not gated for s in SIDES},
                              previous=previous)
        out[k] = result.arm
        previous = result.arm.copy()

    # The output holds its branch and stays continuous where the source did not.
    assert np.degrees(np.abs(np.diff(out, axis=0))).max() < jump / 2.0
    assert np.degrees(np.abs(out[8:16] - source[8:16])).max() \
        < np.degrees(np.abs(flipped[8:16] - source[8:16])).max()
