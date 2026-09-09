"""Pins for tools/sanitize/collision.py: the pair set, the classes, clearance.

Needs Pinocchio, coal and the real URDF; the fixtures skip without them.
Building the collision geometry is slow, so the checker is session-scoped and
these tests only read from it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from clip_audit import HAND_JOINT_NAMES, SIDES
from sanitize import collision as coll
from sanitize import ik as ik_mod

OPEN_HAND = {side: np.zeros(len(HAND_JOINT_NAMES[side])) for side in SIDES}


def pair_named(checker, a, b):
    for pair in checker.pairs:
        if {pair.name_a, pair.name_b} == {a, b}:
            return pair
    return None


def crossed_arms(sm, reach=1.0):
    """A configuration that folds both hands into the middle of the chest.

    Shoulder roll inward, elbows up: the two hands end up in the same place,
    which is the "wrists phase through each other" geometry.
    """
    arm = {}
    for side in SIDES:
        sign = 1.0 if side == "left" else -1.0
        arm[side] = np.array([-0.35, -sign * reach * 0.35, 0.0, reach * 1.05, 0.0, 0.0, 0.0])
    return sm.join_arm(arm)


# -- the pair set -----------------------------------------------------------

def test_pair_set_excludes_adjacent_and_static_pairs(sanitize_checker):
    checker = sanitize_checker
    total = checker.pair_count + checker.excluded_adjacent + checker.excluded_static
    n = len(checker.sm.geom_model.geometryObjects)
    assert total == n * (n - 1) // 2
    assert checker.excluded_adjacent > 0 and checker.excluded_static > 0
    # Every class is populated, and they partition the set.
    counts = checker.counts_by_class()
    assert sum(counts.values()) == checker.pair_count
    assert all(v > 0 for v in counts.values())


def test_a_finger_and_its_own_next_segment_is_not_checked(sanitize_checker):
    # One joint apart, overlapping at the knuckle at every configuration.
    assert pair_named(sanitize_checker,
                      "left_wuji_l_index_finger_middle",
                      "left_wuji_l_index_finger_distal") is None


def test_two_locked_links_are_not_checked(sanitize_checker):
    # Both on the legs: nothing the clip writes can move either one.
    assert pair_named(sanitize_checker, "left_knee_link", "right_knee_link") is None


@pytest.mark.parametrize("a,b,expected", [
    ("left_wrist_yaw_link", "right_wrist_yaw_link", coll.CLASS_CROSS_SIDE),
    ("left_wuji_l_thumb_distal", "right_wuji_r_thumb_distal", coll.CLASS_CROSS_SIDE),
    ("left_wuji_l_thumb_distal", "left_wuji_l_pinky_distal", coll.CLASS_INTRA_HAND),
    ("left_wuji_l_thumb_distal", "torso_link", coll.CLASS_ARM_BODY),
    ("left_elbow_link", "torso_link", coll.CLASS_ARM_BODY),
    ("left_wuji_l_thumb_distal", "left_shoulder_roll_link", coll.CLASS_ARM_SELF),
])
def test_pairs_are_classified_by_the_chains_they_join(sanitize_checker, a, b, expected):
    pair = pair_named(sanitize_checker, a, b)
    assert pair is not None, f"{a} <-> {b} is not in the pair set"
    assert pair.klass == expected
    assert pair.fixable == (expected in coll.FIXABLE_CLASSES)


def test_intra_hand_is_the_only_unfixable_class():
    assert coll.CLASS_INTRA_HAND not in coll.FIXABLE_CLASSES
    assert set(coll.ALL_CLASSES) - set(coll.FIXABLE_CLASSES) == {coll.CLASS_INTRA_HAND}


# -- the model itself -------------------------------------------------------

def test_the_model_does_not_touch_itself_at_rest(sanitize_checker):
    """The check that validates the exclusion set and the adjacency depth.

    If this ever fails, the URDF or ADJACENCY_DEPTH changed and every frame of
    every clip would report the same pairs -- which is why the CLI refuses to
    run rather than reporting them.
    """
    assert sanitize_checker.verify_model_at_rest() == []


# -- contacts ---------------------------------------------------------------

def test_the_hands_folded_together_is_a_cross_side_contact(sanitize_model_geom,
                                                           sanitize_checker):
    sm, checker = sanitize_model_geom, sanitize_checker
    q = sm.configuration(arm_all=crossed_arms(sm), hand=OPEN_HAND)
    contacts = checker.check(q, classes=coll.ALL_CLASSES)
    cross = [c for c in contacts if c.pair.klass == coll.CLASS_CROSS_SIDE]
    assert cross, "folding both hands into the chest has to produce cross-side contact"
    assert any(c.touching for c in cross)
    for contact in cross:
        assert np.isclose(np.linalg.norm(contact.normal), 1.0)
        assert contact.gap_m <= checker.clearance_m


def test_arms_at_rest_report_nothing_fixable(sanitize_model_geom, sanitize_checker):
    sm = sanitize_model_geom
    assert sanitize_checker.check(sm.neutral(), classes=coll.FIXABLE_CLASSES) == []


def test_recheck_agrees_with_a_full_sweep(sanitize_model_geom, sanitize_checker):
    sm, checker = sanitize_model_geom, sanitize_checker
    q = sm.configuration(arm_all=crossed_arms(sm), hand=OPEN_HAND)
    swept = checker.check(q, classes=coll.ALL_CLASSES)
    pairs = [c.pair for c in swept]
    again = checker.recheck(q, pairs)
    assert {c.pair.index for c in again} == {c.pair.index for c in swept}
    by_index = {c.pair.index: c for c in again}
    for contact in swept:
        assert by_index[contact.pair.index].touching == contact.touching


# -- separation rows --------------------------------------------------------

def test_separation_rows_skip_unfixable_pairs_and_ask_for_the_clearance(
        sanitize_model_geom, sanitize_checker):
    sm, checker = sanitize_model_geom, sanitize_checker
    q = sm.configuration(arm_all=crossed_arms(sm), hand=OPEN_HAND)
    contacts = checker.check(q, classes=coll.ALL_CLASSES)
    rows = checker.separation_rows(contacts)
    assert rows
    assert all(row.contact.pair.fixable for row in rows)
    assert all(row.required_gain_m > 0.0 for row in rows)
    # A touching pair is asked for the whole clearance plus the overshoot.
    touching = [r for r in rows if r.contact.touching]
    assert touching
    assert all(r.required_gain_m == pytest.approx(
        checker.clearance_m + coll.REPAIR_OVERSHOOT_M) for r in touching)


def test_a_separation_row_points_the_way_that_separates(sanitize_model_geom,
                                                        sanitize_checker):
    """Following the row must increase the gap, or the solve pushes the wrong way."""
    sm, checker = sanitize_model_geom, sanitize_checker
    arm = crossed_arms(sm)
    q = sm.configuration(arm_all=arm, hand=OPEN_HAND)
    contacts = [c for c in checker.check(q, classes=coll.FIXABLE_CLASSES)
                if c.pair.klass == coll.CLASS_CROSS_SIDE]
    assert contacts
    rows = checker.separation_rows(contacts)

    solver = ik_mod.ArmIK(sm)
    solver._prepare(arm, OPEN_HAND)
    improved = 0
    for row in rows[:8]:
        jacobian = row.jacobian_row(solver)
        if np.allclose(jacobian, 0.0):
            continue
        step = 1e-3 * jacobian / np.linalg.norm(jacobian)
        moved = sm.configuration(arm_all=arm + step, hand=OPEN_HAND)
        before = checker.recheck(q, [row.contact.pair])
        after = checker.recheck(moved, [row.contact.pair])
        # Either it left the clearance band entirely, or its gap grew.
        if not after:
            improved += 1
        elif before and after[0].gap_m >= before[0].gap_m and not (
                before[0].touching and after[0].touching):
            improved += 1
        elif before and before[0].touching and not after[0].touching:
            improved += 1
    assert improved > 0, "no separation row moved its pair apart"


# -- clearance inside the solve ---------------------------------------------


def test_a_constrained_solve_parts_a_clearable_contact(sanitize_model_geom,
                                                       sanitize_checker):
    """The separation rows belong to stage 3, so solve() is what clears a frame.

    Measured by how far the hands are driven together: at reach 0.85 the solve
    parts them (58 pairs inside the clearance down to 48), at 1.00 it cannot
    (17 up to 24), which the next test pins.
    """
    sm, checker = sanitize_model_geom, sanitize_checker
    arm = crossed_arms(sm, reach=0.85)
    targets = sm.targets(sm.configuration(arm_all=arm, hand=OPEN_HAND))
    solver = ik_mod.ArmIK(sm)

    before = checker.check(sm.configuration(arm_all=arm, hand=OPEN_HAND),
                           classes=coll.FIXABLE_CLASSES)
    assert before, "the fixture configuration has to start in contact"
    pairs = tuple(c.pair for c in before)

    result = solver.solve(targets, seed=arm, hand=OPEN_HAND, previous=arm,
                          separation=lambda q: checker.separation_rows(
                              checker.recheck(q, pairs)),
                          separation_iters=ik_mod.DEFAULT_AVOID_ITERS)
    assert result.constrained
    assert "clearance" in result.iterations, "the clearance solve has to be its own stage"

    after = checker.check(sm.configuration(arm_all=result.arm, hand=OPEN_HAND),
                          classes=coll.FIXABLE_CLASSES)
    assert len(after) < len(before), "the constrained solve did not reduce the contact count"
    assert (sum(1 for c in after if c.touching)
            < sum(1 for c in before if c.touching)), "fewer pairs, but no fewer touching"
    assert np.all(result.arm >= sm.arm_lower - 1e-9)
    assert np.all(result.arm <= sm.arm_upper + 1e-9)


def test_an_unclearable_overlap_stays_inside_the_trust_region(sanitize_model_geom,
                                                              sanitize_checker):
    """A contact the arms cannot part must not cost the frame its pose.

    With no penetration depth to read, a touching pair asks for the whole
    clearance forever. Unbounded that walked the arm 48 deg off the pose, into
    more contacts than it started with.
    """
    sm, checker = sanitize_model_geom, sanitize_checker
    arm = crossed_arms(sm, reach=1.0)
    targets = sm.targets(sm.configuration(arm_all=arm, hand=OPEN_HAND))
    solver = ik_mod.ArmIK(sm)
    pairs = tuple(c.pair for c in checker.check(
        sm.configuration(arm_all=arm, hand=OPEN_HAND), classes=coll.FIXABLE_CLASSES))
    assert pairs

    pose_only = solver.solve(targets, seed=arm, hand=OPEN_HAND, previous=arm)
    result = solver.solve(targets, seed=arm, hand=OPEN_HAND, previous=arm,
                          separation=lambda q: checker.separation_rows(
                              checker.recheck(q, pairs)),
                          separation_iters=ik_mod.DEFAULT_AVOID_ITERS)

    budget = math.radians(ik_mod.DEFAULT_AVOID_BUDGET_DEG)
    assert np.all(np.abs(result.arm - pose_only.arm) <= budget + 1e-9)
    assert result.max_pos_err_m < 0.05
    assert np.all(result.arm >= sm.arm_lower - 1e-9)
    assert np.all(result.arm <= sm.arm_upper + 1e-9)


def test_a_second_checker_does_not_double_the_pair_set(sanitize_model_geom):
    """Constructing another checker resets the model's shared pair list."""
    first = coll.CollisionChecker(sanitize_model_geom)
    second = coll.CollisionChecker(sanitize_model_geom)
    assert first.pair_count == second.pair_count
    assert len(sanitize_model_geom.geom_model.collisionPairs) == second.pair_count
