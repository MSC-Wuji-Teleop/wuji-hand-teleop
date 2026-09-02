"""Guard-chain unit tests. No ROS, no hardware -- the guards protect real actuators, so their
logic is proven here where every edge case is cheap to reach."""

import numpy as np
import pytest
from starport_wuji_hand.joint_map import NUM_JOINTS, joint_names, resolve_command
from starport_wuji_hand.safety import GuardChain, Limits

RIGHT_NAMES = joint_names("right")

RAW = {name: (-1.0, 1.0) for name in RIGHT_NAMES}


def make_limits(margin: float = 0.0) -> Limits:
    return Limits.from_mapping(RAW, margin=margin, names=RIGHT_NAMES)


def make_chain(margin: float = 0.0, max_velocity: float = 1e9, timeout_s: float = 1e9) -> GuardChain:
    return GuardChain(
        limits=make_limits(margin),
        max_velocity=np.full(NUM_JOINTS, max_velocity),
        timeout_s=timeout_s,
        initial=np.zeros(NUM_JOINTS),
    )


def test_from_mapping_applies_margin_to_both_sides():
    limits = make_limits(margin=0.05)
    np.testing.assert_allclose(limits.lower, np.full(NUM_JOINTS, -0.95))
    np.testing.assert_allclose(limits.upper, np.full(NUM_JOINTS, 0.95))


def test_from_mapping_rejects_missing_joint():
    partial = {name: (-1.0, 1.0) for name in RIGHT_NAMES[:-1]}
    with pytest.raises(ValueError, match="missing"):
        Limits.from_mapping(partial, margin=0.0, names=RIGHT_NAMES)


def test_from_mapping_rejects_unknown_joint():
    extra = dict(RAW)
    extra["right_finger9_joint1"] = (-1.0, 1.0)
    with pytest.raises(ValueError, match="unknown"):
        Limits.from_mapping(extra, margin=0.0, names=RIGHT_NAMES)


def test_from_mapping_rejects_inverted_range():
    bad = dict(RAW)
    bad["r_thumb_cmc_flex"] = (1.0, -1.0)
    with pytest.raises(ValueError, match="lower >= upper"):
        Limits.from_mapping(bad, margin=0.0, names=RIGHT_NAMES)


def test_from_mapping_rejects_margin_that_inverts_a_range():
    with pytest.raises(ValueError, match="margin"):
        Limits.from_mapping(RAW, margin=1.5, names=RIGHT_NAMES)


def test_cross_check_passes_on_agreement():
    limits = make_limits(margin=0.05)
    # Compared against the PRE-margin bounds, so the margin must not cause a false alarm.
    limits.cross_check(np.full(NUM_JOINTS, -1.0), np.full(NUM_JOINTS, 1.0))


def test_cross_check_raises_and_names_disagreeing_joints():
    limits = make_limits()
    hw_lower = np.full(NUM_JOINTS, -1.0)
    hw_upper = np.full(NUM_JOINTS, 1.0)
    hw_upper[6] = 2.5
    with pytest.raises(ValueError) as excinfo:
        limits.cross_check(hw_lower, hw_upper)
    assert "r_index_finger_pip" in str(excinfo.value)


def test_accepts_and_returns_an_in_range_target():
    chain = make_chain()
    target = np.full(NUM_JOINTS, 0.5)
    safe, report = chain.apply(target, dt=0.01, now=0.0)
    np.testing.assert_allclose(safe, target)
    assert report.accepted
    assert not report.clamped.any()
    assert report.reason == ""


def test_clamps_out_of_range_target_and_reports_which_joints():
    chain = make_chain()
    target = np.zeros(NUM_JOINTS)
    target[3] = 5.0
    target[11] = -5.0
    safe, report = chain.apply(target, dt=0.01, now=0.0)
    assert safe[3] == pytest.approx(1.0)
    assert safe[11] == pytest.approx(-1.0)
    assert report.clamped[3] and report.clamped[11]
    assert report.clamped.sum() == 2
    assert report.accepted


def test_rejects_whole_message_on_nan_not_just_the_bad_joint():
    chain = make_chain()
    chain.apply(np.full(NUM_JOINTS, 0.25), dt=0.01, now=0.0)
    target = np.full(NUM_JOINTS, 0.75)
    target[5] = np.nan
    safe, report = chain.apply(target, dt=0.01, now=0.1)
    assert not report.accepted
    assert "finite" in report.reason
    # The previously accepted target is held -- NOT a partially-honored command.
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.25))


def test_rejects_whole_message_on_inf():
    chain = make_chain()
    target = np.zeros(NUM_JOINTS)
    target[0] = np.inf
    safe, report = chain.apply(target, dt=0.01, now=0.0)
    assert not report.accepted
    np.testing.assert_allclose(safe, np.zeros(NUM_JOINTS))


def test_rejects_wrong_shape_target():
    chain = make_chain()
    safe, report = chain.apply(np.zeros(19), dt=0.01, now=0.0)
    assert not report.accepted
    assert "shape" in report.reason
    np.testing.assert_allclose(safe, np.zeros(NUM_JOINTS))


def test_output_is_always_finite_even_when_rejecting():
    chain = make_chain()
    safe, _ = chain.apply(np.full(NUM_JOINTS, np.nan), dt=0.01, now=0.0)
    assert np.isfinite(safe).all()


def test_last_safe_tracks_the_accepted_target():
    chain = make_chain()
    chain.apply(np.full(NUM_JOINTS, 0.3), dt=0.01, now=0.0)
    np.testing.assert_allclose(chain.last_safe, np.full(NUM_JOINTS, 0.3))


# ----------------------------- guard 03: slew-rate limit -----------------------------


def test_rate_limits_a_large_step():
    chain = make_chain(max_velocity=1.0)  # 1 rad/s
    safe, report = chain.apply(np.full(NUM_JOINTS, 0.9), dt=0.1, now=0.0)
    # 1.0 rad/s * 0.1 s = 0.1 rad of allowed travel from the initial zero.
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.1))
    assert report.accepted
    assert report.rate_limited.all()


def test_rate_limit_is_per_joint():
    chain = make_chain(max_velocity=1.0)
    target = np.zeros(NUM_JOINTS)
    target[0] = 0.9  # far -> limited
    target[1] = 0.05  # near -> untouched
    safe, report = chain.apply(target, dt=0.1, now=0.0)
    assert safe[0] == pytest.approx(0.1)
    assert safe[1] == pytest.approx(0.05)
    assert report.rate_limited[0]
    assert not report.rate_limited[1]


def test_rate_limit_walks_toward_the_target_over_ticks():
    chain = make_chain(max_velocity=1.0)
    for expected in (0.1, 0.2, 0.3):
        safe, _ = chain.apply(np.full(NUM_JOINTS, 0.9), dt=0.1, now=0.0)
        np.testing.assert_allclose(safe, np.full(NUM_JOINTS, expected))


def test_chain_reports_the_post_slew_setpoint_velocity():
    chain = make_chain(max_velocity=1.0)
    _, report = chain.apply(np.full(NUM_JOINTS, 0.05), dt=0.1, now=0.0)
    # 0.05 rad of travel in 0.1 s, inside the 0.1 rad budget, so nothing was truncated.
    np.testing.assert_allclose(report.velocity, np.full(NUM_JOINTS, 0.5))


def test_reported_velocity_is_bounded_by_the_slew_limit():
    # The setpoint cannot move faster than the limiter lets it, so neither can the velocity we
    # hand the hand -- which is what bounds kd*qd.
    chain = make_chain(max_velocity=1.0)
    _, report = chain.apply(np.full(NUM_JOINTS, 10.0), dt=0.1, now=0.0)
    np.testing.assert_allclose(report.velocity, np.full(NUM_JOINTS, 1.0))


def test_a_held_target_reports_zero_velocity():
    chain = make_chain(max_velocity=1.0)
    chain.apply(np.full(NUM_JOINTS, 0.05), dt=0.1, now=0.0)
    _, report = chain.apply(None, dt=0.1, now=0.2)
    np.testing.assert_allclose(report.velocity, np.zeros(NUM_JOINTS))


def test_rate_limit_applies_from_home_on_the_first_command():
    # A distant first target must ramp, not snap -- the initial pose is the rate-limit origin.
    chain = GuardChain(
        limits=make_limits(),
        max_velocity=np.full(NUM_JOINTS, 1.0),
        timeout_s=1e9,
        initial=np.full(NUM_JOINTS, -0.5),
    )
    safe, report = chain.apply(np.full(NUM_JOINTS, 1.0), dt=0.1, now=0.0)
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, -0.4))
    assert report.rate_limited.all()


def test_non_positive_dt_grants_one_nominal_tick_not_unbounded_travel():
    chain = make_chain(max_velocity=1.0)
    safe, report = chain.apply(np.full(NUM_JOINTS, 0.9), dt=0.0, now=0.0)
    # dt<=0 (clock jump) must not divide by zero nor permit a free jump. The value is pinned, not
    # just bounded below the target: 1.0 rad/s * the 0.01 s nominal tick is 0.01 rad of travel.
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.01))
    assert report.accepted


def test_clamp_runs_before_rate_limit_so_travel_targets_a_legal_pose():
    chain = make_chain(max_velocity=1.0)
    safe, report = chain.apply(np.full(NUM_JOINTS, 50.0), dt=0.1, now=0.0)
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.1))
    assert report.clamped.all()
    assert report.rate_limited.all()


# ----------------------------- guard 04: watchdog -----------------------------


def test_watchdog_holds_last_safe_when_the_stream_stops():
    chain = make_chain(timeout_s=0.2)
    chain.apply(np.full(NUM_JOINTS, 0.4), dt=0.01, now=0.0)
    safe, report = chain.apply(None, dt=0.01, now=1.0)
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.4))
    assert not report.accepted
    assert "stale" in report.reason


def test_watchdog_does_not_go_limp_or_home():
    chain = make_chain(timeout_s=0.2)
    chain.apply(np.full(NUM_JOINTS, 0.4), dt=0.01, now=0.0)
    safe, _ = chain.apply(None, dt=0.01, now=99.0)
    # Holding, not zeroing: a loaded finger must not be released.
    assert not np.allclose(safe, 0.0)


def test_within_timeout_a_missing_command_is_not_yet_stale():
    chain = make_chain(timeout_s=0.5)
    chain.apply(np.full(NUM_JOINTS, 0.4), dt=0.01, now=0.0)
    _, report = chain.apply(None, dt=0.01, now=0.1)
    assert report.reason == "no command"


def test_resuming_after_a_stall_rate_limits_from_the_held_pose():
    chain = make_chain(max_velocity=1.0, timeout_s=0.2)
    # dt=0.3 buys the whole 0.3 rad in one tick, so the pose held across the stall really is 0.3.
    chain.apply(np.full(NUM_JOINTS, 0.3), dt=0.3, now=0.0)
    chain.apply(None, dt=0.1, now=5.0)  # stalled, holding 0.3
    safe, report = chain.apply(np.full(NUM_JOINTS, 0.9), dt=0.1, now=5.1)
    # Resume cannot jump: travel is measured from the held target.
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.4))
    assert report.rate_limited.all()


def test_stale_is_false_before_any_command_arrives():
    chain = make_chain(timeout_s=0.2)
    assert chain.stale(now=100.0) is False


# --------------------- non-finite values cannot reach the actuators ---------------------


def test_resolved_non_finite_command_is_rejected_on_both_paths():
    # resolve_command does not police finiteness, so a NaN/Inf position arrives here as a
    # structurally valid (20,) target. Guard 01 is the single place it gets stopped.
    current = np.zeros(NUM_JOINTS)
    named = resolve_command(["r_thumb_cmc_flex"], [float("nan")], current)
    unnamed = resolve_command(None, np.full(NUM_JOINTS, np.inf), current)
    for target in (named, unnamed):
        chain = make_chain()
        chain.apply(np.full(NUM_JOINTS, 0.2), dt=0.01, now=0.0)
        safe, report = chain.apply(target, dt=0.01, now=0.1)
        assert not report.accepted
        assert "finite" in report.reason
        np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.2))


def test_rejects_non_finite_initial():
    # A non-finite seed makes last_safe non-finite forever, so every later output -- including a
    # rejection that only holds the last safe target -- would carry NaN to the hardware.
    with pytest.raises(ValueError, match="finite"):
        GuardChain(
            limits=make_limits(),
            max_velocity=np.full(NUM_JOINTS, 1.0),
            timeout_s=1.0,
            initial=np.full(NUM_JOINTS, np.nan),
        )


def test_rejects_non_finite_or_negative_max_velocity():
    # A NaN/Inf budget makes the rate limit produce NaN or no limit at all; a negative budget
    # inverts the clip and drives away from the target. Both fail at construction.
    for bad in (np.nan, np.inf, -1.0):
        with pytest.raises(ValueError, match="max_velocity"):
            GuardChain(
                limits=make_limits(),
                max_velocity=np.full(NUM_JOINTS, bad),
                timeout_s=1.0,
                initial=np.zeros(NUM_JOINTS),
            )


def test_from_mapping_rejects_non_finite_bound():
    bad = dict(RAW)
    bad["r_middle_finger_mcp_abd"] = (-1.0, float("nan"))
    with pytest.raises(ValueError, match="finite"):
        Limits.from_mapping(bad, margin=0.0, names=RIGHT_NAMES)


def test_from_mapping_rejects_non_finite_margin():
    with pytest.raises(ValueError, match="finite"):
        Limits.from_mapping(RAW, margin=float("nan"), names=RIGHT_NAMES)


def test_unusable_dt_grants_one_nominal_tick_even_with_a_locked_joint():
    # An infinite dt passes `dt > 0.0`, and inf * a zero (locked-joint) budget is NaN, which
    # np.clip then spreads over every joint -- an ACCEPTED, non-finite command. Every unusable dt
    # must take the nominal tick instead.
    max_velocity = np.full(NUM_JOINTS, 1.0)
    max_velocity[7] = 0.0  # deliberately locked
    for dt in (float("inf"), float("-inf"), float("nan"), -1.0, 0.0):
        chain = GuardChain(
            limits=make_limits(),
            max_velocity=max_velocity,
            timeout_s=1.0,
            initial=np.zeros(NUM_JOINTS),
        )
        safe, report = chain.apply(np.full(NUM_JOINTS, 0.9), dt=dt, now=0.0)
        assert report.accepted
        assert np.isfinite(safe).all()
        assert np.isfinite(chain.last_safe).all()
        assert safe[7] == pytest.approx(0.0)  # the locked joint stays put
        assert safe[0] == pytest.approx(0.01)  # one nominal tick for every other joint


def test_an_unusable_dt_cannot_poison_the_watchdog_hold():
    # A single NaN reaching last_safe is permanent: every later output would be non-finite,
    # including the hold that exists to be the safe fallback.
    max_velocity = np.full(NUM_JOINTS, 1.0)
    max_velocity[3] = 0.0
    chain = GuardChain(
        limits=make_limits(),
        max_velocity=max_velocity,
        timeout_s=0.2,
        initial=np.zeros(NUM_JOINTS),
    )
    chain.apply(np.full(NUM_JOINTS, 0.9), dt=float("inf"), now=0.0)
    safe, report = chain.apply(None, dt=0.01, now=99.0)
    assert np.isfinite(safe).all()
    assert "stale" in report.reason


def test_from_mapping_rejects_negative_margin():
    # A negative margin widens the soft limits past the declared envelope, so an out-of-envelope
    # command would pass with clamped=False and nothing would report the truncation.
    with pytest.raises(ValueError, match="margin"):
        Limits.from_mapping(RAW, margin=-0.1, names=RIGHT_NAMES)


def test_cross_check_rejects_a_non_finite_hardware_bound():
    # NaN compares false against any tolerance, so an unreadable hardware bound would otherwise
    # pass as perfect agreement.
    limits = make_limits()
    hw_lower = np.full(NUM_JOINTS, -1.0)
    hw_upper = np.full(NUM_JOINTS, 1.0)
    hw_upper[13] = np.nan
    with pytest.raises(ValueError) as excinfo:
        limits.cross_check(hw_lower, hw_upper)
    assert "r_ring_finger_mcp_abd" in str(excinfo.value)


def test_rejects_non_finite_or_non_positive_timeout():
    # A NaN timeout leaves stale() False forever, silently disabling the staleness signal.
    for bad in (np.nan, np.inf, 0.0, -1.0):
        with pytest.raises(ValueError, match="timeout_s"):
            GuardChain(
                limits=make_limits(),
                max_velocity=np.full(NUM_JOINTS, 1.0),
                timeout_s=bad,
                initial=np.zeros(NUM_JOINTS),
            )


# ------------- guard activity is a judgement about magnitude, not an exact comparison -------------


def make_chain_between(lower: float, upper: float, max_velocity: float = 1e9) -> GuardChain:
    """A chain over a narrower envelope than RAW, for the clamp cases."""
    return GuardChain(
        limits=Limits.from_mapping({name: (lower, upper) for name in RIGHT_NAMES}, margin=0.0, names=RIGHT_NAMES),
        max_velocity=np.full(NUM_JOINTS, max_velocity),
        timeout_s=1.0,
        initial=np.zeros(NUM_JOINTS),
    )


def test_a_step_arithmetically_equal_to_the_budget_is_not_reported_as_rate_limiting():
    # The curl sequence steps by amplitude*step/steps, which in float lands a few units in the last
    # place ABOVE an exactly-representable budget: 0.4 * 6 / 20 - 0.4 * 5 / 20 is
    # 0.020000000000000018 against 2.0 rad/s * 0.01 s = 0.02 exactly. Reported as rate limiting,
    # that made a signal check read as a guard fault on 115 of its 200 frames.
    step, previous = 0.4 * 6 / 20, 0.4 * 5 / 20
    assert step - previous > 2.0 * 0.01, "the float dust this test is about is no longer there"
    chain = GuardChain(
        limits=make_limits(),
        max_velocity=np.full(NUM_JOINTS, 2.0),
        timeout_s=1.0,
        initial=np.full(NUM_JOINTS, previous),
    )
    safe, report = chain.apply(np.full(NUM_JOINTS, step), dt=0.01, now=0.0)
    assert not report.rate_limited.any()
    # Reporting only: the truncation itself still happened, to the last representable double.
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, previous + 0.02))
    assert safe[0] != step


def test_a_rate_truncation_far_above_float_dust_is_still_reported():
    # A nanoradian is invisible on a hand and seven orders of magnitude above the tolerance, so
    # this pins that "indistinguishable from zero" was not widened into "small".
    chain = GuardChain(
        limits=make_limits(),
        max_velocity=np.full(NUM_JOINTS, 2.0),
        timeout_s=1.0,
        initial=np.zeros(NUM_JOINTS),
    )
    safe, report = chain.apply(np.full(NUM_JOINTS, 0.02 + 1e-9), dt=0.01, now=0.0)
    assert report.rate_limited.all()
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.02))


def test_a_clamp_difference_indistinguishable_from_zero_is_not_reported():
    candidate = 0.1 + 0.2  # 0.30000000000000004
    assert candidate > 0.3, "the float dust this test is about is no longer there"
    chain = make_chain_between(-0.3, 0.3)
    safe, report = chain.apply(np.full(NUM_JOINTS, candidate), dt=0.01, now=0.0)
    assert not report.clamped.any()
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.3))


def test_a_clamp_far_above_float_dust_is_still_reported():
    chain = make_chain_between(-0.3, 0.3)
    safe, report = chain.apply(np.full(NUM_JOINTS, 0.3 + 1e-9), dt=0.01, now=0.0)
    assert report.clamped.all()
    np.testing.assert_allclose(safe, np.full(NUM_JOINTS, 0.3))
