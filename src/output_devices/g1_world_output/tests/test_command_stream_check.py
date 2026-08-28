"""Piecewise-linear analyzer: catches the old ZOH, passes D5 output."""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_world_output.command_stream_check import analyze_command_stream  # noqa: E402
from g1_world_output.stream_buffer import StreamBuffer  # noqa: E402

DT_CTRL = 1.0 / 250.0
DT_PLAY = 1.0 / 50.0
VEL = 0.5  # deploy velocity


def synth_stream(zoh: bool, n_frames: int = 40, step: float = 0.002):
    """Simulate the control loop consuming a 50 fps stream at 250 Hz,
    either through the old ZOH (repeat newest) or the D5 StreamBuffer."""
    buf = StreamBuffer()
    t_out, q_out = [], []
    cmd = np.zeros(1)
    now = 0.0
    for i in range(n_frames):
        stamp = i * DT_PLAY
        buf.push(now, stamp, [step * i], current_cmd=cmd)
        for _ in range(int(DT_PLAY / DT_CTRL)):
            now += DT_CTRL
            if zoh:
                cmd = np.array([step * i])       # old behavior: newest sample
            else:
                cmd = buf.interpolate(now)
            t_out.append(now)
            q_out.append(cmd.copy())
    # trailing deliberate hold (end_hold) -- must not fail the check
    for _ in range(100):
        now += DT_CTRL
        t_out.append(now)
        q_out.append(cmd.copy())
    return np.array(t_out), np.array(q_out)


class TestAnalyzer:
    def test_zoh_stream_fails(self):
        t, q = synth_stream(zoh=True)
        check = analyze_command_stream(t, q, VEL)
        assert not check.piecewise_linear
        assert check.duplicate_fraction > 0.5
        assert any('zero-order-hold' in r for r in check.reasons)

    def test_d5_stream_passes(self):
        t, q = synth_stream(zoh=False)
        check = analyze_command_stream(t, q, VEL)
        assert check.piecewise_linear, check.reasons
        assert check.duplicate_fraction < 0.10

    def test_jump_detected(self):
        t, q = synth_stream(zoh=False)
        q[len(q) // 2] += 0.5   # a 0.5 rad discontinuity in one tick
        check = analyze_command_stream(t, q, VEL)
        assert not check.piecewise_linear
        assert any('stepping' in r for r in check.reasons)

    def test_static_stream_reports_nothing_to_judge(self):
        t = np.arange(100) * DT_CTRL
        q = np.zeros((100, 3))
        check = analyze_command_stream(t, q, VEL)
        assert not check.piecewise_linear
        assert 'never moved' in check.reasons[0]

    def test_end_hold_plateau_excluded(self):
        # The trailing hold in synth_stream is longer than the motion; a
        # naive whole-recording duplicate fraction would fail a healthy run.
        t, q = synth_stream(zoh=False, n_frames=10)
        check = analyze_command_stream(t, q, VEL)
        assert check.piecewise_linear, check.reasons
        assert check.moving_ticks < check.total_ticks
