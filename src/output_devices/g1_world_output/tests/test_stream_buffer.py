"""StreamBuffer (ZOH fix): piecewise-linear output, D10/A4 edge semantics."""

from pathlib import Path

import numpy as np
import pytest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_world_output.stream_buffer import StreamBuffer  # noqa: E402

DT_PLAY = 0.02   # 50 fps stream
DT_CTRL = 0.004  # 250 Hz control loop


class TestBasics:
    def test_empty_returns_none(self):
        assert StreamBuffer().interpolate(0.0) is None

    def test_seed_holds(self):
        b = StreamBuffer()
        b.seed([1.0, 2.0])
        np.testing.assert_array_equal(b.interpolate(5.0), [1.0, 2.0])

    def test_first_push_holds_newest(self):
        # No previous stamp: alpha = 1, hold the new sample (D10).
        b = StreamBuffer()
        b.push(arrival_t=0.0, stamp_s=10.0, q=[0.5])
        np.testing.assert_array_equal(b.interpolate(0.0), [0.5])


class TestRamp:
    def test_linear_ramp_from_current_cmd(self):
        b = StreamBuffer()
        b.push(0.0, 10.00, [0.0])
        # Second sample: stamp delta 20 ms; ramp from current cmd 0 -> 1.
        b.push(0.02, 10.02, [1.0], current_cmd=[0.0])
        assert b.interpolate(0.02)[0] == pytest.approx(0.0)
        assert b.interpolate(0.03)[0] == pytest.approx(0.5)
        assert b.interpolate(0.04)[0] == pytest.approx(1.0)
        assert b.interpolate(0.05)[0] == pytest.approx(1.0)  # clamped

    def test_piecewise_linear_stream(self):
        # A constant-velocity stream comes out piecewise-linear (the Stage 0
        # assert), one period behind, never stepping.
        b = StreamBuffer()
        cmd = np.array([0.0])
        outputs = []
        arrival = stamp = 0.0
        for i in range(6):
            b.push(arrival, stamp, [0.1 * i], current_cmd=cmd)
            # control ticks between stream samples
            for j in range(int(DT_PLAY / DT_CTRL)):
                now = arrival + (j + 1) * DT_CTRL
                cmd = b.interpolate(now)
                outputs.append(cmd[0])
            arrival += DT_PLAY
            stamp += DT_PLAY
        out = np.array(outputs)
        steps = np.diff(out)
        # Strictly advancing (after the first held sample), no plateaus, no
        # jumps: every control-tick increment equals v * DT_CTRL = 0.02.
        assert np.all(steps[5:] > 0)
        np.testing.assert_allclose(steps[5:], 0.02, rtol=1e-6)

    def test_zoh_defect_absent(self):
        # The old buffer jumped to the newest sample by the next control
        # tick. Here the first control tick after a push moves only
        # DT_CTRL/DT_PLAY of the way.
        b = StreamBuffer()
        b.push(0.0, 0.0, [0.0])
        b.push(DT_PLAY, DT_PLAY, [1.0], current_cmd=[0.0])
        first_tick = b.interpolate(DT_PLAY + DT_CTRL)[0]
        assert first_tick == pytest.approx(DT_CTRL / DT_PLAY)  # 0.2, not 1.0


class TestEdges:
    def test_duplicate_stamp_holds_newest(self):
        b = StreamBuffer()
        b.push(0.00, 5.0, [0.0])
        b.push(0.02, 5.0, [0.3], current_cmd=[0.1])  # stamp delta 0
        np.testing.assert_array_equal(b.interpolate(0.02), [0.3])

    def test_reordered_stamp_holds_newest(self):
        b = StreamBuffer()
        b.push(0.00, 5.0, [0.0])
        b.push(0.02, 4.9, [0.3], current_cmd=[0.1])
        np.testing.assert_array_equal(b.interpolate(0.02), [0.3])

    def test_dropped_frame_ramp_still_bounded_by_max_period(self):
        b = StreamBuffer(max_period_s=0.1)
        b.push(0.0, 0.0, [0.0])
        b.push(1.0, 1.0, [1.0], current_cmd=[0.0])  # 1 s gap, clamped to 0.1
        assert b.interpolate(1.05)[0] == pytest.approx(0.5)
        assert b.interpolate(1.1)[0] == pytest.approx(1.0)

    def test_tiny_stamp_delta_clamped(self):
        b = StreamBuffer(min_period_s=1e-3)
        b.push(0.0, 0.0, [0.0])
        b.push(0.02, 1e-7, [1.0], current_cmd=[0.0])
        # period clamps to 1 ms; alpha at +1 ms is 1.
        assert b.interpolate(0.021)[0] == pytest.approx(1.0)

    def test_first_sample_after_seed_holds_newest(self):
        # Seed time is node-clock, not a header stamp; there is no previous
        # stamp, so the first sample is held, and first MOTION belongs to
        # the approach phase (A4).
        b = StreamBuffer()
        b.seed([0.2])
        b.push(0.0, 100.0, [0.4], current_cmd=[0.2])
        np.testing.assert_array_equal(b.interpolate(0.0), [0.4])
