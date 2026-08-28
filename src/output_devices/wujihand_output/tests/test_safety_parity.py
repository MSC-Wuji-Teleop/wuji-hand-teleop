"""Parity guard for the deliberately duplicated safety primitives (plan A7).

g1_world_output/replay_safety.py and wujihand_output/hand_safety.py (and
the two stream_buffer.py copies) live in different containers with
different numpy majors, so they are separate files by design. A bug fixed
in one copy must not silently persist in the other: every shared behavior
is exercised here against BOTH implementations. Intentional divergences
(the hand's amp-space EffortGuard, the arm's DivergenceMonitor and
ArmLimits/HandLimits schemas) are not parametrized.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

_SRC = Path(__file__).resolve().parents[3]
for pkg in ('output_devices/g1_world_output', 'output_devices/wujihand_output'):
    p = str(_SRC / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

from g1_world_output import replay_safety as arm_mod  # noqa: E402
from g1_world_output import stream_buffer as arm_buf_mod  # noqa: E402
from wujihand_output import hand_safety as hand_mod  # noqa: E402
from wujihand_output import stream_buffer as hand_buf_mod  # noqa: E402

MODS = [pytest.param(arm_mod, id='arm'), pytest.param(hand_mod, id='hand')]
BUFS = [pytest.param(arm_buf_mod.StreamBuffer, id='arm'),
        pytest.param(hand_buf_mod.StreamBuffer, id='hand')]


@pytest.mark.parametrize('mod', MODS)
class TestClampParity:
    def test_clamp_and_flags(self, mod):
        c = mod.PositionClamp(np.array([-1.0, -2.0]), np.array([1.0, 2.0]),
                              margin=0.1)
        q, hit = c.apply(np.array([2.0, 0.0]))
        assert q[0] == pytest.approx(0.9)
        assert hit.tolist() == [True, False]

    def test_margin_inversion_rejected(self, mod):
        with pytest.raises(ValueError):
            mod.PositionClamp(np.array([-0.1]), np.array([0.1]), margin=0.2)


@pytest.mark.parametrize('mod', MODS)
class TestRateLimitParity:
    def test_uniform_scaling(self, mod):
        out = mod.rate_limit_step(np.zeros(2), np.array([0.1, 0.005]),
                                  np.array([1.0, 1.0]), dt=0.01)
        assert out[0] == pytest.approx(0.01)
        assert out[1] == pytest.approx(0.0005)

    def test_nan_rejected(self, mod):
        with pytest.raises(ValueError):
            mod.rate_limit_step(np.zeros(1), np.array([np.nan]),
                                np.array([1.0]), dt=0.01)


@pytest.mark.parametrize('mod', MODS)
class TestStalenessParity:
    def test_semantics(self, mod):
        s = mod.StalenessTracker(0.1)
        assert s.is_stale(0.0)
        s.mark(1.0)
        assert not s.is_stale(1.05)
        assert s.is_stale(1.2)
        s.mark(1.3)
        s.is_stale(1.31)
        s.is_stale(1.5)
        assert s.stale_episodes == 2


@pytest.mark.parametrize('buf_cls', BUFS)
class TestStreamBufferParity:
    def test_ramp_and_edges(self, buf_cls):
        b = buf_cls()
        b.push(0.0, 10.00, [0.0])
        b.push(0.02, 10.02, [1.0], current_cmd=[0.0])
        assert b.interpolate(0.03)[0] == pytest.approx(0.5)
        # Duplicate stamp holds newest.
        b2 = buf_cls()
        b2.push(0.0, 5.0, [0.0])
        b2.push(0.02, 5.0, [0.3], current_cmd=[0.1])
        assert b2.interpolate(0.02)[0] == pytest.approx(0.3)
