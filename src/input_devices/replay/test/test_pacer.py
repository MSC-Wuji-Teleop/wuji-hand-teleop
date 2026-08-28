"""Pacer core FSM: gates, stamps, scope, fault freeze. ROS-free."""

import numpy as np
import pytest

from conftest import HAND_LIMITS_PATH

from replay.clip_artifact import (
    CANONICAL_HAND_JOINTS,
    hand_joint_names,
    load_artifact,
    synthetic_artifact,
)
from replay.pacer import LoadError, LoadRequest, PacerState, ReplayPacer


@pytest.fixture
def clip(tmp_path):
    synthetic_artifact(tmp_path / 'clip', num_frames=10, k=2)
    return load_artifact(tmp_path / 'clip')


def _full(clip, **kw):
    p = ReplayPacer(**kw)
    p.load(clip, LoadRequest(clip=str(clip.npz_path)))
    return p


class TestNameTables:
    def test_arm_names_match_device_table(self):
        # clip_artifact keeps a literal copy (replay must import without
        # g1_world_output built); a drift would send conditioned columns
        # to the wrong joints.
        from g1_world_output.joint_tables import G1_29_ARM_JOINT_NAMES
        from replay.clip_artifact import CANONICAL_ARM_JOINTS
        assert CANONICAL_ARM_JOINTS == G1_29_ARM_JOINT_NAMES

    def test_hand_names_match_limits_yaml(self):
        # The publisher's q20 names and the hand limits file must agree; a
        # drift here would break the hand branch's name matching.
        from wujihand_output.hand_safety import HandLimits
        lim = HandLimits.from_yaml(HAND_LIMITS_PATH)
        assert lim.names == CANONICAL_HAND_JOINTS
        assert lim.side_names('left') == hand_joint_names('left')
        assert lim.side_names('right') == hand_joint_names('right')


class TestLoadRequest:
    def test_parse_defaults(self):
        r = LoadRequest.from_json('{"clip": "/x.npz"}')
        assert r.speed_scale == 1.0
        assert r.arms == ('left', 'right')
        assert r.hands == ('left', 'right')

    def test_scoped(self):
        r = LoadRequest.from_json('{"clip": "/x", "arms": [], "hands": ["left"]}')
        assert r.arms == () and r.hands == ('left',)

    def test_rejects(self):
        with pytest.raises(LoadError, match='JSON'):
            LoadRequest.from_json('nope')
        with pytest.raises(LoadError, match='clip'):
            LoadRequest.from_json('{}')
        with pytest.raises(LoadError, match='unknown fields'):
            LoadRequest.from_json('{"clip": "/x", "loop": true}')
        with pytest.raises(LoadError, match='unknown sides'):
            LoadRequest.from_json('{"clip": "/x", "arms": ["up"]}')
        with pytest.raises(LoadError, match='empty scope'):
            LoadRequest.from_json('{"clip": "/x", "arms": [], "hands": []}')
        with pytest.raises(LoadError, match='speed_scale'):
            LoadRequest.from_json('{"clip": "/x", "speed_scale": 0}')


class TestGates:
    def test_fail_verdict_refused(self, tmp_path):
        synthetic_artifact(tmp_path / 'bad', verdict='fail')
        clip = load_artifact(tmp_path / 'bad')
        with pytest.raises(LoadError, match="verdict is 'fail'"):
            ReplayPacer().load(clip, LoadRequest(clip='x'))

    def test_force_sim_bypasses(self, tmp_path):
        synthetic_artifact(tmp_path / 'bad', verdict='fail')
        clip = load_artifact(tmp_path / 'bad')
        p = ReplayPacer(force_sim=True)
        p.load(clip, LoadRequest(clip='x'))
        assert p.state is PacerState.LOADED

    def test_overspeed_refused(self, tmp_path):
        synthetic_artifact(tmp_path / 'slow', max_allowed_speed_scale=0.5)
        clip = load_artifact(tmp_path / 'slow')
        with pytest.raises(LoadError, match='max_allowed_speed_scale'):
            ReplayPacer().load(clip, LoadRequest(clip='x', speed_scale=1.0))
        ReplayPacer().load(clip, LoadRequest(clip='x', speed_scale=0.5))

    def test_hand_scope_needs_conditioned_hands(self, tmp_path):
        synthetic_artifact(tmp_path / 'armonly', hands_conditioned=False)
        clip = load_artifact(tmp_path / 'armonly')
        with pytest.raises(LoadError, match='no-hands'):
            ReplayPacer().load(clip, LoadRequest(clip='x'))
        p = ReplayPacer()
        p.load(clip, LoadRequest(clip='x', hands=()))
        assert p.state is PacerState.LOADED

    def test_no_load_while_moving(self, clip):
        p = _full(clip)
        p.publish_first()
        with pytest.raises(LoadError, match='cannot load'):
            p.load(clip, LoadRequest(clip='x'))
        p.start()
        with pytest.raises(LoadError, match='cannot load'):
            p.load(clip, LoadRequest(clip='x'))

    def test_transition_order_enforced(self, clip):
        p = ReplayPacer()
        with pytest.raises(LoadError, match='publish_first'):
            p.publish_first()
        p.load(clip, LoadRequest(clip='x'))
        with pytest.raises(LoadError, match='start requires'):
            p.start()


class TestTicking:
    def test_nothing_before_publish_first(self, clip):
        p = _full(clip)
        assert p.tick() is None

    def test_first_frame_repeats_with_advancing_stamps(self, clip):
        p = _full(clip)
        p.publish_first()
        outs = [p.tick() for _ in range(3)]
        assert all(o.frame_index == 0 for o in outs)
        dt = clip.dt_play(1.0)
        assert [o.stamp_offset_s for o in outs] == pytest.approx([0, dt, 2 * dt])
        assert not any(o.clip_done for o in outs)

    def test_start_continues_stamp_series(self, clip):
        # D10: no stamp discontinuity across start; frames begin advancing.
        p = _full(clip)
        p.publish_first()
        p.tick(); p.tick()                      # stamps 0, dt
        p.start()
        o = p.tick()
        assert o.stamp_offset_s == pytest.approx(2 * clip.dt_play(1.0))
        assert o.frame_index == 0               # first running tick plays frame 0
        assert p.tick().frame_index == 1

    def test_clip_end_holds_last_frame(self, clip):
        p = _full(clip)
        p.publish_first()
        p.start()
        outs = [p.tick() for _ in range(15)]    # 10-frame clip
        assert outs[9].frame_index == 9
        assert outs[9].clip_done
        assert all(o.frame_index == 9 and o.clip_done for o in outs[10:])
        assert p.state is PacerState.FINISHED
        # Stamps never stop advancing (hold stream stays fresh).
        assert outs[14].stamp_offset_s > outs[13].stamp_offset_s

    def test_speed_scale_stretches_dt(self, tmp_path):
        synthetic_artifact(tmp_path / 'c', num_frames=10, k=2)
        clip = load_artifact(tmp_path / 'c')
        p = ReplayPacer()
        p.load(clip, LoadRequest(clip='x', speed_scale=0.5))
        # dt_play = k / (fps * scale) = 2 / (50 * 0.5)
        assert p.dt_play == pytest.approx(2 / 25.0)

    def test_scope_filters_targets(self, clip):
        p = ReplayPacer()
        p.load(clip, LoadRequest(clip='x', arms=('left',), hands=()))
        p.publish_first()
        o = p.tick()
        assert list(o.arm_targets) == ['left']
        assert o.hand_targets == {}
        names, q = o.arm_targets['left']
        assert len(names) == 7 and names[0] == 'left_shoulder_pitch'
        assert q.shape == (7,)

    def test_hand_targets_named(self, clip):
        p = _full(clip)
        p.publish_first()
        o = p.tick()
        names, q = o.hand_targets['right']
        assert names == hand_joint_names('right')
        assert q.shape == (20,)


class TestFault:
    def test_fault_freezes_frame(self, clip):
        p = _full(clip)
        p.publish_first()
        p.start()
        for _ in range(4):
            p.tick()
        p.fault()
        outs = [p.tick() for _ in range(3)]
        frames = {o.frame_index for o in outs}
        assert len(frames) == 1                 # frozen
        assert p.state is PacerState.FAULT
        with pytest.raises(LoadError, match='start requires'):
            p.start()                           # no resume, ever

    def test_fault_before_first_publish_stays_silent(self, clip):
        p = _full(clip)
        p.fault()
        assert p.tick() is None

    def test_reload_after_fault(self, clip):
        p = _full(clip)
        p.publish_first(); p.start(); p.tick(); p.fault()
        p.load(clip, LoadRequest(clip='x'))     # supervisor gates when
        assert p.state is PacerState.LOADED
        assert p.tick() is None                 # publishing stopped until first


class TestStatus:
    def test_schema_keys(self, clip):
        p = _full(clip)
        s = p.status()
        for key in ('state', 'clip', 'sample', 'method', 'speed_scale',
                    'tick', 'total', 'clip_done', 'scope', 'force_sim'):
            assert key in s
        assert s['state'] == 'loaded'
        assert s['total'] == 10
