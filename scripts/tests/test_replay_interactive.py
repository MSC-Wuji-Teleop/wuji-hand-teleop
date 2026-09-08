"""Pin scripts/replay_interactive.py: state rules, the commands it builds, teardown order.

Every test runs with dry_run=True, so nothing is ever executed: the commands
are built and asserted as lists. No Docker, no container, no robot.

The teardown order is the safety-relevant part (docs/spec/spec1_2.md,
"Ctrl-C") and it is asserted as an order, not just as a set of calls.
"""

from __future__ import annotations

import io

import pytest

import replay_interactive as ri
from interactive.terminal import CommandError


def _session(arms="both", hands="both", **kwargs):
    state = ri.State(connected_arms=arms, connected_hands=hands, arms=arms, hands=hands)
    kwargs.setdefault("wait", lambda message: None)
    return ri.Session(state, dry_run=True, out=io.StringIO(), **kwargs)


class Recorder(ri.Session):
    """A session that records the commands it would run, in order."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ran: list[list[str]] = []

    def _call(self, command, *, label):
        self.ran.append(command)
        return 0

    def _popen(self, command, *, label):
        self.ran.append(command)
        return None


def _recorder(arms="both", hands="both"):
    state = ri.State(connected_arms=arms, connected_hands=hands, arms=arms, hands=hands)
    return Recorder(state, dry_run=True, out=io.StringIO(), wait=lambda message: None)


def _labels(ran: list[list[str]]) -> list[str]:
    """A short name per recorded command, so order is readable in a failure."""
    out = []
    for command in ran:
        text = " ".join(command)
        if "hand_drivers.launch.py" in text and "pkill" in text:
            out.append("stop drivers")
        elif "hand_drivers.launch.py" in text:
            out.append("start drivers")
        elif "replay_check" in text:
            out.append("check")
        elif "docker compose" in text:
            out.append("start g1")
        elif "docker kill" in text:
            out.append("stop g1")
        elif "replay_publisher" in text and "pkill" in text:
            out.append("stop publisher")
        elif "replay_publisher" in text:
            out.append("start publisher")
        else:
            out.append(text)
    return out


# ---------------------------------------------------------------- state rules


@pytest.mark.parametrize("connected,allowed", [
    ("none", ("none",)),
    ("both", ("none", "left", "right", "both")),
    ("left", ("none", "left")),
    ("right", ("none", "right")),
])
def test_the_connected_set_bounds_what_may_be_selected(connected, allowed):
    assert ri.State(connected, connected, connected, connected).allowed(connected) == allowed


def test_the_selection_cannot_grow_past_what_was_connected():
    """The rule the whole design rests on: a command never opens a connection."""
    session = _session(hands="left")
    with pytest.raises(CommandError, match="Restart with --hands both"):
        session.cmd_hands(["both"])
    with pytest.raises(CommandError, match="Restart with --hands right"):
        session.cmd_hands(["right"])
    session.cmd_hands(["left"])
    session.cmd_hands(["none"])
    assert session.state.hands == "none"


def test_turning_a_side_off_and_on_again_is_free():
    session = _session()
    session.cmd_hands(["none"])
    assert session.state.hands == "none"
    session.cmd_hands(["both"])
    assert session.state.hands == "both"


def test_selecting_nothing_at_all_is_refused():
    session = _session()
    session.cmd_arms(["none"])
    with pytest.raises(CommandError, match="would publish nothing"):
        session.cmd_hands(["none"])
    assert session.state.hands == "both"


def test_an_unknown_side_is_refused():
    session = _session()
    with pytest.raises(CommandError, match="not one of"):
        session.cmd_arms(["sideways"])
    with pytest.raises(CommandError, match="takes one of"):
        session.cmd_arms([])
    with pytest.raises(CommandError, match="takes one of"):
        session.cmd_arms(["left", "right"])


@pytest.mark.parametrize("bad", ["0", "-1", "1.5", "2", "fast", "nan", "inf"])
def test_a_bad_speed_is_refused(bad):
    session = _session()
    with pytest.raises(CommandError):
        session.cmd_speed([bad])
    assert session.state.speed == "auto"


@pytest.mark.parametrize("good", ["1", "0.5", "0.25", "auto"])
def test_a_good_speed_is_kept(good):
    session = _session()
    session.cmd_speed([good])
    assert session.state.speed == good


# ---------------------------------------------------------------- the commands built


def test_the_publisher_gets_the_live_selection_not_the_connected_one():
    session = _session()
    session.cmd_hands(["left"])
    command = session.publisher_command("90_sweep_joints_GT")
    text = " ".join(command)
    assert "--arms both --hands left" in text
    assert text.endswith(("--arms both --hands left", "--hands left"))
    assert "/home/wuji/ros2_ws/clips/safe/90_sweep_joints_GT" in text


def test_speed_auto_passes_no_speed_flag():
    """auto is the absence of --speed, which is how replay.sh spells it too."""
    session = _session()
    assert "--speed" not in " ".join(session.publisher_command("x"))
    session.cmd_speed(["0.25"])
    assert "--speed 0.25" in " ".join(session.publisher_command("x"))


def test_the_g1_runs_in_joint_replay_at_g1_29():
    text = " ".join(_session().g1_start_command())
    assert "mode:=joint_replay" in text
    assert "arm_type:=G1_29" in text
    assert "control_rate:=250.0" in text
    assert "--name g1-world-output" in text


def test_the_g1_is_stopped_with_sigint_never_sigterm():
    """`ros2 launch` only cancels itself on SIGTERM and the node would be
    SIGKILLed before it could release the arms. Same rule as replay.sh."""
    command = _session().g1_stop_command()
    assert command == ["docker", "kill", "--signal=INT", "g1-world-output"]
    assert "stop" not in command


def test_the_hand_driver_launch_is_the_one_with_no_shutdown():
    """Not replay.launch.py: that ties the drivers to the publisher's lifetime."""
    text = " ".join(_session(hands="left").hand_drivers_command())
    assert "hand_drivers.launch.py side:=left" in text
    assert "replay.launch.py" not in text


def test_the_driver_launch_uses_the_connected_side_not_the_live_one():
    """Drivers are started once from the flags; a later `hands none` must not
    change what was started."""
    session = _session(hands="both")
    session.cmd_hands(["left"])
    assert "side:=both" in " ".join(session.hand_drivers_command())


def test_the_publisher_records_its_pid_so_the_stop_is_exact():
    """`exec` replaces the shell in place, so the PID echoed before it is the
    PID of the process exec becomes."""
    inner = _session().publisher_command("x")[-1]
    assert inner.startswith(f"echo $$ > {ri.PUBLISHER_PID_FILE} && ")
    assert " && exec ros2 run replay replay_publisher" in inner


def test_the_publisher_stop_is_by_pid_with_a_pattern_fallback():
    stop = _session().publisher_stop_command()[-1]
    assert f"kill -INT" in stop and ri.PUBLISHER_PID_FILE in stop
    assert "||" in stop, "the PID file can be missing; there has to be a fallback"


def test_the_fallback_pattern_does_not_assume_ros2_run_is_in_the_command_line():
    """`ros2 run` may exec the node, after which "ros2 run" is gone from the
    command line and a pattern written against it matches nothing."""
    stop = _session().publisher_stop_command()[-1]
    fallback = stop.split("||", 1)[1]
    assert "ros2 run" not in fallback
    assert "replay_publisher.*--clip" in fallback


def test_the_publisher_is_stopped_with_int_never_term():
    stop = _session().publisher_stop_command()[-1]
    assert "-INT" in stop
    assert "-TERM" not in stop and "-9" not in stop


def test_readiness_is_replay_check_not_new_waiting_code():
    session = _session()
    assert "replay_check -- --arms none --hands both" in " ".join(session.check_command("none", "both"))
    assert "replay_check -- --arms both --hands none" in " ".join(session.check_command("both", "none"))


# ---------------------------------------------------------------- startup order


def test_startup_connects_hands_then_verifies_the_arms_and_releases_them():
    session = _recorder()
    assert session.start() is True
    assert _labels(session.ran) == [
        "start drivers", "check",     # hands, once, for the session
        "start g1", "check", "stop g1",  # arms verified then handed back
    ]


def test_startup_with_no_hands_touches_no_driver():
    session = _recorder(hands="none")
    session.start()
    assert _labels(session.ran) == ["start g1", "check", "stop g1"]


def test_startup_with_no_arms_never_starts_the_g1():
    session = _recorder(arms="none")
    session.start()
    assert _labels(session.ran) == ["start drivers", "check"]


def test_a_failed_hand_check_stops_before_touching_the_arms():
    session = _recorder()
    session._call = lambda command, *, label: (session.ran.append(command), 1)[1]
    assert session.start() is False
    assert "start g1" not in _labels(session.ran)


def test_a_failed_arm_check_still_releases_the_g1():
    """The arms must not be left held because a check failed."""
    session = _recorder()
    calls = {"n": 0}

    def call(command, *, label):
        session.ran.append(command)
        calls["n"] += 1
        return 1 if "replay_check" in " ".join(command) and "--arms both" in " ".join(command) else 0

    session._call = call
    assert session.start() is False
    assert _labels(session.ran)[-1] == "stop g1"


# ---------------------------------------------------------------- playing


def test_playing_starts_the_g1_then_the_publisher_then_releases_both():
    session = _recorder()
    session.play("90_sweep_joints_GT")
    assert _labels(session.ran) == ["start g1", "start publisher", "stop publisher", "stop g1"]


def test_playing_with_no_arms_never_starts_the_g1():
    session = _recorder(arms="none")
    session.play("90_sweep_joints_GT")
    assert _labels(session.ran) == ["start publisher", "stop publisher"]


def test_an_unknown_clip_is_refused_before_anything_starts():
    session = _recorder()
    with pytest.raises(CommandError, match="no clip 'nope'"):
        session.play("nope")
    assert session.ran == []


def test_an_incomplete_clip_directory_is_refused_before_anything_starts(monkeypatch, tmp_path):
    """One such directory is in the tree from a Finder copy: clip.json is named
    'clip 3.json' and arm_q.npz is absent."""
    broken = tmp_path / "safe" / "01_broken"
    broken.mkdir(parents=True)
    (broken / "clip 3.json").write_text("{}")
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    session = _recorder()
    with pytest.raises(CommandError, match="missing clip.json, arm_q.npz, hand_q20.npz"):
        session.play("01_broken")
    assert session.ran == []


def test_a_trailing_slash_from_tab_completion_is_tolerated(monkeypatch, tmp_path):
    """readline's filename-ish completion can leave a slash on the name."""
    clip = tmp_path / "safe" / "90_sweep_joints_GT"
    clip.mkdir(parents=True)
    for name in ri.CLIP_FILES:
        (clip / name).write_text("{}")
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    session = _recorder()
    session.play("90_sweep_joints_GT/")
    assert _labels(session.ran) == ["start g1", "start publisher", "stop publisher", "stop g1"]


# ---------------------------------------------------------------- teardown order


def test_teardown_before_anything_played_stops_only_what_started():
    """No clip has played, so there is no publisher and no G1 to stop. Startup
    left the G1 stopped after verifying it."""
    session = _recorder()
    session.start()
    session.ran.clear()
    session.teardown()
    assert _labels(session.ran) == ["stop drivers"]


def test_teardown_order_is_publisher_then_g1_then_drivers():
    """The order is the safety property: the publisher must stop writing before
    the G1 node releases, and the arms must be handed back before the hands
    de-energize."""
    session = _recorder()
    session.start()

    def ctrl_c(message):
        raise KeyboardInterrupt

    session._wait = ctrl_c
    with pytest.raises(KeyboardInterrupt):
        session.play("90_sweep_joints_GT")
    session.ran.clear()
    session.teardown()
    assert _labels(session.ran) == ["stop publisher", "stop g1", "stop drivers"]


def test_teardown_runs_once_however_many_times_it_is_called():
    """With real state to tear down, so a missing guard would show as repeats."""
    session = _recorder()
    session.start()

    def ctrl_c(message):
        raise KeyboardInterrupt

    session._wait = ctrl_c
    with pytest.raises(KeyboardInterrupt):
        session.play("90_sweep_joints_GT")
    session.ran.clear()
    session.teardown()
    first = list(_labels(session.ran))
    assert first == ["stop publisher", "stop g1", "stop drivers"]
    session.teardown()
    session.teardown()
    assert _labels(session.ran) == first


def test_teardown_with_no_hands_stops_no_driver():
    session = _recorder(hands="none")
    session.teardown()
    assert "stop drivers" not in _labels(session.ran)


def test_release_does_not_stop_the_hand_drivers():
    """Between clips the drivers stay up. That is the entire point."""
    session = _recorder()
    session.play("90_sweep_joints_GT")
    session.ran.clear()
    session.release("between clips")
    assert "stop drivers" not in _labels(session.ran)


def test_the_arms_are_not_released_until_the_publisher_is_confirmed_stopped():
    """The G1 weight ramp must not run while a publisher is still writing joint
    targets. Asserted on the stop having *returned* before the ramp is issued,
    not merely on the order the two commands appear in."""
    session = _recorder()
    session.start()
    session._wait = lambda message: None
    order: list[str] = []

    def call(command, *, label):
        text = " ".join(command)
        if "pgrep" in text:
            order.append("publisher stop returned")
        elif "docker kill" in text:
            order.append("g1 ramp issued")
        session.ran.append(command)
        return 0

    session._call = call
    session.play("90_sweep_joints_GT")
    assert order == ["publisher stop returned", "g1 ramp issued"]


def test_a_publisher_that_refuses_sigint_is_killed_before_the_arms_release():
    """A live publisher racing the weight ramp is worse than a killed one."""
    session = _recorder()
    session.start()
    session._wait = lambda message: None
    order: list[str] = []

    def call(command, *, label):
        text = " ".join(command)
        session.ran.append(command)
        if "pgrep" in text:
            order.append("sigint")
            return 1  # still running
        if "pkill -KILL" in text:
            order.append("kill")
            return 0
        if "docker kill" in text:
            order.append("ramp")
        return 0

    session._call = call
    session.play("90_sweep_joints_GT")
    assert order == ["sigint", "kill", "ramp"]


def test_the_publisher_stop_verifies_rather_than_assuming():
    """It must exit non-zero while any publisher survives; that return is the
    only signal the session has that the stop did not take."""
    stop = _session().publisher_stop_command()[-1]
    assert "pgrep" in stop, "the stop has to check, not just signal"
    assert "exit 1" in stop, "and report failure when the publisher survives"


def test_the_pid_kill_does_not_short_circuit_the_pattern_kill():
    """`ros2 run` may fork rather than exec, making the recorded PID the
    wrapper and the node a separate process. Short-circuiting on the wrapper's
    exit status would skip the kill that reaches the node."""
    stop = _session().publisher_stop_command()[-1]
    lines = [line.strip() for line in stop.splitlines() if line.strip()]
    pid_kills = [line for line in lines if line.startswith("kill -INT")]
    pattern_kills = [line for line in lines if line.startswith("pkill -INT")]
    assert len(pid_kills) == 1 and len(pattern_kills) == 1
    # Separate statements. `|| true` inside either one only swallows its own
    # failure; what must not happen is the pattern kill being the right-hand
    # side of the PID kill's success.
    assert "pkill" not in pid_kills[0]
    assert lines.index(pattern_kills[0]) > lines.index(pid_kills[0])


def test_a_recycled_pid_is_not_signalled():
    """The teleop container is long-lived, so a stale PID can belong to
    something else entirely by the time the file is read."""
    stop = _session().publisher_stop_command()[-1]
    assert "/proc/$pid/cmdline" in stop
    assert "grep -q replay_publisher" in stop


def test_a_publisher_that_died_during_the_hold_is_reported():
    """It waits up to 30 s for consumers and exits 1, which is longer than the
    grace after starting it, so the session can have called it playing."""
    session = _recorder()
    session.start()
    session._wait = lambda message: None

    class Dead:
        returncode = 1

        def poll(self):
            return 1

        def wait(self, timeout=None):
            return 1

    session._popen = lambda command, *, label: (session.ran.append(command), Dead())[1]
    session.play("90_sweep_joints_GT")
    assert "had already exited 1" in session.out.getvalue()


# ---------------------------------------------------------------- preflight


def test_preflight_refuses_when_the_teleop_container_is_not_running(monkeypatch):
    session = _recorder()
    session.dry_run = False
    monkeypatch.setattr(session, "_container_state", lambda name: "")
    assert session.preflight() is False
    assert "is not running" in session.out.getvalue()


def test_preflight_refuses_when_something_else_holds_the_arms(monkeypatch):
    """A `docker compose run --name` against a taken name fails, and a session
    that ignored it would later SIGINT the other operator's container and ramp
    their arms out from under them."""
    session = _recorder()
    session.dry_run = False
    monkeypatch.setattr(session, "_container_state",
                        lambda name: "running" if name == ri.G1_CONTAINER else "running")
    assert session.preflight() is False
    assert "already running" in session.out.getvalue()
    assert "holds the arms" in session.out.getvalue()


def test_preflight_does_not_look_at_the_g1_when_no_arms_are_connected(monkeypatch):
    session = _recorder(arms="none")
    session.dry_run = False
    asked: list[str] = []

    def state(name):
        asked.append(name)
        return "running"

    monkeypatch.setattr(session, "_container_state", state)
    assert session.preflight() is True
    assert asked == [ri.TELEOP_CONTAINER]


def test_a_g1_container_that_fails_to_start_is_not_a_thirty_second_wait(monkeypatch):
    """Ignoring the start's exit code produced the message "the arms did not
    report; check network_interface", which is a misdiagnosis."""
    session = _recorder()

    def call(command, *, label):
        session.ran.append(command)
        return 1 if "docker compose" in " ".join(command) else 0

    session._call = call
    assert session.start() is False
    assert "did not start" in session.out.getvalue()
    assert "check" not in _labels(session.ran)[-1]


def test_a_driver_launch_that_dies_at_once_is_noticed(monkeypatch):
    session = _recorder()

    class Dead:
        returncode = 2

        def poll(self):
            return 2

    session._popen = lambda command, *, label: (session.ran.append(command), Dead())[1]
    assert session.start() is False
    assert "exited 2" in session.out.getvalue()
    assert "check" not in _labels(session.ran)


# ---------------------------------------------------------------- reading clips


def test_clip_speeds_reads_the_audited_list(tmp_path, monkeypatch):
    clip = tmp_path / "safe" / "c"
    clip.mkdir(parents=True)
    for name in ri.CLIP_FILES:
        (clip / name).write_text("")
    (clip / "clip.json").write_text('{"safe_speeds": [1.0, 0.5]}')
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    assert _session().clip_speeds("c") == [1.0, 0.5]


def test_an_unreadable_clip_json_is_reported_not_raised(tmp_path, monkeypatch):
    """`ls` must survive a broken clip.json; reading is for display only."""
    clip = tmp_path / "safe" / "c"
    clip.mkdir(parents=True)
    for name in ri.CLIP_FILES:
        (clip / name).write_text("")
    (clip / "clip.json").write_text("{not json")
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    session = _session()
    assert session.clip_speeds("c") is None
    session.cmd_ls([])
    assert "unreadable clip.json" in session.out.getvalue()


def test_ls_names_what_an_unplayable_directory_is_missing(tmp_path, monkeypatch):
    (tmp_path / "safe" / "broken").mkdir(parents=True)
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    session = _session()
    session.cmd_ls([])
    assert "unplayable, missing clip.json, arm_q.npz, hand_q20.npz" in session.out.getvalue()


def test_clip_names_skips_dotfiles(tmp_path, monkeypatch):
    """clips/safe carries .DS_Store and .gitkeep."""
    (tmp_path / "safe" / ".hidden").mkdir(parents=True)
    (tmp_path / "safe" / "real").mkdir()
    (tmp_path / "safe" / ".gitkeep").write_text("")
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "safe")
    assert _session().clip_names() == ["real"]


def test_a_missing_clips_directory_is_not_fatal(tmp_path, monkeypatch):
    monkeypatch.setattr(ri, "SAFE_CLIPS", tmp_path / "nothing here")
    session = _session()
    assert session.clip_names() == []
    session.cmd_ls([])
    assert "no clips under" in session.out.getvalue()


# ---------------------------------------------------------------- arguments


def test_selecting_nothing_implies_a_dry_run(monkeypatch):
    recorded = {}

    class Fake(ri.Session):
        def __init__(self, state, *, dry_run, out=None):
            recorded["dry_run"] = dry_run
            super().__init__(state, dry_run=True, out=io.StringIO())

        def start(self):
            return False

    monkeypatch.setattr(ri, "Session", Fake)
    ri.main(["--arms", "none", "--hands", "none"])
    assert recorded["dry_run"] is True


def test_a_failed_startup_exits_non_zero(monkeypatch):
    class Fake(ri.Session):
        def start(self):
            return False

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO()))
    assert ri.main(["--dry-run"]) == 1


def test_a_normal_invocation_is_not_a_dry_run(monkeypatch):
    """Pins the other side of the dry-run rule. Without it, main() hardcoding
    dry_run=True would leave the rig session running no docker command at all,
    and every other test constructs dry_run=True directly."""
    seen = {}

    class Fake(ri.Session):
        def __init__(self, state, *, dry_run, out=None):
            seen["dry_run"] = dry_run
            super().__init__(state, dry_run=True, out=io.StringIO(),
                             wait=lambda message: None)

        def preflight(self):
            return False

    monkeypatch.setattr(ri, "Session", Fake)
    ri.main([])
    assert seen["dry_run"] is False
    ri.main(["--arms", "left"])
    assert seen["dry_run"] is False


def test_main_tears_down_after_the_loop_returns(monkeypatch):
    """`quit` and Ctrl-D both leave through run() returning normally. Without
    the teardown in main()'s finally the hands would stay energized."""
    torn = []

    class Fake(ri.Session):
        def preflight(self):
            return True

        def start(self):
            return True

        def teardown(self):
            torn.append(True)

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO(), wait=lambda message: None))
    monkeypatch.setattr(ri.Terminal, "run", lambda self: None)
    assert ri.main(["--dry-run"]) == 0
    assert torn == [True]


def test_main_tears_down_when_the_loop_raises(monkeypatch):
    """Any exception, from any source, must not exit with the arms held."""
    torn = []

    class Fake(ri.Session):
        def preflight(self):
            return True

        def start(self):
            return True

        def teardown(self):
            torn.append(True)

    def boom(self):
        raise OSError("docker binary vanished")

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO(), wait=lambda message: None))
    monkeypatch.setattr(ri.Terminal, "run", boom)
    with pytest.raises(OSError):
        ri.main(["--dry-run"])
    assert torn == [True]


def test_main_tears_down_when_startup_is_interrupted(monkeypatch):
    """The longest unprotected window: start() brings the G1 up and can sit in
    a 30 s check. A Ctrl-C there used to reach the default handler and leave
    the arms held at weight 1."""
    torn = []

    class Fake(ri.Session):
        def preflight(self):
            return True

        def start(self):
            raise KeyboardInterrupt

        def teardown(self):
            torn.append(True)

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO(), wait=lambda message: None))
    assert ri.main(["--dry-run"]) == 0
    assert torn == [True]


def test_signal_handlers_are_installed_before_startup(monkeypatch):
    """Not after: the startup window is the one that most needs covering."""
    order = []

    class Fake(ri.Session):
        def preflight(self):
            order.append("preflight")
            return True

        def start(self):
            order.append("start")
            return False

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO(), wait=lambda message: None))
    monkeypatch.setattr(ri.Terminal, "install_signal_handlers",
                        lambda self: order.append("handlers") or True)
    ri.main(["--dry-run"])
    assert order == ["handlers", "preflight", "start"]


def test_a_refused_preflight_never_reaches_startup(monkeypatch):
    started = []

    class Fake(ri.Session):
        def preflight(self):
            return False

        def start(self):
            started.append(True)
            return True

    monkeypatch.setattr(ri, "Session", lambda state, *, dry_run, out=None: Fake(
        state, dry_run=True, out=io.StringIO(), wait=lambda message: None))
    assert ri.main(["--dry-run"]) == 1
    assert started == []
