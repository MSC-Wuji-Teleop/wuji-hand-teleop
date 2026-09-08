#!/usr/bin/env python3
"""Play several clips in one sitting without reconnecting to the hands.

    scripts/replay_interactive.py                            # both hands, both arms
    scripts/replay_interactive.py --hands left               # left hand only, both arms
    scripts/replay_interactive.py --arms right --hands none  # right arm only, no hand driver
    scripts/replay_interactive.py --arms none --hands none   # connect nothing
    scripts/replay_interactive.py --dry-run                  # print commands, run none

Design: docs/spec/spec1_2.md. Runs on the host, outside every container, the
way scripts/replay.sh does. That script is unchanged and stays the one-clip,
scriptable entry point with an exit-code contract; this is a second entry point
beside it.

**What persists and what cycles.** The hand drivers are started once and never
restarted: connecting to a hand costs 10 to 30 s (a UDP broadcast scan,
connect by serial, then the driver's blocking 3 s home) and removing that per
clip is the whole point. The G1 container is started and stopped for every
clip, because stopping it is what ramps the arm_sdk weight 1 to 0 and hands the
arms to the onboard controller. Keeping it up across clips would leave the arms
held stiff at the last frame and Ctrl-C would release nothing.

**Connections are decided once, by the flags.** Nothing in the loop opens a
link, closes one, or starts or stops a hand driver. The commands change only
which topics the next publisher writes. `--hands left` at startup then
`hands both` in the session is refused, because the right driver was never
started and starting it would cost the time this exists to save.

**Ctrl-C always stops everything and exits**, from any state, in one order:
the publisher, then the G1 container for its weight ramp, then the hand
drivers. There is no mid-clip abort that keeps the session alive; that is the
deliberate trade for one unambiguous Ctrl-C.

**Why the publisher runs in the background.** It holds the last frame until it
is killed and never exits on its own. In the foreground it would own the
terminal and this process could not read the Enter that ends the clip.

**How the publisher is stopped.** Not by signalling the local `docker exec`:
that forwards nothing without a pty, which is the same reason
scripts/replay.sh carries a `pkill -INT` fallback. The signal has to be sent
inside the container.

Two mechanisms, primary and fallback, because pattern matching alone is not
safe to rely on here. The inner command writes its own PID before `exec`
replaces the shell, so the file holds the PID of the process that `exec`
became; `kill -INT` on it is exact. The fallback is a pattern, needed because
`ros2 run` may or may not `exec` the node depending on version: if it does,
the command line no longer contains "ros2 run" at all, so a pattern written
against that would silently match nothing. `replay_publisher.*--clip` matches
either shape.

The prompt loop, the completer and the command registry are in
scripts/interactive/terminal.py, which has no Docker and no ROS in it.
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from interactive.terminal import Command, CommandError, Terminal  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
SAFE_CLIPS = REPO_ROOT / "clips" / "safe"

# Same names scripts/replay.sh uses, so one vocabulary covers both entry points.
TELEOP_CONTAINER = "wuji-hand-teleop"
G1_CONTAINER = "g1-world-output"
G1_SERVICE = "g1_world_output"
COMPOSE_FILE = REPO_ROOT / "docker" / "docker-compose.yml"
CONTAINER_WS = "/home/wuji/ros2_ws"

SIDES = ("none", "left", "right", "both")
HISTORY_FILE = Path.home() / ".cache" / "wuji" / "replay_interactive_history"

# The three files a clip directory must have (replay/clip.py). A directory
# under clips/safe/ missing any of them is listed as unplayable rather than
# hidden: the publisher would refuse it, and saying so early is cheaper.
CLIP_FILES = ("clip.json", "arm_q.npz", "hand_q20.npz")

# How long to give a freshly started publisher before deciding it is playing.
# It never exits on success, so any exit inside this window is a refusal (a
# clip it will not play, or a speed the audit did not pass).
PUBLISHER_GRACE_S = 3.0

# Where the publisher records its own PID inside the teleop container, so the
# stop is exact. In the container's /tmp, which no host path is mounted over.
PUBLISHER_PID_FILE = "/tmp/wuji_replay_publisher.pid"

# The G1 container is stopped with SIGINT, never `docker stop`'s SIGTERM:
# `ros2 launch` shuts its nodes down on SIGINT but only cancels itself on
# SIGTERM, and the node would be SIGKILLed before releasing the arms. Same
# reasoning and same grace as scripts/replay.sh's stop_g1.
G1_STOP_GRACE_S = 10


def _inner(command: str) -> str:
    """A command line to run inside the teleop container, workspace sourced."""
    return (
        f"source /opt/ros/humble/setup.bash && "
        f"source {CONTAINER_WS}/install/setup.bash && "
        f"cd {CONTAINER_WS} && exec {command}"
    )


@dataclass
class State:
    """What the flags fixed, and what the operator may still change.

    ``connected_*`` come from the command line and never change. ``arms`` and
    ``hands`` are the live selection and may only ever be a subset of them.
    """

    connected_arms: str
    connected_hands: str
    arms: str
    hands: str
    speed: str = "auto"

    def allowed(self, connected: str) -> tuple[str, ...]:
        """Which live values a given connected set permits."""
        if connected == "none":
            return ("none",)
        if connected == "both":
            return SIDES
        return ("none", connected)


class Session:
    def __init__(self, state: State, *, dry_run: bool, out=None, wait=None) -> None:
        self.state = state
        self.dry_run = dry_run
        self.out = out if out is not None else sys.stdout
        # How the session waits for the operator between clips. Injected rather
        # than calling input() inline so the play path is testable and so the
        # dependency on stdin is visible in one place.
        self._wait = wait if wait is not None else self._wait_for_enter
        self._publisher: subprocess.Popen | None = None
        self._drivers: subprocess.Popen | None = None
        # Intent, not handles. A dry run holds no Popen, and the printed
        # sequence has to match what a real run would do.
        self._g1_started = False
        self._publisher_started = False
        self._drivers_started = False
        self._torn_down = False

    def _wait_for_enter(self, message: str) -> None:
        try:
            input(message)
        except EOFError:
            self.say()

    # ---------------------------------------------------------------- output

    def say(self, message: str = "") -> None:
        print(message, file=self.out, flush=True)

    def _call(self, command: list[str], *, label: str) -> int:
        if self.dry_run:
            self.say(f"  [dry run] {label}")
            self.say(f"            {shlex.join(command)}")
            return 0
        self.say(f"  {label}")
        return subprocess.call(command)

    def _popen(self, command: list[str], *, label: str) -> subprocess.Popen | None:
        if self.dry_run:
            self.say(f"  [dry run] {label}")
            self.say(f"            {shlex.join(command)}")
            return None
        self.say(f"  {label}")
        # stdin closed so a background child can never compete with this
        # process for the terminal; stdout and stderr are inherited so driver
        # and publisher logs stay visible.
        return subprocess.Popen(command, stdin=subprocess.DEVNULL)

    # ---------------------------------------------------------------- commands built

    def hand_drivers_command(self) -> list[str]:
        launch = (
            f"ros2 launch wuji_teleop_bringup hand_drivers.launch.py "
            f"side:={self.state.connected_hands}"
        )
        return ["docker", "exec", TELEOP_CONTAINER, "bash", "-lc", _inner(launch)]

    def check_command(self, arms: str, hands: str) -> list[str]:
        check = f"ros2 run replay replay_check -- --arms {arms} --hands {hands}"
        return ["docker", "exec", TELEOP_CONTAINER, "bash", "-lc", _inner(check)]

    def g1_start_command(self) -> list[str]:
        return [
            "docker", "compose", "-f", str(COMPOSE_FILE), "run", "-d", "--rm",
            "--pull", "never", "--name", G1_CONTAINER, G1_SERVICE,
            "ros2", "launch", "g1_world_output", "g1_world_output.launch.py",
            "mode:=joint_replay", "arm_type:=G1_29", "control_rate:=250.0",
        ]

    def g1_stop_command(self) -> list[str]:
        return ["docker", "kill", "--signal=INT", G1_CONTAINER]

    def publisher_command(self, clip: str) -> list[str]:
        run = "ros2 run replay replay_publisher --"
        run += f" --clip {shlex.quote(f'{CONTAINER_WS}/clips/safe/{clip}')}"
        run += f" --arms {self.state.arms} --hands {self.state.hands}"
        if self.state.speed != "auto":
            run += f" --speed {self.state.speed}"
        # `echo $$` before `exec`: exec replaces this shell in place, so the
        # PID written here is the PID of whatever it becomes. That is what
        # makes the stop exact rather than a pattern guess.
        return ["docker", "exec", TELEOP_CONTAINER, "bash", "-lc",
                f"echo $$ > {PUBLISHER_PID_FILE} && " + _inner(run)]

    def publisher_stop_command(self) -> list[str]:
        """SIGINT the publisher inside the container, then prove it is gone.

        Both mechanisms run, sequenced with `;` and not `||`. The PID kill can
        succeed while leaving the node alive: `ros2 run` may fork rather than
        exec, in which case the recorded PID is the wrapper and the node is a
        separate process. Short-circuiting on the wrapper's exit status would
        then skip the pattern kill, which is the one that reaches the node, and
        the session would ramp the arm_sdk weight with a live publisher still
        writing joint targets.

        The PID is checked against /proc before use, because this container is
        long-lived and a recycled PID would otherwise be signalled instead.

        Exits non-zero while any publisher survives, which is the only signal
        the session has that the stop did not take.
        """
        stop = f"""
        pid=$(cat {PUBLISHER_PID_FILE} 2>/dev/null || true)
        if [ -n "$pid" ] && tr '\\0' ' ' < /proc/$pid/cmdline 2>/dev/null | grep -q replay_publisher; then
            kill -INT "$pid" 2>/dev/null || true
        fi
        pkill -INT -f 'replay_publisher.*--clip' 2>/dev/null || true
        for _ in 1 2 3 4 5 6 7 8 9 10; do
            pgrep -f 'replay_publisher.*--clip' >/dev/null 2>&1 || exit 0
            sleep 0.5
        done
        exit 1
        """
        return ["docker", "exec", TELEOP_CONTAINER, "bash", "-lc", stop]

    def publisher_kill_command(self) -> list[str]:
        """Last resort when SIGINT did not take. A SIGKILLed publisher stops
        writing, and the hands idle-release; a live one racing the weight ramp
        is worse."""
        return ["docker", "exec", TELEOP_CONTAINER, "pkill", "-KILL", "-f",
                "replay_publisher.*--clip"]

    def drivers_stop_command(self) -> list[str]:
        return ["docker", "exec", TELEOP_CONTAINER, "pkill", "-INT", "-f", "hand_drivers.launch.py"]

    # ---------------------------------------------------------------- clips

    def clip_names(self) -> list[str]:
        if not SAFE_CLIPS.is_dir():
            return []
        return sorted(
            entry.name
            for entry in SAFE_CLIPS.iterdir()
            if entry.is_dir() and not entry.name.startswith(".")
        )

    def missing_files(self, name: str) -> list[str]:
        return [f for f in CLIP_FILES if not (SAFE_CLIPS / name / f).is_file()]

    def clip_speeds(self, name: str) -> list[float] | None:
        """The clip's audited safe speeds, or None if clip.json cannot be read.

        Reading to display, not to decide. The publisher is the only thing that
        judges a clip or a speed; this session never refuses one on its own.
        """
        try:
            meta = json.loads((SAFE_CLIPS / name / "clip.json").read_text())
            return [float(s) for s in meta["safe_speeds"]]
        except Exception:
            return None

    # ---------------------------------------------------------------- command handlers

    def _set_side(self, which: str, words: list[str]) -> None:
        if len(words) != 1:
            raise CommandError(f"{which} takes one of: {', '.join(SIDES)}")
        value = words[0]
        if value not in SIDES:
            raise CommandError(f"{value!r} is not one of: {', '.join(SIDES)}")
        connected = getattr(self.state, f"connected_{which}")
        if value not in self.state.allowed(connected):
            raise CommandError(
                f"only {connected!r} was connected at startup, so {which} {value} is not "
                f"available. Restart with --{which} {value}."
            )
        other = "hands" if which == "arms" else "arms"
        if value == "none" and getattr(self.state, other) == "none":
            raise CommandError(f"{which} none with {other} none would publish nothing")
        setattr(self.state, which, value)
        self.say(f"  {which} = {value}")

    def cmd_arms(self, words: list[str]) -> None:
        self._set_side("arms", words)

    def cmd_hands(self, words: list[str]) -> None:
        self._set_side("hands", words)

    def cmd_speed(self, words: list[str]) -> None:
        if len(words) != 1:
            raise CommandError("speed takes a number in (0, 1], or 'auto'")
        value = words[0]
        if value != "auto":
            try:
                number = float(value)
            except ValueError:
                raise CommandError(f"{value!r} is not a number or 'auto'") from None
            if not 0.0 < number <= 1.0:
                raise CommandError(f"speed must be in (0, 1], got {value}")
        self.state.speed = value
        self.say(f"  speed = {value}")

    def cmd_ls(self, words: list[str]) -> None:
        names = self.clip_names()
        if not names:
            self.say(f"  no clips under {SAFE_CLIPS}")
            return
        width = max(len(n) for n in names)
        for name in names:
            missing = self.missing_files(name)
            if missing:
                self.say(f"  {name:<{width}}  unplayable, missing {', '.join(missing)}")
                continue
            speeds = self.clip_speeds(name)
            shown = "unreadable clip.json" if speeds is None else " ".join(f"{s:g}" for s in speeds)
            self.say(f"  {name:<{width}}  safe at {shown}")

    def cmd_state(self, words: list[str]) -> None:
        self.say(f"  connected  arms {self.state.connected_arms}, hands {self.state.connected_hands}")
        self.say(f"  driving    arms {self.state.arms}, hands {self.state.hands}")
        self.say(f"  speed      {self.state.speed}")
        if self.dry_run:
            self.say("  dry run    nothing is started; commands are printed")

    # ---------------------------------------------------------------- preflight

    def _container_state(self, name: str) -> str:
        """running | exited | ... | "" when absent. Same query replay.sh uses."""
        if self.dry_run:
            return ""
        result = subprocess.run(
            ["docker", "ps", "-a", "--filter", f"name=^{name}$", "--format", "{{.State}}"],
            capture_output=True, text=True,
        )
        return result.stdout.strip()

    def preflight(self) -> bool:
        """Refuse to start on the conditions replay.sh refuses to start on.

        Without these a session misdiagnoses its own failures. The important
        one is the second: a `docker compose run --name` against a name that is
        already taken fails, and a session that ignored that would later run
        `docker kill --signal=INT g1-world-output` and ramp somebody else's
        arms out from under them.
        """
        if self.dry_run:
            return True

        if self._container_state(TELEOP_CONTAINER) != "running":
            self.say(f"  the teleop container {TELEOP_CONTAINER!r} is not running.")
            self.say("  start it with: cd docker && docker compose up -d")
            return False

        if self.state.connected_arms != "none":
            state = self._container_state(G1_CONTAINER)
            if state == "running":
                self.say(f"  {G1_CONTAINER} is already running: something else holds the arms.")
                self.say(f"  If that is stale: docker kill --signal=INT {G1_CONTAINER}")
                return False
            if state:
                # A stopped leftover from a run without --rm; its name would
                # block ours.
                subprocess.run(["docker", "rm", "-f", G1_CONTAINER],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if subprocess.run(["docker", "image", "inspect", "g1-world-output:latest"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
                self.say("  the image g1-world-output:latest is missing. Build it once with:")
                self.say("  cd docker && COMPOSE_BAKE=false docker compose build g1_world_output")
                return False
        return True

    # ---------------------------------------------------------------- startup

    def start(self) -> bool:
        """Connect what the flags asked for. False if a device did not report.

        The readiness waits are `replay_check`, which already waits 30 s on the
        same sources the publisher waits on and exits non-zero naming what is
        missing. No waiting logic lives here.
        """
        if self.state.connected_hands != "none":
            # Flag first: a signal landing between the two would otherwise
            # leave the teardown unaware of a driver that is already up. The
            # stop is by pattern, so attempting it after a failed start is
            # harmless.
            self._drivers_started = True
            self._drivers = self._popen(
                self.hand_drivers_command(),
                label=f"start the hand drivers, side {self.state.connected_hands} (once, for the session)",
            )
            if self._drivers is not None and self._drivers.poll() is not None:
                self.say(f"  the hand driver launch exited {self._drivers.returncode} "
                         f"immediately; not waiting for hands that will never report")
                return False
            if self._call(self.check_command("none", self.state.connected_hands),
                          label="wait for the hands (replay_check, up to 30 s)") != 0:
                self.say("  the hands did not report; nothing is playable")
                return False

        if self.state.connected_arms != "none":
            self._g1_started = True  # before the start; see the driver note above
            if self._call(self.g1_start_command(),
                          label=f"start {G1_CONTAINER} to verify the arms") != 0:
                self.say(f"  {G1_CONTAINER} did not start")
                self._stop_g1()
                return False
            ok = self._call(self.check_command(self.state.connected_arms, "none"),
                            label="wait for the arms (replay_check, up to 30 s)") == 0
            self._stop_g1()
            if not ok:
                self.say("  the arms did not report; check the robot and g1_robot.yaml network_interface")
                return False
        return True

    # ---------------------------------------------------------------- playing

    def play(self, clip: str) -> None:
        clip = clip.strip().strip("/")
        if clip not in self.clip_names():
            raise CommandError(f"no clip {clip!r} under clips/safe (try: ls)")
        missing = self.missing_files(clip)
        if missing:
            raise CommandError(f"{clip} is missing {', '.join(missing)}; the publisher would refuse it")

        self.say(
            f"\n  playing {clip}  "
            f"(arms {self.state.arms}, hands {self.state.hands}, speed {self.state.speed})"
        )
        if self.state.arms != "none":
            self._g1_started = True  # before the start; see the note in start()
            if self._call(self.g1_start_command(), label=f"start {G1_CONTAINER}") != 0:
                self._stop_g1()
                raise CommandError(f"{G1_CONTAINER} did not start; nothing played")

        self._publisher_started = True
        self._publisher = self._popen(self.publisher_command(clip), label="replay_publisher")

        if self.dry_run:
            self.say("  [dry run] the clip would play once, then hold its last frame")
        elif self._publisher is not None:
            try:
                code = self._publisher.wait(timeout=PUBLISHER_GRACE_S)
            except subprocess.TimeoutExpired:
                code = None
            if code is not None:
                # The publisher never exits on success, so this is a refusal.
                self._publisher = None
                self._publisher_started = False
                self.say(f"  the publisher exited {code}; the clip did not play")
                self.release("nothing played")
                return

        self._wait("\n  Enter to release the arms and choose another clip: ")
        self.release("released")

    def release(self, why: str) -> None:
        """Stop the publisher, prove it stopped, then release the arms.

        The order is the safety property. The G1 node's weight ramp must not
        run while a publisher is still writing joint targets, so the ramp is
        gated on the publisher actually being gone rather than on having asked
        it to stop.
        """
        if self._publisher_started:
            # A publisher that died on its own is the ready-timeout case: it
            # waits up to 30 s for consumers and exits 1, which is longer than
            # the grace after starting it, so the session can have reported the
            # clip as playing.
            if self._publisher is not None and self._publisher.poll() is not None:
                self.say(f"  the publisher had already exited {self._publisher.returncode}; "
                         f"the clip did not finish")
            if self._call(self.publisher_stop_command(), label="stop replay_publisher") != 0:
                self.say("  the publisher did not stop on SIGINT; killing it before releasing "
                         "the arms")
                self._call(self.publisher_kill_command(), label="kill replay_publisher")
            if self._publisher is not None:
                try:
                    self._publisher.wait(timeout=G1_STOP_GRACE_S)
                except subprocess.TimeoutExpired:
                    # Killing this handle would only end the local docker exec
                    # client, never the process in the container, so there is
                    # nothing further to do here. The in-container kill above
                    # is the mechanism that matters.
                    self.say("  the docker exec client for the publisher is still open")
                self._publisher = None
            self._publisher_started = False
        self._stop_g1()
        self.say(f"  {why}\n")

    def _stop_g1(self) -> None:
        """SIGINT, wait out the grace, then fall back. Same shape as replay.sh's stop_g1."""
        if not self._g1_started:
            return
        self._call(self.g1_stop_command(),
                   label=f"stop {G1_CONTAINER}: arm_sdk weight 1 to 0 over 1.02 s")
        if not self.dry_run:
            try:
                subprocess.call(["docker", "wait", G1_CONTAINER],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                timeout=G1_STOP_GRACE_S)
            except subprocess.TimeoutExpired:
                self.say(f"  {G1_CONTAINER} did not exit in {G1_STOP_GRACE_S}s; stopping it")
                subprocess.call(["docker", "stop", "-t", "2", G1_CONTAINER],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._g1_started = False

    # ---------------------------------------------------------------- teardown

    def teardown(self) -> None:
        """Ctrl-C or quit, from any state. Publisher, then G1, then the drivers."""
        if self._torn_down:
            return
        self._torn_down = True
        self.say("\n  stopping")
        try:
            self.release("arms released")
        finally:
            # Reached even if releasing the arms raised. Leaving the hands
            # energized because an earlier step failed is not an option.
            if self._drivers_started:
                self._call(self.drivers_stop_command(),
                           label="stop the hand drivers: hands de-energize and disconnect")
                if self._drivers is not None:
                    try:
                        self._drivers.wait(timeout=G1_STOP_GRACE_S)
                    except subprocess.TimeoutExpired:
                        # As with the publisher, this handle is a docker exec
                        # client; killing it would not touch the drivers.
                        self.say("  the docker exec client for the drivers is still open")
                    self._drivers = None
                self._drivers_started = False


def build_terminal(session: Session) -> Terminal:
    state = session.state

    def quit_(words: list[str]) -> None:
        session.teardown()
        terminal.stop()

    commands = [
        Command("arms", session.cmd_arms, "which arms the next clip drives", SIDES),
        Command("hands", session.cmd_hands, "which hands the next clip drives", SIDES),
        Command("speed", session.cmd_speed, "playback speed for the next clip", ("auto",)),
        Command("ls", session.cmd_ls, "clips under clips/safe, with their safe speeds"),
        Command("state", session.cmd_state, "what is connected, and what is driving"),
        Command("help", lambda words: session.say(terminal.help_text()), "this list"),
        Command("quit", quit_, "stop everything and leave"),
    ]

    terminal = Terminal(
        prompt=lambda: f"[{state.arms}|{state.hands}] clip> ",
        commands=commands,
        fallback=session.play,
        fallback_candidates=session.clip_names,
        on_interrupt=session.teardown,
        history_file=HISTORY_FILE,
    )
    return terminal


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="replay_interactive.py",
        description="Interactive clip replay: connect once, play many clips.",
    )
    parser.add_argument("--arms", choices=SIDES, default="both",
                        help="arms to connect and drive (default both)")
    parser.add_argument("--hands", choices=SIDES, default="both",
                        help="hands to connect and drive (default both)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print every command instead of running it; needs no Docker or hardware")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    # Selecting nothing implies a dry run: there is no device to talk to.
    dry_run = args.dry_run or (args.arms == "none" and args.hands == "none")

    state = State(connected_arms=args.arms, connected_hands=args.hands,
                  arms=args.arms, hands=args.hands)
    session = Session(state, dry_run=dry_run)

    session.say("wuji interactive replay  (docs/spec/spec1_2.md)")
    if dry_run:
        session.say("Dry run: every command is printed instead of run. No Docker, no robot.")
    session.say()
    session.cmd_state([])
    session.say()

    terminal = build_terminal(session)

    # The handlers go on before start(), not after. start() brings the G1
    # container up to verify the arms and can sit in a 30 s replay_check, and a
    # Ctrl-C in that window used to reach the default handler and leave the
    # arms held at weight 1. The whole of main() is inside the try so that no
    # exception, from any source, can exit without a teardown.
    terminal.install_signal_handlers()
    try:
        if not session.preflight():
            return 1
        if not session.start():
            return 1

        session.say("Tab completes clips and commands; Tab on an empty line lists the commands.")
        session.say("Ctrl-C stops everything and exits, from anywhere.")
        session.say()
        terminal.run()
    except KeyboardInterrupt:
        pass
    finally:
        terminal.restore_signal_handlers()
        session.teardown()
    return 130 if terminal.interrupted else 0


if __name__ == "__main__":
    sys.exit(main())
