#!/usr/bin/env python3
"""Pin scripts/replay.sh: what each mode resolves to, and what it refuses.

The script's hidden --print-plan resolves every command it would run and exits
without touching Docker, so these tests read the resolved plan rather than the
source. Nothing here starts a container, a node, or the robot.

The refusals matter more than the plan. --home is the one command in the repo
whose whole job is to move the arms without a clip the operator chose, so every
flag it cannot honour has to be refused rather than quietly dropped: silently
ignoring --speed or --hands would let an operator believe they had asked for
something they did not get.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "scripts" / "replay.sh"

# The script takes a bare clip name as well as a path, and only the path form
# goes through realpath. Most tests use the bare name so they run anywhere.
CLIP_NAME = "90_sweep_joints_GT"
CLIP_PATH = f"clips/safe/{CLIP_NAME}"

pytestmark = pytest.mark.skipif(not SCRIPT.is_file(), reason="scripts/replay.sh not in this tree")

# clip_name() uses `realpath -m`, which is GNU. The rig host is Linux; a
# developer machine may not be, so the path-form tests skip rather than fail.
HAS_GNU_REALPATH = subprocess.run(
    ["realpath", "-m", "/tmp/x"], capture_output=True).returncode == 0
needs_gnu_realpath = pytest.mark.skipif(
    not HAS_GNU_REALPATH, reason="clip paths need GNU realpath -m")


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([str(SCRIPT), "--print-plan", *args],
                          capture_output=True, text=True, cwd=REPO_ROOT)


def plan(*args: str) -> dict[str, str]:
    result = run(*args)
    assert result.returncode == 0, result.stderr
    rows = {}
    for line in result.stdout.splitlines():
        key, _, value = line.partition(":")
        rows[key.strip()] = value.strip()
    return rows


# --- the ordinary replay plan ------------------------------------------------


def test_a_clip_replay_starts_the_g1_container_and_the_teleop_launch():
    rows = plan(CLIP_NAME)
    assert rows["clip (container)"].endswith("/clips/safe/90_sweep_joints_GT")
    assert "mode:=joint_replay arm_type:=G1_29 control_rate:=250.0" in rows["g1 container"]
    assert "arms:=both hands:=both" in rows["teleop launch"]
    assert rows["exit status"].startswith("replay_publisher's")


def test_arms_none_skips_the_g1_container():
    assert plan(CLIP_NAME, "--arms", "none", "--hands", "left")["g1 container"] == "not started (--arms none)"


def test_sim_adds_dry_run_and_asks_for_the_viewer():
    rows = plan(CLIP_NAME, "--sim")
    assert "dry_run:=true" in rows["g1 container"]
    assert "sim:=true" in rows["teleop launch"]


# --- the home plan -----------------------------------------------------------


def test_home_resolves_capture_then_generate_then_the_same_launch():
    rows = plan("--home")
    assert rows["mode"].startswith("home")
    assert rows["start pose"].startswith("json:/tmp/home_pose_")
    assert "/clips/home/home_" in rows["clip (container)"]
    assert "ros2 run replay capture_arm_pose" in rows["step capture"]
    assert "tools/make_home_clip.py" in rows["step generate"]
    # The motion goes through the ordinary launch and the ordinary publisher.
    assert "ros2 launch wuji_teleop_bringup replay.launch.py" in rows["teleop launch"]


def test_home_starts_no_hand_driver():
    rows = plan("--home")
    assert "hands:=none" in rows["teleop launch"]


def test_home_disables_the_publishers_approach_ramp():
    """The clip's frame 0 is the measured pose, so approaching frame 0 is a move
    to where the arms already are, and its 2 s is not in the printed duration."""
    assert "ramp:=0" in plan("--home")["teleop launch"]


def test_an_ordinary_replay_keeps_the_publishers_default_ramp():
    assert "ramp:=" not in plan(CLIP_NAME)["teleop launch"]


def test_home_plays_the_clip_it_generated():
    rows = plan("--home")
    clip = rows["clip (container)"]
    assert f"clip:={clip}" in rows["teleop launch"]
    # The generator is told --out clips --name <stamp>; the launch plays
    # clips/home/<stamp>. The name is what ties the two together.
    name = clip.rsplit("/", 1)[-1]
    assert f"--name {name}" in rows["step generate"]
    assert clip.endswith(f"/clips/home/{name}")


def test_home_passes_the_captured_pose_to_the_generator():
    rows = plan("--home")
    pose_file = rows["start pose"][len("json:"):]
    assert pose_file in rows["step capture"]
    assert pose_file in rows["step generate"]


def test_home_arms_selection_reaches_the_launch_and_the_capture():
    rows = plan("--home", "--arms", "left")
    assert "arms:=left" in rows["teleop launch"]
    assert "--arms left" in rows["step capture"]


def test_home_with_from_skips_the_capture_entirely():
    rows = plan("--home", "--from", "stand")
    assert rows["step capture"] == "skipped (--from stand)"
    assert rows["start pose"] == "stand"
    assert "--start-pose stand" in rows["step generate"]


def test_home_sim_needs_from_because_a_dry_run_node_publishes_no_state():
    rows = plan("--home", "--sim", "--from", "clip:clips/safe/90_sweep_joints_GT@last")
    assert "dry_run:=true" in rows["g1 container"]
    assert rows["step capture"].startswith("skipped")


def test_home_reports_the_failing_step_in_its_exit_status():
    assert plan("--home")["exit status"] == (
        "the capture's or the generator's if either fails, else replay_publisher's"
    )


# --- refusals ----------------------------------------------------------------


@pytest.mark.parametrize("args,fragment", [
    (["--home", CLIP_NAME], "takes no clip"),
    (["--home", "--speed", "0.5"], "takes no --speed"),
    (["--home", "--hands", "both"], "takes no --hands"),
    (["--home", "--hands", "none"], "takes no --hands"),
    (["--home", "--arms", "none"], "moves nothing"),
    (["--home", "--check"], "separate modes"),
    (["--home", "--sim"], "needs --from"),
    (["--from", "stand"], "only meaningful with --home"),
    ([], "a clip is required"),
    (["--home", "--arms", "sideways"], "--arms must be one of"),
])
def test_refusals(args, fragment):
    result = run(*args)
    assert result.returncode != 0
    assert fragment in result.stderr, result.stderr


def test_home_takes_no_clip_even_when_the_clip_exists():
    """The guard is the mode, not whether the path resolves."""
    assert (REPO_ROOT / CLIP_PATH).is_dir()
    assert run("--home", CLIP_NAME).returncode != 0


@needs_gnu_realpath
def test_a_clip_path_under_clips_safe_resolves_to_its_name():
    assert plan(CLIP_PATH)["clip (container)"].endswith(f"/clips/safe/{CLIP_NAME}")


@needs_gnu_realpath
def test_a_clip_outside_clips_safe_is_refused_before_docker():
    result = run("clips/rejected/whatever")
    assert result.returncode != 0
    assert "must be under clips/safe" in result.stderr


def test_help_names_home_as_not_an_e_stop():
    """An operator reading --help must not mistake this for a fast stop."""
    result = subprocess.run([str(SCRIPT), "--help"], capture_output=True, text=True, cwd=REPO_ROOT)
    assert result.returncode == 0
    assert "--home" in result.stdout
    assert "NOT an e-stop" in result.stdout
