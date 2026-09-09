#!/usr/bin/env python3
"""Pin scripts/replay.sh: what each mode resolves to, and what it refuses.

The script's hidden --print-plan resolves every command it would run and exits
without touching Docker, so these tests read the resolved plan rather than the
source. Nothing here starts a container, a node, or the robot.

The refusals matter as much as the plan: a flag the script cannot honour has
to be refused rather than quietly dropped, or an operator believes they asked
for something they did not get.
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


def test_the_publishers_default_ramp_is_left_alone():
    """Nothing passes ramp:= any more; the publisher's own default stands."""
    assert "ramp:=" not in plan(CLIP_NAME)["teleop launch"]


# --- refusals ----------------------------------------------------------------


@pytest.mark.parametrize("args,fragment", [
    ([], "a clip is required"),
    (["--arms", "sideways"], "--arms must be one of"),
    ([CLIP_NAME, "--hands", "sideways"], "--hands must be one of"),
    ([CLIP_NAME, "--arms", "none", "--hands", "none"], "selects nothing"),
    ([CLIP_NAME, "--speed", "2"], "--speed must be a number"),
    ([CLIP_NAME, "--speed", "0"], "--speed must be a number"),
])
def test_refusals(args, fragment):
    result = run(*args)
    assert result.returncode != 0
    assert fragment in result.stderr, result.stderr


# --- the rehome is gone (docs/spec/spec1_2.md) -------------------------------


@pytest.mark.parametrize("flag", ["--home", "--from"])
def test_the_rehome_flags_are_gone(flag):
    """Removed 2026-09-08. They must fail as unknown flags, not be ignored:
    an operator with the old command in shell history has to be told."""
    result = run(flag)
    assert result.returncode != 0
    assert f"unknown flag '{flag}'" in result.stderr, result.stderr


@needs_gnu_realpath
def test_a_clip_path_under_clips_safe_resolves_to_its_name():
    assert plan(CLIP_PATH)["clip (container)"].endswith(f"/clips/safe/{CLIP_NAME}")


@needs_gnu_realpath
def test_a_clip_outside_clips_safe_is_refused_before_docker():
    result = run("clips/rejected/whatever")
    assert result.returncode != 0
    assert "must be under clips/safe" in result.stderr


def test_help_does_not_advertise_the_removed_rehome():
    result = subprocess.run([str(SCRIPT), "--help"], capture_output=True, text=True, cwd=REPO_ROOT)
    assert result.returncode == 0
    assert "--home" not in result.stdout
    assert "--from" not in result.stdout
