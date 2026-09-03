#!/usr/bin/env bash
# One host command for a clip replay (docs/replay.md).
#
#   scripts/replay.sh [clips/safe/<clip>] [--arms none|left|right|both] [--hands none|left|right|both]
#                     [--speed S] [--check] [--sim] [-h]
#
# In order:
#   1. unless --arms none, starts the G1 node detached in its own container
#      (docker compose run ... g1_world_output ros2 launch g1_world_output g1_world_output.launch.py
#       mode:=joint_replay arm_type:=G1_29 control_rate:=250.0, plus dry_run:=true with --sim);
#   2. runs replay.launch.py in the teleop container in the foreground: the hand drivers for the
#      selected sides and the publisher, or replay_check instead of the publisher with --check, or no
#      drivers and the MuJoCo viewer with --sim;
#   3. on exit -- the launch ended, Ctrl-C, a signal -- stops the G1 container it started.
#
# Stop order, and why the G1 node goes last: Ctrl-C reaches the launch inside the teleop container
# first (publisher and hand drivers shut down), then this script's trap stops the G1 container,
# whose node releases the arm_sdk weight on shutdown so the onboard controller takes the arms back.
# The G1 container is stopped with SIGINT, not `docker stop`'s SIGTERM: `ros2 launch` (its PID 1)
# shuts its nodes down on SIGINT, but on SIGTERM it only cancels itself and warns about orphaned
# processes, and the node would then be SIGKILLed with the container before it could release the
# arms (launch/launch_service.py, Humble). `docker stop` stays as the fallback after the grace period.
#
# Exit status. On Humble `ros2 launch` returns 0 even when a required process (on_exit=Shutdown)
# died with a non-zero code -- established in the container with a throwaway launch file whose only
# process ran `exit 3`: the launch shut down and returned 0. Only an exception inside the launch
# itself (a refused argument combination) returns 1. So this script always captures the launch
# output and reads launch's own line for the process that decides the run -- replay_check with
# --check, replay_publisher otherwise:
#     [<node>-N]: process has finished cleanly                     -> 0
#     [<node>-N]: process has died [pid P, exit code E, cmd '...'] -> E
#     [launch]: user interrupted with ctrl-c                       -> 130
#     none of these (it never started, or died from a signal)      -> 1
# The publisher path needs this as much as the check does: a clip or speed the publisher refuses
# exits 2, and without the scrape `ros2 launch` would report that refusal to the operator's shell
# as success.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/docker-compose.yml"

TELEOP_CONTAINER="wuji-hand-teleop"     # docker-compose.yml container_name of the teleop service
G1_CONTAINER="g1-world-output"          # the --name docs/replay.md gives the G1 node's one-off container
G1_SERVICE="g1_world_output"            # docker-compose.yml service; naming it directly enables its "g1" profile
G1_LAUNCH=(ros2 launch g1_world_output g1_world_output.launch.py mode:=joint_replay arm_type:=G1_29 control_rate:=250.0)
G1_STOP_GRACE_S=10                      # SIGINT grace: launch waits 5 s on its node before escalating, the release itself is quick
CONTAINER_WS="/home/wuji/ros2_ws"       # the workspace in the teleop container (docker-compose.yml mounts)
HOST_SAFE_CLIPS="$REPO_ROOT/clips/safe"
CONTAINER_SAFE_CLIPS="$CONTAINER_WS/clips/safe"
SIDES="none left right both"

usage() {
    cat <<EOF
usage: scripts/replay.sh [clips/safe/<clip>] [--arms none|left|right|both] [--hands none|left|right|both]
                         [--speed S] [--check] [--sim] [-h]

Plays one prepared clip on the rig: starts the G1 node in its own container (unless --arms none),
runs replay.launch.py in the teleop container (hand drivers for the selected sides + the publisher),
and stops the G1 container on exit. Run on the host with the containers up (cd docker && docker compose up -d).

  clips/safe/<clip>  the clip directory; a bare <clip> name or an absolute path under clips/safe also works
  --arms SIDE        arm topics the publisher writes (default both); none also skips the G1 container
  --hands SIDE       hand driver topics the publisher writes (default both); none skips the hand drivers
  --speed S          0 < S <= 1 (default: the clip's fastest safe speed); a faster one is refused by the publisher
  --check            connection check only: drivers and G1 node with replay_check, no publisher; exits 0 when
                     every selected source reported within 20 s, 1 otherwise
  --sim              G1 node with dry_run:=true, no hand drivers, MuJoCo viewer on the composed model
  -h, --help         this text

Runbook: docs/replay.md
EOF
}

die() { printf 'replay.sh: %s\n' "$*" >&2; exit 1; }
die_usage() { printf 'replay.sh: %s\n\n' "$*" >&2; usage >&2; exit 1; }
need_value() { [[ $# -ge 2 && -n $2 ]] || die_usage "$1 needs a value"; }
is_side() { case " $SIDES " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }
as_bool() { if [[ $1 == 1 ]]; then echo true; else echo false; fi; }

# ---------------------------------------------------------------- arguments

CLIP_ARG=""
ARMS="both"
HANDS="both"
SPEED=""
CHECK=0
SIM=0
PRINT_PLAN=0   # hidden: print the resolved plan and exit 0 without touching docker (tests)

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --arms) need_value "$@"; ARMS="$2"; shift 2 ;;
        --arms=*) ARMS="${1#--arms=}"; shift ;;
        --hands) need_value "$@"; HANDS="$2"; shift 2 ;;
        --hands=*) HANDS="${1#--hands=}"; shift ;;
        --speed) need_value "$@"; SPEED="$2"; shift 2 ;;
        --speed=*) SPEED="${1#--speed=}"; shift ;;
        --check) CHECK=1; shift ;;
        --sim) SIM=1; shift ;;
        --print-plan) PRINT_PLAN=1; shift ;;
        -*) die_usage "unknown flag '$1'" ;;
        *)
            [[ -z $CLIP_ARG ]] || die_usage "two clips given: '$CLIP_ARG' and '$1'"
            CLIP_ARG="$1"; shift ;;
    esac
done

is_side "$ARMS" || die_usage "--arms must be one of: $SIDES (got '$ARMS')"
is_side "$HANDS" || die_usage "--hands must be one of: $SIDES (got '$HANDS')"
[[ $ARMS != none || $HANDS != none ]] || die_usage "--arms none with --hands none selects nothing"
if [[ -n $SPEED ]]; then
    if ! [[ $SPEED =~ ^[0-9]*\.?[0-9]+$ ]] || ! awk -v s="$SPEED" 'BEGIN { exit !(s > 0 && s <= 1) }'; then
        die_usage "--speed must be a number with 0 < S <= 1 (got '$SPEED')"
    fi
fi

# Normalise the clip argument to its directory name under clips/safe. Accepted: 'clips/safe/<name>'
# (as docs/replay.md writes it, resolved against the repo root so it also works from another cwd),
# an absolute host path inside clips/safe, or a bare '<name>'. Anything else is refused up front:
# the publisher only plays directories whose parent is clips/safe.
clip_name() {
    local arg="${1%/}" path safe rest
    case "$arg" in
        ""|.|..) die "'$1' is not a clip name" ;;
        */*)
            if [[ $arg == /* ]]; then path="$arg"
            elif [[ $arg == clips/* ]]; then path="$REPO_ROOT/$arg"
            else path="$PWD/$arg"
            fi
            path="$(realpath -m -- "$path")"
            safe="$(realpath -m -- "$HOST_SAFE_CLIPS")"
            [[ $path == "$safe"/* ]] || die "clip must be under clips/safe (got '$1')"
            rest="${path#"$safe"/}"
            [[ $rest != */* ]] || die "clip must be a directory directly under clips/safe (got '$1')"
            printf '%s\n' "$rest" ;;
        *) printf '%s\n' "$arg" ;;
    esac
}

CLIP_NAME=""
if [[ -n $CLIP_ARG ]]; then
    CLIP_NAME="$(clip_name "$CLIP_ARG")" || exit 1
    if [[ $CHECK == 0 && ! -d "$HOST_SAFE_CLIPS/$CLIP_NAME" ]]; then
        die "no clip directory $HOST_SAFE_CLIPS/$CLIP_NAME (see: ls clips/safe)"
    fi
elif [[ $CHECK == 0 ]]; then
    die_usage "a clip is required unless --check"
fi

# ---------------------------------------------------------------- the plan

# `ros2 launch` rejects `name:=` with an empty value as malformed, so clip and speed are passed
# only when set; the launch file's defaults ('') cover the rest.
LAUNCH_ARGS=()
[[ -z $CLIP_NAME ]] || LAUNCH_ARGS+=("clip:=$CONTAINER_SAFE_CLIPS/$CLIP_NAME")
LAUNCH_ARGS+=("arms:=$ARMS" "hands:=$HANDS")
[[ -z $SPEED ]] || LAUNCH_ARGS+=("speed:=$SPEED")
LAUNCH_ARGS+=("check:=$(as_bool "$CHECK")" "sim:=$(as_bool "$SIM")")

INNER="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/ros2_ws"
INNER+=" && exec ros2 launch wuji_teleop_bringup replay.launch.py"
for arg in "${LAUNCH_ARGS[@]}"; do INNER+=" $(printf '%q' "$arg")"; done

# A pty is what carries Ctrl-C into the container to the launch (docker exec forwards no signals);
# without a terminal on stdin the trap below asks the launch to shut down instead.
EXEC_CMD=(docker exec)
if [[ -t 0 ]]; then EXEC_CMD+=(-it); fi
if [[ $SIM == 1 && -n ${DISPLAY:-} ]]; then EXEC_CMD+=(-e "DISPLAY=$DISPLAY"); fi
EXEC_CMD+=("$TELEOP_CONTAINER" bash -lc "$INNER")

[[ $SIM == 0 ]] || G1_LAUNCH+=(dry_run:=true)
G1_CMD=(docker compose -f "$COMPOSE_FILE" run -d --rm --name "$G1_CONTAINER" "$G1_SERVICE" "${G1_LAUNCH[@]}")

if [[ $PRINT_PLAN == 1 ]]; then
    if [[ -n $CLIP_NAME ]]; then
        echo "clip (host):       $HOST_SAFE_CLIPS/$CLIP_NAME"
        echo "clip (container):  $CONTAINER_SAFE_CLIPS/$CLIP_NAME"
    else
        echo "clip:              none (--check)"
    fi
    if [[ $ARMS == none ]]; then
        echo "g1 container:      not started (--arms none)"
    else
        echo "g1 container:      ${G1_CMD[*]}"
    fi
    echo "teleop launch:     ${EXEC_CMD[*]}"
    if [[ $CHECK == 1 ]]; then
        echo "exit status:       replay_check's, read from the launch output"
    else
        echo "exit status:       replay_publisher's, read from the launch output"
    fi
    exit 0
fi

# ---------------------------------------------------------------- containers

container_state() { docker ps -a --filter "name=^$1\$" --format '{{.State}}'; }   # running | exited | ... | "" (absent)

[[ $(container_state "$TELEOP_CONTAINER") == running ]] \
    || die "teleop container '$TELEOP_CONTAINER' is not running; start it with: cd docker && docker compose up -d"

G1_STARTED=0
LAUNCH_RUNNING=0

stop_g1() {
    echo "replay.sh: stopping $G1_CONTAINER" >&2
    # Ctrl-C to the container's launch (see the header for why not docker stop's SIGTERM). A container
    # started with --rm removes itself when the launch exits, which is what `docker wait` sees.
    docker kill --signal=INT "$G1_CONTAINER" >/dev/null 2>&1 || return 0
    if ! timeout "$G1_STOP_GRACE_S" docker wait "$G1_CONTAINER" >/dev/null 2>&1; then
        docker stop -t 2 "$G1_CONTAINER" >/dev/null 2>&1 || true
    fi
}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM
    if [[ $LAUNCH_RUNNING == 1 ]]; then
        # Only reached when this script was signalled while the launch ran without a pty: docker
        # exec forwards nothing, so ask the launch to shut down itself (publisher and drivers first).
        docker exec "$TELEOP_CONTAINER" pkill -INT -f 'ros2 launch wuji_teleop_bringup replay.launch.py' >/dev/null 2>&1 || true
    fi
    [[ $G1_STARTED == 0 ]] || stop_g1
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ $ARMS != none ]]; then
    case "$(container_state "$G1_CONTAINER")" in
        running)
            die "container '$G1_CONTAINER' is already running: someone else holds the arms." \
                "If that is stale, stop it first: docker kill --signal=INT $G1_CONTAINER" ;;
        "") ;;
        *)  # a stopped leftover (a run without --rm); its name would block ours
            docker rm -f "$G1_CONTAINER" >/dev/null ;;
    esac
    echo "replay.sh: starting $G1_CONTAINER: ${G1_LAUNCH[*]}" >&2
    G1_ID="$("${G1_CMD[@]}")"
    G1_STARTED=1
    echo "replay.sh: started $G1_CONTAINER (${G1_ID:0:12})" >&2
fi

if [[ $SIM == 1 && -n ${DISPLAY:-} ]]; then
    xhost +local:docker >/dev/null 2>&1 || true   # once per host session (docs/usage.md); harmless to repeat
fi

# ---------------------------------------------------------------- the launch

# The exit status $2 earned, read from the captured launch output in $1 (see the header).
node_status() {
    local log code node
    node="$2"
    log="$(tr -d '\r' < "$1" | sed $'s/\e\\[[0-9;]*m//g')"
    if grep -q 'user interrupted with ctrl-c' <<<"$log"; then echo 130; return; fi
    if grep -Eq "\[$node-[0-9]+\]: process has finished cleanly" <<<"$log"; then echo 0; return; fi
    code="$(grep -Eo "\[$node-[0-9]+\]: process has died \[pid [0-9]+, exit code -?[0-9]+" <<<"$log" \
        | grep -Eo -- '-?[0-9]+$' | head -n 1 || true)"
    if [[ $code =~ ^[1-9][0-9]*$ ]]; then echo "$code"; else echo 1; fi
}

echo "replay.sh: ${LAUNCH_ARGS[*]}" >&2
if [[ $CHECK == 1 ]]; then DECIDING_NODE=replay_check; else DECIDING_NODE=replay_publisher; fi
LOG="$(mktemp "${TMPDIR:-/tmp}/replay-launch.XXXXXX")"
set +e
LAUNCH_RUNNING=1
"${EXEC_CMD[@]}" 2>&1 | tee "$LOG"
LAUNCH_RC=${PIPESTATUS[0]}
LAUNCH_RUNNING=0
set -e
if [[ $LAUNCH_RC -ne 0 ]]; then RC=$LAUNCH_RC; else RC="$(node_status "$LOG" "$DECIDING_NODE")"; fi
rm -f "$LOG"
if [[ $CHECK == 1 ]]; then
    if [[ $RC == 0 ]]; then echo "replay.sh: check OK" >&2; else echo "replay.sh: check FAILED (exit $RC)" >&2; fi
elif [[ $RC != 0 && $RC != 130 ]]; then
    echo "replay.sh: the publisher exited $RC; the clip did not play" >&2
fi
exit "$RC"
