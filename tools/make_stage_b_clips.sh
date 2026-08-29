#!/usr/bin/env bash
# Generate the full Stage B single-joint artifact set (spec_1 / TUITION 7B):
# one slow raised-cosine ramp per joint, everything else held at zero, each
# through the same conditioning audit and the same load path as a real clip.
#
# Run inside the teleop container:
#   bash tools/make_stage_b_clips.sh [OUT_DIR] [ARM_AMP] [HAND_AMP]
#
# Defaults: OUT_DIR=~/wuji_clips/stage_b, ARM_AMP=0.2 rad, HAND_AMP=0.3 rad
# (inside every joint's position bounds; ramps sit at half the deploy
# velocity/acceleration caps).
#
# 7B order is enforced by the OPERATOR, not this script: all 20 joints on
# each hand first, then left arm, then right arm, one artifact at a time
# through run_ctl. This script only pre-generates the artifacts.

set -euo pipefail

OUT_DIR="${1:-$HOME/wuji_clips/stage_b}"
ARM_AMP="${2:-0.2}"
HAND_AMP="${3:-0.3}"

ARM_JOINTS=(
  left_shoulder_pitch left_shoulder_roll left_shoulder_yaw left_elbow
  left_wrist_roll left_wrist_pitch left_wrist_yaw
  right_shoulder_pitch right_shoulder_roll right_shoulder_yaw right_elbow
  right_wrist_roll right_wrist_pitch right_wrist_yaw
)
HAND_JOINTS=(
  thumb_cmc_flex thumb_cmc_abd thumb_mcp thumb_ip
  index_finger_mcp_flex index_finger_mcp_abd index_finger_pip index_finger_dip
  middle_finger_mcp_flex middle_finger_mcp_abd middle_finger_pip middle_finger_dip
  ring_finger_mcp_flex ring_finger_mcp_abd ring_finger_pip ring_finger_dip
  pinky_mcp_flex pinky_mcp_abd pinky_pip pinky_dip
)

fails=0
gen() {
  local spec="$1" amp="$2"
  if ! ros2 run replay condition_clip --single-joint "$spec" \
      --amplitude "$amp" --out-dir "$OUT_DIR" > /dev/null; then
    echo "FAIL: $spec"
    fails=$((fails + 1))
  fi
}

for side in left right; do
  for j in "${HAND_JOINTS[@]}"; do
    gen "${side}_hand:${j}" "$HAND_AMP"
  done
done
for j in "${ARM_JOINTS[@]}"; do
  gen "arm:${j}" "$ARM_AMP"
done

total=$(( ${#ARM_JOINTS[@]} + 2 * ${#HAND_JOINTS[@]} ))
echo
echo "generated $((total - fails))/$total single-joint artifacts in $OUT_DIR"
echo "load one at a time, e.g.:"
echo "  ros2 run replay run_ctl load $OUT_DIR/single_joint_left_hand_thumb_cmc_flex/conditioned_clip_v1.npz --arms '' --hands left --speed 1.0"
echo "  ros2 run replay run_ctl load $OUT_DIR/single_joint_arm_left_elbow/conditioned_clip_v1.npz --hands '' --speed 1.0"
exit "$fails"
