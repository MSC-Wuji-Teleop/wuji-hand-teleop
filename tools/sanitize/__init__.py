"""Offline drop-in sanitizer for RoboSTAR retargeting output.

docs/spec/RoboSTAR_dropin_fix.md. Reads a joint-space clip (npz arrays or a
replay clip directory), extracts the wrist and elbow poses its arm joints
imply, re-solves them against src/g1_wuji2_description/g1_29_wuji2.urdf with
a continuity-seeded IK that also enforces joint limits, a per-frame step
clamp and full-mesh self-collision clearance, and writes the same layout back
out so the result drops straight into the existing replay pipeline.

The three defects it targets, from the spec:

    1. wrists phase through each other -- no collision checking anywhere on
       the replay path. The clip audit measures contact FORCE in MuJoCo,
       which is a different question from whether two meshes overlap and by
       how much; this tool checks the URDF's collision meshes with coal and
       constrains the frame's own IK solve with what it finds.
    2. branch flipping -- one wrist pose admits many exact arm solutions
       (measured: 32 distinct ones for a single pose when only the 6-DoF
       wrist task is imposed), so an unseeded per-frame solve is free to jump
       between them. The fix is the solve order the spec asks for: the three
       shoulder joints are solved for the elbow centre first (a square
       solve, since the elbow centre depends on nothing else), and only then
       are the remaining four joints solved for the wrist pose. With the
       elbow pinned the same experiment returns exactly one solution.
    3. wrist rotation drift -- a slow accumulation that ends the clip with
       the hands inverted. Measured per wrist joint and capped.

Module map:

    model      URDF + collision geometry, index maps, locked joints, FK
    clipio     the two npz layouts, auto-detected, echoed back on write
    unwrap     2pi wraps, drift measurement and capping, flip detection
    ik         both arms in one 14-DoF continuity-seeded damped least squares
    collision  pair set, classification, per-frame check, separation rows
    report     sanitize.json and the operator-facing failure lines
    cli        the command line

Nothing here talks to ROS, DDS or hardware, and nothing re-solves the hand
joints: hand angles pass through (clamped to the URDF limits) exactly as they
arrived, because regenerating them is the retargeter's job (CLAUDE.md: hand
joints are always regenerated from the bundle keypoints, never adapted).
"""

from __future__ import annotations

TOOL_ID = "sanitize/1"

__all__ = ["TOOL_ID"]
