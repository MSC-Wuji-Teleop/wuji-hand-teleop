"""Arm joint-name tables, import-light (no SDK, no ROS).

robot_arm.py re-exports these beside its DDS controller, but that module
imports unitree_sdk2py at module level and cannot be loaded offline. The
replay package's conditioning/artifact code keeps its own literal copy of
the G1_29 table (it must run in containers where this package is not
built); the parity test in replay/test/test_pacer.py imports THIS module to
assert the two copies never drift -- a silent reorder here would send
conditioned columns to the wrong joints.

Order is the DDS motor-slot order: left 7 (shoulder pitch/roll/yaw, elbow,
wrist roll/pitch/yaw), then right 7 -- slots 15-21 and 22-28.
"""

G1_23_ARM_JOINT_NAMES = [
    'left_shoulder_pitch',
    'left_shoulder_roll',
    'left_shoulder_yaw',
    'left_elbow',
    'left_wrist_roll',
    'right_shoulder_pitch',
    'right_shoulder_roll',
    'right_shoulder_yaw',
    'right_elbow',
    'right_wrist_roll',
]

G1_29_ARM_JOINT_NAMES = [
    'left_shoulder_pitch',
    'left_shoulder_roll',
    'left_shoulder_yaw',
    'left_elbow',
    'left_wrist_roll',
    'left_wrist_pitch',
    'left_wrist_yaw',
    'right_shoulder_pitch',
    'right_shoulder_roll',
    'right_shoulder_yaw',
    'right_elbow',
    'right_wrist_roll',
    'right_wrist_pitch',
    'right_wrist_yaw',
]
