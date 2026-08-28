"""g1_world_output — Unitree G1 arm output node.

Three modes: 'pose' remaps chest-frame targets into the pelvis frame and
solves Pinocchio/CasADi IK; 'joint_replay' takes named joint targets and
skips IK; 'idle' holds position. Output goes over Unitree SDK2 DDS, which
is G1_23 only (5 DoF per arm).
"""
