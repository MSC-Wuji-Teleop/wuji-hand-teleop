"""g1_world_output — Unitree G1_23 arm output for PICO teleoperation.

Subscribes to the same chest-frame pose topics as tianji_world_output,
remaps into the G1 pelvis frame, then runs Pinocchio/Casadi IK and DDS
arm control (5 DoF per arm).
"""
