# replay

Replay input device for the SOT handoff bundle
(`RobotSTAR_demos/`, repo root). One node,
`replay_publisher`, reads a sample's `GT/` or `Ours/` method directory
and publishes, on a single timer so arms and hands stay time-aligned:

| Topic | Type | Content |
|---|---|---|
| `/left_arm/joint_targets`, `/right_arm/joint_targets` | `sensor_msgs/JointState` | Arm joints from `g1_reference/controller_reference_v7.npz` `body_q`, **named** per `target_meta.json` (7/side for the bundle's 29-DoF layout). Consumed by `g1_world_output` in `mode:=joint_replay` |
| `/left_hand/keypoints21`, `/right_hand/keypoints21` | `std_msgs/Float64MultiArray` (63 floats) | 21-point MediaPipe hand keypoints in meters from `hand2_input/*_human_targets_v5.npz`. Consumed by `wujihand_controller` with `input_source: "keypoints_topic"`, which retargets them live |

The bundle's precomputed hand joint columns are **never** published — per the
bundle's TUITION.md they target the legacy hand model and must not reach a
real Hand 2. Hands are always regenerated from the keypoints downstream.

Pure `rclpy` + `numpy`: runs in the main teleop container, never touches DDS
or any device SDK. Holds the final frame at clip end (the bundle's
`hold_last_target` semantics) unless `--loop`.

```bash
ros2 run replay replay_publisher -- \
    --method-dir <bundle>/samples/<sample>/GT \
    [--rate HZ] [--loop] [--no-arms] [--no-hands]
```

Full runbook (all four processes + MuJoCo viewer):
[docs/usage.md — SOT bundle replay](../../../docs/usage.md#sot-bundle-replay-sim).
Design and safety context:
[docs/architecture.md — SOT bundle replay](../../../docs/architecture.md#sot-bundle-replay).
