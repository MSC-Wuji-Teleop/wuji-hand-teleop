# wujihand_urdf

Wuji Hand 2 URDF models, used by the retargeting optimizer as its IK model.

## What is here

| File | Model | Robot name |
|---|---|---|
| `wujihand_left.urdf` | Wuji Hand 2, **Beta 2**, left | `wujihand2-beta2-left` |
| `wujihand_right.urdf` | Wuji Hand 2, **Beta 2**, right | `wujihand2-beta2-right` |
| `deprecated/wujihand_hand_1_{left,right}.urdf` | Wuji **Hand 1** | `wujihand-{side}-v1.0.0` |

## Source

Both files are byte-for-byte copies of the official vendor package, taken from
the `wuji-description` submodule nested under `src/wuji-retargeting`:

```
repo    https://github.com/wuji-technology/wuji-description
tag     v2026.8.19  (commit b13f7d52b23cb79e35357303c72b7f61f1d2fda2)
path    hand2/hand2_beta2/body/urdf/{left,right}.urdf
```

To re-derive them:

```bash
cd src/wuji-retargeting/wuji_retargeting/wuji-description
git show v2026.8.19:hand2/hand2_beta2/body/urdf/left.urdf  > ../../../wujihand_urdf/wujihand_left.urdf
git show v2026.8.19:hand2/hand2_beta2/body/urdf/right.urdf > ../../../wujihand_urdf/wujihand_right.urdf
```

SHA-256, unchanged from upstream:

```
4515cb77eb7e6bc5da18edde3063ed44242a5597d8d38cb04a66b648d198ea30  wujihand_left.urdf
8c514f579944b49635027f5308990a68d0cddc5190800e4b5dfd86fc21c27332  wujihand_right.urdf
```

## Why Beta 2, and why the revision does not change IK

Upstream ships Beta 1 and Beta 2 as separate ROS2 packages
(`wuji_hand2_description` and `wuji_hand2_beta2_description`). Beta 2 is Beta 1
plus one tactile-sensor pad link per fingertip. Firmware v2.0.0 targets Beta 2;
Beta 1 does not receive it, so the revision decides the firmware line as well as
the description package.

Beta 2 is copied here because it matches the composed models in
`src/g1_wuji2_description/` and the firmware line above.

For retargeting the choice does not matter. The two revisions were diffed joint
by joint: all 25 joints Beta 1 and Beta 2 have in common carry **identical**
origin `xyz` and `rpy` (max absolute difference 0.000e+00), the same names, the
same order, and the same limits. Beta 2 adds only 5 fixed
`*_tip_sensor_frame_fixed` joints, which add no degree of freedom. IK output is
therefore the same on either revision.

## Joint order

Both hands declare 20 revolute joints, thumb to pinky, 4 per finger:

```
[cmc_flex, cmc_abd, mcp, ip]                 thumb
[mcp_flex, mcp_abd, pip, dip]                index, middle, ring, pinky
```

Position 0 is flexion and position 1 is abduction, confirmed by expressing each
joint axis in the root frame: the joint at position 0 is parallel to the PIP and
DIP axes, and the joint at position 1 is perpendicular to them. Hand 1 uses the
same convention (`finger{i}_joint1` is flexion, `joint2` is abduction), so the
flat 20-element order is unchanged by the Hand 1 to Hand 2 switch. That order
also matches the hand driver's hardware order (`starport_wuji_hand`
`joint_map.py`; the USB driver's `JOINT_NAMES`) and the `HAND_CODES` actuator
mapping in `output_devices/g1_world_output/scripts/_mujoco_common.py`
(`THJ0..THJ3, FFJ0..FFJ3, ...`).

## Meshes are not vendored

The `<visual>` and `<collision>` elements reference `../meshes/{side}/*.STL`,
which are not copied here (17 MB per hand). Those paths do not resolve from this
directory. This is deliberate: the only consumer is Pinocchio via
`optimizer.urdf_path`, and `pin.buildModelFromUrdf` builds the kinematic and
inertial model without reading geometry. For visualization, use the MJCF in
`src/g1_wuji2_description/`, which has its meshes alongside it.

## Consumers

`src/output_devices/wujihand_output/config/retarget_*.yaml` point
`optimizer.urdf_path` here, relative to the config directory
(`../../../wujihand_urdf/wujihand_{side}.urdf`). That relative path resolves
against the config YAML's own directory after symlink resolution, so it assumes
the workspace is built with `colcon build --symlink-install`, which is what the
README and `docker/entrypoint.sh` already use. A plain `colcon build` fails loudly
at load time with `FileNotFoundError`, it does not silently fall back.

The Hand 1 files under `deprecated/` are kept for provenance and for diffing
against old recordings. Nothing loads them.
