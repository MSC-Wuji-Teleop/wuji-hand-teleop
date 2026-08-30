# g1_wuji2_description

Composed Unitree G1 + dual Wuji Hand 2 models, in both G1 DoF variants.
Self-contained: every referenced mesh lives under `meshes/`.

Which variant the rig ends up as is unsettled, possibly both, so both are
carried here. See [hardware_spec.md](../../docs/spec/hardware_spec.md).

| File | Variant | What |
|---|---|---|
| `g1_23_wuji2.xml` | 23-DoF | MJCF, floating base. nq 70, nv 69, nu 63 |
| `g1_23_wuji2_fixed.xml` | 23-DoF | MJCF, pelvis welded at the stand pose. nq = nv = nu = 63 |
| `scene_g1_23_wuji2.xml` | 23-DoF | Floor and lighting around `g1_23_wuji2.xml`, for the viewer |
| `g1_23_wuji2.urdf` | 23-DoF | URDF mirror, fixed base, 63 revolute joints |
| `g1_29_wuji2.xml` | 29-DoF | MJCF, floating base. nq 76, nv 75, nu 69 |
| `g1_29_wuji2_fixed.xml` | 29-DoF | MJCF, pelvis welded at the stand pose. nq = nv = nu = 69 |
| `scene_g1_29_wuji2.xml` | 29-DoF | Floor and lighting around `g1_29_wuji2.xml` |
| `g1_29_wuji2.urdf` | 29-DoF | URDF mirror, fixed base, 69 revolute joints |

Use a fixed-base file for upper-body sim control: on hardware the legs are
owned by the G1's onboard balance controller.

Facts:

- Actuator and `ctrl` order: G1 body joints first, then left hand 20, then
  right hand 20. The body block is 23 (legs 12, waist yaw, arms 5 + 5) or 29
  (legs 12, waist 3, arms 7 + 7), in Unitree's joint order.
- All actuators are position servos. G1 joints: kp 500, critically damped per
  joint. Hand joints: vendor gains (kp 0.2 to 0.7, clamps 0.3 to 0.6 N m).
- Every MJCF carries a `stand` keyframe. Joint limits in each URDF are
  cross-checked against its MJCF at 1e-4.
- `meshes/g1/` is shared by both variants. Link names disambiguate the
  variant-specific parts: the 23-DoF model uses `torso_link_23dof_rev_1_0.STL`
  and `*_wrist_roll_bare.STL`, the 29-DoF model uses `torso_link_rev_1_0.STL`,
  `waist_{roll,yaw}_link_rev_1_0.STL`, and `*_wrist_{pitch,roll,yaw}_link.STL`.
- Unitree ships no hand-less 23-DoF wrist: the stock URDF fuses the wrist-roll
  module and rubber hand into one link. In the 23-DoF model that link carries a
  derived bare-wrist inertial (0.18693 kg, rubber hand subtracted by parallel
  axis) and a cropped mesh. The Wuji hand mounts at the ICP-located palm
  flange, wrist_roll + [0.1220, +-0.003, 0]. The 29-DoF model mounts on
  `wrist_yaw_link` instead.
- Since 2026-08-29 each hand hangs off a `{left,right}_hand_dock` link
  (`meshes/g1-hand-dock.stl`, 0.01136153 kg per side) placed at the flange
  above, with the hand mount 3.75 mm beyond the dock origin: 0.12575 m from
  wrist_roll on the 23-DoF model, 0.04525 m from `wrist_yaw_link` on the
  29-DoF model. The URDF makes the dock the mount's parent link; the MJCF
  keeps them siblings under the wrist body at the same composed offsets.
- The wrist-to-hand transform is still provisional in both variants. Whether
  `g1-hand-dock.stl` is a real G1-to-Hand2 adapter's CAD or a modelling
  placeholder is unrecorded, and no dimension of it has been measured against
  a physical part. The models regenerate when a measured adapter exists; see
  [hardware_spec.md](../../docs/spec/hardware_spec.md#mounting-adapter-modelled-cad-provenance-unconfirmed).
- Generated files. Do not hand-edit; regenerate from the composition build
  scripts.

Viewer check:

```bash
python -m mujoco.viewer --mjcf="$PWD/src/g1_wuji2_description/scene_g1_23_wuji2.xml"
python -m mujoco.viewer --mjcf="$PWD/src/g1_wuji2_description/scene_g1_29_wuji2.xml"
```
