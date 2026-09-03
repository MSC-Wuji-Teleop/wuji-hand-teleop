# Where this clip comes from

Unlike every other clip under `clips/safe/`, this one is not a recording. It
is generated: small joint ramps written to look like a bundle sample, so it
goes through the same preparation and audit as the rest.

```bash
# from the repo root
python3 tools/generate_sweep_sample.py
python3 tools/prepare_clip.py \
    --method-dir RobotSTAR_demos/sweep-test/samples/90_sweep_joints/GT \
    --out clips
```

The generator writes `RobotSTAR_demos/sweep-test/`, which is gitignored along
with the rest of the bundle, so run the first command before the second on a
fresh checkout. What the motion is, and why the hands only move their thumbs,
is in the generator's own docstring.

Regenerating reproduces the arm trajectory byte for byte and the hand
keypoints to 3e-17. The manifest hash still changes, because
`target_meta.json` records the time it was generated, so a clip prepared from
a regenerated sample carries a different `source.bundle_manifest_sha256` than
this one. The motion is the same.
