## Camera pipeline: what feeds the PICO headset once the G1 head cameras are in?

### Context:

`src/camera/` is kept in the tree but unwired: nothing in the launch graph
starts it, and it targets hardware this rig does not have — an HBVCAM USB UVC
stereo head plus two RealSense D405 wrist cameras. The G1's own cameras are a
RealSense D435i built into the head and a D455 as an attachment. Most of the
package survives that change cheaply: `create_realsense_camera()` forwards
`device_type` straight to the stock `rs_launch.py`, and `d435i` is already a
supported and in fact default type, so the driver side is a two-line edit plus
config. The `unified_stereo_node.py` capture front-end is a contained rewrite,
roughly fifty lines, swapping one `cv2.VideoCapture` call for a ROS2
subscriber; the H.264 encode, TCP server, and XRobo protocol downstream of it
are source-agnostic and need no change. The part that does not carry over is the
stereo geometry. The HBVCAM was a true side-by-side stereo pair with a roughly
IPD-scale baseline, which is what made the headset view work. Both the D435i and
the D455 expose a single RGB sensor plus a stereo *infrared* pair, and their IR
baselines differ from each other and from human IPD. So "what does the operator
actually see in the headset" is an open design question, not a port.

### Options:

1. **Stream the D435i IR stereo pair** — geometrically the closest single-device
   match to a head-mounted stereo view, and it needs no inter-camera
   synchronization because both sensors sit on one device behind one clock. But
   the imagery is greyscale, and with the depth projector enabled it carries the
   structured-light dot pattern, so the emitter has to be disabled and the scene
   relit or accepted as ambient-IR; colour cues that matter for grasping are
   lost.

2. **Pair the D435i and D455 as a wide-baseline stereo rig** — gives two colour
   streams and uses hardware the rig will already have mounted, and
   `camera_config.yaml`'s `enable_sync` field already anticipates RealSense
   master/slave sync. But the effective baseline is set by where the D455
   attachment physically sits rather than by optics, it will not match IPD, and
   mismatched baseline is a known cause of operator eye strain and depth
   misjudgement; it also makes the headset view depend on the attachment staying
   put.

3. **Send mono colour from the D435i** — simplest by a wide margin, full colour,
   no sync and no geometry to get right, and it reuses the existing encode and
   transport untouched. But it discards stereo depth perception entirely, which
   is the reason the headset path exists rather than a flat monitor.

### Recommendation:

Option 1, but decided by a bench test rather than on paper. Confirm the exact IR
baselines from the D435i and D455 datasheets, then run both option 1 and option
3 through the existing `ffmpeg_utils` and `xrobo_protocol` path to the headset
and have an operator attempt a grasp under each. The choice turns on whether
greyscale IR imagery is good enough for hand-eye work, which cannot be predicted
from specifications, and the test is cheap because the transport is already built
and camera-agnostic. Option 2 becomes the answer only if the bench test shows IR
imagery is unusable *and* colour stereo is judged necessary, since it is the only
option that delivers both — at the cost of accepting a non-IPD baseline.

---

### Reference: migration cost for the rest of the package

Independent of the stereo decision above. Nothing was deleted, so this is
scoping, not recovery.

| Component | Change | Size |
|---|---|---|
| `create_realsense_camera()` | add `'d455'` to the two type checks (`camera_launch.py:330`, `:364`). `d435i` already works | 2 lines |
| `config/camera_config.yaml` | rename slots, set `type:`, new serials, new TF frames | config only |
| slot iteration | the hardcoded `['head','left_wrist','right_wrist']` list (`:337`) | 1 list |
| `head` special-case | currently skipped in the loop (`:352-359`) so `unified_stereo` can own the UVC device; inverts to an ordinary RealSense call | ~10 lines |
| `unified_stereo_node.py` capture | `_open_camera` + `_capture_loop` become a ROS2 subscriber yielding numpy frames | ~50 lines |
| `ffmpeg_utils.build_camera_input_args` | V4L2 input args become piped raw frames | ~20 lines |
| encode / TCP serve / XRobo protocol | none, source-agnostic | 0 |

The Docker layer is already provisioned: `ros-humble-realsense2-camera`,
`ffmpeg`, the `c 81:*` V4L2 cgroup rule (librealsense uses the V4L2 backend, so
this is not UVC-only), and the commented-out NVENC block all remain in place.

The Camera Preview GUI (`ros2 run wuji_teleop_monitor camera`) was removed with
the rest of the Tianji-era Monitor entry points and would need rebuilding.
