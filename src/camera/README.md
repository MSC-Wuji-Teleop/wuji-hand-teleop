
# Camera Package

Wuji teleoperation system camera integration package.

> ## STATUS: staged, not wired
>
> **Nothing in the launch graph starts this package.** It was unwired from
> `pico_teleop.launch.py` on 2026-08-25, because the hardware it targets is not
> on this rig: an HBVCAM USB UVC stereo head plus two RealSense **D405** wrist
> cameras. It is kept, and kept building, because the G1's own head cameras
> (RealSense **D435i** built-in, **D455** attachment) are planned, and most of
> this package applies to them directly.
>
> It still runs standalone if you have the hardware:
> `ros2 launch camera camera_launch.py`.
>
> ### What changes for D435i / D455
>
> | Component | Change | Size |
> |---|---|---|
> | `create_realsense_camera()` | add `'d455'` to the two type checks (`camera_launch.py:330`, `:364`). **`d435i` already works** and is in fact the default type; `device_type` is forwarded straight to the stock `rs_launch.py` | 2 lines |
> | `config/camera_config.yaml` | rename slots, set `type:`, new serials, new TF frames | config only |
> | slot iteration | the hardcoded `['head','left_wrist','right_wrist']` list (`:337`) | 1 list |
> | `head` special-case | `head` is currently skipped in the loop (`:352-359`) so `unified_stereo` can own the UVC device. With a RealSense head this inverts to an ordinary `create_realsense_camera()` call | ~10 lines |
> | `unified_stereo_node.py` capture | `_open_camera` + `_capture_loop` swap from `cv2.VideoCapture` on V4L2 to a ROS2 subscriber yielding numpy frames. Everything downstream already consumes a plain frame | ~50 lines |
> | `ffmpeg_utils.build_camera_input_args` | V4L2 input args become piped raw frames | ~20 lines |
> | H.264 encode, TCP serve, XRobo protocol | **none.** `ffmpeg_utils.py` and `xrobo_protocol.py` are source-agnostic | 0 |
> | stereo pair for the headset | **open design question.** The HBVCAM was a true 60 mm-baseline stereo pair, which is what made PICO stereo viewing work. D435i's *color* stream is mono. Either use D435i + D455 as a wide-baseline pair (needs hardware sync, which `enable_sync` already anticipates) or send mono | new work |
>
> The Docker layer is already provisioned: `ros-humble-realsense2-camera`,
> `ffmpeg`, the `c 81:*` V4L2 cgroup rule (librealsense uses the V4L2 backend),
> and the commented-out NVENC block all remain in place.
>
> The Camera Preview GUI (`ros2 run wuji_teleop_monitor camera`) was removed
> with the rest of the Tianji-era Monitor entry points.

## Hardware Configuration

| Position | Default Camera | Sensor | Shutter | Interface | Device Symlink |
|----------|---------------|--------|---------|-----------|---------------|
| Head | HBVCAM-F2439GS-2 V11 | AR0234CS | Global shutter | USB UVC | `/dev/stereo_camera` |
| Left wrist | RealSense D405 | - | Global shutter | USB | `/dev/cam_left_wrist` |
| Right wrist | RealSense D405 | - | Global shutter | USB | `/dev/cam_right_wrist` |

> Global shutter cameras are selected for dexterous hand teleoperation to avoid the jello effect during fast motion.

## Architecture

Two data paths, managed uniformly by `camera_launch.py`:

### Path 1: Wrist RealSense → ROS2

```
camera_launch.py → realsense2_camera driver → /cam_{left,right}_wrist/color/image_rect_raw
```

Configuration: `config/camera_config.yaml`

### Path 2: Head Stereo → unified_stereo (ROS2 + PICO)

```
camera_launch.py → unified_stereo (single process, OpenCV capture)
  ├── ROS2: MJPEG→BGR→JPEG → /stereo/{left,right}/compressed (30fps)
  └── PICO: MJPEG→H.264 (FFmpeg) → TCP:12345 → PICO VR (60fps, on-demand)
```

- ROS2 publishing is always enabled (`enable_head:=true`)
- PICO H.264 streaming is enabled on demand (`enable_pico:=true`), automatically handshakes and streams when PICO connects
- Configuration: `config/stereo_head/stereo_head_config.yaml`

## First-Time Setup

```bash
# Install udev rules, create fixed device symlinks
bash src/camera/setup_cameras.sh
```

## Launch Commands

```bash
# Wrist + head stereo → ROS2 (default)
ros2 launch camera camera_launch.py

# Wrist RealSense only (disable head)
ros2 launch camera camera_launch.py enable_head:=false

# Wrist + head stereo → ROS2 + PICO streaming
ros2 launch camera camera_launch.py enable_pico:=true

# Head stereo only — PICO H.264 streaming, no wrist cameras
# (useful when the operator only needs the VR headset view)
ros2 launch camera camera_launch.py enable_wrist:=false enable_pico:=true

# Full teleoperation (cameras + hand + arm).
# Cameras spawn alongside teleop unless you pass enable_camera:=false.
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py
ros2 launch wuji_teleop_bringup pico_teleop.launch.py
```

## ROS2 Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cam_left_wrist/color/image_rect_raw` | sensor_msgs/Image | RealSense D405 driver |
| `/cam_right_wrist/color/image_rect_raw` | sensor_msgs/Image | RealSense D405 driver |
| `/stereo/left/compressed` | CompressedImage | unified_stereo |
| `/stereo/right/compressed` | CompressedImage | unified_stereo |

## Entry Points

| Name | Module | Caller |
|------|--------|--------|
| `unified_stereo` | stereocamera.unified_stereo | camera_launch.py (enable_head/enable_pico) |

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wrist_type` | (YAML) | Override wrist camera type (empty = use YAML, currently `d405`) |
| `enable_head` | true | Enable head stereo camera → ROS2 publishing |
| `enable_pico` | false | Enable head stereo camera → PICO H.264 streaming |
| `enable_wrist` | true | Enable wrist RealSense cameras (set `false` for head/PICO-only stereo) |
| `head_device` | /dev/stereo_camera | Head stereo camera device path |
| `head_fps` | 30 | Head stereo camera ROS2 publishing frame rate |
| `head_quality` | 70 | Head stereo camera JPEG compression quality |
| `head_width` | 2560 | Head stereo camera frame width (left+right stitched) |
| `head_height` | 720 | Head stereo camera frame height |

## Supported Cameras

### Head (Stereo RGB, side-by-side)

| Camera | Sensor | Shutter | Baseline | H-FOV | Resolution | Notes |
|--------|--------|---------|----------|-------|------------|-------|
| **HBVCAM-F2439GS-2 V11** | AR0234CS | **Global** | 60mm | 125° | 2560x720@60 | **Default**, recommended for dexterous hand |
| ZED Mini | OV4689 | Rolling | 63mm | 85° | 2560x720@60 | Alternative, USB 3.0 required |
| ZED 2i (4mm) | - | Rolling | 120mm | 65° | 2560x720@60 | Narrower FOV |
| Other USB stereo | - | - | - | - | Custom | Must output side-by-side |

> Head camera is read via OpenCV UVC, **no dedicated SDK or NVIDIA GPU required**.
> Any USB camera that outputs side-by-side stereo can be used.
> ZED series cameras **must be connected to USB 3.0 ports**; USB 2.0 only exposes IMU (HID), no video.

### Wrist (RealSense)

| Camera | Shutter | Resolution | Config Type | Notes |
|--------|---------|------------|-------------|-------|
| **RealSense D405** | **Global** | 848x480@30 | `d405` | **Default**, close range, recommended for dexterous hand |

## Switching Cameras

Edit `config/camera_config.yaml` — set the wrist `type` field per side
(only `d405` is qualified today; the `wrist_type` launch argument is
kept as the escape hatch for one-off swaps to other RealSense models):

```yaml
left_wrist:
  type: "d405"
right_wrist:
  type: "d405"
```

To switch head cameras, just change `video_device` and `resolution` (all UVC stereo cameras use the same `type: "usb"`):

```yaml
head:
  type: "usb"
  video_device: "/dev/stereo_camera"   # udev symlink
  resolution:
    width: 2560       # Total stereo width (single eye 1280)
    height: 720       # Must match actual camera specifications
    fps: 60
```

### Method 3: Replace Physical Camera Hardware

**Replacing wrist RealSense (same model D405):**

Only the serial number needs to be changed. Two D405s have the same vendor/product ID, so serial numbers must be used to distinguish left from right.

1. Check serial number: `rs-enumerate-devices | grep "Serial Number"`
2. Modify `serial_number` in `config/camera_config.yaml` (**required**, RealSense SDK uses this to open the camera)
3. Modify corresponding serial number in `config/udev/99-teleop-cameras.rules` (needed when Docker mount requires symlinks)
4. Re-run `bash src/camera/setup_cameras.sh` (only needed when udev rules are changed)

> **Note:** Step 2 is required. Steps 3-4 are only needed when symlinks like `/dev/cam_left_wrist` are required.
> When not using Docker, only steps 1-2 are needed.

**Replacing head stereo camera (same model HBVCAM):**

udev matches by vendor/product ID; same model can be swapped directly with no configuration changes needed.

**Replacing head stereo camera (different model):**

1. Confirm the new camera supports UVC side-by-side output
2. Modify vendor/product ID in `config/udev/99-teleop-cameras.rules`
3. Modify `head.resolution` in `config/camera_config.yaml` to match new camera specifications
4. Modify corresponding resolution in `config/stereo_head/stereo_head_config.yaml`
5. Re-run `bash src/camera/setup_cameras.sh`

## Docker

```bash
docker run \
  --device /dev/stereo_camera \
  --device /dev/cam_left_wrist \
  --device /dev/cam_right_wrist \
  your-image
```

## Directory Structure

```
camera/
├── config/
│   ├── camera_config.yaml              # Main config (camera type/device/resolution/serial number)
│   ├── stereo_head/stereo_head_config.yaml  # Head stereo runtime parameters
│   └── udev/99-teleop-cameras.rules    # udev rules (device symlinks)
├── launch/
│   └── camera_launch.py                # Unified entry point (wrist+head+PICO)
├── stereocamera/
│   ├── config_loader.py                # Configuration file loader
│   ├── unified_stereo.py               # Head stereo entry point (ROS2 + PICO H.264)
│   └── teleopVision/                   # Core library
│       ├── ffmpeg_utils.py             # FFmpeg encoder detection + NAL parsing
│       ├── unified_stereo_node.py      # Main node (OpenCV capture + ROS2 publish + PICO streaming)
│       └── xrobo_protocol.py           # XRoboToolkit compatible protocol
└── setup_cameras.sh                    # udev rules installation script
```
