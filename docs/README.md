# `docs/` — Guides and References

Two kinds of documents live here: **operator setup guides**, read once before
first use, and **repo-level references** for developing on or debugging the
stack. Package-specific deep-dives stay next to the code they describe (e.g.
`src/input_devices/pico_input/ARCHITECTURE.md`).

The main entry point for the whole project is [`../README.md`](../README.md).
The documents below cover what is too long or too device-specific to put
inline there.

## Setup guides (read once, before first use)

| If you are setting up… | Read |
|---|---|
| The **HTC Vive Tracker** arm path (default, `wuji_teleop.launch.py`) — SteamVR null driver, base stations, dongles, tracker pairing | [STEAMVR.md](STEAMVR.md) |
| The **PICO 4 + Motion Trackers** arm path (`pico_teleop.launch.py`) — Developer Mode, XRoboToolkit APK, ADB reverse-forwarding, H.264 stereo streaming | [PICO.md](PICO.md) |
| **Mounting trackers on the body** — straps, orientation, demo video | [tracker-wearing-guide.md](tracker-wearing-guide.md) |

Pick one arm path. The two are alternatives; you don't need both.

## References (developing and debugging)

| Topic | Read |
|---|---|
| Daily developer commands: container lifecycle, build, test, launch, sim modes, rebuild rules | [usage.md](usage.md) |
| System architecture: data flow, per-package roles, process/container model, config convention, invariants | [architecture.md](architecture.md) |
| Camera topic map, the 2x2 preview, blank-tile and D405-serial troubleshooting | [wuji-camera-topics.md](wuji-camera-topics.md) |
| Wuji Glove UDP networking and the multi-NIC routing gotcha (glove discovered but connect times out) | [wuji-glove-network.md](wuji-glove-network.md) |

## What this folder is not

- **Not** a per-package code reference. For ROS2 topics, node parameters, and
  package layout, see the package READMEs under `src/`.
- **Not** install instructions for the codebase itself. The supported
  deployment is Docker — see [Quick Start (Docker)](../README.md#quick-start-docker)
  in the main README. The Dockerfile is the canonical recipe for host
  dependencies if you want to roll a bare-metal install yourself (unsupported).

## Media

`images/` holds figures linked from the guides above. `teleop-demo.mp4` and `tracker-wearing-demo.mp4` are operator-facing demo clips referenced from the main README and from `tracker-wearing-guide.md`.
