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
| The **PICO 4 + Motion Trackers** arm path (`pico_teleop.launch.py`) — Developer Mode, XRoboToolkit APK, ADB reverse-forwarding | [PICO.md](PICO.md) |
| **Wuji Glove networking** — UDP on the glove LAN, and the multi-NIC routing gotcha (glove discovered but connect times out) | [wuji-glove-network.md](wuji-glove-network.md) |

## References (developing and debugging)

| Topic | Read |
|---|---|
| **Hardware spec**: G1 23-DoF, Wuji Hand 2, gloves + PICO. What the code assumes vs. what exists. Source of truth | [hardware_spec.md](hardware_spec.md) |
| Daily developer commands: container lifecycle, build, test, launch, sim modes, rebuild rules | [usage.md](usage.md) |
| System architecture: data flow, per-package roles, process/container model, config convention, invariants | [architecture.md](architecture.md) |
| **What was removed from the upstream fork, and why** | [cleanup.md](cleanup.md) |
| Camera topic map and troubleshooting. **Staged, not wired** — targets the G1 head cameras | [wuji-camera-topics.md](wuji-camera-topics.md) |

## What this folder is not

- **Not** a per-package code reference. For ROS2 topics, node parameters, and
  package layout, see the package READMEs under `src/`.
- **Not** install instructions for the codebase itself. The supported
  deployment is Docker — see [Install](../README.md#install)
  in the main README. The Dockerfile is the canonical recipe for host
  dependencies if you want to roll a bare-metal install yourself (unsupported).
