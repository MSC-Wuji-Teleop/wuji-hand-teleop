## Monitor GUI: how should the operator start the G1 arms from one place?

### Context:

The Monitor GUI exposes two presets, `Hand only (Wuji Glove)` and `Hand + PICO
input`, and neither starts an arm output. A full PICO session therefore needs
two terminals: `pico_teleop.launch.py` in the `teleop` container, and
`g1_world_output.launch.py` in the `g1-world-output` container. The split is
structural, not an oversight — `g1_world_output` needs a Pinocchio + CasADi
build linked against NumPy 1.x while the rest of the stack needs 2.x, so it
ships as a separate image. The GUI itself runs *inside* the `teleop` container
(`scripts/launch_ui_docker.sh` does `docker exec wuji-hand-teleop ... ros2 run
wuji_teleop_monitor monitor`), so its `subprocess.Popen` launches land in that
container and cannot reach the other one. Adding a `LAUNCH_CONFIGS` entry is
therefore not sufficient; the reach problem has to be solved first.

### Options:

1. **Mount the Docker socket into the teleop container** — add
   `/var/run/docker.sock` as a volume and have the GUI shell out to `docker
   compose run --rm g1_world_output ...`. Smallest change, roughly a compose
   volume plus twenty lines in `run_teleop.py`, and preserves the current
   on-demand start; but it grants the container effective root on the host, and
   the GUI gains a Docker dependency that has nothing to do with teleop.

2. **Keep the G1 container running and control the node over ROS2** — bring it
   up once with `docker compose --profile g1 up -d`, and give
   `g1_world_output_node` a service or lifecycle interface the GUI calls to arm
   and disarm the control loop. No privilege escalation, no Docker knowledge in
   the GUI, and it uses the DDS link that already works between the two
   containers. It also fixes a second gap: the GUI currently has no way to know
   whether an arm output is alive at all. Costs more than the socket approach,
   since the node needs a real standby state, and it means the container idles
   with a DDS connection open.

3. **Leave it manual and document the two-terminal flow** — zero code, zero
   risk, and the flow is already written up in `docs/usage.md` and
   `README.md`; but every session carries a step that is easy to forget, and
   forgetting it looks like "the arms are broken" rather than "the arm node was
   never started."

### Recommendation:

Option 2. It is the only one that puts arm state into the same ROS2 graph the
GUI already reads joint topics from, so the GUI can show whether the arms are
live instead of just firing a command into the dark, and it avoids handing the
container root on the host for what is fundamentally a UI convenience. Option 1
wins instead if the G1 container must not idle — for example if holding the DDS
link to the robot while disarmed is itself considered unsafe, in which case
on-demand process start is the requirement and the socket is the direct way to
get it.
