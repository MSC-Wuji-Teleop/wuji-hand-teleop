# ===========================================
# g1_world_output — Unitree G1 arm teleop node
# Standalone container: talks to the rest of wuji-hand-teleop over ROS2/DDS
# (host networking + shared ROS_DOMAIN_ID / cyclonedds config), not via a
# shared filesystem or shared Python environment.
# ===========================================
#
# Why this node gets its own image instead of living in docker/Dockerfile:
# G1 IK needs Pinocchio compiled WITH CasADi Python bindings
# (`from pinocchio import casadi as cpin`). The PyPI wheel (`pin`, any
# version) never ships that binding — verified empirically across 4.1.0,
# 4.0.0 and 3.8.0, all missing `pinocchio.casadi` regardless of whether
# `casadi` is also pip-installed. The only prebuilt package that does ship
# a working `pinocchio.casadi` for this Ubuntu 22.04 / Python 3.10 target is
# the robotpkg apt build (`robotpkg-py310-pinocchio` + `robotpkg-py310-casadi`),
# and that build is linked against NumPy 1.x — it segfaults-on-import
# ("_ARRAY_API not found") under NumPy 2.x. The rest of wuji-hand-teleop
# (wuji_retargeting) pins NumPy 2.2.6 for its own cmeel `pin==4.0.0` build,
# which was specifically rebuilt for the NumPy 2 ABI. The two Pinocchio
# builds are mutually incompatible in one Python environment, so this node
# runs in its own container with its own NumPy 1.26.4 (matching the version
# unitree's own xr_teleoperate README validates with conda-forge Pinocchio).
#
# Build:
#   cd docker && docker compose build g1_world_output
# Run:
#   cd docker && docker compose up -d g1_world_output
# ===========================================

FROM ros:humble-ros-base

LABEL description="Unitree G1 dual-arm teleop output node (Pinocchio + CasADi IK, unitree_sdk2py DDS)"

ARG USERNAME=wuji
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# -------------------------------------------
# 1. System dependencies
# -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev \
    ros-humble-tf2-ros \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    # opencv-python (pulled in by unitree_sdk2py) needs libGL at runtime
    libgl1-mesa-glx \
    lsb-release curl gnupg ca-certificates \
    vim sudo \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------
# 2. Pinocchio + CasADi with working `pinocchio.casadi` bindings (robotpkg apt)
# -------------------------------------------
# See header comment: this is the only prebuilt combo (for jammy/py310) that
# actually exposes `from pinocchio import casadi as cpin`. Pinned implicitly
# to whatever robotpkg currently publishes for jammy; both packages come from
# the same build so their ABI always matches each other.
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc -o /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
        > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        robotpkg-py310-pinocchio \
        robotpkg-py310-casadi \
    && rm -rf /var/lib/apt/lists/*

# robotpkg installs under /opt/openrobots; not on the default Python/loader path.
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/openrobots/bin:$PATH

# -------------------------------------------
# 3. Python dependencies (versions pinned)
# -------------------------------------------
# NumPy 1.26.4: matches xr_teleoperate's own validated combo and satisfies
# robotpkg's Pinocchio/CasADi/eigenpy build (compiled against NumPy 1.x ABI;
# NumPy 2.x makes them fail with "_ARRAY_API not found" at import time).
# Do NOT bump this to 2.x in this image; it is deliberately independent of
# the numpy==2.2.6 pin in the main wuji-hand-teleop image.
ARG PIP_INDEX_URL=""
ARG PIP_TIMEOUT=120
RUN if [ -n "${PIP_INDEX_URL}" ]; then EXTRA="--index-url ${PIP_INDEX_URL}"; else EXTRA=""; fi \
    && pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} $EXTRA \
        numpy==1.26.4 \
        scipy==1.14.1 \
        pyyaml==6.0.2 \
        "opencv-python<5"

# -------------------------------------------
# 4. unitree_sdk2py (DDS LowCmd/LowState for the physical G1)
# -------------------------------------------
# Not on PyPI; vendored via the `src/unitree_sdk2_python` git submodule
# (same pattern as `src/wujihandros2` / `src/wuji-retargeting` in the main
# Dockerfile). Installed with --no-deps: its setup.py pulls unpinned
# "numpy" and "opencv-python", and an unpinned opencv-python resolves to a
# NumPy-2-only build that would silently upgrade NumPy out from under
# Pinocchio (verified: this is exactly what breaks the import). cyclonedds
# is pinned to the exact version unitree_sdk2py declares.
COPY src/unitree_sdk2_python /tmp/unitree_sdk2_python
RUN pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} cyclonedds==0.10.2 \
    && pip3 install --no-cache-dir --no-deps /tmp/unitree_sdk2_python \
    && rm -rf /tmp/unitree_sdk2_python
# Upstream setup.py has no package_data, so the wheel build drops the CRC
# shared libraries (utils/lib/crc_*.so) that the LowCmd write path dlopens;
# upstream's own instructions assume `pip install -e .`, which keeps them.
COPY src/unitree_sdk2_python/unitree_sdk2py/utils/lib \
    /usr/local/lib/python3.10/dist-packages/unitree_sdk2py/utils/lib

# -------------------------------------------
# 5. User setup (non-root, matches main wuji-hand-teleop container's UID)
# -------------------------------------------
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN mkdir -p /home/$USERNAME/ros2_ws/src/g1_world_output \
    && mkdir -p /home/$USERNAME/ros2_ws/src/g1_wuji2_description \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# g1_wuji2_description (URDF + meshes) is bind-mounted at runtime by
# docker-compose.yml (read-only), not baked in here -- it keeps this image
# and the main `teleop` image reading the exact same files, so regenerating
# the model can't silently go stale in just one of the two containers.
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# -------------------------------------------
# 6. Entrypoint
# -------------------------------------------
USER root
COPY src/output_devices/g1_world_output/docker/entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
COPY docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml
USER $USERNAME

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "g1_world_output", "g1_world_output.launch.py"]
