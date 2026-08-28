# ===========================================
# Wuji Hand Teleop ROS2 Docker Image
# Unitree G1 (23-DoF) + 2x Wuji Hand 2, driven by Wuji Gloves + PICO 4
# ===========================================
#
# Features:
#   - Wuji Glove drives the Wuji Hand (wuji_sdk UDP -> wuji_retargeting -> wujihand_driver)
#   - PICO 4 headset + 4 Motion Trackers for arm input (docs/PICO.md)
#   - MuJoCo tooling for the G1 sim/dry-run path
#
# The G1 arm controller is NOT in this image. It runs in its own container
# (see src/output_devices/g1_world_output/docker/Dockerfile) because its
# Pinocchio + CasADi build needs NumPy 1.x while this stack needs 2.x.
#
# Prerequisites on the host, both required before building:
#   git submodule update --init --recursive
#   git lfs install && git lfs pull
#
# Build:
#   cd docker && docker compose build
#
# Run:
#   cd docker && docker compose up -d
#   docker exec -it wuji-hand-teleop bash
#
# Camera notes:
#   ros-humble-realsense2-camera and ffmpeg are installed for the camera
#   pipeline in src/camera/, which is currently unwired: it targets D405 wrist
#   + HBVCAM head hardware we do not have, pending the G1 head cameras
#   (RealSense D435i built-in, D455 attachment). See docs/wuji-camera-topics.md.
#   H.264 encoding auto-detects NVENC when an NVIDIA GPU is present, else
#   libx264. NVENC needs nvidia-container-toolkit on the host plus the GPU
#   section uncommented in docker-compose.yml.
#
# ===========================================

FROM osrf/ros:humble-desktop

LABEL description="Wuji Hand Teleop: Unitree G1 23-DoF + 2x Wuji Hand 2, Wuji Gloves + PICO 4 input"

# -------------------------------------------
# 1. Build Arguments
# -------------------------------------------
ARG USERNAME=wuji
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Timezone
ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# China users: uncomment to switch to the Tsinghua mirror.
# RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list

# -------------------------------------------
# 2. System Dependencies
# -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake git \
    python3-pip python3-dev python3-pybind11 \
    # Hardware access
    libusb-1.0-0-dev libncursesw5-dev udev usbutils \
    # Camera drivers (src/camera/ is unwired; see the header note)
    ros-humble-realsense2-camera \
    ros-humble-compressed-image-transport \
    # H.264 encoding (head stereo -> PICO VR stream)
    ffmpeg \
    # DDS (remote monitoring; off by default in single-machine)
    ros-humble-rmw-cyclonedds-cpp \
    # XRoboToolkit PC-Service dependency
    xdg-utils \
    # ADB (wired PICO link: USB data transfer, port forwarding)
    adb \
    # Network tools (PICO connectivity debugging)
    net-tools iputils-ping \
    # GUI: CJK font (for Qt5) + OpenGL software rendering
    fonts-noto-cjk \
    libgl1-mesa-glx libgl1-mesa-dri \
    # Kernel tools (wuji_teleop_monitor lsmod probe)
    kmod \
    # Dev tools
    vim tmux wget curl sudo \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------
# 3. XRoboToolkit PC-Service (PICO data receiver)
# -------------------------------------------
# Upstream: https://github.com/XR-Robotics/XRoboToolkit-PC-Service (Apache-2.0)
# Source vendored at src/input_devices/pico_input/vendor/XRoboToolkit-PC-Service/
# The .deb ships via Git LFS at docker/prebuilt/ and is COPYed in at build time.
# After install: /opt/apps/roboticsservice/runService.sh
COPY docker/prebuilt/XRoboToolkit_PC_Service_*_ubuntu_22.04_amd64.deb /tmp/xrobotoolkit-pc-service.deb
# Fail fast if the .deb is still an LFS pointer (forgot to run `git lfs pull`):
# without this check dpkg below would silently fail with a cryptic
# "archive corrupt" error.
RUN if head -c 100 /tmp/xrobotoolkit-pc-service.deb | grep -q 'git-lfs.github.com/spec'; then \
        echo "ERROR: xrobotoolkit-pc-service.deb is a Git LFS pointer, not the real binary." >&2; \
        echo "       Run \`git lfs install && git lfs pull\` on the host before \`docker compose build\`." >&2; \
        exit 1; \
    fi \
    && (dpkg -i /tmp/xrobotoolkit-pc-service.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/xrobotoolkit-pc-service.deb

# -------------------------------------------
# 3b. WujiHandCpp C++ SDK (build-time dependency of wujihand_driver ROS2 node)
# -------------------------------------------
# ROS2 driver: https://github.com/wuji-technology/wujihandros2 (submodule at src/wujihandros2/)
# C++ SDK deb: https://github.com/wuji-technology/wujihandpy/releases
# After install: /usr/lib/libwujihandcpp.so + /usr/include/wujihandcpp/
ARG WUJIHANDCPP_VERSION=1.8.0
RUN wget -q https://github.com/wuji-technology/wujihandpy/releases/download/v${WUJIHANDCPP_VERSION}/wujihandcpp-${WUJIHANDCPP_VERSION}-amd64.deb \
        -O /tmp/wujihandcpp.deb \
    && (dpkg -i /tmp/wujihandcpp.deb || apt-get update && apt-get install -f -y) \
    && rm /tmp/wujihandcpp.deb

# -------------------------------------------
# 4. User Setup (non-root, hardware group permissions)
# -------------------------------------------
# dialout: serial | plugdev: USB | input: input devices | video: cameras
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,input,video $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# -------------------------------------------
# 4.5 Pinocchio (cmeel pip wheel, pinned to 4.0.0)
# -------------------------------------------
# pin 3.9.0 wheels bundled an urdfdom build with a soname (4.0) that wasn't
# carried by the cmeel-urdfdom dep, so `import pinocchio` failed at runtime
# with "liburdfdom_sensor.so.4.0: cannot open". pin 4.0.0 fixed the bundled
# urdfdom + rebuilt eigenpy against the NumPy 2 ABI, so the wheel imports
# cleanly with numpy==2.2.6 below (§5). Pinned to 4.0.0 for reproducibility.

# -------------------------------------------
# 5. Python Dependencies (versions pinned)
# -------------------------------------------
ARG PIP_INDEX_URL=""
ARG PIP_TIMEOUT=120

# Pinned versions: validated compatible combo.
# numpy 2.2.6 + scipy 1.14.1: no compatibility warnings.
# pin==4.0.0: Pinocchio rigid-body library used by wuji_retargeting (cmeel wheel
#   bundles eigenpy + urdfdom; 4.0.0 is the first release whose bundled urdfdom
#   sonames resolve cleanly and whose eigenpy ABI is rebuilt against NumPy 2).
# mujoco==3.12.0: needed by g1_world_output's MuJoCo sim-mode tooling --
#   scripts/sweep_and_visualize.py and scripts/mujoco_visualizer.py, both of
#   which render g1_wuji2_fixed.xml from this (NumPy 2.x) container, since
#   g1_world_output's own Pinocchio/NumPy-1.x container is deliberately kept
#   dependency-minimal (see that package's README). Not used by
#   wuji-retargeting itself: its installed package doesn't import mujoco
#   (viz/ isn't on the import path), so this pin exists solely for the G1
#   sim tooling.
# Dexterous hand: C++ SDK comes from the deb (see 3b); ROS2 driver via the wujihandros2 submodule.
RUN if [ -n "${PIP_INDEX_URL}" ]; then \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 ruamel.yaml==0.18.6 \
            opencv-python==4.13.0.92 \
            pin==4.0.0 \
            mujoco==3.12.0 \
            --index-url ${PIP_INDEX_URL}; \
    else \
        pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} \
            numpy==2.2.6 scipy==1.14.1 pyyaml==6.0.2 ruamel.yaml==0.18.6 \
            opencv-python==4.13.0.92 \
            pin==4.0.0 \
            mujoco==3.12.0; \
    fi

# -------------------------------------------
# 5.5 wuji-sdk (Wuji Glove SDK, UDP transport)
# -------------------------------------------
# Required by the wuji_glove input path: wujihand_node imports wuji_sdk
# lazily when wujihand_ik.yaml::input_source is "wuji_glove" (the default).
# Public PyPI release tracks Wuji Studio (current: 2026.5.26 <-> Studio 5.18).
ARG WUJI_SDK_VERSION=2026.5.26
RUN pip3 install --no-cache-dir --timeout ${PIP_TIMEOUT} wuji-sdk==${WUJI_SDK_VERSION}

# -------------------------------------------
# 6. wuji-retargeting (Wuji-owned, hand retargeting algorithm)
# -------------------------------------------
# Vendored via the `src/wuji-retargeting` git submodule — same pattern as
# `src/wujihandros2`. The Docker build context is `public/` (set by docker-
# compose.yml `context: ..`), so the submodule appears here at
# `src/wuji-retargeting`. Users MUST initialise submodules before building:
#   git clone --recurse-submodules https://github.com/.../wuji-hand-teleop.git
# or, post-clone: `git submodule update --init --recursive`.
#
# We pip-install (not colcon-build) at image-build time because wuji-retargeting
# is a pure-Python package. Upstream `pyproject.toml` declares [project] but
# omits [build-system], which pip 22's legacy mode rejects; the shim setup.py
# below restores legacy compat without mutating the submodule source. Pinocchio
# is supplied by the cmeel `pin==4.0.0` wheel in §5.
#
# URDF sideload: wuji-retargeting falls back to URDFs at
# `wuji_retargeting/wuji-description/hand/body/urdf/{left,right}.urdf`, normally
# populated by its `wuji-description` sub-submodule. `git submodule update
# --init --recursive` from the outer repo doesn't traverse that nested submodule
# reliably, so we copy the same URDFs from `wujihandros2/external/wuji-description`
# (same upstream, same tag-tracked content) into the installed package after pip.
#
# These are Wuji Hand 1 models and are only the FALLBACK, taken when a config
# sets no `optimizer.urdf_path`. Every retarget config under
# `output_devices/wujihand_output/config/` sets it, pointing at the Hand 2 models
# in `src/wujihand_urdf/` (bind-mounted from the host, so no image rebuild is
# needed to change them). The sideload stays for upstream's own example configs.
COPY src/wuji-retargeting /tmp/wuji-retargeting
COPY src/wujihandros2/external/wuji-description/hand/body/urdf /tmp/wuji-description-urdf
RUN printf 'from setuptools import setup, find_packages\nsetup(\n    name="wuji-retargeting",\n    version="2026.8.17",\n    packages=find_packages(include=["wuji_retargeting*"]),\n    install_requires=["numpy==2.2.6", "nlopt==2.10.0", "pyyaml", "scipy>=1.11.0"],\n)\n' > /tmp/wuji-retargeting/setup.py \
    && pip3 install --no-cache-dir /tmp/wuji-retargeting \
    && WUJI_RT_DIR=$(python3 -c "import wuji_retargeting, os; print(os.path.dirname(wuji_retargeting.__file__))") \
    && mkdir -p "$WUJI_RT_DIR/wuji-description/hand/body/urdf" \
    && cp /tmp/wuji-description-urdf/*.urdf "$WUJI_RT_DIR/wuji-description/hand/body/urdf/" \
    && rm -rf /tmp/wuji-retargeting /tmp/wuji-description-urdf

# -------------------------------------------
# 7. ROS2 Workspace (directory only; source mounted from host)
# -------------------------------------------
# Sources are bind-mounted to src/ via docker-compose volumes.
# entrypoint.sh handles first-run colcon build + shared-library symlinks.
RUN mkdir -p /home/$USERNAME/ros2_ws && chown $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# -------------------------------------------
# 8. Entrypoint & Config
# -------------------------------------------
USER root
# Legacy-builder compatible (no BuildKit --chmod).
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/scripts/ /entrypoint-scripts/
RUN chmod 755 /entrypoint.sh /entrypoint-scripts/*.sh
COPY docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

USER $USERNAME

# Auto-source ROS2 + workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc

# CycloneDDS RMW (used when distributed nodes are added)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
# xrobotoolkit_sdk: vendored .cpython-310.so; not installed by colcon install.
ENV PYTHONPATH=/home/$USERNAME/ros2_ws/src/input_devices/pico_input/prebuilt/x86_64:$PYTHONPATH
# Qt5/GUI: silence the XDG_RUNTIME_DIR warning.
ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p /tmp/runtime-$USERNAME && chmod 700 /tmp/runtime-$USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
