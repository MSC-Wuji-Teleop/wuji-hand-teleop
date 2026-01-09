# Wuji Hand Teleop ROS2

A ROS2-based teleoperation system for Wuji Hand and Tianji Arm, supporting multiple input devices including Apple Vision Pro, MANUS Gloves, HTC Vive Trackers, and custom devices.

基于 ROS2 的 Wuji Hand 和天机机械臂遥操作系统，支持 Apple Vision Pro、MANUS 数据手套、HTC Vive Tracker 等多种输入设备，也支持自定义输入设备（只需将节点输出维护为对应的话题格式）。

We welcome contributions to the Wuji ecosystem!

我们欢迎朋友们一起进行二次开发，一起打造 Wuji 生态。

---

## Table of Contents / 目录

1. [Overview / 概述](#1-overview--概述)
2. [System Architecture / 系统架构](#2-system-architecture--系统架构)
3. [Installation / 环境配置](#3-installation--环境配置)
4. [Quick Start / 快速开始](#4-quick-start--快速开始)
5. [Node Reference / 节点说明](#5-node-reference--节点说明)
6. [Configuration / 配置文件](#6-configuration--配置文件)
7. [Topic Interface / 话题接口](#7-topic-interface--话题接口)
8. [Directory Structure / 目录结构](#8-directory-structure--目录结构)
9. [Troubleshooting / 常见问题](#9-troubleshooting--常见问题)
10. [Development / 开发与贡献](#10-development--开发与贡献)
11. [MANUS Glove 使用指南](#11-manus-glove-使用指南)

---

## 1. Overview / 概述

### Supported Devices / 支持的设备

**手部输入 → Wuji Hand 输出:**
| 输入设备 | 说明 |
|---------|------|
| ✅ Apple Vision Pro | 手部追踪 |
| ✅ MANUS Glove | 数据手套 |
| ✅ 自定义设备 | 输出 `/hand_input` 话题即可 |

**机械臂输入 → Tianji Arm 输出:**
| 输入设备 | 说明 |
|---------|------|
| ✅ Apple Vision Pro | 腕部追踪 |
| ✅ HTC Vive Tracker | 外部追踪器 |
| ✅ 自定义设备 | 发布 TF 到 `left_wrist`/`right_wrist` 即可 |

### System Requirements / 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+

---

## 2. System Architecture / 系统架构

### Data Flow / 数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                     Input Devices / 输入设备                      │
├─────────────────┬─────────────────┬─────────────────────────────┤
│  Apple Vision   │   MANUS Glove   │   HTC Vive Tracker          │
│  Pro (AVP)      │                 │   (OpenVR)                  │
└────────┬────────┴────────┬────────┴─────────────┬───────────────┘
         │                 │                      │
         ▼                 ▼                      ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────────────┐
│   avp_input     │ │  manus_input    │ │     openvr_input        │
│ (手部+腕部)     │ │  (仅手部)       │ │     (仅腕部)            │
└────────┬────────┘ └────────┬────────┘ └────────────┬────────────┘
         │                   │                       │
         ▼                   ▼                       ▼
┌────────────────────────────────────────────────────────────────┐
│                    Standard Topic Interface                     │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐    │
│  │ /hand_input  │  │ /left_wrist  │  │ TF: world->chest   │    │
│  │ (手部关键点) │  │ /right_wrist │  │     world->*_wrist │    │
│  └──────┬───────┘  └──────┬───────┘  └─────────┬──────────┘    │
└─────────┼─────────────────┼────────────────────┼───────────────┘
          │                 │                    │
          ▼                 ▼                    ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────────────┐
│ wujihand_ik     │ │  tf_broadcaster │ │    static_tf (x4)       │
│ (手部重定向)    │ │  (AVP 动态 TF)  │ │  chest->*_chest         │
└────────┬────────┘ └────────┬────────┘ │  *_wrist->tianji_*      │
         │                   │          └────────────┬────────────┘
         ▼                   └───────────────────────┤
┌─────────────────┐                                  ▼
│   Wuji Hand     │                      ┌─────────────────────┐
│   (Hardware)    │                      │     TF Buffer       │
└─────────────────┘                      │   (ROS2 自动维护)   │
                                         └──────────┬──────────┘
                                                    │ lookup_transform()
                                                    ▼
                                         ┌─────────────────────┐
                                         │   tianji_output     │
                                         │   (查询 TF)         │
                                         └──────────┬──────────┘
                                                    │
                                                    ▼
                                         ┌─────────────────────┐
                                         │    Tianji Arm       │
                                         │    (Hardware)       │
                                         └─────────────────────┘
```

### Custom Input Device / 自定义输入设备

只需将您的输入节点输出维护为以下格式：

- **手部控制**: 发布 `/hand_input` 话题 (`Float32MultiArray`, MediaPipe 21点格式)
- **机械臂控制**: 发布 TF 到 `left_wrist` / `right_wrist` / `chest` 坐标系

### TF Tree / TF 树结构

```
world
├── head
├── chest (computed from head, -0.3m in Z / 由头部计算，Z轴下移0.3m)
│   ├── left_chest (static: Rx=-90°, Rz=90°, Tz=37mm)
│   └── right_chest (static: Rx=90°, Rz=90°, Tz=37mm)
├── left_wrist
│   └── tianji_left (static: Rz=180° Ry=-90° Rz=225°)
└── right_wrist
    └── tianji_right (static: Ry=270° Rz=225°)
```

**Key insight / 关键点**: `tianji_output_node` 查询 `left_chest -> tianji_left` 时，ROS2 TF 会自动计算完整的变换链。

---

## 3. Installation / 环境配置

### 3.1 Prerequisites / 前置条件

```bash
# Install ROS2 Humble (Ubuntu 22.04)
# 安装 ROS2 Humble - 可参考鱼香ROS一键安装
sudo apt install ros-humble-desktop

# Install ROS2 dependencies
# 安装 ROS2 依赖
sudo apt install ros-humble-ament-cmake ros-humble-rclpy ros-humble-std-msgs ros-humble-tf2-ros

# Install Python dependencies
# 安装 Python 依赖
pip install numpy scipy pyyaml
```

### 3.2 External Dependencies / 外部依赖

```bash
#pip3 install wujihandpy (Wuji Hand SDK)
# 安装Wuji SDK
pip3 install --user wujihandpy

# 2. Install avp_stream (Vision Pro streaming)
# 安装 Vision Pro 数据流
pip3 install --user avp-stream

# 3. Install wuji_retargeting (hand retargeting algorithm)
# 安装手部重定向算法
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/Wuji-Technology-Co-Ltd/wuji_retargeting.git
cd wuji_retargeting && pip install -e .
```

> ⚠️ **Important / 重要**: `--recurse-submodules` is required because the repo contains git submodules.
>

### 3.3 Build / 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash  # or setup.zsh
```

---

## 4. Quick Start / 快速开始

### 4.1 Launch Commands / 启动命令

#### 完整遥操作（手 + 机械臂）

```bash
# AVP 手部 + AVP 机械臂
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=avp arm_input:=avp

# Manus 手部 + Tracker 机械臂
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker

# AVP 手部 + Tracker 机械臂
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=avp arm_input:=tracker

# 带 RViz 可视化
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=avp arm_input:=avp enable_rviz:=true
```

#### 单侧遥操作（单手 + 单臂）

```bash
# 右侧：AVP 输入
ros2 launch wuji_teleop_bringup wuji_teleop_single.launch.py side:=right hand_input:=avp arm_input:=avp

# 左侧：Manus + Tracker
ros2 launch wuji_teleop_bringup wuji_teleop_single.launch.py side:=left hand_input:=manus arm_input:=tracker
```

#### 仅手部控制

```bash
# AVP 手部输入
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=avp

# Manus 手部输入
ros2 launch wuji_teleop_bringup wuji_teleop_hand.launch.py hand_input:=manus
```

#### 仅机械臂控制

```bash
# AVP 机械臂输入
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=avp

# Tracker 机械臂输入
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker

# 带 RViz 可视化
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker enable_rviz:=true
```

### 4.2 Launch Parameters / 启动参数

| Parameter | Default | Description / 说明 |
|-----------|---------|-------------------|
| `hand_input` | avp | 手部输入：`avp` 或 `manus` |
| `arm_input` | avp | 机械臂输入：`avp` 或 `tracker` |
| `side` | right | 单侧模式：`left` 或 `right` |
| `enable_rviz` | false | 启用 RViz 可视化 |
| `hand_config` | (default path) | 手部配置文件路径 |


## 5. Node Reference / 节点说明

| Node / 节点 | Package / 包 | Description / 说明 |
|------------|-------------|-------------------|
| `avp_input` | wujihand_input | Vision Pro 数据采集，发布手部和腕部数据 |
| `manus_input` | wujihand_input | Manus 手套数据采集，发布手部数据 |
| `tf_broadcaster` | wujihand_input | 将腕部/头部数据转换为 TF 变换 |
| `wujihand_retargeting` | wujihand_ik | 手部 IK 重定向，控制 Wuji 手 |
| `tianji_output_node` | tianji_output | 双臂 IK 解算和控制 |

### Node Arguments / 节点参数

#### tianji_output_node

```bash
ros2 run tianji_output tianji_output_node [OPTIONS]

Options:
  --robot-ip IP        Tianji robot IP (default: 192.168.1.190)

# Enable debug logging / 启用调试日志
ros2 run tianji_output tianji_output_node --ros-args --log-level debug
```

**Note / 说明**: 该节点只从 TF tree 查询 `left_chest -> tianji_left` 和 `right_chest -> tianji_right`，不发布任何 TF。

---

## 6. Configuration / 配置文件

### 6.1 Apple Vision Pro 配置

**文件位置:** `src/input_devices/avp_input/avp_input/config/avp_input.yaml`

```yaml
avp_ip: 192.168.2.13           # 修改为你的 Vision Pro IP 地址
publish_rate_hz: 200.0         # 发布频率
include_right_hand: true       # 启用右手
include_left_hand: true        # 启用左手
```

### 6.2 Wuji Hand 配置

**文件位置:** `src/wujihand_ik/wujihand_ik/config/wujihand_ik.yaml`

```yaml
# 手部序列号（设为 null 禁用该手）
right_hand_serial: "337A386F3233"
left_hand_serial: "337438793233"

# 输入模式
use_joint_input: false
```

**获取 Wuji Hand 序列号:**
```bash
lsusb -v -d 0483:2000 | grep iSerial
```

### 6.3 手部重定向配置 (Retarget Config)

重定向配置文件控制手部关键点到 Wuji Hand 关节角度的映射算法参数。

**文件位置:** `src/wujihand_ik/wujihand_ik/config/`

| 输入源 | 配置文件 | 说明 |
|--------|----------|------|
| AVP | `retarget_avp.yaml` | 左右手共用同一配置 |
| Manus | `retarget_manus_right.yaml` | 右手配置 (z旋转: -15°) |
| Manus | `retarget_manus_left.yaml` | 左手配置 (z旋转: +15°) |

> **Note / 说明**: Manus 手套由于左右手坐标系差异，需要分别使用不同的 `mediapipe_rotation.z` 参数。修改 Manus 重定向参数时，请同时修改 `retarget_manus_right.yaml` 和 `retarget_manus_left.yaml` 两个文件（除 `mediapipe_rotation.z` 外其他参数相同）。

**关键参数说明:**

```yaml
retarget:
  mediapipe_rotation:    # 输入数据坐标系旋转校正
    x: 0.0
    y: 0.0
    z: -15.0             # Manus 右手: -15°, 左手: +15°, AVP: 0°

  segment_scaling:       # 手指段长度缩放
    thumb:  [0.98, 0.95, 0.95]
    index:  [0.9, 0.95, 0.98]
    # ...

  lp_alpha: 0.2          # 低通滤波器系数 (越小越平滑)
```

### 6.4 Manus 手套配置

**文件位置:** `src/input_devices/manus_input/manus_input_py/manus_input_py/config/manus_input.yaml`

```yaml
# 启用哪只手
include_right_hand: true
include_left_hand: true

# 手套 ID 映射
left_glove_id: 0
right_glove_id: 1
```

### 6.5 HTC Vive Tracker 配置

**文件位置:** `src/input_devices/openvr_input/config/openvr_input.yaml`

```yaml
# Tracker 序列号（以下为默认配置）
tracker_serials:
  chest: "LHR-952F919D"        # 胸部 tracker
  right_wrist: "LHR-D8219256"  # 右腕 tracker
  left_wrist: "LHR-6662A330"   # 左腕 tracker
```

**获取 Tracker 序列号 (需先运行 SteamVR):**
```bash
python3 -c "import openvr; openvr.init(openvr.VRApplication_Other); vr=openvr.VRSystem(); [print(f'Tracker: {vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)}') for i in range(64) if vr.getTrackedDeviceClass(i)==openvr.TrackedDeviceClass_GenericTracker]; openvr.shutdown()"
```

### 6.6 Tianji 机械臂配置

**文件位置:** `src/output_devices/tianji_output/tianji_output/config/tianji_output.yaml`

```yaml
# 机械臂 IP 地址
robot_ip: "192.168.1.190"
```

---

## 7. Topic Interface / 话题接口

### 7.1 Published Topics / 发布的话题

| Topic | Type | Publisher | Description / 说明 |
|-------|------|-----------|-------------------|
| `/hand_input` | Float32MultiArray | avp_input, manus_input | MediaPipe 手部关键点 (126 值 = 双手各 21×3) |
| `/right_wrist` | Float32MultiArray | avp_input | 右腕变换矩阵 (16 值 = 4×4) |
| `/left_wrist` | Float32MultiArray | avp_input | 左腕变换矩阵 (16 值 = 4×4) |
| `/head_pose` | Float32MultiArray | avp_input | 头部变换矩阵 (16 值 = 4×4) |
| `/manus_glove_0` | ManusGlove | manus_data_publisher | 左手 Manus 原始数据 |
| `/manus_glove_1` | ManusGlove | manus_data_publisher | 右手 Manus 原始数据 |
| `/tf` | TFMessage | tf_broadcaster | TF 变换 |

### 7.2 Custom Input Source Interface / 自定义输入源接口

⚠️ **Important / 重要**: If developing a custom input source, follow this interface:

**Topic Name:** `/hand_input` (fixed / 固定)

**Message Type:** `std_msgs/Float32MultiArray`

**Data Format (MediaPipe 21-point):**
- Single hand: 63 values (21 keypoints × 3 coordinates)
- Dual hands: 126 values (right hand first, then left)

**Keypoint Order (21 points per hand / 每只手 21 个关键点):**
```
0: WRIST / 腕部
1-4: THUMB / 拇指 (CMC, MCP, IP, TIP)
5-8: INDEX / 食指 (MCP, PIP, DIP, TIP)
9-12: MIDDLE / 中指 (MCP, PIP, DIP, TIP)
13-16: RING / 无名指 (MCP, PIP, DIP, TIP)
17-20: PINKY / 小指 (MCP, PIP, DIP, TIP)
```

---

## 8. Directory Structure / 目录结构

```
wuji-hand-teleop-ros2/
├── README.md
├── src/
│   ├── wuji_teleop_bringup/              # 启动文件包（主入口）
│   │   └── launch/
│   │       ├── wuji_teleop.launch.py         # 完整遥操作（双手+双臂）
│   │       ├── wuji_teleop_single.launch.py  # 单侧遥操作（单手+单臂）
│   │       ├── wuji_teleop_hand.launch.py    # 仅手部控制
│   │       └── wuji_teleop_arm.launch.py     # 仅机械臂控制
│   │
│   ├── input_devices/                    # 输入设备包
│   │   ├── avp_input/                    # Apple Vision Pro 输入
│   │   │   ├── avp_input/avp_input_node.py
│   │   │   └── config/avp_input.yaml
│   │   │
│   │   ├── openvr_input/                 # HTC Vive Tracker 输入
│   │   │   ├── openvr_input/openvr_input_node.py
│   │   │   └── config/openvr_input.yaml
│   │   │
│   │   ├── manus_input/                  # Manus 手套输入
│   │   │   ├── manus_input_py/           # Python 节点
│   │   │   ├── manus_ros2/               # C++ 驱动
│   │   │   └── manus_ros2_msgs/          # 消息定义
│   │   │
│   │   └── common_input/                 # 通用工具
│   │       └── tf_broadcaster_node.py
│   │
│   ├── output_devices/                   # 输出设备包
│   │   └── tianji_output/                # 天机臂控制
│   │       ├── tianji_output_node.py
│   │       └── cartesian_controller.py
│   │
│   ├── wujihand_ik/                      # Wuji Hand 控制
│   │   ├── ik_node.py
│   │   └── config/wujihand_ik.yaml
│   │
│   └── wujihand_urdf/                    # URDF 模型
│       ├── wujihand_left.urdf
│       └── wujihand_right.urdf
```

---

## 9. Troubleshooting / 常见问题

| Problem / 问题 | Solution / 解决方案 |
|----------------|---------------------|
| Cannot connect to Vision Pro | 1. 确保设备在同一局域网<br>2. 检查 `avp_ip` 配置<br>3. Vision Pro 上运行 avp_stream 应用 |
| Hand serial not found | 运行 `lsusb -v -d 0483:2000 \| grep iSerial` |
| Robot connection failed | 1. 检查机器人已上电<br>2. 确认 IP 地址正确 (ping 测试)<br>3. 检查网络连接 |
| TF tree incomplete | 确保运行了 `tf_broadcaster` 节点 |
| `ImportError: wuji_retargeting` | 从 GitHub 源码安装: `pip install -e .` |
| `ImportError: wujihandpy` | `pip install wujihandpy` |
| Package not found | 1. `colcon build`<br>2. `source install/setup.bash` |

**Enable Debug Logging / 启用调试日志:**
```bash
ros2 run tianji_output tianji_output_node --ros-args --log-level debug
```

---

## 10. Development / 开发与贡献

### 10.1 Adding New Input Sources / 添加新输入源

1. Create new node in `wujihand_input/wujihand_input/`
2. Implement the same topic interface (`/hand_input`, `/right_wrist`, etc.)
3. Add entry point in `setup.py`
4. Update config files

### 10.2 Related Projects / 相关项目

- **[wuji_retargeting](https://github.com/Wuji-Technology-Co-Ltd/wuji_retargeting)** - 手部重定向算法库
- **[wujihandpy](https://pypi.org/project/wujihandpy/)** - Wuji 手控制 SDK
- **[avp-stream](https://pypi.org/project/avp-stream/)** - Apple Vision Pro 数据流库

### 10.3 License / 许可证

MIT License

### 10.4 Maintainer / 维护者

Wuji Tech (support@wuji.tech)

---

## 11. MANUS Glove 使用指南

### 11.1 安装 MANUS SDK

```bash
# 下载 MANUS SDK Python (从 MANUS 官网获取)
# 安装 SDK
cd /path/to/MANUS_SDK_Python_3.1.1-Beta
pip install .
```

### 11.2 运行 MANUS 节点

```bash
# 确保手套已通过蓝牙配对并开机

# 构建（首次或更新后）
cd ~/ros2_ws/wuji-hand-teleop-ros2
colcon build --packages-select manus_ros2
source install/setup.bash

# 运行节点
ros2 run manus_ros2 manus_driver

# 或使用 launch 文件
ros2 launch manus_ros2 manus_driver.launch.py
```

### 11.3 发布的话题 (manus_ros2)

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/manus/left_hand/skeleton` | `geometry_msgs/PoseArray` | 200Hz | 左手骨骼位置+旋转 |
| `/manus/right_hand/skeleton` | `geometry_msgs/PoseArray` | 200Hz | 右手骨骼位置+旋转 |
| `/manus/left_hand/joint_states` | `sensor_msgs/JointState` | 200Hz | 左手关节角度 |
| `/manus/right_hand/joint_states` | `sensor_msgs/JointState` | 200Hz | 右手关节角度 |

### 11.4 数据格式

#### Skeleton 数据 (`geometry_msgs/PoseArray`)

每个 pose 代表一个关节节点：

```python
poses[i].position.x, y, z        # 3D 位置（米）
poses[i].orientation.w, x, y, z  # 四元数旋转
```

**节点顺序：**
| Index | Node | Description |
|-------|------|-------------|
| 0 | hand_metacarpal | 手腕/手掌根部 |
| 22-25 | thumb_* | 拇指 (metacarpal → tip) |
| 2-6 | index_* | 食指 |
| 7-11 | middle_* | 中指 |
| 12-16 | ring_* | 无名指 |
| 17-21 | pinky_* | 小指 |

#### Joint States 数据 (`sensor_msgs/JointState`)

每只手 20 个关节角度值（归一化 0.0 ~ 1.0）：

```
name[0-3]:   thumb_mcp_spread, thumb_mcp_stretch, thumb_pip_stretch, thumb_dip_stretch
name[4-7]:   index_mcp_spread, index_mcp_stretch, index_pip_stretch, index_dip_stretch
name[8-11]:  middle_*
name[12-15]: ring_*
name[16-19]: pinky_*
```

### 11.5 数据处理说明

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  MANUS 手套      │ ──► │  MANUS SDK      │ ──► │  manus_ros2     │
│  (硬件传感器)    │     │  (算法处理)      │     │  (ROS2 话题)    │
│  - IMU          │     │  - 传感器融合    │     │  - PoseArray    │
│  - 弯曲传感器   │     │  - 手部模型      │     │  - JointState   │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

**注意**: 所有输出数据都经过 MANUS SDK 内部算法处理，不是原始传感器值。

### 11.6 查看数据

```bash
# 查看话题列表
ros2 topic list | grep manus

# 查看骨骼数据
ros2 topic echo /manus/left_hand/skeleton

# 查看关节角度
ros2 topic echo /manus/left_hand/joint_states

# 查看发布频率
ros2 topic hz /manus/left_hand/skeleton
```

---

Issues and PRs welcome at: https://github.com/Wuji-Technology-Co-Ltd/wuji_teleop
