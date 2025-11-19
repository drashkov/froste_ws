Here is the definitive **Technical Reference & Deployment Guide** for the "Frost-E" perception stack.

This document captures the exact configuration, build "hacks," and runtime parameters required to get the **Luxonis OAK-D Pro** and **NVIDIA Isaac ROS Nvblox** running on an 8GB Jetson Orin Nano.

Save this file as `SETUP.md` in the root of your repository.

-----

# Project Frost-E: Perception Stack Setup Guide

**Classification:** Technical Blueprint
**Target Hardware:** NVIDIA Jetson Orin Nano (8GB)
**OS:** JetPack 6.0 (Ubuntu 22.04 Host)
**Sensors:** Luxonis OAK-D Pro (USB 3.0)
**Software:** ROS 2 Humble (Containerized)

-----

## I. Host System Architecture

The system relies on a persistent Docker container with specific hardware passthroughs to handle the OAK-D's USB re-enumeration and the 8GB RAM constraint.

### 1\. The "Master Key" Script (`run_froste_dev.sh`)

**Location:** `~/run_froste_dev.sh` (Host Home Directory)
**Purpose:** Creates/Restarts the container with `cgroup` rules for USB access and maps the workspace.

```bash
#!/bin/bash
# Project: Sasquatch Robotics "Frost-E"
# Purpose: Persistent Development Container with X11 & Hardware Access

# --- Configuration ---
IMAGE_NAME="nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c"
CONTAINER_NAME="froste_dev_container"
HOST_WS_DIR="$HOME/froste_ws"
# --- End Configuration ---

# Safety Check
if [ -z "$CONTAINER_NAME" ]; then
    echo "ERROR: CONTAINER_NAME is not set."
    exit 1
fi

# 1. Enable X-Forwarding (Safe Mode)
if [ -n "$DISPLAY" ]; then
    xhost +local:docker > /dev/null 2>&1
else
    echo "WARNING: \$DISPLAY is not set. RVIZ will not work."
fi

# 2. Check if container exists
if [ -z "$(docker ps -a -q -f "name=^/${CONTAINER_NAME}$")" ]; then
    echo "--- Creating persistent container... ---"
    
    docker run -it -d \
        --network=host \
        --privileged \
        --name ${CONTAINER_NAME} \
        \
        # HARDWARE: UART (Sub-controller) & USB Bus (Camera Re-boot)
        --device=/dev/THS1 \
        --device-cgroup-rule='c 189:* rmw' \
        -v /dev/bus/usb:/dev/bus/usb \
        -v /dev/*:/dev/* \
        \
        # ENVIRONMENT: Workspace & Graphics
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev/src/froste_ws \
        -v ${HOST_WS_DIR}:/workspaces/isaac_ros-dev/src/froste_ws \
        -v /etc/localtime:/etc/localtime:ro \
        \
        ${IMAGE_NAME} \
        /bin/bash

    echo "--- Setup complete. Entering... ---"
    docker exec -it -w /workspaces/isaac_ros-dev/src/froste_ws ${CONTAINER_NAME} /bin/bash

else
    echo "--- Found existing container. Restarting... ---"
    docker start ${CONTAINER_NAME} > /dev/null
    docker exec -it \
      -w /workspaces/isaac_ros-dev/src/froste_ws \
      -e DISPLAY=$DISPLAY \
      ${CONTAINER_NAME} /bin/bash
fi
```

-----

## II. Build Strategy (The "Split Build")

We must build from source to fix a header mismatch in `cv_bridge` (needed by Nvblox).

### 1\. Dependency Injection

Run these **once** inside the new container:

```bash
sudo apt-get update && sudo apt-get install -y \
    git-lfs \
    ros-humble-depthai-ros \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv
```

### 2\. The Header Fix (Symlink Hack)

The NVIDIA source expects `.hpp`, but ROS 2 provides `.h`.

```bash
cd /workspaces/isaac_ros-dev/src/froste_ws/src/vision_opencv/cv_bridge/include/cv_bridge
ln -s cv_bridge.h cv_bridge.hpp
cd /workspaces/isaac_ros-dev/src/froste_ws
```

### 3\. The Build Command

We use a specific sequence to force the local `cv_bridge` to override the system one, while ignoring broken example packages.

```bash
# 1. Build the Bridge First
colcon build --symlink-install \
    --packages-select cv_bridge \
    --allow-overriding cv_bridge

source install/setup.bash

# 2. Build the Stack (Ignoring 'test_data' and 'examples' to prevent download errors)
colcon build --symlink-install \
    --packages-up-to isaac_ros_nvblox \
    --packages-ignore nvblox_test_data nvblox_examples_bringup isaac_ros_common \
    --allow-overriding cv_bridge
```

-----

## III. Runtime Configuration

These files are located in the custom `froste_bringup` package.

### 1\. `nvblox.yaml` (Critical Tuning)

**Key Fixes:**

  * **Memory:** `max_block_count: 10000` prevents 8GB Nano from crashing (OOM).
  * **Lag:** `clear_map_outside_radius_rate_hz: 0.0` prevents CPU freeze loop.
  * **Frame:** `global_frame: map` ensures static mapping.

<!-- end list -->

```yaml
/**:
  ros__parameters:
    # --- CRITICAL: Force Static Global Frame ---
    global_frame: map
    
    # --- CRITICAL: Memory Optimization for 8GB Nano ---
    max_block_count: 10000
    
    # --- CRITICAL: Disable CPU-Heavy Clearing ---
    clear_map_outside_radius_rate_hz: 0.0
    map_clearing_radius_m: 5.0

    # Mapping Parameters
    voxel_size: 0.05
    esdf_update_rate_hz: 1.0
    mesh_update_rate_hz: 2.0
    tsdf_integrator_max_integration_distance_m: 5.0
    tsdf_integrator_truncation_distance_m: 0.2
    tsdf_decay_factor: 0.95
    decay_tsdf_rate_hz: 0.0

    # Inputs
    use_depth: true
    use_lidar: false
    use_color: false
    
    # Auto-Save
    after_shutdown_map_save_path: "/workspaces/isaac_ros-dev/src/froste_ws/maps/slushe_map"
```

### 2\. `nvblox.launch.py` (Topic Patch)

**Key Fix:** Remapping the internal `camera_0` topics to the actual OAK-D topics.

```python
        remappings=[
            # FIX: Connect OAK-D topics to Nvblox internal names
            ('camera_0/depth/image', '/oak/stereo/image_raw'),
            ('camera_0/depth/camera_info', '/oak/stereo/camera_info'),
            ('pose', '/visual_slam/tracking/vo_pose'),
        ]
```

### 3\. `m1_mapping.launch.py` (The "Nuclear" TF Tree)

**Key Fix:** Hardcoding the TF tree (`map` -\> `odom` -\> `base_link`) using static publishers because Visual SLAM was failing to publish them reliably.

```python
    # TF 1: Camera -> Robot
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak-d-base-frame']
    )

    # TF 2: Map -> Odom (Global Anchor)
    map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # TF 3: Odom -> Base_Link (The Pin)
    # Forces the robot to exist at (0,0,0) so mapping works even if VSLAM fails
    odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
```

-----

## IV. Troubleshooting Reference

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| **RVIZ Crash (Exit Code -6)** | Qt Plugin conflict in Docker. | Run `unset QT_PLUGIN_PATH` before launch. |
| **"No map to save"** | Nvblox not receiving data. | Check topics. Ensure remapping targets `camera_0/depth/image`. |
| **System Freeze / Lag** | Map clearing loop consuming CPU. | Set `clear_map_outside_radius_rate_hz: 0.0` in YAML. |
| **Crash (Exit Code 99)** | Out of Memory (CUDA Error 2). | Set `max_block_count: 10000` in YAML. |
| **"Lookup transform failed"** | Broken TF Tree. | Ensure all 3 static TF nodes are running in launch file. |

[ROS2 Tutorial: Humble Nvidia Jetson Nano and RPLidar Interfacing for Autonomous Robotics](https://www.youtube.com/watch?v=IRtIurWqNLk)
This video covers fundamental ROS 2 setup on Jetson Nano, reinforcing the environment configuration steps discussed.

http://googleusercontent.com/youtube_content/57

