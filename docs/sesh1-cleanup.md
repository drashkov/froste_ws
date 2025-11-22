FrostE Phase 1: Transitioning UGV Beast to Vision-Only AutonomyObjective: Achieve "Room-Scale Autonomy" using only the OAK-D Pro camera.Success Criteria: The robot can map a room, save the map, and autonomously navigate to a goal point selected in RViz, without using LiDAR.Source Codebase: ugv_ws1. Repository Cleanup & Structure AnalysisWe will perform "surgical" edits to the existing ugv_ws to create our froste variants. We will not delete the original files but create copies with the froste_ prefix to maintain a fallback.Target Package Modification:Keep: ugv_base_node (Keep as is. It handles the critical Odom+IMU fusion).Modify: ugv_bringup, ugv_vision, ugv_slam, ugv_nav.Ignore: ugv_chat_ai, ugv_web_app (These are bloat for this milestone).2. Step 1: The Driver & Camera (The "Body")We need a launch file that brings up the robot without the LiDAR and with the OAK-D Pro active stereo enabled.2.1 Configure OAK-D Pro for "White Wall" MappingThe default oak_d_lite.launch.py is insufficient because the Lite lacks the IR projector. The Pro needs the projector enabled to see depth on featureless indoor walls.Action: Create src/ugv_main/ugv_vision/launch/froste_camera.launch.pyThis should wrap the standard depthai_ros_driver but inject specific parameters.Python# Key Parameter Changes for OAK-D Pro
parameters =
2.2 Create the FrostE BringupWe must replace bringup_imu_ekf.launch.py. The original launches ldlidar. If we leave that running, it will publish empty or error-filled transforms/scans that will confuse the Nav stack.Action: Create src/ugv_main/ugv_bringup/launch/froste_bringup.launch.pyInclude: ugv_bringup/launch/driver.launch.py (Motor Control).Include: ugv_base_node/launch/base_node.launch.py (Odometry Fusion).Include: ugv_vision/launch/froste_camera.launch.py (The new camera launch).REMOVE: Any reference to ldlidar.REMOVE: robot_localization (EKF) if you plan to let RTAB-Map handle odom->map. However, ugv_ws uses EKF to fuse Wheel+IMU into /odom. Keep the EKF, but ensure it doesn't wait for LiDAR.3. Step 2: RTAB-Map Configuration (The "Mapper")We need to configure RTAB-Map to act as our SLAM engine and our LiDAR emulator. It must ingest depth images and output a 2D Occupancy Grid.Action: Create src/ugv_main/ugv_slam/launch/froste_mapping.launch.pyCopy rtabmap_rgbd.launch.py and modify the arguments passed to the rtabmap node.Critical Parameter Overrides:subscribe_scan: false (We have no LiDAR).subscribe_depth: true.frame_id: base_footprint (Ensure this matches the URDF).map_frame_id: map.visual_odometry: false (We trust the ugv_base_node EKF odometry more than pure visual odometry for this chassis).odom_topic: /odom (From ugv_base_node).RTAB-Map Internal Args (rtabmap_args):These are the "Magic Numbers" for LiDAR-less mapping.--Grid/FromDepth true: The most important setting. Tells RTAB-Map to raytrace the 3D depth cloud to create the 2D map.--Reg/Force3DoF true: Forces the graph optimizer to ignore Roll/Pitch/Z. Keeps the map flat even if the robot hits a bump.--Grid/RayTracing true: Clears empty space. Essential for navigation; otherwise, dynamic obstacles (like people) become permanent walls.Verification (RVIZ):Launch froste_mapping.launch.py.Open RViz.Add display -> Map -> Topic /map (or /grid_map).Test: Drive the robot toward a wall. Does the map show a black line appearing where the wall is? If yes, the depth-to-scan projection is working.4. Step 3: Map Saving (The "Artifact")The ugv_ws repo uses a script save_2d_gmapping_map.sh. We will adapt this.Action: Create save_froste_map.sh in the root workspace.Bash#!/bin/bash
# Save the map using the standard Nav2 map_saver
ros2 run nav2_map_server map_saver_cli -f ~/froste_ws/src/ugv_main/ugv_nav/maps/my_room
Note: Run this while the mapping node is still running. It grabs the current /map topic and saves my_room.pgm and my_room.yaml.5. Step 4: Navigation (The "Navigator")This is the hardest part. The default nav2_params.yaml in ugv_nav is tuned for LiDAR. A LiDAR ObstacleLayer expects a LaserScan. If we give it nothing, the robot will be blind to new obstacles.Action: Create src/ugv_main/ugv_nav/param/froste_nav2_params.yaml5.1 Modifying the CostmapsWe need to switch from a 2D Laser Layer to a 3D Voxel Layer (or a Depth Observation Layer).Local Costmap Config:YAMLlocal_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        observation_sources: pointcloud
        pointcloud:
          topic: /camera/points  # CHECK THIS TOPIC NAME via 'ros2 topic list'
          max_obstacle_height: 2.0
          min_obstacle_height: 0.1
          clearing: True
          marking: True
          data_type: "PointCloud2"
Why? This allows the robot to "see" chairs and tables using the depth camera for obstacle avoidance, even if they aren't on the static map.5.2 Switching LocalizationThe ugv_nav launch likely starts amcl (Adaptive Monte Carlo Localization). AMCL requires a LaserScan. It will not work for us. We must disable AMCL and use RTAB-Map in localization mode.Action: Create src/ugv_main/ugv_nav/launch/froste_nav.launch.pyDo not launch amcl.Launch rtabmap_rgbd.launch.py (from Step 2) but add argument localization:=true.This puts RTAB-Map in read-only mode. It will publish the map -> odom transform based on visual loop closures.Launch nav2_bringup (lifecycle manager, planner, controller) pointing to froste_nav2_params.yaml.6. Execution Sequence (Milestone 1 Complete)Session A: Mappingros2 launch froste_bringup froste_bringup.launch.py (Starts Driver + OAK-D Pro)ros2 launch froste_bringup froste_mapping.launch.py (Starts RTAB-Map in Mapping Mode)Drive: Use the joystick/keyboard to paint the room. Perform 360 spins at corners.Save: Run ./save_froste_map.sh.Shutdown: Ctrl-C everything.Session B: Navigationros2 launch froste_bringup froste_bringup.launch.pyros2 launch froste_bringup froste_nav.launch.py (Starts RTAB-Map Localization + Nav2)Wake Up: The robot may not know where it is initially. Drive forward slightly until RTAB-Map recognizes a visual feature. The map should "snap" into place in RViz.Command: Use "2D Nav Goal" in RViz. Click a point on the map.Observe: The robot should plan a path (Global Planner) and drive to it (Local Planner), avoiding obstacles seen by the depth camera.



A second opinion implementaiton plan:
# FrostE Phase 1: Development Plan (Vision-Only Transition)

To answer your immediate question: **No, you do not need Cartographer or Gmapping.** These are strictly LiDAR-based 2D SLAM algorithms. Since you are removing the LiDAR to rely on the OAK-D Pro, these packages are obsolete for your configuration. You can safely ignore or remove them from your workspace to save compile time.

Below is the technical execution plan to transform the `ugv_ws` codebase for **Milestone 1: Room-Scale Vision Autonomy**.

---

## Phase 1: Codebase Transformation Strategy

**Goal:** Modify `ugv_ws` to map and navigate using *only* the OAK-D Pro and RTAB-Map.
**Core Constraint:** Do not write from scratch. Modify existing launch files and params.

### Step 1: Hardware Abstraction (The "Body")
We must modify the startup sequence to prevent the LiDAR driver from launching (which would spam errors) and ensure the OAK-D Pro is in "Active Stereo" mode (essential for white walls).

**1.1 Modify `ugv_vision` for Active Stereo**
The default `oak_d_lite.launch.py` is insufficient. You need a specific configuration for the OAK-D **Pro** to turn on the IR projector.
*   **File:** Create `src/ugv_main/ugv_vision/launch/oak_d_pro.launch.py` (copy the lite version).
*   **Edit:** In the `declare_launchable_description`, ensure you pass parameters to the `depthai_ros_driver` node. You specifically need:
    *   `"dotProjectorBrightness": 800` (mA) – This projects the dots for depth on textureless walls.
    *   `"floodLightBrightness": 0` – Turn off floodlight to save power/reduce glare.

**1.2 Create FrostE Bringup**
*   **File:** Create `src/ugv_main/ugv_bringup/launch/froste_bringup.launch.py` (copy `bringup_imu_ekf.launch.py`).
*   **Edits:**
    *   **Keep:** `ugv_bringup.driver.launch.py` (Motor control).
    *   **Keep:** `ugv_base_node.base_node.launch.py` (Odom publishing).
    *   **Keep:** `robot_localization` (EKF). *Note: Ensure the EKF config does not wait for the LiDAR topic.*
    *   **Remove:** `ldlidar` launch inclusion.
    *   **Replace:** `ugv_vision.oak_d_lite.launch.py` with your new `oak_d_pro.launch.py`.

### Step 2: Perception & Mapping (The "Eyes")
We need to configure RTAB-Map to act as both our SLAM system and our "fake laser scanner" by projecting 3D depth data into 2D.

**2.1 Configure RTAB-Map**
*   **File:** Modify `src/ugv_main/ugv_slam/launch/rtabmap_rgbd.launch.py`.
*   **Critical Parameter Changes:**
    *   `subscribe_scan`: Set to `false` (No LiDAR).
    *   `subscribe_depth`: Set to `true`.
    *   `visual_odometry`: Set to `false` (We will trust the wheel+IMU odometry from `ugv_base_node` as the guess, which is more stable on the Beast).
    *   `odom_topic`: Set to `/odom` (or `/odometry/filtered` if using EKF).
*   **Add RTAB-Map Arguments:** You must add these arguments to the `rtabmap` node to generate a valid 2D map without a laser:
    *   `--Grid/FromDepth true`: Creates the 2D occupancy grid from the camera depth.
    *   `--Reg/Force3DoF true`: Forces the map to be 2D (x, y, yaw only), preventing the map from tilting if the robot hits a bump.
    *   `--Grid/RayTracing true`: Clears empty space in the map so the robot knows where it can drive.

### Step 3: Navigation Stack (The "Brain")
The existing navigation stack is likely tuned for a 2D LaserScan. If we feed it nothing, it will think the world is empty or error out. We must switch it to use the 3D Voxel layer.

**3.1 Modify Costmap Parameters**
*   **File:** Create `src/ugv_main/ugv_nav/param/froste_nav2.yaml` (copy `nav2_params.yaml`).
*   **Edit `local_costmap` and `global_costmap` sections:**
    *   Look for `plugins: ["obstacle_layer", "inflation_layer"]`.
    *   Change `obstacle_layer` to use `nav2_costmap_2d::VoxelLayer` instead of `ObstacleLayer`.
    *   **Observation Sources:** Change the source from `scan` to `pointcloud`.
        *   Topic: `/oak/points` (Check `ros2 topic list` when camera is running to confirm exact name).
        *   Data Type: `PointCloud2`.
        *   Clearing: `true` (Essential for dynamic obstacles).

**3.2 Switch Localization Source**
Standard `ugv_nav` likely launches `amcl` (Adaptive Monte Carlo Localization). AMCL **requires** a LiDAR scan. It will not work with a depth camera point cloud.
*   **File:** Create `src/ugv_main/ugv_nav/launch/froste_nav.launch.py`.
*   **Action:**
    *   **Remove:** Any include related to `amcl`.
    *   **Add:** An include for your `rtabmap_rgbd.launch.py` but with argument `localization:=true`.
    *   **Reasoning:** RTAB-Map in localization mode replaces AMCL. It looks at the camera image, finds features it recognizes from the saved database, and calculates the `map -> odom` transform just like AMCL would.

### Step 4: Verification & Execution (The "Workflow")

**Milestone 1.1: The Mapping Run**
1.  Launch: `ros2 launch ugv_bringup froste_bringup.launch.py`
2.  Launch: `ros2 launch ugv_slam rtabmap_rgbd.launch.py`
3.  **Action:** Drive the robot manually. Rotate 360 degrees at key intersections to "teach" the camera the view.
4.  **Verify:** In RViz, do you see a 2D grid map appearing? Is the depth cloud overlaying it correctly?
5.  **Save:** Run the `save_2d_gmapping_map.sh` script (it uses `nav2_map_server`, so it works for RTAB-Map too) *before* killing the nodes.

**Milestone 1.2: The Navigation Run**
1.  Kill all previous nodes.
2.  Launch: `ros2 launch ugv_bringup froste_bringup.launch.py`
3.  Launch: `ros2 launch ugv_nav froste_nav.launch.py` (This starts RTAB-Map in localization mode + Nav2).
4.  **Wake Up:** The robot might start "lost". Drive it forward/rotate slightly until RTAB-Map matches the view. You will see the map "snap" into position in RViz.
5.  **Execute:** Use the "2D Nav Goal" button in RViz to click a point on the map. The robot should plan a path and drive there.