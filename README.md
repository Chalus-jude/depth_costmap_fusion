# r1_mapless_navigation

Depth-only, mapless autonomous navigation for the R1 off-road AMR (ROS 2 Humble).

The robot navigates with **no lidar and no pre-built map**. A custom C++ fusion node turns the stereo depth image into a clean, IMU tilt-corrected point cloud. That cloud feeds a Spatio-Temporal Voxel Layer (STVL) in Nav2's rolling-window costmaps, and a TEB local planner drives the robot.

---

## Table of contents

1. [Architecture](#1-architecture)
2. [Package layout](#2-package-layout)
3. [Prerequisites](#3-prerequisites)
4. [Build](#4-build)
5. [Run](#5-run)
6. [Launch arguments](#6-launch-arguments)
7. [Fusion node](#7-fusion-node)
8. [Frames and TF](#8-frames-and-tf)
9. [Nav2 configuration](#9-nav2-configuration)
10. [RViz](#10-rviz)
11. [Deploying on another robot (OAK-D Lite)](#11-deploying-on-another-robot-oak-d-lite)
12. [Simulation (Gazebo)](#12-simulation-gazebo)
13. [Troubleshooting](#13-troubleshooting)
14. [Design decisions](#14-design-decisions)

---

## 1. Architecture

```
 Depth camera ──► depth image + camera_info ─┐
                                              ▼
 BNO055 IMU ────► /imu/data ───────────► depth_fusion_node
                                              │  /filtered_points (PointCloud2, base_link)
                                              ▼
                              STVL layer (local + global costmap)
                                              │
 wheel odom ─► EKF ─► odom->base_link TF      ▼
                              Nav2: planner_server (NavFn) + controller_server (TEB)
                                              │
                                              ▼
                              velocity_smoother ─► /cmd_vel ─► base
```

What each part does:

| Stage | Component | Role |
|---|---|---|
| Perception | `depth_fusion_node` (this package) | Deprojects depth to 3D, clips range, tilt-corrects with the IMU, publishes the cloud in `base_link` |
| Costmap | STVL (`spatio_temporal_voxel_layer`) | Voxelizes the cloud with temporal decay and flattens it into 2D costmaps |
| Planning | NavFn | Global path over the rolling global costmap |
| Control | TEB local planner (`teb_local_planner`) | Local trajectory optimization over the local costmap |
| Localization | `robot_localization` EKF (external) | Provides `odom -> base_link` from wheel odometry and IMU yaw |

---

## 2. Package layout

```
r1_mapless_navigation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── nav2_params_mapless.yaml     # all Nav2 params, STVL, TEB
│   └── r1_mapless.rviz              # RViz config with all relevant displays
├── launch/
│   └── r1_navigation_depth.launch.py   # single launch file: fusion + Nav2 + RViz
└── src/
    └── fusion_node.cpp              # depth_fusion_node
```

`include/r1_mapless_navigation/` can be added later if the fusion node is split into headers.

---

## 3. Prerequisites

**Platform:** Ubuntu 22.04 with ROS 2 Humble.

**ROS packages**

```bash
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-robot-localization \
  ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-rviz2
```

**TEB local planner:** there is no apt package for Humble, so it is built from source (workspace `~/teb_ws`), together with its dependencies such as `costmap_converter`. Overlay that workspace when building and running:

```bash
source /opt/ros/humble/setup.bash
source ~/teb_ws/install/setup.bash
source ~/r1_ws/install/setup.bash
```

**Things that must already be running on the robot** (not started by this package):

| Needed | Provided by | Notes |
|---|---|---|
| Depth image + `camera_info` | Camera driver (RealSense or OAK-D Lite) | See [Launch arguments](#6-launch-arguments) for topic names |
| `/imu/data` | `bno055_i2c_node` | About 33–50 Hz, orientation quaternion |
| `odom -> base_link` TF | `ekf_filter_node` | Fuses `/r1a006/wheel_odom` and IMU yaw |
| URDF TF (`base_link -> camera_link`, plus the camera's internal frames) | `robot_state_publisher` + camera driver | Must include the camera optical frame |

---

## 4. Build

```bash
cd ~/r1_ws
colcon build --packages-select r1_mapless_navigation --symlink-install
source install/setup.bash
```

After removing or renaming an executable, do a clean rebuild of the package:

```bash
rm -rf build/r1_mapless_navigation install/r1_mapless_navigation
colcon build --packages-select r1_mapless_navigation --symlink-install
```

---

## 5. Run

Start the camera driver, IMU driver and EKF first, then:

```bash
# Real robot
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py

# Headless (e.g. over SSH): no RViz
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py rviz:=false

# Gazebo / any sim clock
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py use_sim_time:=true
```

The single launch file starts:

1. `depth_fusion_node` (the fusion node)
2. Nav2's `navigation_launch.py` (controller, smoother, planner, behaviors, BT navigator, waypoint follower, velocity smoother, lifecycle manager). There is no map server and no AMCL.
3. RViz2 with `config/r1_mapless.rviz` (optional)

Send goals from RViz with the **2D Goal Pose** tool, with the Fixed Frame set to `odom`.

**Healthy startup:** the lifecycle manager ends with all nodes active. If `planner_server` hangs on a missing `map` frame, see [Troubleshooting](#13-troubleshooting).

---

## 6. Launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use `/clock` for all nodes |
| `autostart` | `true` | Auto-activate the Nav2 lifecycle nodes |
| `params_file` | `config/nav2_params_mapless.yaml` | Nav2 parameter file |
| `rviz` | `true` | Start RViz2 |
| `rviz_config` | `config/r1_mapless.rviz` | RViz config file |
| `depth_topic` | `/camera/camera/depth/image_rect_raw` | Depth image (RealSense default) |
| `camera_info_topic` | `/camera/camera/depth/camera_info` | Depth camera intrinsics |
| `imu_topic` | `/imu/data` | IMU with fused orientation |
| `base_frame` | `base_link` | Output frame of the fusion node |

Example for the OAK-D Lite robot (adjust to your driver's topic names):

```bash
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py \
  depth_topic:=/oak/stereo/image_raw \
  camera_info_topic:=/oak/stereo/camera_info
```

---

## 7. Fusion node

**Node:** `depth_fusion_node` (executable `fusion_node`, class `DepthFusionNode`, C++/rclcpp)

**Inputs**

| Topic (param) | Type | Notes |
|---|---|---|
| `depth_topic` | `sensor_msgs/Image` | 16UC1 depth, e.g. 848x480 on the RealSense |
| `camera_info_topic` | `sensor_msgs/CameraInfo` | Intrinsics for deprojection |
| `imu_topic` | `sensor_msgs/Imu` | Orientation quaternion used for tilt correction |

Depth and `camera_info` are synchronized with an `ApproximateTime` policy.

**Output:** `/filtered_points` (`sensor_msgs/PointCloud2`) in `base_link`. It is an organized cloud (height/width equal to the image size) with NaN for invalid pixels, selectable between flat (dense) and organized output with the `organized_output` parameter.

**Processing**

1. Deproject each depth pixel with pinhole math into 3D points in the camera optical frame.
2. Clip by minimum and maximum range (`min_range_m`, `max_range_m`).
3. Look up the static `camera_depth_optical_frame -> base_link` extrinsic through TF.
4. Take the IMU orientation at the cloud's exact timestamp. Interpolate with **SLERP between the two IMU samples that bracket the cloud stamp**, keep only the **roll and pitch** component (yaw is discarded because the EKF already fuses it) and compose it with the static extrinsic.
5. Apply the corrected transform to the points in code and publish the cloud already in `base_link`.

**Frame dropping:** if no pair of IMU samples brackets the cloud timestamp, the frame is dropped with the warning `no bracketing IMU samples ... -- dropping frame`. Stale or extrapolated orientation is never used.

**Why tilt is handled here:** the EKF only fuses yaw and yaw rate from the IMU, so the `odom -> base_link` TF is flat regardless of real robot tilt. The EKF config, URDF and TF tree are not modified. The fusion node does the tilt compensation itself.

**Not implemented (by design for now):** RGB fusion and confidence masking. The pipeline is depth-only.

---

## 8. Frames and TF

```
odom ──► base_link ──► camera_link ──► camera_depth_frame ──► camera_depth_optical_frame
  ▲          │
  │          ├──► laser  (unused here)
 EKF         └──► imu
```

| Transform | Source |
|---|---|
| `odom -> base_link` | `ekf_filter_node` |
| `base_link -> camera_link` | URDF through `robot_state_publisher` |
| Camera internal frames | Camera driver |

Robot 1 (`r1_mini`) mounts: `base_link -> camera_link` at (0.15065, 0.0, 0.08505) m with no rotation, and `base_link -> imu` at (0.07215, 0.0, 0.0773) m with no rotation.

**There is no `map` frame.** Both costmaps use `odom` as the global frame with a rolling window. The `global_frame` of every Nav2 component must be `odom`.

---

## 9. Nav2 configuration

All parameters are in `config/nav2_params_mapless.yaml`.

**Mapless essentials**

| Parameter | Value |
|---|---|
| `bt_navigator.global_frame` | `odom` |
| `local_costmap.global_frame` | `odom` |
| `global_costmap.global_frame` | `odom` |
| `global_costmap.rolling_window` | `true` |
| `robot_base_frame` | `base_link` |
| `amcl`, `map_server` | not used (the `amcl` block can be deleted) |

**Costmap layers**

- STVL layer: `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`, observing `/filtered_points`
- Inflation layer
- `static_layer` must not be present

**STVL notes**

- Local `voxel_decay` is kept at **1.0 s** because the voxel costmap is very dynamic. Controller tuning adapts to that.
- `publish_voxel_map` is `false` by default. Set it to `true` only when debugging, then add a PointCloud2 display in RViz for the voxel grid topic.

**Controller (TEB)**

- Plugin: `teb_local_planner::TebLocalPlannerROS`
- Controller frequency: 10 Hz
- Homotopy class planning: disabled
- Footprint model: two circles
- Costmap converter plugin must be fully qualified, for example `costmap_converter::CostmapToPolygonsDBSMCCH`, or set to `""` to disable it
- `max_vel_x_backwards` must be greater than `penalty_epsilon`, otherwise TEB warns about a negative bound

**Planner:** `nav2_navfn_planner/NavfnPlanner` on the global rolling costmap.

---

## 10. RViz

`config/r1_mapless.rviz` has Fixed Frame `odom` and the camera following `base_link`. Displays:

| Display | Topic |
|---|---|
| TF | `/tf`, `/tf_static` |
| Filtered Points (fusion output) | `/filtered_points` |
| Global Costmap | `/global_costmap/costmap` |
| Local Costmap | `/local_costmap/costmap` |
| Robot Footprint | `/local_costmap/published_footprint` |
| Global Plan | `/plan` |
| TEB Local Plan | `/local_plan` |
| TEB Markers | `/teb_markers` |
| Odometry (EKF) | `/odometry/filtered` |

If a display stays empty, check the topic name with `ros2 topic list | grep -E "plan|teb"`.

---

## 11. Deploying on another robot (OAK-D Lite)

The fusion node and costmap code are camera-agnostic. Only the topics and the camera frame change.

1. Install and run the DepthAI ROS driver, attached to `camera_link` with a zero pose through its `parent_frame` parameter. The URDF mounts match robot 1.
2. Start the BNO055 driver (`bno055_i2c_node`, I2C bus 1, address 0x28) so `/imu/data` is publishing at about 50 Hz.
3. Launch with the OAK topic names:

```bash
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py \
  depth_topic:=<oak depth image topic> \
  camera_info_topic:=<oak camera_info topic>
```

4. Set the camera, IMU and EKF drivers to **auto-start at boot** (systemd units) so the robot is ready without manual steps.

Resolution and intrinsics are read from `camera_info`, so no code changes are needed (for example, the OAK depth is 1280x720 versus 848x480 on the RealSense).

---

## 12. Simulation (Gazebo)

The simulation files live in the `r1_simulations` package (robot xacros, Gazebo and Nav2 launch files, bridge config). To run this stack against the simulation:

1. Start Gazebo and the robot with the sim launch file.
2. Launch this package with the sim clock:

```bash
ros2 launch r1_mapless_navigation r1_navigation_depth.launch.py use_sim_time:=true
```

The depth camera must see a proper floor in the chosen world. The `small_city.world` was replaced for this reason.

---

## 13. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Invalid frame ID "map" passed to canTransform` | A `global_frame` is still `map` in the params file | Set `global_frame: odom` under `bt_navigator`, `global_costmap` (and `behavior_server` if present) |
| `planner_server` never activates | Same as above | Same as above |
| `no bracketing IMU samples ... dropping frame` | The IMU isn't publishing, uses a different clock, or its stamps lag the camera | Run `ros2 topic hz /imu/data`. Compare `header.stamp` with `date +%s`. A few drops at startup are normal. |
| Costmap empty in RViz | `/filtered_points` not publishing, or TF missing | `ros2 topic hz /filtered_points`, and check that `odom -> base_link` exists |
| Point cloud is vertical or tilted | Missing `base_link -> camera_link` TF | Confirm `robot_state_publisher` is running with the URDF |
| TEB: `max_vel_x_backwards <= penalty_epsilon` | Parameter conflict | Raise `max_vel_x_backwards` above `penalty_epsilon`, or set both to 0 |
| TEB: costmap converter plugin cannot be loaded | Short plugin name | Use `costmap_converter::CostmapToPolygonsDBSMCCH`, or `""` |
| Controller loop misses 10 Hz, robot creeps or wobbles | Controller or costmap too heavy for the dynamic voxel costmap | Tune TEB and local costmap parameters (see below) |

Useful checks:

```bash
ros2 topic hz /filtered_points
ros2 topic hz /imu/data
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 lifecycle get /controller_server
```

---

## 14. Design decisions

- **Depth-only pipeline.** RGB was dropped from the fusion node because depth and color are not pre-registered and no filtering step needs color yet.
- **Custom fusion node feeding STVL**, rather than reimplementing voxelization from scratch.
- **No lidar.** The costmap is built from camera data alone, even though a 2D lidar is available on the robot.
- **Tilt compensation inside the fusion node.** The EKF, URDF and TF tree cannot be changed, and the EKF fuses only yaw from the IMU.
- **Exact SLERP interpolation, not a cached IMU sample.** A cached orientation is stale and approximate. Frames are dropped when no bracketing pair exists.
- **Rolling window with an `odom` global frame**, because there is no map and therefore no `map` frame.
- **No `odom_tf_broadcaster`.** `ekf_filter_node` already publishes `odom -> base_link` on the robot and in simulation, so the bridge node was removed from this package.
- **Short voxel decay (1.0 s)** because the costmap is intentionally very dynamic. Controller tuning (`failure_tolerance`, TEB horizons) is set to ride out costmap flicker.
- **Homotopy class planning disabled in TEB**, which runs fine without it and costs less CPU.

---

## Maintainer notes

- Robot hostnames: `r1a006` (RealSense), `r1a008` (OAK-D Lite, aarch64).
- When adding a new camera, only `depth_topic`, `camera_info_topic` and the driver's parent frame should need changing.
