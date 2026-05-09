# DLIO++

ROS2 perception stack for the AV-24 Cybertruck autonomous race car. Combines GPU-accelerated LiDAR-inertial mapping with map-based localization for offline mapping and online operation against a pre-built map.

## Packages

| Package | Purpose |
|---|---|
| [`GLIM/`](GLIM/) | LiDAR-inertial SLAM. Builds a 3D map from IMU + multi-LiDAR + GNSS. Fork of [koide3/GLIM](https://github.com/koide3/glim) with multi-LiDAR concatenation, URDF-based extrinsics, per-point timestamp handling, and GNSS-to-map transform export. |
| [`gicp_localization/`](gicp_localization/) | GICP scan-to-map localization against a PCD map produced by GLIM (or compatible). IMU + LiDAR pipeline with dead-reckoning fallback, layered rejection gates (fitness / combined-hessian / large-jump), and optional ground-truth-driven recovery for catastrophic corner failures. |

Each package has its own README covering install, configuration, and usage.

## Sensor Setup

The configs target an AV-24 Cybertruck with:

- 3× Luminar Iris LiDAR (`luminar_front` primary; `luminar_left`, `luminar_right` concatenated via URDF transforms)
- Novatel INS publishing IMU on `/gps_na/imu` and odometry on `/localization/global/odom` (`novatel_a` URDF link is both `base_frame` and `imu_frame` in the localization config)
- RTK GPS publishing `nav_msgs/msg/Odometry` on `/gps_na/odom` (used by GLIM for the world-to-UTM transform)
- Optional camera (used only by extension modules)

Sensor extrinsics are derived from [`av24.urdf`](av24.urdf) at runtime; see `GLIM/glim/config/config_sensors.json` for the URDF frame names that drive the lookup.

## Workflow

1. **Record** a bag containing IMU + LiDAR + GNSS topics during a driving session.
2. **Map** the run offline with GLIM:
   ```bash
   ros2 run glim_ros glim_rosbag <bag_path> --ros-args -p dump_path:=/tmp/dump
   ```
   Outputs `graph.bin`, optimized trajectories, submap point clouds, and `T_world_utm.txt` (GNSS-to-map SE(3)) under `dump_path`.
3. **Convert** the dumped submaps into a single PCD map (use `glim_ros offline_viewer` or your own merging pass).
4. **Localize** online against the PCD map with `gicp_localization`.

## Build

ROS2 Humble + colcon. Built and tested inside an Ubuntu 22.04 distrobox (`distrobox enter ros2-humble`).

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

GLIM and gicp_localization each pull in their own dependencies — see the per-package READMEs. Headline external libraries:

- GTSAM 4.2, gtsam_points (GPU factors), Eigen3, PCL, OpenMP
- Optional: CUDA 11.8+ (GPU acceleration), Iridescence (viewer), OpenCV

## Quick Reference

```bash
# Live SLAM with real sensors
ros2 launch glim_ros glim_ros.launch.py config_path:=config

# Offline bag → map
ros2 run glim_ros glim_rosbag <bag_path> --ros-args -p dump_path:=<out_dir>

# Inspect a saved map
ros2 run glim_ros offline_viewer

# Localization against pre-built PCD map
ros2 launch gicp_localization localization_with_tf.launch.py rviz:=true \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_na/imu \
    gt_odom_topic:=/localization/global/odom
```

If `ros2 pkg prefix glim` does not point inside this workspace's `install/`, you are running an apt-installed `ros-humble-glim-*` package instead of this fork — re-source the workspace overlay (`source install/setup.bash` *after* `/opt/ros/humble/setup.bash`). Same caveat applies to `gicp_localization` if a sibling workspace is also sourced.

## Repo Layout

```
DLIO_plusplus/
├── GLIM/                # SLAM workspace (glim, glim_ext, glim_ros2)
├── gicp_localization/   # Map-based localization package
├── dlio/                # Convenience metapackage that pulls all of the above
├── av24.urdf            # Vehicle URDF used for sensor extrinsics
├── CLAUDE.md            # Developer-facing project summary
├── AGENTS.md            # Notes for AI reviewers (false positives, watch-conditions)
└── README.md
```

## License

GLIM and gtsam_points are MIT-licensed; GTSAM is BSD. See the upstream repositories and `GLIM/README.md` for details.
