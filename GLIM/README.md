# GLIM ROS2 Workspace

ROS2 workspace for **GLIM** (Graph-based LiDAR-Inertial Mapping) maintained as an `airacingtech` fork of the upstream `koide3/GLIM` project family.

## Overview

This repository contains a monorepo-style workspace with:
- `glim` for the core SLAM framework
- `glim_ext` for extension modules
- `glim_ros2` for ROS2 integration

The current workspace state is primarily synced from the local `glim_ws` copy and then committed into this monorepo.

## Differences From Upstream GLIM

- This fork keeps `glim`, `glim_ext`, and `glim_ros2` together in one repository instead of separate sibling repositories.
- The monorepo contents are currently aligned to the versions in the local `glim_ws` workspace rather than the previous `ros2_ws` state.
- `glim` includes optional `flip_points_y` preprocessing support for mirrored LiDAR clouds.
- `glim` includes packed LiDAR per-point timestamp parsing support for `UINT8[8]` timestamp fields.
- `glim_ext` includes the GNSS-related modules and configs from the synced `glim_ws` copy.
- `glim_ext` preserves export of the recovered GNSS-to-map SE(3) transform as `T_world_utm.txt` when GNSS alignment is initialized.
- ROS2 and configuration defaults in this fork may differ from upstream to match local vehicle and bag-processing workflows.

### Key Features

- GNSS extension support through `glim_ext`
- ROS2 bag processing through `glim_ros2`
- Local configuration presets for the current mapping setup

The exact behavior of this fork should be taken from the checked-in config and source files in this repository, not assumed to match upstream defaults.

## Repository Structure

```
.
├── glim/          # Core SLAM framework
│   ├── config/    # Configuration files (optimized for cybertruck)
│   ├── include/   # Header files
│   └── src/       # Source code
├── glim_ext/      # Extension modules
│   ├── modules/
│   │   └── mapping/
│   │       └── gnss_global/  # RTK-GPS constraint module
│   └── config/    # Extension configs
└── glim_ros2/     # ROS2 interface
    ├── launch/    # Launch files
    └── src/       # ROS2 nodes
```

## Dependencies

### System Requirements
- Ubuntu 22.04 (recommended)
- ROS2 Humble
- CUDA 11.8+ (optional, for GPU acceleration)

### Core Dependencies
```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libfmt-dev \
  libomp-dev \
  libmetis-dev \
  ros-humble-tf2-eigen \
  ros-humble-pcl-ros
```

### GTSAM (Required)
```bash
# Install GTSAM
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc)
sudo make install
```

### gtsam_points (Required)
```bash
# Install gtsam_points
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points
mkdir build && cd build
cmake .. -DBUILD_WITH_CUDA=ON  # Set OFF if no GPU
make -j$(nproc)
sudo make install
```

### iridescence (Optional, for visualization)
```bash
git clone https://github.com/koide3/iridescence.git
cd iridescence
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Building

### Clone and Build
```bash
# Clone this repository
cd ~/ros2_ws/src
git clone https://github.com/airacingtech/GLIM.git .

# Build with colcon
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Build Options
- **CPU-only build**: Remove `-DBUILD_WITH_CUDA=ON` from gtsam_points build
- **Debug build**: Use `-DCMAKE_BUILD_TYPE=Debug` instead of Release

## Usage

### Running GLIM

**Live mode (with real sensors):**
```bash
ros2 launch glim_ros glim_ros.launch.py config_path:=config
```

**Offline mode (rosbag processing):**
```bash
ros2 run glim_ros glim_rosbag <rosbag_path> --ros-args -p dump_path:=<output_directory>
```

**Example:**
```bash
ros2 run glim_ros glim_rosbag /path/to/rosbag --ros-args -p dump_path:=/home/user/glim_maps/my_map
```

**Offline mode (rosbag replay with launch):**
```bash
# Terminal 1: Launch GLIM
ros2 launch glim_ros glim_ros.launch.py config_path:=config use_sim_time:=true

# Terminal 2: Play rosbag
ros2 bag play <your_bag_file.db3> --clock
```

**With logging:**
```bash
ros2 launch glim_ros glim_ros.launch.py config_path:=config use_sim_time:=true | tee /tmp/glim_live.log
```

### Monitoring GNSS Alignment

Watch for these key log messages:
```
[gnss_global] initializing GNSS global constraints
[gnss_global] gnss_global_config_path=<path>
[gnss_global] T_world_utm=<transformation>
[gnss_global] insert <N> GNSS prior factors        # debug level
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

### Map Output

When using `glim_rosbag`, maps are saved to the specified `dump_path`:
```bash
ros2 run glim_ros glim_rosbag <rosbag> --ros-args -p dump_path:=<output_directory>
```

Each directory contains:
- `graph.txt` / `graph.bin` - Pose graph structure
- `000000/`, `000001/`, ... - Submap directories with point clouds
- `odom_lidar.txt` / `odom_imu.txt` - Odometry trajectories
- `traj_lidar.txt` / `traj_imu.txt` - Optimized trajectories
- `T_world_utm.txt` - **SE(3) transformation from GNSS/UTM to odom frame** (if GNSS enabled)
- `config/` - Configuration files used for this map

## Configuration

### Main Configuration Files

**GLIM Core (`glim/config/`):**
- `config.json` - Main config (points to other configs)
- `config_ros.json` - ROS topics and extension modules
- `config_sensors.json` - Sensor calibration (IMU-LiDAR transform)
- `config_odometry_gpu.json` - Odometry settings and threading
- `config_preprocess.json` - Point cloud preprocessing
- `config_global_mapping_pose_graph.json` - Loop closure and global optimization

**GNSS Extension (`glim_ext/config/`):**
- `config_gnss_global.json` - GPS constraint parameters

### Key Parameters

**GNSS Constraints:**
```json
{
  "gnss": {
    "gnss_topic": "/gps_nav/odom",
    "gnss_msg_type": "nav_msgs/msg/Odometry",
    "min_baseline": 1.0,              // Minimum travel for alignment (meters)
    "prior_inf_scale": [1e4, 1e4, 1e4],  // X, Y, Z information values
    "enable_orientation_prior": true,
    "orientation_prior_inf_scale": [1e2, 1e2, 1e2]  // Roll, pitch, yaw information values
  }
}
```

**Threading (adjust based on your CPU):**
```json
"odometry_estimation": { "num_threads": 2 },
"preprocess":          { "num_threads": 2 }
```
Sub/global mapping use library defaults; tune up if you have spare cores.

## Coordinate Transformation

The GNSS module automatically computes the transformation between:
- **SLAM world frame** (local mapping frame)
- **GPS/UTM frame** (global coordinates)

**Transformation variable:** `T_world_utm`

This transformation is:
- Computed once per session after achieving minimum baseline distance (default: 1.0m)
- Remains static throughout the mapping run
- **Automatically saved to `T_world_utm.txt` in the map directory**

**Convert map point to GPS:**
```cpp
Eigen::Vector3d gps_position = T_world_utm.inverse() * map_position;
```

**Convert GPS to map:**
```cpp
Eigen::Vector3d map_position = T_world_utm * gps_position;
```

The transformation is logged when alignment initializes:
```
[gnss_global] T_world_utm=<transformation>
```

And saved to the map directory when mapping completes:
```
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

## Troubleshooting

### GNSS not aligning
- Check GPS messages are being received: `ros2 topic echo /gps_nav/odom`
- Verify timestamps match between sensors (check for retiming issues)
- Ensure vehicle has traveled > `min_baseline` distance
- Check logs for timestamp warnings

### Low performance
- Reduce thread counts if CPU usage is 100%
- Increase downsampling: lower `random_downsample_target` from the default `10000`
- Disable viewers if running headless

### CUDA errors
- Build gtsam_points with `-DBUILD_WITH_CUDA=OFF`
- System falls back to CPU automatically

### Map not saving
- Use `tee` for logging instead of piping through `grep` so the SIGINT shutdown
  sequence reaches GLIM directly: `ros2 launch ... | tee output.log`
- Default dump path is `/tmp/dump`; override with `-p dump_path:=<dir>` and
  check write permissions on the chosen directory

## Credits

This workspace is based on:

- **GLIM** by Kenji Koide
  - Repository: https://github.com/koide3/glim
  - Paper: [Graph-based LiDAR-Inertial Mapping](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)

- **gtsam_points** by Kenji Koide
  - Repository: https://github.com/koide3/gtsam_points

- **GTSAM** by Georgia Tech
  - Repository: https://github.com/borglab/gtsam

## License

This workspace inherits licenses from its constituent packages:
- GLIM: MIT License
- gtsam_points: MIT License
- GTSAM: BSD License

See individual package directories for full license texts.

## Modifications

This fork includes:
- RTK-GPS global constraint configuration
- Automatic SE(3) transformation saving (T_world_utm.txt)
- Optimized threading parameters for real-time performance
- GNSS module fixes for ROS2 compatibility
- Enhanced logging for debugging
- Cybertruck sensor topic configuration

## Citation

If you use this work, please cite the original GLIM paper:

```bibtex
@article{koide2024glim,
  title={GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author={Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal={IEEE Robotics and Automation Letters},
  year={2024}
}
```
