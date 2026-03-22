# GICP Localization

GICP-based localization node for localizing against pre-built point cloud maps. This package provides a standalone localization solution that uses Generalized Iterative Closest Point (GICP) for scan-to-map matching.

## Features

- **Pure GICP Localization**: Scan-to-map matching without odometry fusion
- **Pre-built Map Support**: Works with PCD format maps (GLIM, DLIO, etc.)
- **Automatic Frame Transformation**: Built-in TF2 support for sensor frame transformations
- **Efficient Matching**: Utilizes nano_gicp for fast point cloud registration
- **RViz Visualization**: Integrated visualization of map, aligned scans, and pose

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- Eigen3
- OpenMP
- `direct_lidar_inertial_odometry` - For PointType and nano_gicp

## Building

```bash
cd ~/cybertruck_dlio
colcon build --packages-select gicp_localization
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch gicp_localization localization_with_tf.launch.py \
    map_path:=/path/to/your/map.pcd \
    pointcloud_topic:=/your/lidar/topic \
    rviz:=true
```

### Launch Parameters

- `map_path`: Path to pre-built PCD map file (required)
- `pointcloud_topic`: Input point cloud topic (default: `points_raw`)
- `rviz`: Launch RViz visualization (default: `false`)
- `tf_x`, `tf_y`, `tf_z`: Static transform from base_link to sensor (default: `1.45 0.0 1.91`)
- `tf_qx`, `tf_qy`, `tf_qz`, `tf_qw`: Rotation quaternion for static transform (default: `0 0.7071068 0 0.7071068`)
- `parent_frame`: Parent frame for static TF (default: `base_link`)
- `child_frame`: Child frame for static TF (default: `innovusion`)

### Setting Initial Pose

Use RViz's "2D Pose Estimate" tool to set the initial pose on the `/initialpose` topic.

## Configuration

Edit `cfg/localization.yaml` to adjust parameters:

### GICP Parameters
- `maxIterations`: Maximum GICP iterations (default: 128)
- `maxCorrespondenceDistance`: Max distance for point correspondences in meters (default: 2.0)
- `transformationEpsilon`: Convergence threshold for transformation (default: 0.001)
- `rotationEpsilon`: Convergence threshold for rotation (default: 0.001)

### Preprocessing Parameters
- `crop_size`: Crop box size for scan preprocessing (default: 100.0)
- `voxel_filter/use`: Enable voxel grid filtering (default: true)
- `voxel_filter/resolution`: Voxel size for downsampling (default: 0.25)

## Topics

### Subscribed
- `pointcloud` (sensor_msgs/PointCloud2): Input point cloud from LiDAR
- `initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose estimate

### Published
- `gicp/localization/pose` (geometry_msgs/PoseStamped): Localized pose
- `gicp/localization/odom` (nav_msgs/Odometry): Localized odometry
- `gicp/localization/aligned_cloud` (sensor_msgs/PointCloud2): Aligned scan
- `gicp/localization/map` (sensor_msgs/PointCloud2): Downsampled map for visualization

### TF Frames
- Publishes transform: `map` → `base_link`

## Map Preparation

### Converting GLIM Maps

GLIM produces PLY format maps. Use the provided converter:

```bash
python3 gicp_localization/scripts/convert_ply_to_pcd.py \
    /path/to/glim_map.pcd \
    /path/to/output_map.pcd
```

Note: Despite the `.pcd` extension, GLIM maps are actually in PLY format internally.

## Algorithm

This package implements naive GICP localization:

1. **Initialization**: User provides initial pose via RViz
2. **Frame Transformation**: Incoming scans are transformed from sensor frame to base_link using TF2
3. **Preprocessing**: Scans are cropped and optionally downsampled
4. **GICP Alignment**: Current scan is aligned to the entire pre-built map
5. **Pose Update**: Aligned pose becomes the new localization estimate
6. **Initial Guess**: Previous pose is used as the initial guess for next alignment

This is a "naive" approach in that it:
- Does not use odometry for motion prediction
- Matches against the entire map (not submaps)
- Uses only GICP without sensor fusion

## Performance Notes

- **Map Size**: Successfully tested with 8.4M point maps
- **Update Rate**: Depends on scan density and GICP parameters
- **Convergence**: Fitness scores < 0.5 indicate good alignment

## Troubleshooting

### Localization Fails to Converge
- Increase `maxIterations` or `maxCorrespondenceDistance`
- Ensure initial pose is close to actual position
- Check if sensor frame transformations are correct

### Point Clouds Facing Wrong Direction
- Verify static transform parameters (tf_x, tf_y, tf_z, rotations)
- Check that `child_frame` matches your LiDAR frame ID

### Map Not Loading
- Ensure map is in PCD format (convert from PLY if needed)
- Check file path and permissions
