/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_localization/localization.h"
#include "dlio/utils.h"

#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <algorithm>

gicp_localization::LocalizationNode::LocalizationNode() : Node("gicp_localization_node") {

  this->getParams();

  // Initialize flags
  this->initialized = false;
  this->first_imu_received = false;

  // Initialize pose
  this->current_pose = Eigen::Matrix4f::Identity();
  this->T_prior = Eigen::Matrix4f::Identity();
  this->last_gicp_pose_ = Eigen::Matrix4f::Identity();
  this->last_gicp_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  this->last_gicp_valid_ = false;

  // Initialize IMU buffer
  this->imu_buffer.set_capacity(this->imu_buffer_size_);

  // Initialize previous scan stamp
  this->prev_scan_stamp = 0.0;

  // Initialize lidar pose
  this->lidarPose.p = Eigen::Vector3f::Zero();
  this->lidarPose.q = Eigen::Quaternionf::Identity();

  // Initialize previous velocity
  this->prev_vel = Eigen::Vector3f::Zero();

  // Initialize geometric observer state
  this->state.p = Eigen::Vector3f::Zero();
  this->state.q = Eigen::Quaternionf::Identity();
  this->state.v.lin.b = Eigen::Vector3f::Zero();
  this->state.v.lin.w = Eigen::Vector3f::Zero();
  this->state.v.ang.b = Eigen::Vector3f::Zero();
  this->state.v.ang.w = Eigen::Vector3f::Zero();
  this->state.b.gyro = Eigen::Vector3f::Zero();
  this->state.b.accel = Eigen::Vector3f::Zero();

  this->geo.first_opt_done = false;
  this->geo.dp = 0.0;
  this->geo.dq_deg = 0.0;
  this->geo.prev_p = Eigen::Vector3f::Zero();
  this->geo.prev_q = Eigen::Quaternionf::Identity();
  this->geo.prev_vel = Eigen::Vector3f::Zero();

  // Initialize sensor type (default to OUSTER, can be configured)
  this->sensor = dlio::SensorType::OUSTER;

  // Initialize extrinsics to identity (should be configured from parameters)
  this->extrinsics.baselink2imu.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2imu.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2lidar.t = Eigen::Vector3f::Zero();
  this->extrinsics.baselink2lidar.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();
  this->extrinsics.baselink2lidar_T = Eigen::Matrix4f::Identity();

  // Initialize point clouds
  this->map_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  this->map_cloud_ds = std::make_shared<pcl::PointCloud<PointType>>();
  this->current_scan = std::make_shared<pcl::PointCloud<PointType>>();
  this->original_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Load map
  if (!this->loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Exiting...");
    throw std::runtime_error("Failed to load map");
  }

  // Setup GICP
  this->gicp.setNumThreads(omp_get_max_threads());
  this->gicp.setCorrespondenceRandomness(this->gicp_corr_randomness_);
  this->gicp.setMaxCorrespondenceDistance(this->gicp_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicp_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicp_transformation_epsilon_);
  this->gicp.setRotationEpsilon(this->gicp_rotation_epsilon_);

  // Set target (map)
  this->gicp.setInputTarget(this->map_cloud);
  this->gicp.calculateTargetCovariances();

  // Setup subscribers
  this->pointcloud_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto pointcloud_sub_opt = rclcpp::SubscriptionOptions();
  pointcloud_sub_opt.callback_group = this->pointcloud_cb_group;
  // Use sensor-data QoS so rosbag/sensor publishers with BEST_EFFORT are compatible.
  this->pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&gicp_localization::LocalizationNode::callbackPointCloud, this, std::placeholders::_1),
      pointcloud_sub_opt);

  this->initial_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = this->initial_pose_cb_group;
  this->initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&gicp_localization::LocalizationNode::callbackInitialPose, this, std::placeholders::_1),
      initial_pose_sub_opt);

  // Use Reentrant callback group so IMU can process in parallel with pointcloud processing
  this->imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = this->imu_cb_group;

  // Set QoS for IMU subscriber (BEST_EFFORT to match sensor publishers)
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000));
  imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", imu_qos,
      std::bind(&gicp_localization::LocalizationNode::callbackImu, this, std::placeholders::_1),
      imu_sub_opt);

  // Setup publishers
  this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("localized_pose", 10);

  // Note: Pose is published at IMU rate from propagateState() for perfect time synchronization
  RCLCPP_INFO(this->get_logger(), "Pose will be published at IMU rate (~100 Hz) from propagateState()");

  // High-frequency odometry publisher (100Hz from IMU propagation)
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1000));  // Large queue for 100Hz
  odom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  this->localized_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("localized_odom", odom_qos);

  this->path_pub = this->create_publisher<nav_msgs::msg::Path>("localized_path", 10);
  this->aligned_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 10);

  // Debug publishers (small scalar topics for plotting)
  this->dbg_fitness_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/fitness", 10);
  this->dbg_corr_norm_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/corr_norm", 10);
  this->dbg_scan_dt_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/scan_dt", 10);
  this->dbg_imu_age_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/imu_age", 10);
  this->dbg_jump_trans_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_trans", 10);
  this->dbg_jump_rot_deg_pub = this->create_publisher<std_msgs::msg::Float64>("gicp/localization/debug/jump_rot_deg", 10);
  this->dbg_converged_pub = this->create_publisher<std_msgs::msg::Bool>("gicp/localization/debug/converged", 10);

  if (this->visualize_map_) {
    this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  }

  // TF broadcaster
  if (this->publish_tf_) {
    this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  // TF buffer and listener for transforming incoming point clouds
  this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

  RCLCPP_INFO(this->get_logger(), "DLIO Localization Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Map loaded with %lu points", this->map_cloud->points.size());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

  this->applyInitialPoseFromParams();
}

gicp_localization::LocalizationNode::~LocalizationNode() {}

void gicp_localization::LocalizationNode::getParams() {

  // Frame IDs
  this->declare_parameter<std::string>("localization/map_frame", "map");
  this->declare_parameter<std::string>("localization/base_frame", "base_link");
  this->declare_parameter<std::string>("odom/odom_frame", "odom");
  this->declare_parameter<std::string>("localization/imu_frame", "imu");
  this->declare_parameter<std::string>("localization/lidar_frame", "lidar");

  this->get_parameter("localization/map_frame", this->map_frame);
  this->get_parameter("localization/base_frame", this->base_frame);
  this->get_parameter("odom/odom_frame", this->odom_frame);
  this->get_parameter("localization/imu_frame", this->imu_frame);
  this->get_parameter("localization/lidar_frame", this->lidar_frame);

  // Map parameters
  this->declare_parameter<std::string>("localization/map_path", "");
  this->declare_parameter<double>("localization/voxel_leaf_size", 0.25);
  this->declare_parameter<bool>("localization/visualize_map", true);
  this->declare_parameter<double>("localization/map_voxel_size_vis", 0.5);
  this->declare_parameter<double>("localization/map_rotation/roll_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/pitch_deg", 0.0);
  this->declare_parameter<double>("localization/map_rotation/yaw_deg", 0.0);

  this->get_parameter("localization/map_path", this->map_path_);
  this->get_parameter("localization/voxel_leaf_size", this->voxel_leaf_size_);
  this->get_parameter("localization/visualize_map", this->visualize_map_);
  this->get_parameter("localization/map_voxel_size_vis", this->map_voxel_size_vis_);
  this->get_parameter("localization/map_rotation/roll_deg", this->map_roll_deg_);
  this->get_parameter("localization/map_rotation/pitch_deg", this->map_pitch_deg_);
  this->get_parameter("localization/map_rotation/yaw_deg", this->map_yaw_deg_);

  // Localization parameters
  this->declare_parameter<bool>("localization/publish_tf", true);
  this->declare_parameter<bool>("localization/imu_only", false);
  this->declare_parameter<bool>("localization/use_odom_init", true);
  this->declare_parameter<double>("localization/publish_rate", 10.0);
  this->declare_parameter<bool>("localization/initial_pose/use", false);
  this->declare_parameter<double>("localization/initial_pose/x", 0.0);
  this->declare_parameter<double>("localization/initial_pose/y", 0.0);
  this->declare_parameter<double>("localization/initial_pose/z", 0.0);
  this->declare_parameter<double>("localization/initial_pose/roll", 0.0);
  this->declare_parameter<double>("localization/initial_pose/pitch", 0.0);
  this->declare_parameter<double>("localization/initial_pose/yaw", 0.0);

  this->get_parameter("localization/publish_tf", this->publish_tf_);
  this->get_parameter("localization/imu_only", this->imu_only_mode_);
  this->get_parameter("localization/use_odom_init", this->use_odom_init_);
  this->get_parameter("localization/publish_rate", this->publish_rate_);
  this->get_parameter("localization/initial_pose/use", this->use_param_initial_pose_);
  this->get_parameter("localization/initial_pose/x", this->initial_pose_x_);
  this->get_parameter("localization/initial_pose/y", this->initial_pose_y_);
  this->get_parameter("localization/initial_pose/z", this->initial_pose_z_);
  this->get_parameter("localization/initial_pose/roll", this->initial_pose_roll_);
  this->get_parameter("localization/initial_pose/pitch", this->initial_pose_pitch_);
  this->get_parameter("localization/initial_pose/yaw", this->initial_pose_yaw_);

  // GICP parameters
  this->declare_parameter<int>("gicp/maxIterations", 32);
  this->declare_parameter<int>("gicp/correspondenceRandomness", 20);
  this->declare_parameter<double>("gicp/maxCorrespondenceDistance", 1.0);
  this->declare_parameter<double>("gicp/transformationEpsilon", 0.0001);
  this->declare_parameter<double>("gicp/rotationEpsilon", 0.0001);

  this->get_parameter("gicp/maxIterations", this->gicp_max_iter_);
  this->get_parameter("gicp/correspondenceRandomness", this->gicp_corr_randomness_);
  this->get_parameter("gicp/maxCorrespondenceDistance", this->gicp_max_corr_dist_);
  this->get_parameter("gicp/transformationEpsilon", this->gicp_transformation_epsilon_);
  this->get_parameter("gicp/rotationEpsilon", this->gicp_rotation_epsilon_);

  // Preprocessing parameters
  this->declare_parameter<double>("dlio/preprocessing/cropBoxFilter/size", 0.0);  // Disabled by default
  this->declare_parameter<bool>("dlio/preprocessing/voxelFilter/use", false);  // DISABLED by default - was removing all points
  this->declare_parameter<double>("dlio/preprocessing/voxelFilter/res", 0.25);

  this->get_parameter("dlio/preprocessing/cropBoxFilter/size", this->crop_size_);
  this->get_parameter("dlio/preprocessing/voxelFilter/use", this->vf_use_);
  this->get_parameter("dlio/preprocessing/voxelFilter/res", this->vf_res_);

  // IMU and deskewing parameters
  this->declare_parameter<bool>("dlio/deskew", true);
  this->declare_parameter<double>("dlio/gravity", 9.81);
  this->declare_parameter<int>("dlio/imu/bufferSize", 2000);

  this->get_parameter("dlio/deskew", this->deskew_);
  this->get_parameter("dlio/gravity", this->gravity_);
  this->get_parameter("dlio/imu/bufferSize", this->imu_buffer_size_);

  this->declare_parameter<bool>("localization/flip_y", false);
  this->get_parameter("localization/flip_y", this->flip_y_);

  // Sensor type for per-point timestamp handling during deskewing
  this->declare_parameter<std::string>("localization/sensor_type", "ouster");
  std::string sensor_type_str;
  this->get_parameter("localization/sensor_type", sensor_type_str);
  if (sensor_type_str == "luminar") {
    this->sensor = dlio::SensorType::LUMINAR;
  } else if (sensor_type_str == "velodyne") {
    this->sensor = dlio::SensorType::VELODYNE;
  } else if (sensor_type_str == "hesai") {
    this->sensor = dlio::SensorType::HESAI;
  } else if (sensor_type_str == "livox") {
    this->sensor = dlio::SensorType::LIVOX;
  } else {
    this->sensor = dlio::SensorType::OUSTER;
  }
  RCLCPP_INFO(this->get_logger(), "Sensor type: %s", sensor_type_str.c_str());

  // Geometric Observer parameters
  this->declare_parameter<double>("odom/geo/Kp", 1.0);
  this->declare_parameter<double>("odom/geo/Kv", 1.0);
  this->declare_parameter<double>("odom/geo/Kq", 1.0);
  this->declare_parameter<double>("odom/geo/Kab", 1.0);
  this->declare_parameter<double>("odom/geo/Kgb", 1.0);
  this->declare_parameter<double>("odom/geo/abias_max", 1.0);
  this->declare_parameter<double>("odom/geo/gbias_max", 1.0);

  this->get_parameter("odom/geo/Kp", this->geo_Kp_);
  this->get_parameter("odom/geo/Kv", this->geo_Kv_);
  this->get_parameter("odom/geo/Kq", this->geo_Kq_);
  this->get_parameter("odom/geo/Kab", this->geo_Kab_);
  this->get_parameter("odom/geo/Kgb", this->geo_Kgb_);
  this->get_parameter("odom/geo/abias_max", this->geo_abias_max_);
  this->get_parameter("odom/geo/gbias_max", this->geo_gbias_max_);

  // Debug parameters
  this->declare_parameter<bool>("localization/debug/enable_pub", true);
  this->declare_parameter<bool>("localization/debug/enable_jump_log", true);
  this->declare_parameter<double>("localization/debug/jump_trans_m", 1.0);
  this->declare_parameter<double>("localization/debug/jump_rot_deg", 10.0);

  this->get_parameter("localization/debug/enable_pub", this->debug_pub_enabled_);
  this->get_parameter("localization/debug/enable_jump_log", this->debug_jump_log_enabled_);
  this->get_parameter("localization/debug/jump_trans_m", this->debug_jump_trans_m_);
  this->get_parameter("localization/debug/jump_rot_deg", this->debug_jump_rot_deg_);

  RCLCPP_INFO(this->get_logger(), "Preprocessing config: crop_size=%.2f, voxel_filter=%s, voxel_res=%.2f",
              this->crop_size_, this->vf_use_ ? "ENABLED" : "DISABLED", this->vf_res_);
  RCLCPP_INFO(this->get_logger(), "IMU config: deskew=%s, gravity=%.2f, buffer_size=%d",
              this->deskew_ ? "ENABLED" : "DISABLED", this->gravity_, this->imu_buffer_size_);
  RCLCPP_INFO(this->get_logger(), "Geometric Observer: Kp=%.2f, Kv=%.2f, Kq=%.2f",
              this->geo_Kp_, this->geo_Kv_, this->geo_Kq_);
  RCLCPP_INFO(this->get_logger(), "Localization mode: %s",
              this->imu_only_mode_ ? "IMU-only (GICP disabled)" : "GICP + IMU");
  RCLCPP_INFO(this->get_logger(), "Debug: publish=%s jump_log=%s thresholds=[%.2fm, %.1fdeg]",
              this->debug_pub_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_log_enabled_ ? "ENABLED" : "DISABLED",
              this->debug_jump_trans_m_, this->debug_jump_rot_deg_);
}

bool gicp_localization::LocalizationNode::loadMap() {

  if (this->map_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Map path is empty! Please set localization/map_path parameter.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  // Load PCD file
  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *this->map_cloud) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", this->map_path_.c_str());
    return false;
  }

  if (this->map_cloud->points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }

  // Optional static map rotation to correct coordinate-frame differences from source map files.
  if (std::abs(this->map_roll_deg_) > 1e-6 ||
      std::abs(this->map_pitch_deg_) > 1e-6 ||
      std::abs(this->map_yaw_deg_) > 1e-6) {
    constexpr float kDeg2Rad = 0.017453292519943295f;
    const float roll = static_cast<float>(this->map_roll_deg_ * kDeg2Rad);
    const float pitch = static_cast<float>(this->map_pitch_deg_ * kDeg2Rad);
    const float yaw = static_cast<float>(this->map_yaw_deg_ * kDeg2Rad);

    Eigen::Affine3f map_tf = Eigen::Affine3f::Identity();
    map_tf.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*this->map_cloud, *this->map_cloud, map_tf.matrix());

    RCLCPP_INFO(this->get_logger(),
                "Applied map rotation [roll=%.2f, pitch=%.2f, yaw=%.2f] deg",
                this->map_roll_deg_, this->map_pitch_deg_, this->map_yaw_deg_);
  }

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %lu points", this->map_cloud->points.size());

  // Downsample map for visualization if needed
  if (this->visualize_map_) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(this->map_voxel_size_vis_, this->map_voxel_size_vis_, this->map_voxel_size_vis_);
    vg.setInputCloud(this->map_cloud);
    vg.filter(*this->map_cloud_ds);
    RCLCPP_INFO(this->get_logger(), "Downsampled map for visualization: %lu points", this->map_cloud_ds->points.size());
  }

  return true;
}

void gicp_localization::LocalizationNode::start() {
  // Publish map periodically
  if (this->visualize_map_) {
    auto timer_callback = [this]() {
      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(*this->map_cloud_ds, map_msg);
      map_msg.header.stamp = this->now();
      map_msg.header.frame_id = this->map_frame;
      this->map_pub->publish(map_msg);
    };
    this->map_pub_timer_ = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
  }
}

void gicp_localization::LocalizationNode::applyInitialPoseFromParams() {

  if (!this->use_param_initial_pose_) {
    return;
  }

  Eigen::Vector3f position(static_cast<float>(this->initial_pose_x_),
                           static_cast<float>(this->initial_pose_y_),
                           static_cast<float>(this->initial_pose_z_));

  Eigen::AngleAxisf roll_angle(static_cast<float>(this->initial_pose_roll_), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(this->initial_pose_pitch_), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(this->initial_pose_yaw_), Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf orientation = yaw_angle * pitch_angle * roll_angle;
  orientation.normalize();

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = position;
    this->initialized = true;
  }

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = position;
    this->state.q = orientation;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = position;
    this->geo.prev_q = orientation;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->lidarPose.p = position;
  this->lidarPose.q = orientation;

  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = this->now();

  RCLCPP_INFO(this->get_logger(),
              "Initial pose loaded from parameters at [%.2f, %.2f, %.2f] m with RPY [%.2f, %.2f, %.2f] rad",
              this->initial_pose_x_, this->initial_pose_y_, this->initial_pose_z_,
              this->initial_pose_roll_, this->initial_pose_pitch_, this->initial_pose_yaw_);
}

void gicp_localization::LocalizationNode::applyInitialPose(const Eigen::Vector3f& p,
                                                          const Eigen::Quaternionf& q_in,
                                                          const rclcpp::Time& stamp,
                                                          const std::string& source) {

  Eigen::Quaternionf q = q_in;
  q.normalize();

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = p;
    this->initialized = true;
    if (stamp.nanoseconds() > 0) {
      this->scan_stamp = stamp;
    }
  }

  // Reset geometric observer state to prevent drift from previous estimates
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = p;
    this->state.q = q;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.v.lin.b = Eigen::Vector3f::Zero();
    this->state.v.ang.w = Eigen::Vector3f::Zero();
    this->state.v.ang.b = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
    this->geo.prev_p = p;
    this->geo.prev_q = q;
    this->geo.prev_vel = Eigen::Vector3f::Zero();
    if (this->imu_only_mode_) {
      this->geo.first_opt_done = true;
    }
  }
  this->lidarPose.p = p;
  this->lidarPose.q = q;

  // Clear trajectory path on reinitialization
  this->path_msg.poses.clear();
  this->path_msg.header.frame_id = this->map_frame;
  this->path_msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : this->now();

  RCLCPP_INFO(this->get_logger(), "Received initial pose (%s) at [%.2f, %.2f, %.2f]",
              source.c_str(), p.x(), p.y(), p.z());
}

void gicp_localization::LocalizationNode::callbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose) {

  Eigen::Quaternionf q(
      pose->pose.pose.orientation.w,
      pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z);

  Eigen::Vector3f p(
      pose->pose.pose.position.x,
      pose->pose.pose.position.y,
      pose->pose.pose.position.z);

  this->applyInitialPose(p, q, pose->header.stamp, "PoseWithCovarianceStamped");
}

void gicp_localization::LocalizationNode::callbackPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc) {

  if (!this->initialized) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for initialization (odom or initial pose)...");
    return;
  }

  if (this->imu_only_mode_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "IMU-only mode enabled: skipping pointcloud/GICP updates.");
    return;
  }

  // Transform point cloud to base_link frame if needed
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_transformed = pc;
  if (pc->header.frame_id != this->base_frame) {
    try {
      // Look up transform from sensor frame to base_link
      geometry_msgs::msg::TransformStamped transform_stamped = this->tf_buffer->lookupTransform(
          this->base_frame,
          pc->header.frame_id,
          pc->header.stamp,
          rclcpp::Duration::from_seconds(0.1));

      // Transform the point cloud
      sensor_msgs::msg::PointCloud2 pc_transformed_msg;
      tf2::doTransform(*pc, pc_transformed_msg, transform_stamped);
      pc_transformed = std::make_shared<sensor_msgs::msg::PointCloud2>(pc_transformed_msg);

      RCLCPP_INFO_ONCE(this->get_logger(), "Transforming point clouds from '%s' to '%s'",
                       pc->header.frame_id.c_str(), this->base_frame.c_str());
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not transform point cloud from '%s' to '%s': %s",
                           pc->header.frame_id.c_str(), this->base_frame.c_str(), ex.what());
      return;
    }
  }

  this->scan_stamp = pc_transformed->header.stamp;

  // Convert to PCL format using manual field extraction for robustness
  pcl::PointCloud<PointType>::Ptr raw_scan = std::make_shared<pcl::PointCloud<PointType>>();

  // Calculate number of points
  size_t num_points = pc_transformed->width * pc_transformed->height;

  RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2: width=%d, height=%d, num_points=%lu, data_size=%lu",
               pc_transformed->width, pc_transformed->height, num_points, pc_transformed->data.size());

  if (num_points == 0) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud (width=%d, height=%d)", pc_transformed->width, pc_transformed->height);
    return;
  }

  // Manual conversion using field iterators (most robust for custom formats)
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_transformed, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_transformed, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_transformed, "z");

  RCLCPP_DEBUG(this->get_logger(), "Created XYZ iterators successfully");

  // Check if intensity field exists and determine its type
  bool has_intensity = false;
  uint8_t intensity_datatype = 0;
  for (const auto& field : pc_transformed->fields) {
    if (field.name == "intensity") {
      has_intensity = true;
      intensity_datatype = field.datatype;
      break;
    }
  }

  raw_scan->points.resize(num_points);
  raw_scan->width = pc_transformed->width;
  raw_scan->height = pc_transformed->height;
  raw_scan->is_dense = pc_transformed->is_dense;

  RCLCPP_DEBUG(this->get_logger(), "Resized cloud to %lu points, has_intensity=%d, datatype=%d",
               num_points, has_intensity, intensity_datatype);

  try {
    if (has_intensity) {
      RCLCPP_DEBUG(this->get_logger(), "Converting with intensity (type %d)", intensity_datatype);
      // Handle different intensity data types
      if (intensity_datatype == sensor_msgs::msg::PointField::UINT16) {
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_i(*pc_transformed, "intensity");
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
          raw_scan->points[i].x = *iter_x;
          raw_scan->points[i].y = *iter_y;
          raw_scan->points[i].z = *iter_z;
          raw_scan->points[i].intensity = static_cast<float>(*iter_i);
          raw_scan->points[i].time = 0.0f;
        }
        RCLCPP_DEBUG(this->get_logger(), "Converted %lu points with UINT16 intensity", num_points);
      } else if (intensity_datatype == sensor_msgs::msg::PointField::UINT8) {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_i(*pc_transformed, "intensity");
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
          raw_scan->points[i].x = *iter_x;
          raw_scan->points[i].y = *iter_y;
          raw_scan->points[i].z = *iter_z;
          raw_scan->points[i].intensity = static_cast<float>(*iter_i);
          raw_scan->points[i].time = 0.0f;
        }
      } else if (intensity_datatype == sensor_msgs::msg::PointField::FLOAT32) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*pc_transformed, "intensity");
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
          raw_scan->points[i].x = *iter_x;
          raw_scan->points[i].y = *iter_y;
          raw_scan->points[i].z = *iter_z;
          raw_scan->points[i].intensity = *iter_i;
          raw_scan->points[i].time = 0.0f;
        }
      } else if (intensity_datatype == sensor_msgs::msg::PointField::FLOAT64) {
        sensor_msgs::PointCloud2ConstIterator<double> iter_i(*pc_transformed, "intensity");
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
          raw_scan->points[i].x = *iter_x;
          raw_scan->points[i].y = *iter_y;
          raw_scan->points[i].z = *iter_z;
          raw_scan->points[i].intensity = static_cast<float>(*iter_i);
          raw_scan->points[i].time = 0.0f;
        }
      } else {
        // Fallback for unknown intensity type
        RCLCPP_WARN(this->get_logger(), "Unknown intensity type %d, ignoring", intensity_datatype);
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
          raw_scan->points[i].x = *iter_x;
          raw_scan->points[i].y = *iter_y;
          raw_scan->points[i].z = *iter_z;
          raw_scan->points[i].intensity = 0.0f;
          raw_scan->points[i].time = 0.0f;
        }
      }
    } else {
      // No intensity field
      RCLCPP_DEBUG(this->get_logger(), "Converting without intensity");
      for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        raw_scan->points[i].x = *iter_x;
        raw_scan->points[i].y = *iter_y;
        raw_scan->points[i].z = *iter_z;
        raw_scan->points[i].intensity = 0.0f;
        raw_scan->points[i].time = 0.0f;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during point cloud conversion: %s", e.what());
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully converted, raw_scan has %lu points", raw_scan->points.size());

  // For Luminar: read per-point timestamps (uint64 nanoseconds) from the raw PointCloud2 bytes.
  // The manual conversion loop above only reads x/y/z and leaves pt.timestamp as zero.
  if (this->sensor == dlio::SensorType::LUMINAR) {
    uint32_t ts_offset = 0;
    bool ts_found = false;
    for (const auto& field : pc_transformed->fields) {
      if (field.name == "timestamp") {
        ts_offset = field.offset;
        ts_found = true;
        break;
      }
    }
    if (ts_found) {
      for (size_t i = 0; i < num_points; ++i) {
        uint64_t ts_raw;
        memcpy(&ts_raw, &pc_transformed->data[i * pc_transformed->point_step + ts_offset], sizeof(uint64_t));
        // Store raw uint64 bytes into pt.timestamp (double field, same 8-byte size).
        // deskewPointcloud will memcpy them back out as uint64.
        memcpy(&raw_scan->points[i].timestamp, &ts_raw, sizeof(uint64_t));
      }
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), "Luminar sensor type set but 'timestamp' field not found in point cloud!");
    }
  }

  // Optionally negate Y axis (e.g. to convert SAE Y-right to ROS Y-left without flipping Z)
  if (this->flip_y_) {
    for (auto& pt : raw_scan->points) {
      pt.y = -pt.y;
    }
  }

  // Store as original scan for deskewing
  this->original_scan = raw_scan;

  // Deskew using IMU
  this->deskewPointcloud();

  RCLCPP_DEBUG(this->get_logger(), "After deskewing: current_scan has %lu points",
               this->current_scan->points.size());

  // Preprocess the deskewed scan
  this->preprocessPointCloud(this->current_scan);

  RCLCPP_DEBUG(this->get_logger(), "Before preprocessing: current_scan has %lu points",
               this->current_scan->points.size());

  RCLCPP_DEBUG(this->get_logger(), "After preprocessing: current_scan has %lu points",
               this->current_scan->points.size());

  if (this->current_scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud empty after preprocessing (original had %lu points)",
                raw_scan->points.size());
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "After preprocessing: %lu points", this->current_scan->points.size());

  // Perform localization
  RCLCPP_DEBUG(this->get_logger(), "Calling performLocalization()...");
  this->performLocalization();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization() completed");

  // Publish results
  RCLCPP_DEBUG(this->get_logger(), "Calling publishPose()...");
  this->publishPose();
  RCLCPP_DEBUG(this->get_logger(), "publishPose() completed");
}

void gicp_localization::LocalizationNode::deskewPointcloud() {

  if (!this->deskew_ || !this->first_imu_received) {
    // If deskewing is disabled or no IMU data yet, just use original scan
    this->current_scan = this->original_scan;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return;
  }

  pcl::PointCloud<PointType>::Ptr deskewed_scan_ =
      std::make_shared<pcl::PointCloud<PointType>>(1, this->original_scan->points.size());

  // Individual point timestamps should be relative to this time
  double sweep_ref_time = this->scan_stamp.seconds();

  // Sort points by timestamp and build list of timestamps
  std::function<bool(const PointType&, const PointType&)> point_time_cmp;
  std::function<double(const PointType&)> extract_point_time_from_point;

  if (this->sensor == dlio::SensorType::OUSTER) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.t < p2.t; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) { return sweep_ref_time + pt.t * 1e-9f; };
  } else if (this->sensor == dlio::SensorType::VELODYNE) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) { return sweep_ref_time + pt.time; };
  } else if (this->sensor == dlio::SensorType::HESAI) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) { return pt.timestamp; };
  } else if (this->sensor == dlio::SensorType::LIVOX) {
    point_time_cmp = [](const PointType& p1, const PointType& p2) { return p1.timestamp < p2.timestamp; };
    extract_point_time_from_point = [&sweep_ref_time](const PointType& pt) { return pt.timestamp * 1e-9f; };
  } else if (this->sensor == dlio::SensorType::LUMINAR) {
    // Luminar publishes timestamp as uint64 nanoseconds in hardware clock domain (not Unix epoch).
    // PCL copies the 8 raw bytes into pt.timestamp (double), so we reinterpret as uint64.
    // Use relative offset from the earliest point in the scan, anchored to scan header time.
    uint64_t min_ts = UINT64_MAX;
    for (const auto& pt : this->original_scan->points) {
      uint64_t ts;
      memcpy(&ts, &pt.timestamp, sizeof(uint64_t));
      if (ts < min_ts) min_ts = ts;
    }
    const uint64_t min_ts_captured = min_ts;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Luminar scan: min_ts=%lu ns, sweep_ref=%.3f s", min_ts_captured, sweep_ref_time);
    point_time_cmp = [](const PointType& p1, const PointType& p2) {
      uint64_t t1, t2;
      memcpy(&t1, &p1.timestamp, sizeof(uint64_t));
      memcpy(&t2, &p2.timestamp, sizeof(uint64_t));
      return t1 < t2;
    };
    extract_point_time_from_point = [&sweep_ref_time, min_ts_captured](const PointType& pt) {
      uint64_t ts;
      memcpy(&ts, &pt.timestamp, sizeof(uint64_t));
      return sweep_ref_time + static_cast<double>(ts - min_ts_captured) * 1e-9;
    };
  }

  // Copy points into deskewed_scan_ in order of timestamp
  std::partial_sort_copy(this->original_scan->points.begin(), this->original_scan->points.end(),
                         deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

  // Extract timestamps from points and build list of unique timestamps
  std::vector<double> timestamps;
  std::vector<int> unique_time_indices;

  double prev_timestamp = -1.0;
  for (size_t i = 0; i < deskewed_scan_->points.size(); i++) {
    double curr_timestamp = extract_point_time_from_point(deskewed_scan_->points[i]);
    if (std::abs(curr_timestamp - prev_timestamp) > 1e-9) {  // Unique timestamp
      timestamps.push_back(curr_timestamp);
      unique_time_indices.push_back(i);
      prev_timestamp = curr_timestamp;
    }
  }
  unique_time_indices.push_back(deskewed_scan_->points.size());

  if (timestamps.empty()) {
    RCLCPP_WARN(this->get_logger(), "No timestamps extracted from point cloud, skipping deskewing");
    this->current_scan = this->original_scan;
    return;
  }

  int median_pt_index = timestamps.size() / 2;

  // Don't process scans on first iteration
  if (this->prev_scan_stamp == 0.0) {
    this->prev_scan_stamp = this->scan_stamp.seconds();
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    return;
  }

  // Check if we have sufficient IMU history
  // We need IMU data from BEFORE prev_scan_stamp to integrate
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (this->imu_buffer.empty()) {
      RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping deskewing");
      this->current_scan = deskewed_scan_;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }

    // Check if oldest IMU is before prev_scan_stamp (need some margin)
    double oldest_imu_time = this->imu_buffer.back().stamp;
    double margin = 0.1;  // 100ms margin

    if (oldest_imu_time > this->prev_scan_stamp - margin) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for sufficient IMU history (oldest: %.3f, need: %.3f). Skipping deskewing.",
                           oldest_imu_time, this->prev_scan_stamp);
      this->T_prior = this->current_pose;
      pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
      this->current_scan = deskewed_scan_;
      this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
      return;
    }
  }

  // IMU prior & deskewing
  RCLCPP_DEBUG(this->get_logger(),
               "Integrating IMU: prev_stamp=%.3f, pos=[%.2f,%.2f,%.2f], vel=[%.2f,%.2f,%.2f]",
               this->prev_scan_stamp,
               this->lidarPose.p.x(), this->lidarPose.p.y(), this->lidarPose.p.z(),
               this->prev_vel.x(), this->prev_vel.y(), this->prev_vel.z());

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> frames;
  frames = this->integrateImu(this->prev_scan_stamp, this->lidarPose.q, this->lidarPose.p,
                              this->prev_vel, timestamps);

  // If there are no frames between the start and end of the sweep, use previous transform
  if (frames.size() != timestamps.size()) {
    RCLCPP_WARN(this->get_logger(),
                "IMU integration failed! Got %lu frames for %lu timestamps. "
                "Time range: [%.3f, %.3f], IMU buffer size: %lu, first IMU: %.3f",
                frames.size(), timestamps.size(),
                this->prev_scan_stamp, timestamps.back(),
                this->imu_buffer.size(),
                this->imu_buffer.empty() ? 0.0 : this->imu_buffer.back().stamp);
    this->T_prior = this->current_pose;
    pcl::transformPointCloud(*deskewed_scan_, *deskewed_scan_, this->T_prior * this->extrinsics.baselink2lidar_T);
    this->current_scan = deskewed_scan_;
    this->prev_scan_stamp = this->scan_stamp.seconds();  // Update timestamp
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Deskewing OK: %lu frames, scan time [%.3f, %.3f]",
                       frames.size(), timestamps.front(), timestamps.back());

  // Update prior to be the estimated pose at the median time of the scan
  this->T_prior = frames[median_pt_index];

  // Deskew each point using its timestamp
  #pragma omp parallel for
  for (size_t i = 0; i < timestamps.size(); i++) {
    Eigen::Matrix4f T = frames[i] * this->extrinsics.baselink2lidar_T;

    // Transform point to world frame
    for (int k = unique_time_indices[i]; k < unique_time_indices[i+1]; k++) {
      auto &pt = deskewed_scan_->points[k];
      pt.getVector4fMap()[3] = 1.;
      pt.getVector4fMap() = T * pt.getVector4fMap();
    }
  }

  this->current_scan = deskewed_scan_;
  this->prev_scan_stamp = this->scan_stamp.seconds();
}

void gicp_localization::LocalizationNode::preprocessPointCloud(pcl::PointCloud<PointType>::Ptr& cloud) {

  size_t original_size = cloud->points.size();

  // Crop box filter
  if (this->crop_size_ > 0.0 && this->crop_size_ < 1000.0) {  // Only apply if reasonable size
    pcl::CropBox<PointType> crop;
    crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));
    crop.setInputCloud(cloud);
    crop.filter(*cloud);
    RCLCPP_DEBUG(this->get_logger(), "Crop box filter: %lu -> %lu points", original_size, cloud->points.size());
  }

  // Voxel filter
  if (this->vf_use_) {
    size_t before_voxel = cloud->points.size();
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(this->vf_res_, this->vf_res_, this->vf_res_);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud);
    RCLCPP_DEBUG(this->get_logger(), "Voxel filter: %lu -> %lu points", before_voxel, cloud->points.size());
  }

  RCLCPP_DEBUG(this->get_logger(), "Preprocessing: %lu -> %lu points total", original_size, cloud->points.size());
}

void gicp_localization::LocalizationNode::performLocalization() {

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Acquiring mutex lock...");
  std::lock_guard<std::mutex> lock(this->pose_mutex);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Mutex acquired");

  // Set source cloud
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Setting input source (%lu points)...",
               this->current_scan->points.size());
  this->gicp.setInputSource(this->current_scan);
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Input source set");

  // Align using IMU-based prior as initial guess (if deskewing is enabled)
  // Otherwise use the previous pose
  pcl::PointCloud<PointType>::Ptr aligned = std::make_shared<pcl::PointCloud<PointType>>();

  // If deskewing is enabled, the points are already in world frame at T_prior
  // So we align with identity, and the result is the correction T_corr
  // Final pose = T_corr * T_prior (similar to DLIO)
  Eigen::Matrix4f initial_guess = this->deskew_ ? Eigen::Matrix4f::Identity() : this->current_pose;

  RCLCPP_DEBUG(this->get_logger(), "performLocalization: Starting GICP alignment...");
  auto start = std::chrono::high_resolution_clock::now();
  this->gicp.align(*aligned, initial_guess);
  auto end = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "performLocalization: GICP alignment completed");

  double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

  double fitness_score = this->gicp.getFitnessScore();
  bool converged = this->gicp.hasConverged();

  if (converged) {
    Eigen::Matrix4f T_corr = this->gicp.getFinalTransformation();

    // If deskewing is enabled, combine correction with IMU prior
    // Otherwise, T_corr is the full pose
    if (this->deskew_) {
      this->current_pose = T_corr * this->T_prior;
    } else {
      this->current_pose = T_corr;
    }

    // Update lidar pose for next iteration
    Eigen::Vector3f new_p = this->current_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotSO3 = this->current_pose.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rotSO3);
    q.normalize();

    this->lidarPose.p = new_p;
    this->lidarPose.q = q;

    // Validate GICP result before using it
    bool gicp_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) && std::isfinite(new_p.z()) &&
                      std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());

    if (!gicp_valid) {
      RCLCPP_WARN(this->get_logger(), "GICP result contains invalid values, skipping geometric observer update");
    } else {
      // Initialize or update geometric observer
      if (!this->geo.first_opt_done) {
        // First time: initialize state to GICP result
        this->state.p = new_p;
        this->state.q = q;
        this->state.v.lin.w = Eigen::Vector3f::Zero();
        this->state.v.lin.b = Eigen::Vector3f::Zero();
        this->state.v.ang.w = Eigen::Vector3f::Zero();
        this->state.v.ang.b = Eigen::Vector3f::Zero();
        this->state.b.accel = Eigen::Vector3f::Zero();
        this->state.b.gyro = Eigen::Vector3f::Zero();

        // Initialize geo tracking
        this->geo.prev_p = this->state.p;
        this->geo.prev_q = this->state.q;
        this->geo.prev_vel = Eigen::Vector3f::Zero();

        // Mark as initialized
        this->geo.first_opt_done = true;

        RCLCPP_INFO(this->get_logger(), "Geometric observer initialized to pos=[%.2f,%.2f,%.2f]",
                    new_p.x(), new_p.y(), new_p.z());
      } else {
        // Update geometric observer with GICP measurement (skip on first scan)
        this->updateState();
      }
    }

    // Use geometric observer velocity for next IMU integration
    this->prev_vel = this->geo.prev_vel;

    // Debug metrics
    double scan_dt = 0.0;
    if (this->last_gicp_valid_) {
      scan_dt = (this->scan_stamp - this->last_gicp_stamp_).seconds();
    }

    double imu_age = -1.0;
    {
      std::lock_guard<std::mutex> lock(this->mtx_imu);
      if (!this->imu_buffer.empty()) {
        imu_age = this->imu_buffer.front().stamp - this->imu_buffer.back().stamp;
      }
    }

    double jump_trans = -1.0;
    double jump_rot_deg = -1.0;
    if (gicp_valid && this->last_gicp_valid_) {
      const Eigen::Vector3f last_p = this->last_gicp_pose_.block<3, 1>(0, 3);
      const Eigen::Matrix3f last_R = this->last_gicp_pose_.block<3, 3>(0, 0);
      Eigen::Quaternionf last_q(last_R);
      last_q.normalize();

      Eigen::Quaternionf dq = last_q.conjugate() * q;
      dq.normalize();
      double dq_w = std::max(-1.0, std::min(1.0, static_cast<double>(dq.w())));
      double angle_rad = 2.0 * std::acos(dq_w);

      jump_trans = (new_p - last_p).norm();
      jump_rot_deg = angle_rad * 57.29577951308232;
    }

    if (this->debug_pub_enabled_) {
      std_msgs::msg::Float64 f;
      f.data = fitness_score;
      this->dbg_fitness_pub->publish(f);
      f.data = T_corr.block<3, 1>(0, 3).norm();
      this->dbg_corr_norm_pub->publish(f);
      f.data = scan_dt;
      this->dbg_scan_dt_pub->publish(f);
      f.data = imu_age;
      this->dbg_imu_age_pub->publish(f);
      f.data = jump_trans;
      this->dbg_jump_trans_pub->publish(f);
      f.data = jump_rot_deg;
      this->dbg_jump_rot_deg_pub->publish(f);
      std_msgs::msg::Bool b;
      b.data = true;
      this->dbg_converged_pub->publish(b);
    }

    if (this->debug_jump_log_enabled_ && gicp_valid && this->last_gicp_valid_) {
      if (jump_trans > this->debug_jump_trans_m_ || jump_rot_deg > this->debug_jump_rot_deg_) {
        const Eigen::Vector3f t_prior = this->T_prior.block<3, 1>(0, 3);
        const Eigen::Vector3f t_corr = T_corr.block<3, 1>(0, 3);
        RCLCPP_WARN(this->get_logger(),
                    "JUMP DETECTED: dT=%.3fm dR=%.2fdeg | dt=%.3fs fitness=%.6f | prior=[%.2f,%.2f,%.2f] corr=[%.2f,%.2f,%.2f]",
                    jump_trans, jump_rot_deg, scan_dt, fitness_score,
                    t_prior.x(), t_prior.y(), t_prior.z(),
                    t_corr.x(), t_corr.y(), t_corr.z());
      }
    }

    // Update last GICP pose after computing jump metrics
    if (gicp_valid) {
      this->last_gicp_pose_ = this->current_pose;
      this->last_gicp_stamp_ = this->scan_stamp;
      this->last_gicp_valid_ = true;
    }

    // Log pose and correction
    Eigen::Vector3f t_corr = T_corr.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(),
                "Localization: ✓ CONVERGED | fitness=%.6f | time=%.2fms | "
                "correction=[%.3f, %.3f, %.3f] | pose=[%.2f, %.2f, %.2f]",
                fitness_score, elapsed_ms,
                t_corr.x(), t_corr.y(), t_corr.z(),
                this->lidarPose.p.x(), this->lidarPose.p.y(), this->lidarPose.p.z());

    // Publish aligned cloud for visualization
    if (this->aligned_cloud_pub->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header.stamp = this->scan_stamp;
      aligned_msg.header.frame_id = this->map_frame;
      this->aligned_cloud_pub->publish(aligned_msg);
    }
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Localization: ✗ FAILED TO CONVERGE | fitness=%.6f | time=%.2fms | Pose NOT updated!",
                fitness_score, elapsed_ms);

    if (this->debug_pub_enabled_) {
      std_msgs::msg::Float64 f;
      f.data = fitness_score;
      this->dbg_fitness_pub->publish(f);
      std_msgs::msg::Bool b;
      b.data = false;
      this->dbg_converged_pub->publish(b);
    }
  }
}

void gicp_localization::LocalizationNode::publishPose() {

  std::lock_guard<std::mutex> lock(this->pose_mutex);

  // Extract position and orientation from localized pose
  Eigen::Vector3f position = this->current_pose.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = this->current_pose.block<3, 3>(0, 0);
  Eigen::Quaternionf orientation(rotation);
  orientation.normalize();

  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->scan_stamp;
  pose_msg.header.frame_id = this->map_frame;
  pose_msg.pose.position.x = position.x();
  pose_msg.pose.position.y = position.y();
  pose_msg.pose.position.z = position.z();
  pose_msg.pose.orientation.w = orientation.w();
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();

  // Publish GICP-corrected pose
  // With unreliable IMU, we publish only GICP results instead of propagated poses
  this->pose_pub->publish(pose_msg);

  // Add to trajectory path (capped to avoid unbounded memory growth)
  this->path_msg.header.stamp = this->scan_stamp;
  this->path_msg.header.frame_id = this->map_frame;
  if (this->path_msg.poses.size() >= 10000) {
    this->path_msg.poses.erase(this->path_msg.poses.begin());
  }
  this->path_msg.poses.push_back(pose_msg);
  this->path_pub->publish(this->path_msg);

  // Publish TF
  if (this->publish_tf_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = pose_msg.header;
    transform_stamped.child_frame_id = this->base_frame;
    transform_stamped.transform.translation.x = position.x();
    transform_stamped.transform.translation.y = position.y();
    transform_stamped.transform.translation.z = position.z();
    transform_stamped.transform.rotation = pose_msg.pose.orientation;
    this->tf_broadcaster->sendTransform(transform_stamped);
  }
}

void gicp_localization::LocalizationNode::callbackImu(const sensor_msgs::msg::Imu::SharedPtr imu) {

  double stamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;

  ImuMeas imu_meas_temp;
  imu_meas_temp.stamp = stamp;
  imu_meas_temp.ang_vel << imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z;
  imu_meas_temp.lin_accel << imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z;

  // Calculate dt
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    if (!this->imu_buffer.empty()) {
      imu_meas_temp.dt = stamp - this->imu_buffer.front().stamp;
    } else {
      imu_meas_temp.dt = 0.0;
    }

    this->imu_buffer.push_front(imu_meas_temp);
    this->imu_meas = imu_meas_temp;
  }

  if (!this->first_imu_received) {
    this->first_imu_received = true;
    RCLCPP_INFO(this->get_logger(), "First IMU message received");
  }

  // Propagate state with geometric observer (only after initialization)
  static int propagate_calls = 0;
  static int imu_total = 0;
  static bool logged_first_propagate = false;
  imu_total++;

  if (this->initialized && (this->geo.first_opt_done || this->imu_only_mode_)) {
    this->propagateState();
    propagate_calls++;

    // Log first successful propagation
    if (!logged_first_propagate) {
      RCLCPP_INFO(this->get_logger(), "First IMU propagation successful! Starting high-frequency odometry.");
      logged_first_propagate = true;
    }
  }

  // Debug: Log IMU and propagation rates periodically
  static int imu_count = 0;
  if (++imu_count % 100 == 0) {  // Log every 100 IMU messages (~1 second)
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    RCLCPP_INFO(this->get_logger(), "IMU rate check: %d callbacks, %d propagations, initialized=%d, geo_init=%d",
                imu_total, propagate_calls, this->initialized, this->geo.first_opt_done.load());
    imu_total = 0;
    propagate_calls = 0;
  }
}

bool gicp_localization::LocalizationNode::imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {

  std::lock_guard<std::mutex> lock(this->mtx_imu);

  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Not enough IMU data yet
    return false;
  }

  auto imu_it = this->imu_buffer.begin();

  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->imu_buffer.end()) {
    // not enough IMU measurements
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImu(
    double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init,
    Eigen::Vector3f v_init, const std::vector<double>& sorted_timestamps) {

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> empty;

  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    // invalid input, return empty vector
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(), begin_imu_it, end_imu_it) == false) {
    // not enough IMU measurements, return empty vector
    return empty;
  }

  // Backwards integration to find pose at first IMU sample
  if ((begin_imu_it + 1) == end_imu_it) {
    // Need at least two IMU measurements for backwards integration
    return empty;
  }

  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it+1);

  // Time between first two IMU samples
  double dt = f2.dt;

  if (dt <= 0.0) {
    return empty;
  }

  // Time between first IMU sample and start_time
  double idt = start_time - f1.stamp;

  // Angular acceleration between first two IMU samples
  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  // Average angular velocity (reversed) between first IMU sample and start_time
  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5*alpha*idt);

  // Set q_init to orientation at first IMU sample
  q_init = Eigen::Quaternionf (
    q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
    q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
    q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
    q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
  );
  q_init.normalize();

  // Average angular velocity between first two IMU samples
  Eigen::Vector3f omega = f1.ang_vel + 0.5*alpha_dt;

  // Orientation at second IMU sample
  Eigen::Quaternionf q2 (
    q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
    q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
    q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
    q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
  );
  q2.normalize();

  // Acceleration at first IMU sample
  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  a1[2] -= this->gravity_;

  // Acceleration at second IMU sample
  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  a2[2] -= this->gravity_;

  // Jerk between first two IMU samples
  Eigen::Vector3f j = (a2 - a1) / dt;

  // Set v_init to velocity at first IMU sample (go backwards from start_time)
  v_init -= a1*idt + 0.5*j*idt*idt;

  // Set p_init to position at first IMU sample (go backwards from start_time)
  p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
gicp_localization::LocalizationNode::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  a[2] -= this->gravity_;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;

  auto stamp_it = sorted_timestamps.begin();

  for (; imu_it != end_imu_it; imu_it++) {

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    if (dt <= 0.0) {
      prev_imu_it = imu_it;
      continue;
    }

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5*alpha_dt;

    // Orientation
    q = Eigen::Quaternionf (
      q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
      q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
      q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
      q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
    );
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    a[2] -= this->gravity_;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5*alpha*idt;

      // Orientation
      Eigen::Quaternionf q_i (
        q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
        q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
        q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
        q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
      );
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // Position
    p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

    // Velocity
    v += a0*dt + 0.5*j_dt*dt;

    prev_imu_it = imu_it;

  }

  return imu_se3;

}

void gicp_localization::LocalizationNode::propagateState() {

  ImuMeas imu_local;
  {
    std::lock_guard<std::mutex> lock(this->mtx_imu);
    imu_local = this->imu_meas;
  }

  double dt = imu_local.dt;

  if (dt <= 0.0 || dt > 1.0) {
    static int skip_count = 0;
    if (++skip_count % 100 == 0) {
      RCLCPP_WARN(this->get_logger(), "Skipping propagation due to invalid dt: %.6f (skipped %d times)", dt, skip_count);
    }
    return;  // Skip invalid dt
  }

  // Read current state with minimal lock time
  Eigen::Vector3f current_p;
  Eigen::Quaternionf current_q;
  Eigen::Vector3f current_v_lin_w;
  Eigen::Vector3f bias_gyro;
  Eigen::Vector3f bias_accel;

  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    current_p = this->state.p;
    current_q = this->state.q;
    current_v_lin_w = this->state.v.lin.w;
    bias_gyro = this->state.b.gyro;
    bias_accel = this->state.b.accel;
  }

  // Do computation without holding lock
  Eigen::Quaternionf qhat = current_q;
  Eigen::Quaternionf omega;
  Eigen::Vector3f world_accel;

  // Apply gyro bias correction
  Eigen::Vector3f ang_vel_corrected = imu_local.ang_vel - bias_gyro;

  // Apply accel bias correction
  Eigen::Vector3f lin_accel_corrected = imu_local.lin_accel - bias_accel;

  // Transform accel from body to world frame
  world_accel = qhat._transformVector(lin_accel_corrected);

  // Log propagation status periodically
  static int propagate_count = 0;
  if (++propagate_count % 1000 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer: pos_z=%.3f vel_z=%.3f | accel_raw_z=%.3f bias_z=%.3f world_accel_z=%.3f | gravity=%.2f",
                current_p.z(), current_v_lin_w.z(),
                imu_local.lin_accel.z(), bias_accel.z(), world_accel.z(),
                this->gravity_);
  }

  // Position propagation (with gravity compensation)
  // For ground vehicles: only propagate x,y from IMU; z comes from GICP only
  Eigen::Vector3f new_p = current_p;
  new_p[0] += current_v_lin_w[0]*dt + 0.5*dt*dt*world_accel[0];
  new_p[1] += current_v_lin_w[1]*dt + 0.5*dt*dt*world_accel[1];
  // new_p[2] stays unchanged - no z propagation to avoid IMU drift

  // Velocity propagation
  // Also zero out z-velocity since we assume ground vehicle (no sustained vertical motion)
  Eigen::Vector3f new_v_lin_w = current_v_lin_w;
  new_v_lin_w[0] += world_accel[0]*dt;
  new_v_lin_w[1] += world_accel[1]*dt;
  new_v_lin_w[2] = 0.0;  // Zero z-velocity for ground vehicle

  // Orientation propagation
  omega.w() = 0;
  omega.vec() = ang_vel_corrected;
  Eigen::Quaternionf tmp = qhat * omega;
  Eigen::Quaternionf new_q;
  new_q.w() = qhat.w() + 0.5 * dt * tmp.w();
  new_q.vec() = qhat.vec() + 0.5 * dt * tmp.vec();

  // Ensure quaternion is properly normalized
  new_q.normalize();

  // Store angular velocity
  Eigen::Vector3f new_v_ang_b = ang_vel_corrected;
  Eigen::Vector3f new_v_ang_w = new_q.toRotationMatrix() * new_v_ang_b;

  // Validate computed state before publishing
  bool state_valid = std::isfinite(new_p.x()) && std::isfinite(new_p.y()) &&
                     std::isfinite(new_p.z()) && std::isfinite(new_q.w()) &&
                     std::isfinite(new_q.x()) && std::isfinite(new_q.y()) &&
                     std::isfinite(new_q.z()) &&
                     std::isfinite(new_v_lin_w.x()) && std::isfinite(new_v_lin_w.y()) &&
                     std::isfinite(new_v_lin_w.z());

  if (!state_valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Skipping odometry publish - state contains invalid values (p=[%.3f,%.3f,%.3f], q=[%.3f,%.3f,%.3f,%.3f])",
                         new_p.x(), new_p.y(), new_p.z(),
                         new_q.w(), new_q.x(), new_q.y(), new_q.z());
    return;  // Skip publishing if state contains invalid values
  }

  // Use IMU timestamp (from bag file or sensor)
  rclcpp::Time current_time;
  current_time = rclcpp::Time(static_cast<int64_t>(imu_local.stamp * 1e9));

  // Log successful validation on first publish
  static bool logged_first_publish = false;
  if (!logged_first_publish) {
    RCLCPP_INFO(this->get_logger(), "First odometry publish! Using IMU timestamp: %.3f", imu_local.stamp);
    logged_first_publish = true;
  }

  // Build odometry message from computed values (not from this->state to avoid race condition)
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = this->map_frame;
  odom_msg.child_frame_id = this->base_frame;

  // Position and orientation from propagated state
  odom_msg.pose.pose.position.x = new_p.x();
  odom_msg.pose.pose.position.y = new_p.y();
  odom_msg.pose.pose.position.z = new_p.z();
  odom_msg.pose.pose.orientation.w = new_q.w();
  odom_msg.pose.pose.orientation.x = new_q.x();
  odom_msg.pose.pose.orientation.y = new_q.y();
  odom_msg.pose.pose.orientation.z = new_q.z();

  // Velocity from propagated state (in world frame)
  odom_msg.twist.twist.linear.x = new_v_lin_w.x();
  odom_msg.twist.twist.linear.y = new_v_lin_w.y();
  odom_msg.twist.twist.linear.z = new_v_lin_w.z();
  odom_msg.twist.twist.angular.x = new_v_ang_w.x();
  odom_msg.twist.twist.angular.y = new_v_ang_w.y();
  odom_msg.twist.twist.angular.z = new_v_ang_w.z();

  this->localized_odom_pub->publish(odom_msg);

  if (this->imu_only_mode_) {
    // Publish pose/TF directly from propagated IMU state when GICP is disabled.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = this->map_frame;
    pose_msg.pose.position.x = new_p.x();
    pose_msg.pose.position.y = new_p.y();
    pose_msg.pose.position.z = new_p.z();
    pose_msg.pose.orientation.w = new_q.w();
    pose_msg.pose.orientation.x = new_q.x();
    pose_msg.pose.orientation.y = new_q.y();
    pose_msg.pose.orientation.z = new_q.z();
    this->pose_pub->publish(pose_msg);

    static int path_decimator = 0;
    if (++path_decimator % 10 == 0) {
      std::lock_guard<std::mutex> path_lock(this->pose_mutex);
      this->path_msg.header.stamp = current_time;
      this->path_msg.header.frame_id = this->map_frame;
      if (this->path_msg.poses.size() >= 10000) {
        this->path_msg.poses.erase(this->path_msg.poses.begin());
      }
      this->path_msg.poses.push_back(pose_msg);
      this->path_pub->publish(this->path_msg);
    }

    if (this->publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = current_time;
      transform_stamped.header.frame_id = this->map_frame;
      transform_stamped.child_frame_id = this->base_frame;
      transform_stamped.transform.translation.x = new_p.x();
      transform_stamped.transform.translation.y = new_p.y();
      transform_stamped.transform.translation.z = new_p.z();
      transform_stamped.transform.rotation.w = new_q.w();
      transform_stamped.transform.rotation.x = new_q.x();
      transform_stamped.transform.rotation.y = new_q.y();
      transform_stamped.transform.rotation.z = new_q.z();
      this->tf_broadcaster->sendTransform(transform_stamped);
    }
  }

  // Note: Pose publishing is done from publishPose() at GICP rate only
  // With unreliable IMU data, geometric observer propagation is not accurate
  // Better to publish only GICP-corrected poses at ~15 Hz than poorly-propagated poses at 100 Hz

  // Debug: Count published messages
  static int odom_publish_count = 0;
  static auto last_report_time = std::chrono::steady_clock::now();
  odom_publish_count++;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time).count();
  if (elapsed >= 1000) {  // Report every second
    RCLCPP_INFO(this->get_logger(), "Odometry publish rate: %d Hz", odom_publish_count);
    odom_publish_count = 0;
    last_report_time = now;
  }

  // Update state AFTER publishing to avoid race condition
  {
    std::lock_guard<std::mutex> lock(this->geo.mtx);
    this->state.p = new_p;
    this->state.q = new_q;
    this->state.v.lin.w = new_v_lin_w;
    this->state.v.lin.b = new_q.toRotationMatrix().inverse() * new_v_lin_w;
    this->state.v.ang.b = new_v_ang_b;
    this->state.v.ang.w = new_v_ang_w;
  }

  if (this->imu_only_mode_) {
    std::lock_guard<std::mutex> lock(this->pose_mutex);
    this->current_pose.setIdentity();
    this->current_pose.block<3, 3>(0, 0) = new_q.toRotationMatrix();
    this->current_pose.block<3, 1>(0, 3) = new_p;
  }

  // Don't publish TF from propagated state - only from GICP-corrected pose in publishPose()
  // High-frequency TF from IMU propagation drifts between GICP corrections
  // TF publishing is handled in publishPose() at GICP rate (15 Hz) with corrected pose

}

void gicp_localization::LocalizationNode::updateState() {

  // Lock thread to prevent state from being accessed by propagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->lidarPose.p;
  Eigen::Quaternionf qin = this->lidarPose.q;
  double dt = this->scan_stamp.seconds() - this->prev_scan_stamp;

  // On very first update after initialization, dt might be large
  // Just skip the update but don't warn
  if (dt <= 0.0) {
    return;  // Skip invalid dt
  }

  if (dt > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Large dt in updateState: %.3f sec, skipping update", dt);
    return;  // Skip if dt is too large (probably first update or dropped scans)
  }

  // Validate inputs
  bool inputs_valid = std::isfinite(pin.x()) && std::isfinite(pin.y()) && std::isfinite(pin.z()) &&
                      std::isfinite(qin.w()) && std::isfinite(qin.x()) && std::isfinite(qin.y()) && std::isfinite(qin.z()) &&
                      std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) && std::isfinite(this->state.p.z()) &&
                      std::isfinite(this->state.q.w()) && std::isfinite(this->state.q.x()) &&
                      std::isfinite(this->state.q.y()) && std::isfinite(this->state.q.z());

  if (!inputs_valid) {
    RCLCPP_WARN(this->get_logger(), "Invalid inputs in updateState - pin=[%.3f,%.3f,%.3f] state.p=[%.3f,%.3f,%.3f]",
                pin.x(), pin.y(), pin.z(), this->state.p.x(), this->state.p.y(), this->state.p.z());
    return;
  }

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Construct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.0;
  if (qe.w() < 0) {
    sgn = -1.0;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - fabs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  // Position error
  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel = this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro = this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // For localization: directly snap position and orientation to GICP measurement
  // The gradual correction gains (Kp, Kq) are too slow for 30 Hz GICP updates
  // Instead, trust GICP when it converges and directly update state
  this->state.p = pin;
  this->state.q = qin;

  // Estimate velocity from GICP-to-GICP displacement (not from IMU correction error).
  // geo.prev_p holds the previous GICP result, so this gives true vehicle velocity.
  // Using err/dt would include IMU drift in the velocity, causing oscillating T_prior.
  if (dt > 0.001) {
    this->state.v.lin.w = (pin - this->geo.prev_p) / dt;
  }

  // Validate updated state
  bool state_valid_after = std::isfinite(this->state.p.x()) && std::isfinite(this->state.p.y()) &&
                           std::isfinite(this->state.p.z()) && std::isfinite(this->state.q.w()) &&
                           std::isfinite(this->state.v.lin.w.x()) && std::isfinite(this->state.v.lin.w.y()) &&
                           std::isfinite(this->state.v.lin.w.z());

  if (!state_valid_after) {
    RCLCPP_ERROR(this->get_logger(), "State became invalid after update! Resetting to GICP measurement.");
    // Reset to valid GICP measurement
    this->state.p = pin;
    this->state.q = qin;
    this->state.v.lin.w = Eigen::Vector3f::Zero();
    this->state.b.accel = Eigen::Vector3f::Zero();
    this->state.b.gyro = Eigen::Vector3f::Zero();
  }

  // Store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

  // Log update status periodically
  static int update_count = 0;
  if (++update_count % 20 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Geo Observer | pos_err=[%.3f,%.3f,%.3f]m vel=[%.2f,%.2f,%.2f]m/s | bias_gyro=[%.4f,%.4f,%.4f] bias_accel=[%.3f,%.3f,%.3f]",
                err.x(), err.y(), err.z(),
                this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
                this->state.b.gyro.x(), this->state.b.gyro.y(), this->state.b.gyro.z(),
                this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());
  }

  RCLCPP_DEBUG(this->get_logger(),
               "Geo Observer: pos_err=[%.3f,%.3f,%.3f] vel=[%.2f,%.2f,%.2f] bias_a=[%.3f,%.3f,%.3f]",
               err.x(), err.y(), err.z(),
               this->state.v.lin.w.x(), this->state.v.lin.w.y(), this->state.v.lin.w.z(),
               this->state.b.accel.x(), this->state.b.accel.y(), this->state.b.accel.z());

}
