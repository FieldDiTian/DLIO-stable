#pragma once

#include <any>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

#include <glim_ros/lidar_concat.hpp>

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void imu_callback_live(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void external_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void external_odom_callback_live(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Live subscription entry point for the primary LiDAR. Merges any buffered
  // auxiliary clouds into the primary and then forwards the result to
  // points_callback().
  void points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Buffers an auxiliary LiDAR cloud for later time-matched merging.
  void aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  enum class LiveInputType { IMU, POINTS, AUX_POINTS, EXTERNAL_ODOM };

  struct LiveInputEvent {
    double stamp = 0.0;
    uint64_t seq = 0;
    LiveInputType type = LiveInputType::IMU;
    size_t aux_index = 0;
    sensor_msgs::msg::Imu::SharedPtr imu;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr points;
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
  };

  void enqueue_live_input(LiveInputEvent&& event);
  void live_input_loop();
  bool pop_live_input(LiveInputEvent& event, bool drain_all);
  void dispatch_live_input(const LiveInputEvent& event);
  void stop_live_input(bool drain_all);
  void process_points_callback_live(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void process_aux_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, size_t aux_index);

  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool dump_on_unload;
  bool deterministic_live_input;
  double live_input_reorder_window_sec;
  size_t live_input_max_queue_size;

  std::string intensity_field, ring_field;
  bool flip_points_y;

  std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> flush_ready_live_points_locked();
  bool aux_watermarks_cover_primary_locked(double primary_stamp) const;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  std::mutex live_input_mutex;
  std::condition_variable live_input_cv;
  std::vector<LiveInputEvent> live_input_queue;
  std::thread live_input_thread;
  std::atomic_bool live_input_stop;
  double live_input_latest_stamp;
  uint64_t live_input_next_seq;

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odom_sub;

  // Multi-LiDAR concatenation (primary luminar_front + auxiliary left/right).
  // Live ROS callback order is not guaranteed to follow sensor timestamps. Keep
  // primary scans queued until every auxiliary stream's timestamp watermark has
  // passed the primary matching window, then merge with the same time threshold
  // used by offline glim_rosbag / glim_pcap_rosbag.
  glim_ros::AuxConcatConfig aux_concat;
  std::mutex aux_buffers_mutex;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> primary_points_buffer;
  std::vector<double> aux_latest_stamps;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> aux_points_subs;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
