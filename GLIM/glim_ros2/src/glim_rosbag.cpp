#include <glob.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iostream>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/urdf_transforms.hpp>

// ============================================================================
// Multi-LiDAR concatenation support
// ============================================================================

struct AuxLidarSensor {
  std::string topic;
  Eigen::Isometry3d T_primary_sensor;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> buffer;
  size_t buffer_size;
};

static double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

static bool find_xyz_offsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

static void transform_cloud_data(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int x_off,
  int y_off,
  int z_off,
  const Eigen::Isometry3d& T) {
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  const size_t num_points = data.size() / point_step;

  for (size_t i = 0; i < num_points; i++) {
    const size_t base = i * point_step;
    float x, y, z;
    std::memcpy(&x, &data[base + x_off], sizeof(float));
    std::memcpy(&y, &data[base + y_off], sizeof(float));
    std::memcpy(&z, &data[base + z_off], sizeof(float));

    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(&data[base + x_off], &p.x(), sizeof(float));
    std::memcpy(&data[base + y_off], &p.y(), sizeof(float));
    std::memcpy(&data[base + z_off], &p.z(), sizeof(float));
  }
}

static sensor_msgs::msg::PointCloud2::SharedPtr find_nearest(
  const std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>& buffer,
  double target_sec,
  double threshold) {
  sensor_msgs::msg::PointCloud2::SharedPtr best;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto& msg : buffer) {
    double dt = std::abs(stamp_to_sec(msg->header.stamp) - target_sec);
    if (dt < best_dt) {
      best_dt = dt;
      best = msg;
    }
  }
  return (best && best_dt <= threshold) ? best : nullptr;
}

/// Find per-point time field offset and datatype in a PointCloud2 message.
/// Returns true if a time field was found.
static bool find_time_field(const sensor_msgs::msg::PointCloud2& msg, int& time_off, uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = f.offset;
      time_datatype = f.datatype;
      time_count = f.count;
      return true;
    }
  }
  return false;
}

/// Shift per-point timestamps in raw cloud data by dt seconds.
/// dt = t_aux_header - t_primary_header, so aux points get rebased to primary's timebase.
static void shift_cloud_timestamps(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int time_off,
  uint8_t time_datatype,
  int time_count,
  double dt) {
  if (time_off < 0) return;

  const size_t num_points = data.size() / point_step;
  for (size_t i = 0; i < num_points; i++) {
    uint8_t* time_ptr = &data[i * point_step + time_off];
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
        val = static_cast<uint32_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint32_t));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float val;
        std::memcpy(&val, time_ptr, sizeof(float));
        val += static_cast<float>(dt);
        std::memcpy(time_ptr, &val, sizeof(float));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double val;
        std::memcpy(&val, time_ptr, sizeof(double));
        val += dt;
        std::memcpy(time_ptr, &val, sizeof(double));
        break;
      }
      case sensor_msgs::msg::PointField::UINT8: {
        if (time_count == 8) {
          uint64_t val;
          std::memcpy(&val, time_ptr, sizeof(uint64_t));
          int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
          val = static_cast<uint64_t>(std::max<int64_t>(0, shifted));
          std::memcpy(time_ptr, &val, sizeof(uint64_t));
        }
        break;
      }
      default:
        break;
    }
  }
}

static sensor_msgs::msg::PointCloud2::ConstSharedPtr merge_clouds(
  const sensor_msgs::msg::PointCloud2::SharedPtr& primary,
  std::vector<AuxLidarSensor>& aux_sensors,
  double time_threshold) {
  const double t_primary = stamp_to_sec(primary->header.stamp);
  const uint32_t point_step = primary->point_step;

  int x_off, y_off, z_off;
  if (!find_xyz_offsets(*primary, x_off, y_off, z_off)) {
    spdlog::warn("lidar_concat: cannot find xyz fields in primary cloud");
    return primary;
  }

  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  size_t total_points = primary->width * primary->height;

  for (auto& aux : aux_sensors) {
    auto match = find_nearest(aux.buffer, t_primary, time_threshold);
    if (!match) {
      spdlog::debug("lidar_concat: no match for {} (t={:.3f})", aux.topic, t_primary);
      continue;
    }
    if (match->point_step != point_step) {
      spdlog::warn("lidar_concat: point_step mismatch for {} ({} vs {})", aux.topic, match->point_step, point_step);
      continue;
    }

    std::vector<uint8_t> data(match->data.begin(), match->data.end());
    int ax, ay, az;
    if (find_xyz_offsets(*match, ax, ay, az)) {
      transform_cloud_data(data, point_step, ax, ay, az, aux.T_primary_sensor);
    }

    // Rebase per-point timestamps from aux clock to primary clock.
    // dt is the header time difference: points in the aux cloud need their
    // timestamps shifted so they are relative to the primary scan's timebase.
    int time_off;
    uint8_t time_datatype;
    int time_count;
    if (find_time_field(*match, time_off, time_datatype, time_count)) {
      double dt = stamp_to_sec(match->header.stamp) - t_primary;
      shift_cloud_timestamps(data, point_step, time_off, time_datatype, time_count, dt);
      spdlog::debug("lidar_concat: shifted timestamps for {} by {:.6f}s", aux.topic, dt);
    }

    merged->data.insert(merged->data.end(), data.begin(), data.end());
    total_points += match->width * match->height;

    double dt = std::abs(stamp_to_sec(match->header.stamp) - t_primary);
    spdlog::debug("lidar_concat: merged {} (dt={:.4f}s, {} pts)", aux.topic, dt, match->width * match->height);
  }

  merged->width = total_points;
  merged->height = 1;
  merged->row_step = point_step * total_points;
  return merged;
}

class SpeedCounter {
public:
  SpeedCounter() : last_sim_time(0.0), last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const double& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(5)) {
      return;
    }

    if (last_sim_time > 0.0) {
      const auto real = now - last_real_time;
      const auto sim = stamp - last_sim_time;
      const double playback_speed = sim / (std::chrono::duration_cast<std::chrono::nanoseconds>(real).count() / 1e9);
      spdlog::info("playback speed: {:.3f}x", playback_speed);
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  double last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

class KeyboardHandler {
public:
  KeyboardHandler() : paused_(false), active_(false) {
    if (isatty(STDIN_FILENO)) {
      tcgetattr(STDIN_FILENO, &original_termios_);
      struct termios raw = original_termios_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &raw);
      active_ = true;
    }
  }

  ~KeyboardHandler() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }
  }

  void update() {
    if (!active_) return;
    char c;
    while (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == ' ') {
        paused_ = !paused_;
        if (paused_) {
          spdlog::info("playback paused (press space to resume)");
        } else {
          spdlog::info("playback resumed");
        }
      }
    }
  }

  bool is_paused() const { return paused_; }

private:
  bool paused_;
  bool active_;
  struct termios original_termios_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // List topics
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "/image");
  std::vector<std::string> topics = {imu_topic, points_topic, image_topic};

  // Load multi-LiDAR concatenation config
  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  const bool concat_enabled = config_sensors.param<bool>("lidar_concat", "enabled", false);
  double concat_time_threshold = config_sensors.param<double>("lidar_concat", "time_threshold", 0.05);
  const int concat_buffer_size = config_sensors.param<int>("lidar_concat", "buffer_size", 200);
  std::vector<AuxLidarSensor> aux_sensors;

  if (concat_enabled) {
    const auto aux_topics = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_topics", {});

    // Try to load transforms from URDF if configured
    const std::string urdf_path = config_sensors.param<std::string>("lidar_concat", "urdf_path", "");
    const std::string primary_frame = config_sensors.param<std::string>("lidar_concat", "primary_frame", "");
    std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
    bool use_urdf = !urdf_path.empty() && !primary_frame.empty();

    if (use_urdf) {
      try {
        urdf_transforms = glim::parse_urdf_transforms(urdf_path);
        spdlog::info("lidar_concat: loaded URDF from {} (primary_frame={})", urdf_path, primary_frame);
      } catch (const std::exception& e) {
        spdlog::error("lidar_concat: failed to parse URDF: {}", e.what());
        use_urdf = false;
      }
    }

    const auto aux_frames = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_frames", {});
    if (use_urdf && aux_frames.size() != aux_topics.size()) {
      spdlog::error("lidar_concat: aux_frames size ({}) must match aux_topics size ({})", aux_frames.size(), aux_topics.size());
      use_urdf = false;
    }

    for (size_t i = 0; i < aux_topics.size(); i++) {
      const auto& topic = aux_topics[i];
      AuxLidarSensor sensor;
      sensor.topic = topic;
      sensor.buffer_size = concat_buffer_size;

      if (use_urdf) {
        const std::string& aux_frame = aux_frames[i];
        try {
          sensor.T_primary_sensor = glim::compute_transform(urdf_transforms, primary_frame, aux_frame);
          std::stringstream ss;
          ss << sensor.T_primary_sensor.matrix();
          spdlog::info("lidar_concat: T_{}_{}:\n{}", primary_frame, aux_frame, ss.str());
        } catch (const std::exception& e) {
          spdlog::error("lidar_concat: failed to compute transform {} -> {}: {}", primary_frame, aux_frame, e.what());
          continue;
        }
      } else {
        // Fall back to reading the 4x4 matrix from config
        std::string key = topic;
        for (auto& c : key) {
          if (c == '/') c = '_';
        }
        if (!key.empty() && key[0] == '_') key = key.substr(1);
        key = "T_primary_" + key;

        auto flat = config_sensors.param<std::vector<double>>("lidar_concat", key);
        if (!flat || flat->size() != 16) {
          spdlog::error("lidar_concat: missing or invalid transform '{}' for topic '{}'", key, topic);
          continue;
        }

        Eigen::Matrix4d mat;
        for (int r = 0; r < 4; r++)
          for (int c = 0; c < 4; c++)
            mat(r, c) = (*flat)[r * 4 + c];
        sensor.T_primary_sensor = Eigen::Isometry3d(mat);
      }

      topics.push_back(sensor.topic);
      spdlog::info("lidar_concat: auxiliary sensor {} enabled", sensor.topic);
      aux_sensors.push_back(std::move(sensor));
    }
    spdlog::info("lidar_concat: {} auxiliary sensors, threshold={:.3f}s", aux_sensors.size(), concat_time_threshold);
  }

  rosbag2_storage::StorageFilter filter;
  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
    filter.topics.push_back(topic);
  }

  //
  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim->extension_subscriptions()) {
    spdlog::info("- {} (ext)", sub->topic);
    filter.topics.push_back(sub->topic);
    subscription_map[sub->topic].push_back(sub);
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  // Playback range settings
  double delay = 0.0;
  glim->declare_parameter<double>("delay", delay);
  glim->get_parameter<double>("delay", delay);

  double start_offset = 0.0;
  glim->declare_parameter<double>("start_offset", start_offset);
  glim->get_parameter<double>("start_offset", start_offset);

  double playback_duration = 0.0;
  glim->declare_parameter<double>("playback_duration", playback_duration);
  glim->get_parameter<double>("playback_duration", playback_duration);

  double playback_until = 0.0;
  glim->declare_parameter<double>("playback_until", playback_until);
  glim->get_parameter<double>("playback_until", playback_until);

  // Playback speed settings
  const double playback_speed = config_ros.param<double>("glim_ros", "playback_speed", 100.0);
  std::chrono::system_clock::time_point real_t0;
  rcutils_time_point_value_t bag_t0 = 0;
  SpeedCounter speed_counter;

  double end_time = std::numeric_limits<double>::max();
  glim->declare_parameter<double>("end_time", end_time);
  glim->get_parameter<double>("end_time", end_time);

  if (delay > 0.0) {
    spdlog::info("delaying {} sec", delay);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
  }

  // Keyboard handler for pause/resume
  KeyboardHandler keyboard;

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename) {
    spdlog::info("opening {}", bag_filename);
    rosbag2_storage::StorageOptions options;
    options.uri = bag_filename;

    bool is_mcap = bag_filename.size() > 5 && bag_filename.rfind(".mcap") == (bag_filename.size() - 5);
    if (is_mcap) {
      options.storage_id = "mcap";
    } else if (std::filesystem::is_directory(bag_filename)) {
      try {
        rosbag2_storage::MetadataIo metadata_io;
        const auto metadata = metadata_io.read_metadata(bag_filename);
        options.storage_id = metadata.storage_identifier;

        if (options.storage_id.empty()) {
          spdlog::warn("storage_identifier not found in metadata.yaml (uri={}), fallback to sqlite3", bag_filename);
          options.storage_id = "sqlite3";
        } else {
          spdlog::info("detected storage_id={} from metadata.yaml", options.storage_id);
        }
      } catch (const std::exception& e) {
        spdlog::warn("failed to read metadata.yaml (uri={}): {} (fallback to sqlite3)", bag_filename, e.what());
        options.storage_id = "sqlite3";
      }
    } else {
      options.storage_id = "sqlite3";
    }

    rosbag2_cpp::ConverterOptions converter_options;

    // rosbag2_cpp::Reader reader;
    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_;
    reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    reader_->open(options, converter_options);

    if (reader_->get_metadata().compression_format != "") {
      spdlog::info("compression detected (format={})", reader_->get_metadata().compression_format);
      spdlog::info("opening bag with SequentialCompressionReader");
      reader_ = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
      reader_->open(options, converter_options);
    }

    auto& reader = *reader_;
    reader.set_filter(filter);

    const auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto& topic : topics_and_types) {
      topic_type_map[topic.name] = topic.type;
    }

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> points_serialization;
#ifdef BUILD_WITH_CV_BRIDGE
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;
#endif

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        return false;
      }
      rclcpp::spin_some(glim);

      const auto msg = reader.read_next();
      const std::string topic_type = topic_type_map[msg->topic_name];
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (real_t0.time_since_epoch().count() == 0) {
        real_t0 = std::chrono::system_clock::now();
      }

      const auto msg_time = get_msg_recv_timestamp(*msg);
      if (bag_t0 == 0) {
        bag_t0 = msg_time;
      }
      spdlog::debug("msg_time: {} ({} sec)", msg_time / 1e9, (msg_time - bag_t0) / 1e9);

      if (start_offset > 0.0) {
        spdlog::info("skipping msg for start_offset {}", start_offset);
        reader.seek(bag_t0 + start_offset * 1e9);

        start_offset = 0.0;
        bag_t0 = 0;
        real_t0 = std::chrono::system_clock::from_time_t(0);
        continue;
      }

      if (playback_until > 0.0 && msg_time / 1e9 > playback_until) {
        spdlog::info("reached playback_until ({} < {})", msg_time / 1e9, playback_until);
        return false;
      }

      if (playback_duration > 0.0 && (msg_time - bag_t0) / 1e9 > playback_duration) {
        spdlog::info("reached playback_duration ({} > {})", (msg_time - bag_t0) / 1e9, playback_duration);
        return false;
      }

      // Pause/resume handling
      keyboard.update();
      if (keyboard.is_paused()) {
        auto pause_start = std::chrono::system_clock::now();
        while (keyboard.is_paused() && rclcpp::ok()) {
          rclcpp::spin_some(glim);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          keyboard.update();
        }
        if (!rclcpp::ok()) {
          return false;
        }
        // Adjust real_t0 to account for pause duration to avoid fast-forward
        real_t0 += std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::system_clock::now() - pause_start);
      }

      const auto bag_elapsed = std::chrono::nanoseconds(msg_time - bag_t0);
      while (playback_speed > 0.0 && (std::chrono::system_clock::now() - real_t0) * playback_speed < bag_elapsed) {
        const double real_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - real_t0).count() / 1e9;
        spdlog::debug("throttling (real_elapsed={} bag_elapsed={} playback_speed={})", real_elapsed, bag_elapsed.count() / 1e9, playback_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // Check if this message is for an auxiliary LiDAR sensor
      bool is_aux_sensor = false;
      if (concat_enabled) {
        for (auto& aux : aux_sensors) {
          if (msg->topic_name == aux.topic) {
            if (topic_type != "sensor_msgs/msg/PointCloud2") {
              spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/PointCloud2 (topic={})", topic_type, msg->topic_name);
              return false;
            }
            auto aux_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            points_serialization.deserialize_message(&serialized_msg, aux_msg.get());
            aux.buffer.push_back(aux_msg);
            while (aux.buffer.size() > aux.buffer_size) {
              aux.buffer.pop_front();
            }
            is_aux_sensor = true;
            break;
          }
        }
      }

      if (is_aux_sensor) {
        // Already handled above; skip to next message
      } else if (msg->topic_name == imu_topic) {
        if (topic_type != "sensor_msgs/msg/Imu") {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/Imu (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        glim->imu_callback(imu_msg);
      } else if (msg->topic_name == points_topic) {
        if (topic_type != "sensor_msgs/msg/PointCloud2") {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/PointCloud2 (topic={})", topic_type, msg->topic_name);
          return false;
        }
        auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        points_serialization.deserialize_message(&serialized_msg, points_msg.get());

        // Merge auxiliary LiDAR clouds if concatenation is enabled
        sensor_msgs::msg::PointCloud2::ConstSharedPtr final_points = points_msg;
        if (concat_enabled && !aux_sensors.empty()) {
          final_points = merge_clouds(points_msg, aux_sensors, concat_time_threshold);
        }
        const size_t workload = glim->points_callback(final_points);

        if (points_msg->header.stamp.sec + points_msg->header.stamp.nanosec * 1e-9 > end_time) {
          spdlog::info("end_time reached");
          return false;
        }

        if (workload > 5) {
          // Odometry estimation is behind
          const size_t sleep_msec = (workload - 4) * 5;
          spdlog::debug("throttling: {} msec (workload={})", sleep_msec, workload);
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_msec));
        }
      }
#ifdef BUILD_WITH_CV_BRIDGE
      else if (msg->topic_name == image_topic) {
        if (topic_type == "sensor_msgs/msg/Image") {
          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          image_serialization.deserialize_message(&serialized_msg, image_msg.get());
          glim->image_callback(image_msg);
        } else if (topic_type == "sensor_msgs/msg/CompressedImage") {
          auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
          compressed_image_serialization.deserialize_message(&serialized_msg, compressed_image_msg.get());

          auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
          cv_bridge::toCvCopy(*compressed_image_msg, "bgr8")->toImageMsg(*image_msg);
          glim->image_callback(image_msg);
        } else {
          spdlog::error("topic_type mismatch: {} != sensor_msgs/msg/(Image|CompressedImage) (topic={})", topic_type, msg->topic_name);
          return false;
        }
      }
#endif

      auto found = subscription_map.find(msg->topic_name);
      if (found != subscription_map.end()) {
        for (const auto& sub : found->second) {
          sub->insert_message_instance(serialized_msg, topic_type);
        }
      }

      glim->timer_callback();
      speed_counter.update(msg_time / 1e9);

      const auto t0 = std::chrono::high_resolution_clock::now();
      while (glim->needs_wait()) {
        rclcpp::spin_some(glim);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        spdlog::debug("throttling (waiting for odometry estimation)");
        if (std::chrono::high_resolution_clock::now() - t0 > std::chrono::seconds(1)) {
          spdlog::warn("throttling timeout (an extension module may be hanged)");
          break;
        }
      }
    }

    return true;
  };

  // Read all rosbags
  bool auto_quit = false;
  glim->declare_parameter<bool>("auto_quit", auto_quit);
  glim->get_parameter<bool>("auto_quit", auto_quit);

  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      auto_quit = true;
      break;
    }
  }

  if (!auto_quit) {
    rclcpp::spin(glim);
  }

  glim->wait(auto_quit);
  glim->save(dump_path);

  return 0;
}
