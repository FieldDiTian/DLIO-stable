#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

// Forward-declare libpcap so we can keep <pcap.h> out of the header.
struct pcap;
typedef struct pcap pcap_t;

namespace glim_ros {

struct IrisLidarConfig {
  std::string ip;
  std::string topic;
  std::string frame_id;
  uint16_t dst_port = 0;
};

struct IrisPcapConfig {
  uint16_t iris_data_src_port = 4371;
  double inactivity_sec = 0.05;
  uint64_t scan_reorder_lookahead_ns = 200'000'000ULL;
  std::vector<IrisLidarConfig> lidars;
};

// One assembled scan, with timestamps still in the original PTP epoch.
// The executable applies the PTP -> ROS shift before dispatch.
struct AssembledScan {
  uint64_t t_ns = 0;                        // header time in PTP epoch ns (first ray's base ns)
  uint64_t wall_clock_first_packet_ns = 0;  // wall-clock epoch ns of the FIRST packet for this scan.
                                            // Use this as the anchor for the PTP -> wall shift so the
                                            // shift = first_wall - first_ptp doesn't bake the scan
                                            // duration (~50ms) into the timeline.
  uint64_t wall_clock_last_packet_ns = 0;   // wall-clock epoch ns of the last packet (kept for diagnostics).
  std::string topic;                        // configured topic for this lidar
  std::string frame_id;                     // configured frame_id for this lidar
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
};

class IrisPcapReader {
public:
  IrisPcapReader();
  ~IrisPcapReader();

  // Open a classic .pcap file. Throws std::runtime_error on:
  //   - file not found / unreadable
  //   - pcapng input (use editcap -F pcap to convert)
  //   - unsupported magic / link type
  void open(const std::string& pcap_path, const IrisPcapConfig& config);

  // Returns the wall-clock epoch ns of the first packet record header.
  // Exact O(1): single record header read at offset sizeof(pcap_file_header).
  uint64_t peek_first_pcap_epoch_ns() const { return first_packet_epoch_ns_; }

  // Returns UINT64_MAX. The reader does not pre-scan to the last record —
  // for multi-GB pcaps that pass would cost tens of seconds at startup. The
  // executable should treat the pcap as unbounded-end and rely on the bag
  // metadata (+ inactivity_sec tail) for the upper window bound.
  uint64_t peek_last_pcap_epoch_ns() const { return last_packet_epoch_ns_; }

  // Source-side packet-time filter for the iteration pass. Packets whose
  // wall-clock ns is outside [lo_ns, hi_ns] are dropped before parsing.
  void set_window(uint64_t lo_ns, uint64_t hi_ns);

  // Pull the next assembled scan, applying the inactivity timeout and the
  // 200ms reorder heap. Returns std::nullopt at EOF (after flush_all has
  // already been drained).
  std::optional<AssembledScan> next();

  // Drain remaining open scan accumulators. Call once at PCAP EOF.
  // After this call, next() will return std::nullopt.
  std::vector<AssembledScan> flush_all();

private:
  void close();
  bool feed_next_packet();           // pulls one UDP payload, feeds assembler
  void emit_completed(std::vector<AssembledScan>&& scans);

  // libpcap state
  pcap_t* handle_ = nullptr;
  int link_type_ = 0;                 // DLT_*
  int link_header_len_ = 0;           // for fixed-length link types

  // Pre-scan results
  uint64_t first_packet_epoch_ns_ = 0;
  uint64_t last_packet_epoch_ns_ = 0;

  // Time window
  uint64_t window_lo_ns_ = 0;
  uint64_t window_hi_ns_ = std::numeric_limits<uint64_t>::max();

  // Config
  IrisPcapConfig config_;
  // ip -> (topic, frame_id, dst_port)
  std::unordered_map<std::string, IrisLidarConfig> lidar_by_ip_;
  // (ip|dst_port) one-shot warn dedup
  std::unordered_map<std::string, bool> warned_unknown_;

  // Output buffer: scans completed but held by the reorder lookahead.
  // pop_front when (head.t_ns + scan_reorder_lookahead_ns) <= newest_seen_t_ns_.
  std::deque<AssembledScan> ready_;
  uint64_t newest_completed_t_ns_ = 0;
  bool eof_ = false;

  // Forward-declared assembler (impl in .cpp).
  struct AssemblerImpl;
  std::unique_ptr<AssemblerImpl> assembler_;
};

}  // namespace glim_ros
