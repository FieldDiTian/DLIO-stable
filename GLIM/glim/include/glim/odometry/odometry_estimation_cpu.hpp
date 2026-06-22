#pragma once

#include <glim/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {

class GaussianVoxelMapCPU;
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  virtual ~OdometryEstimationCPUParams();

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP or VGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levesl (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)

  bool enable_registration_guard;              ///< Reject frame-to-model registrations that are inconsistent with prediction
  double max_registration_correction_trans;    ///< Max allowed correction from IMU prediction [m] (<=0 disables)
  double max_registration_correction_rot;      ///< Max allowed correction from IMU prediction [rad] (<=0 disables)
  double min_registration_overlap;             ///< Min accepted frame-to-model overlap for VGICP (<=0 disables)
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU odometry
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;

  virtual void fallback_smoother() override;
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors) override;

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);
  void reset_target();
  void rebuild_target_from_active_frames();

private:
  // Registration params
  std::mt19937 mt;                                                                   ///< RNG
  Eigen::Isometry3d last_T_target_imu;                                               ///< Last IMU pose w.r.t. target model
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_points::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                       ///< Target points (just for visualization)
  bool pending_target_update;                                                        ///< True if the current frame should be inserted after smoother success
  int pending_target_frame;                                                          ///< Frame id associated with pending target update
};

}  // namespace glim
