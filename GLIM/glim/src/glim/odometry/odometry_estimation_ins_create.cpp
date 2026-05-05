#include <glim/odometry/odometry_estimation_ins.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationINSParams params;
  return new glim::OdometryEstimationINS(params);
}
