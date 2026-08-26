#pragma once

#include "vantage/geometry.hpp"

namespace vantage {

// Differential-drive SE(2) odometry. Distances and track width may use any
// consistent unit; heading is continuous radians and may come from fused IMUs.
class DifferentialDriveOdometry {
 public:
  explicit DifferentialDriveOdometry(Pose2d initialPose = {});
  Pose2d update(double leftDistance, double rightDistance,
                double gyroHeading);
  void reset(Pose2d pose, double leftDistance, double rightDistance,
             double gyroHeading);
  const Pose2d& pose() const { return pose_; }

 private:
  Pose2d pose_;
  double previousLeft_ = 0.0;
  double previousRight_ = 0.0;
  double gyroOffset_ = 0.0;
  double previousHeading_ = 0.0;
  bool initialized_ = false;
};

}  // namespace vantage
