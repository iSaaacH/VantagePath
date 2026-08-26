#include "vantage/odometry.hpp"

#include <cmath>

namespace vantage {

DifferentialDriveOdometry::DifferentialDriveOdometry(Pose2d initialPose)
    : pose_(initialPose), previousHeading_(initialPose.theta) {}

Pose2d DifferentialDriveOdometry::update(double leftDistance,
                                         double rightDistance,
                                         double gyroHeading) {
  if (!initialized_) {
    reset(pose_, leftDistance, rightDistance, gyroHeading);
    return pose_;
  }
  const double dl = leftDistance - previousLeft_;
  const double dr = rightDistance - previousRight_;
  const double ds = (dl + dr) * 0.5;
  const double heading = gyroHeading + gyroOffset_;
  const double dtheta = wrapAngle(heading - previousHeading_);
  const double localX = ds * sinc(dtheta);
  const double localY = ds * (std::abs(dtheta) < 1e-6
                                  ? dtheta * 0.5 : (1.0 - std::cos(dtheta)) / dtheta);
  const double c = std::cos(previousHeading_);
  const double s = std::sin(previousHeading_);
  pose_.x += c * localX - s * localY;
  pose_.y += s * localX + c * localY;
  pose_.theta = wrapAngle(heading);
  previousLeft_ = leftDistance;
  previousRight_ = rightDistance;
  previousHeading_ = heading;
  return pose_;
}

void DifferentialDriveOdometry::reset(Pose2d pose, double leftDistance,
                                      double rightDistance,
                                      double gyroHeading) {
  pose_ = pose;
  previousLeft_ = leftDistance;
  previousRight_ = rightDistance;
  gyroOffset_ = wrapAngle(pose.theta - gyroHeading);
  previousHeading_ = pose.theta;
  initialized_ = true;
}

}  // namespace vantage
