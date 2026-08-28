#include "vantage/field.hpp"

#include <stdexcept>

namespace vantage {
namespace {

void validateField(const FieldDimensions& field) {
  if (!(field.width > 0.0 && field.height > 0.0)) {
    throw std::invalid_argument("field dimensions must be > 0");
  }
}

}  // namespace

Pose2d cornerToVexGps(const Pose2d& pose, FieldDimensions field) {
  validateField(field);
  return {pose.x - field.width * 0.5,
          pose.y - field.height * 0.5,
          wrapAngle(kPi * 0.5 - pose.theta)};
}

Pose2d vexGpsToCorner(const Pose2d& pose, FieldDimensions field) {
  validateField(field);
  return {pose.x + field.width * 0.5,
          pose.y + field.height * 0.5,
          wrapAngle(kPi * 0.5 - pose.theta)};
}

Pose2d mirrorPose(const Pose2d& pose, FieldMirror mirror,
                  FieldDimensions field) {
  validateField(field);
  switch (mirror) {
    case FieldMirror::kLeftRight:
      return {field.width - pose.x, pose.y,
              wrapAngle(kPi - pose.theta)};
    case FieldMirror::kBottomTop:
      return {pose.x, field.height - pose.y, wrapAngle(-pose.theta)};
    case FieldMirror::kAlliance180:
      return {field.width - pose.x, field.height - pose.y,
              wrapAngle(pose.theta + kPi)};
  }
  return pose;
}

std::vector<Waypoint> mirrorWaypoints(
    const std::vector<Waypoint>& waypoints, FieldMirror mirror,
    FieldDimensions field) {
  std::vector<Waypoint> result = waypoints;
  for (Waypoint& waypoint : result) {
    waypoint.pose = mirrorPose(waypoint.pose, mirror, field);
  }
  return result;
}

std::vector<Waypoint> reverseWaypoints(
    const std::vector<Waypoint>& waypoints) {
  std::vector<Waypoint> result(waypoints.rbegin(), waypoints.rend());
  for (Waypoint& waypoint : result) {
    waypoint.pose.theta = wrapAngle(waypoint.pose.theta + kPi);
  }
  return result;
}

}  // namespace vantage
