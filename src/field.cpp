#include "vantage/field.hpp"

#include <stdexcept>
#include <algorithm>

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
    for (auto& control : waypoint.controlPoints) control = mirrorPose(control, mirror, field);
  }
  return result;
}

std::vector<Waypoint> reverseWaypoints(
    const std::vector<Waypoint>& waypoints) {
  std::vector<Waypoint> result(waypoints.rbegin(), waypoints.rend());
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i].pose.theta = wrapAngle(result[i].pose.theta + kPi);
    result[i].bezierToNext = i + 1 < result.size() && waypoints[waypoints.size()-2-i].bezierToNext;
    result[i].controlPoints = i + 1 < result.size() ? waypoints[waypoints.size()-2-i].controlPoints : std::vector<Pose2d>{};
    std::reverse(result[i].controlPoints.begin(), result[i].controlPoints.end());
  }
  return result;
}

}  // namespace vantage
