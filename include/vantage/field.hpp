#pragma once

#include <vector>

#include "vantage/geometry.hpp"
#include "vantage/trajectory.hpp"

namespace vantage {

struct FieldDimensions {
  double width = 144.0;
  double height = 144.0;
};

enum class FieldMirror {
  kLeftRight,
  kBottomTop,
  kAlliance180,
};

// The authoring frame uses the bottom-left inside corner as (0, 0), +X right,
// +Y up, and a mathematical heading (+X zero, counter-clockwise). Official
// VEX GPS uses the field centre as (0, 0) and a compass heading (+Y zero,
// clockwise). Linear units are preserved by these conversions.
Pose2d cornerToVexGps(const Pose2d& pose, FieldDimensions field = {});
Pose2d vexGpsToCorner(const Pose2d& pose, FieldDimensions field = {});

Pose2d mirrorPose(const Pose2d& pose, FieldMirror mirror,
                  FieldDimensions field = {});
std::vector<Waypoint> mirrorWaypoints(
    const std::vector<Waypoint>& waypoints, FieldMirror mirror,
    FieldDimensions field = {});

// Reorders a geometric path while preserving its shape. This differs from
// TrajectoryConfig::reversed, which drives the same ordered path backwards.
std::vector<Waypoint> reverseWaypoints(
    const std::vector<Waypoint>& waypoints);

}  // namespace vantage
