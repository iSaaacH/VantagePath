#pragma once

#include <cstddef>
#include <vector>

#include "vantage/geometry.hpp"

namespace vantage {

struct Waypoint {
  Pose2d pose;
  double tangentScale = 0.0;  // 0 chooses a distance-based tangent.
  // Optional Bézier geometry to the next waypoint. No controls = straight line.
  // Control poses use x/y only; heading follows the curve's tangent.
  bool bezierToNext = false;
  std::vector<Pose2d> controlPoints;

};

struct DriveFeedforwardConstraint {
  double staticGain = 0.0;
  double velocityGain = 0.0;
  double accelerationGain = 0.0;
};

struct TrajectoryConfig {
  double maxVelocity = 1.5;
  double maxAcceleration = 2.0;
  double maxDeceleration = 2.5;
  double maxCentripetalAcceleration = 2.0;
  double maxWheelVelocity = 1.8;
  // Set maxVoltage > 0 and identify both feedforwards to constrain local
  // acceleration against motor voltage, including curvature-rate wheel accel.
  double maxVoltage = 0.0;
  DriveFeedforwardConstraint leftFeedforward;
  DriveFeedforwardConstraint rightFeedforward;
  double trackWidth = 0.30;
  double startVelocity = 0.0;
  double endVelocity = 0.0;
  double sampleDistance = 0.025;
  bool reversed = false;
};

struct TrajectoryState {
  double time = 0.0;
  double distance = 0.0;
  Pose2d pose;
  double curvature = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double angularVelocity = 0.0;
  // Preserves the intended drive direction when velocity is exactly zero at a
  // trajectory endpoint. This matters to signed lateral pose feedback.
  double direction = 1.0;
};

class Trajectory {
 public:
  Trajectory() = default;
  explicit Trajectory(std::vector<TrajectoryState> states);

  bool empty() const { return states_.empty(); }
  double duration() const;
  double length() const;
  const std::vector<TrajectoryState>& states() const { return states_; }
  TrajectoryState sample(double time) const;

 private:
  std::vector<TrajectoryState> states_;
};

// Quintic Hermite or arbitrary-degree Bezier geometry plus forward/backward
// time parameterization. Stops at joins involving Bezier segments.
// Throws std::invalid_argument for unsafe configuration.
Trajectory generateTrajectory(const std::vector<Waypoint>& waypoints,
                              const TrajectoryConfig& config);

}  // namespace vantage
