#pragma once

#include <cstddef>
#include <vector>

#include "vantage/geometry.hpp"

namespace vantage {

struct Waypoint {
  Pose2d pose;
  double tangentScale = 0.0;  // 0 chooses a distance-based tangent.
};

struct TrajectoryConfig {
  double maxVelocity = 1.5;
  double maxAcceleration = 2.0;
  double maxDeceleration = 2.5;
  double maxCentripetalAcceleration = 2.0;
  double maxWheelVelocity = 1.8;
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

// C2-continuous quintic Hermite geometry plus forward/backward time
// parameterization. Throws std::invalid_argument for unsafe configuration.
Trajectory generateTrajectory(const std::vector<Waypoint>& waypoints,
                              const TrajectoryConfig& config);

}  // namespace vantage
