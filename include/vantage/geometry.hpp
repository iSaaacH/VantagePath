#pragma once

#include <algorithm>
#include <cmath>

namespace vantage {

constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double radians) {
  return std::remainder(radians, 2.0 * kPi);
}

inline double sinc(double x) {
  if (std::abs(x) < 1e-6) return 1.0 - x * x / 6.0;
  return std::sin(x) / x;
}

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct PoseError {
  double longitudinal = 0.0;
  double lateral = 0.0;
  double heading = 0.0;
};

inline PoseError errorInRobotFrame(const Pose2d& current,
                                   const Pose2d& target) {
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  const double c = std::cos(current.theta);
  const double s = std::sin(current.theta);
  return {c * dx + s * dy, -s * dx + c * dy,
          wrapAngle(target.theta - current.theta)};
}

inline Pose2d interpolate(const Pose2d& a, const Pose2d& b, double u) {
  u = std::clamp(u, 0.0, 1.0);
  return {a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u,
          wrapAngle(a.theta + wrapAngle(b.theta - a.theta) * u)};
}

}  // namespace vantage
