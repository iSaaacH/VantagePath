#include "vantage/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vantage {
namespace {

struct Sample {
  Pose2d pose;
  double curvature;
  double distance;
};

struct Quintic {
  double a0, a1, a2, a3, a4, a5;

  static Quintic connect(double p0, double v0, double acc0,
                         double p1, double v1, double acc1) {
    return {p0,
            v0,
            acc0 * 0.5,
            -10.0 * p0 - 6.0 * v0 - 1.5 * acc0 + 10.0 * p1 -
                4.0 * v1 + 0.5 * acc1,
            15.0 * p0 + 8.0 * v0 + 1.5 * acc0 - 15.0 * p1 +
                7.0 * v1 - acc1,
            -6.0 * p0 - 3.0 * v0 - 0.5 * acc0 + 6.0 * p1 -
                3.0 * v1 + 0.5 * acc1};
  }

  double value(double t) const {
    return a0 + t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
  }
  double first(double t) const {
    return a1 + t * (2.0 * a2 + t * (3.0 * a3 +
           t * (4.0 * a4 + t * 5.0 * a5)));
  }
  double second(double t) const {
    return 2.0 * a2 + t * (6.0 * a3 +
           t * (12.0 * a4 + t * 20.0 * a5));
  }
};

double distance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

void validate(const TrajectoryConfig& c) {
  if (!(c.maxVelocity > 0.0 && c.maxAcceleration > 0.0 &&
        c.maxDeceleration > 0.0 && c.maxCentripetalAcceleration > 0.0 &&
        c.maxWheelVelocity > 0.0 && c.trackWidth > 0.0 &&
        c.sampleDistance > 0.0)) {
    throw std::invalid_argument("trajectory limits and dimensions must be > 0");
  }
  if (c.startVelocity < 0.0 || c.endVelocity < 0.0 ||
      c.startVelocity > c.maxVelocity || c.endVelocity > c.maxVelocity) {
    throw std::invalid_argument("boundary velocities must be within limits");
  }
}

}  // namespace

Trajectory::Trajectory(std::vector<TrajectoryState> states)
    : states_(std::move(states)) {}

double Trajectory::duration() const {
  return states_.empty() ? 0.0 : states_.back().time;
}

double Trajectory::length() const {
  return states_.empty() ? 0.0 : states_.back().distance;
}

TrajectoryState Trajectory::sample(double time) const {
  if (states_.empty()) return {};
  if (time <= 0.0) return states_.front();
  if (time >= duration()) return states_.back();
  const auto upper = std::lower_bound(
      states_.begin(), states_.end(), time,
      [](const TrajectoryState& state, double value) {
        return state.time < value;
      });
  const auto lower = upper - 1;
  const double span = upper->time - lower->time;
  const double u = span > 1e-9 ? (time - lower->time) / span : 0.0;
  TrajectoryState out;
  out.time = time;
  out.distance = lower->distance + (upper->distance - lower->distance) * u;
  out.pose = interpolate(lower->pose, upper->pose, u);
  out.curvature = lower->curvature + (upper->curvature - lower->curvature) * u;
  out.velocity = lower->velocity + (upper->velocity - lower->velocity) * u;
  out.acceleration = lower->acceleration +
                     (upper->acceleration - lower->acceleration) * u;
  out.angularVelocity = out.velocity * out.curvature;
  return out;
}

Trajectory generateTrajectory(const std::vector<Waypoint>& waypoints,
                              const TrajectoryConfig& config) {
  validate(config);
  if (waypoints.size() < 2) {
    throw std::invalid_argument("at least two waypoints are required");
  }

  std::vector<Sample> samples;
  double arcLength = 0.0;
  for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment) {
    const Waypoint& start = waypoints[segment];
    const Waypoint& end = waypoints[segment + 1];
    const double chord = distance(start.pose, end.pose);
    if (chord < 1e-8) throw std::invalid_argument("adjacent waypoints overlap");
    const double startScale = start.tangentScale > 0.0
                                  ? start.tangentScale : chord * 1.2;
    const double endScale = end.tangentScale > 0.0
                                ? end.tangentScale : chord * 1.2;
    const Quintic x = Quintic::connect(
        start.pose.x, std::cos(start.pose.theta) * startScale, 0.0,
        end.pose.x, std::cos(end.pose.theta) * endScale, 0.0);
    const Quintic y = Quintic::connect(
        start.pose.y, std::sin(start.pose.theta) * startScale, 0.0,
        end.pose.y, std::sin(end.pose.theta) * endScale, 0.0);
    const int count = std::max(16, static_cast<int>(std::ceil(
        chord / config.sampleDistance * 2.0)));
    for (int i = segment == 0 ? 0 : 1; i <= count; ++i) {
      const double t = static_cast<double>(i) / count;
      const double dx = x.first(t);
      const double dy = y.first(t);
      const double ddx = x.second(t);
      const double ddy = y.second(t);
      const double denom = std::pow(dx * dx + dy * dy, 1.5);
      const double curvature = denom > 1e-12
                                   ? (dx * ddy - dy * ddx) / denom : 0.0;
      Sample sample{{x.value(t), y.value(t), std::atan2(dy, dx)},
                    curvature, arcLength};
      if (!samples.empty()) {
        arcLength += distance(samples.back().pose, sample.pose);
        sample.distance = arcLength;
      }
      samples.push_back(sample);
    }
  }

  std::vector<double> velocity(samples.size(), config.maxVelocity);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double curvature = std::abs(samples[i].curvature);
    const double wheelFactor = std::max(
        std::abs(1.0 - samples[i].curvature * config.trackWidth * 0.5),
        std::abs(1.0 + samples[i].curvature * config.trackWidth * 0.5));
    velocity[i] = std::min(velocity[i], config.maxWheelVelocity / wheelFactor);
    if (curvature > 1e-9) {
      velocity[i] = std::min(
          velocity[i], std::sqrt(config.maxCentripetalAcceleration / curvature));
    }
  }
  velocity.front() = std::min(velocity.front(), config.startVelocity);
  for (std::size_t i = 1; i < velocity.size(); ++i) {
    const double ds = samples[i].distance - samples[i - 1].distance;
    velocity[i] = std::min(velocity[i], std::sqrt(
        velocity[i - 1] * velocity[i - 1] + 2.0 * config.maxAcceleration * ds));
  }
  velocity.back() = std::min(velocity.back(), config.endVelocity);
  for (std::size_t i = velocity.size() - 1; i-- > 0;) {
    const double ds = samples[i + 1].distance - samples[i].distance;
    velocity[i] = std::min(velocity[i], std::sqrt(
        velocity[i + 1] * velocity[i + 1] + 2.0 * config.maxDeceleration * ds));
  }

  const double direction = config.reversed ? -1.0 : 1.0;
  std::vector<TrajectoryState> states(samples.size());
  for (std::size_t i = 0; i < states.size(); ++i) {
    states[i].distance = samples[i].distance;
    states[i].pose = samples[i].pose;
    if (config.reversed) {
      states[i].pose.theta = wrapAngle(states[i].pose.theta + kPi);
    }
    states[i].curvature = direction * samples[i].curvature;
    states[i].velocity = direction * velocity[i];
    states[i].angularVelocity = states[i].velocity * states[i].curvature;
    if (i > 0) {
      const double ds = samples[i].distance - samples[i - 1].distance;
      const double sum = velocity[i] + velocity[i - 1];
      const double dt = sum > 1e-9 ? 2.0 * ds / sum : 0.0;
      states[i].time = states[i - 1].time + dt;
      if (dt > 1e-9) {
        states[i - 1].acceleration = direction *
            (velocity[i] - velocity[i - 1]) / dt;
      }
    }
  }
  states.back().acceleration = 0.0;
  return Trajectory(std::move(states));
}

}  // namespace vantage
