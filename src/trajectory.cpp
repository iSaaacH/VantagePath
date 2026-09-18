#include "vantage/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vantage {
namespace {

struct Sample {
  Pose2d pose;
  double curvature;
  double curvatureDerivative = 0.0;
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

Pose2d bezierValue(std::vector<Pose2d> points, double t) {
  for (std::size_t size = points.size() - 1; size > 0; --size) {
    for (std::size_t i = 0; i < size; ++i) {
      points[i].x += (points[i + 1].x - points[i].x) * t;
      points[i].y += (points[i + 1].y - points[i].y) * t;
    }
  }
  return points.front();
}

std::vector<Pose2d> bezierDerivative(const std::vector<Pose2d>& points) {
  std::vector<Pose2d> result;
  const double degree = points.size() - 1;
  for (std::size_t i = 1; i < points.size(); ++i)
    result.push_back({degree * (points[i].x - points[i-1].x),
                      degree * (points[i].y - points[i-1].y), 0.0});
  if (result.empty()) result.push_back({0.0, 0.0, 0.0});
  return result;
}

double distance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

void validate(const TrajectoryConfig& c) {
  const double values[] = {
      c.maxVelocity, c.maxAcceleration, c.maxDeceleration,
      c.maxCentripetalAcceleration, c.maxWheelVelocity, c.maxVoltage,
      c.maxAngularVelocity,
      c.trackWidth, c.startVelocity, c.endVelocity, c.sampleDistance,
      c.leftFeedforward.staticGain, c.leftFeedforward.velocityGain,
      c.leftFeedforward.accelerationGain, c.rightFeedforward.staticGain,
      c.rightFeedforward.velocityGain, c.rightFeedforward.accelerationGain};
  if (std::any_of(std::begin(values), std::end(values),
                  [](double value) { return !std::isfinite(value); })) {
    throw std::invalid_argument("trajectory configuration must be finite");
  }
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
  if (c.maxAngularVelocity < 0.0) {
    throw std::invalid_argument("maxAngularVelocity must be nonnegative");
  }
  if (c.maxVoltage > 0.0 &&
      (!(c.leftFeedforward.accelerationGain > 0.0) ||
       !(c.rightFeedforward.accelerationGain > 0.0))) {
    throw std::invalid_argument(
        "voltage constraint requires positive left/right acceleration gains");
  }
  for (const auto& ff : {c.leftFeedforward, c.rightFeedforward}) {
    if (ff.staticGain < 0 || ff.velocityGain < 0 || ff.accelerationGain < 0 ||
        (c.maxVoltage > 0 && c.maxVoltage <= ff.staticGain)) {
      throw std::invalid_argument("voltage budget must exceed nonnegative static feedforward");
    }
  }
}

struct AccelerationBounds { double minimum; double maximum; };

AccelerationBounds voltageAccelerationBounds(const Sample& sample, double speed,
                                              const TrajectoryConfig& config) {
  AccelerationBounds bounds{-config.maxDeceleration, config.maxAcceleration};
  if (!(config.maxVoltage > 0.0)) return bounds;
  const double halfTrack = config.trackWidth * 0.5;
  // The time passes operate on positive geometric path speed. Convert that
  // speed to signed physical wheel speeds here. On a reverse path this also
  // swaps which physical side is the curve's inner/outer wheel; using the
  // forward factors can violate voltage limits when the two sides have
  // different feedforward constants.
  const double direction = config.reversed ? -1.0 : 1.0;
  const double factors[2] = {direction - sample.curvature * halfTrack,
                             direction + sample.curvature * halfTrack};
  const double factorDerivatives[2] = {
      -sample.curvatureDerivative * halfTrack,
       sample.curvatureDerivative * halfTrack};
  const DriveFeedforwardConstraint feeds[2] = {
      config.leftFeedforward, config.rightFeedforward};
  for (int side = 0; side < 2; ++side) {
    const double wheelVelocity = speed * factors[side];
    const double sign = wheelVelocity > 1e-9 ? 1.0
                        : wheelVelocity < -1e-9 ? -1.0 : 0.0;
    const double curveAcceleration = factorDerivatives[side] * speed * speed;
    const double baseVoltage = feeds[side].staticGain * sign +
        feeds[side].velocityGain * wheelVelocity +
        feeds[side].accelerationGain * curveAcceleration;
    const double coefficient = feeds[side].accelerationGain * factors[side];
    if (std::abs(coefficient) < 1e-9) continue;
    double low = (-config.maxVoltage - baseVoltage) / coefficient;
    double high = (config.maxVoltage - baseVoltage) / coefficient;
    if (low > high) std::swap(low, high);
    bounds.minimum = std::max(bounds.minimum, low);
    bounds.maximum = std::min(bounds.maximum, high);
  }
  if (bounds.minimum > bounds.maximum) {
    // The final conservative voltage retiming pass below must resolve this;
    // zero acceleration alone does NOT make this speed feasible.
    return {0.0, 0.0};
  }
  return bounds;
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
  if (time <= states_.front().time) return states_.front();
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
  const double elapsed = time - lower->time;
  const double acceleration = span > 1e-9 ? (upper->velocity-lower->velocity)/span : 0;
  const double travel = std::abs(lower->velocity * elapsed + .5 * acceleration * elapsed * elapsed);
  const double ds = upper->distance - lower->distance;
  const double geometryFraction = ds > 1e-9 ? std::clamp(travel / ds, 0.0, 1.0) : u;
  out.distance = lower->distance + ds * geometryFraction;
  out.pose = interpolate(lower->pose, upper->pose, geometryFraction);
  out.curvature = lower->curvature + (upper->curvature - lower->curvature) * geometryFraction;
  out.velocity = lower->velocity + (upper->velocity - lower->velocity) * u;
  out.acceleration = acceleration;
  out.angularVelocity = out.velocity * out.curvature;
  out.direction = lower->direction;
  return out;
}

Trajectory generateTrajectory(const std::vector<Waypoint>& waypoints,
                              const TrajectoryConfig& config) {
  validate(config);
  if (waypoints.size() < 2) {
    throw std::invalid_argument("at least two waypoints are required");
  }

  std::vector<Sample> samples;
  std::vector<double> tangentScales(waypoints.size(), 0.0);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    if (!std::isfinite(waypoints[i].pose.x) || !std::isfinite(waypoints[i].pose.y) ||
        !std::isfinite(waypoints[i].pose.theta) || !std::isfinite(waypoints[i].tangentScale)) {
      throw std::invalid_argument("waypoint values must be finite");
    }
    if (waypoints[i].tangentScale < 0.0) {
      throw std::invalid_argument("waypoint tangent scale must be nonnegative");
    }
    if (waypoints[i].tangentScale > 0.0) {
      tangentScales[i] = waypoints[i].tangentScale;
    } else if (i == 0) {
      tangentScales[i] = 1.2 * distance(waypoints[0].pose, waypoints[1].pose);
    } else if (i + 1 == waypoints.size()) {
      tangentScales[i] = 1.2 * distance(waypoints[i - 1].pose, waypoints[i].pose);
    } else {
      tangentScales[i] = 0.6 *
          (distance(waypoints[i - 1].pose, waypoints[i].pose) +
           distance(waypoints[i].pose, waypoints[i + 1].pose));
    }
  }
  double arcLength = 0.0;
  std::vector<std::size_t> stopIndices;
  for (std::size_t segment = 0; segment + 1 < waypoints.size(); ++segment) {
    // Independently edited Bézier segments may meet at a corner. Stop at
    // their shared anchor instead of carrying speed through a heading jump.
    if (segment > 0 && (waypoints[segment-1].bezierToNext || waypoints[segment].bezierToNext))
      stopIndices.push_back(samples.size()-1);
    const Waypoint& start = waypoints[segment];
    const Waypoint& end = waypoints[segment + 1];
    const double chord = distance(start.pose, end.pose);
    if (chord < 1e-8) throw std::invalid_argument("adjacent waypoints overlap");
    const double startScale = tangentScales[segment];
    const double endScale = tangentScales[segment + 1];
    const Quintic x = Quintic::connect(
        start.pose.x, std::cos(start.pose.theta) * startScale, 0.0,
        end.pose.x, std::cos(end.pose.theta) * endScale, 0.0);
    const Quintic y = Quintic::connect(
        start.pose.y, std::sin(start.pose.theta) * startScale, 0.0,
        end.pose.y, std::sin(end.pose.theta) * endScale, 0.0);
    std::vector<Pose2d> controls{start.pose};
    if (start.bezierToNext) controls.insert(controls.end(), start.controlPoints.begin(), start.controlPoints.end());
    controls.push_back(end.pose);
    double polygonLength = 0.0;
    for (std::size_t c = 1; c < controls.size(); ++c) {
      if (!std::isfinite(controls[c].x) || !std::isfinite(controls[c].y))
        throw std::invalid_argument("control coordinates must be finite");
      polygonLength += distance(controls[c-1], controls[c]);
    }
    const auto first = bezierDerivative(controls);
    const auto second = bezierDerivative(first);
    const int count = std::max(16, static_cast<int>(std::ceil(
        (start.bezierToNext ? polygonLength : chord) / config.sampleDistance * 2.0)));
    for (int i = 0; i <= count; ++i) {
      const double t = static_cast<double>(i) / count;
      const auto position = start.bezierToNext ? bezierValue(controls, t) : Pose2d{x.value(t), y.value(t), 0.0};
      const auto velocity = start.bezierToNext ? bezierValue(first, t) : Pose2d{x.first(t), y.first(t), 0.0};
      const auto acceleration = start.bezierToNext ? bezierValue(second, t) : Pose2d{x.second(t), y.second(t), 0.0};
      const double dx = velocity.x, dy = velocity.y;
      const double ddx = acceleration.x, ddy = acceleration.y;
      const double denom = std::pow(dx * dx + dy * dy, 1.5);
      const double curvature = denom > 1e-12
                                   ? (dx * ddy - dy * ddx) / denom : 0.0;
      double heading = std::atan2(dy, dx);
      // Repeated endpoint controls have a zero derivative, but still have a
      // well-defined limiting tangent toward the first distinct control.
      if (start.bezierToNext && std::hypot(dx, dy) < 1e-9 && (i == 0 || i == count)) {
        for (std::size_t c = 1; c < controls.size(); ++c) {
          const auto& other = i == 0 ? controls[c] : controls[controls.size()-1-c];
          const double tx = i == 0 ? other.x-position.x : position.x-other.x;
          const double ty = i == 0 ? other.y-position.y : position.y-other.y;
          if (std::hypot(tx, ty) > 1e-9) { heading = std::atan2(ty, tx); break; }
        }
      }
      if (segment > 0 && i == 0) {
        if (std::abs(wrapAngle(heading - samples.back().pose.theta)) > 1e-3) {
          throw std::invalid_argument("path join is not tangent-continuous; split into explicit motions or smooth controls");
        }
        continue;
      }
      Sample sample{{position.x, position.y, heading},
                    curvature, 0.0, arcLength};
      if (!samples.empty()) {
        arcLength += distance(samples.back().pose, sample.pose);
        sample.distance = arcLength;
      }
      samples.push_back(sample);
    }
  }

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const std::size_t before = i == 0 ? 0 : i - 1;
    const std::size_t after = i + 1 < samples.size() ? i + 1 : i;
    const double ds = samples[after].distance - samples[before].distance;
    samples[i].curvatureDerivative = ds > 1e-9
        ? (samples[after].curvature - samples[before].curvature) / ds : 0.0;
  }

  std::vector<double> velocity(samples.size(), config.maxVelocity);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const double curvature = std::abs(samples[i].curvature);
    // Bound both endpoints of each interpolation interval. Limiting only the
    // sampled wheel speed lets interpolated v * (1 +/- curvature * halfTrack)
    // exceed the wheel ceiling between samples.
    const double curvatureBound = std::max({
        curvature,
        std::abs(samples[i == 0 ? 0 : i - 1].curvature),
        std::abs(samples[std::min(i + 1, samples.size() - 1)].curvature)});
    const double wheelFactor = 1.0 + curvatureBound * config.trackWidth * 0.5;
    velocity[i] = std::min(velocity[i], config.maxWheelVelocity / wheelFactor);
    if (config.maxAngularVelocity > 0.0) {
      // v <= omega_max / |curvature|. Include adjacent samples so linear
      // interpolation of velocity and curvature also respects this ceiling.
      // Applied before time passes: the robot brakes BEFORE a tight bend.
      if (curvatureBound > 1e-9) {
        velocity[i] = std::min(velocity[i],
                              config.maxAngularVelocity / curvatureBound);
      }
    }
    if (curvature > 1e-9) {
      velocity[i] = std::min(
          velocity[i], std::sqrt(config.maxCentripetalAcceleration / curvature));
    }
  }
  for (auto index : stopIndices) velocity[index] = 0.0;
  velocity.front() = std::min(velocity.front(), config.startVelocity);
  for (std::size_t i = 1; i < velocity.size(); ++i) {
    const double ds = samples[i].distance - samples[i - 1].distance;
    const double allowedAcceleration = std::max(0.0,
        voltageAccelerationBounds(samples[i - 1], velocity[i - 1], config).maximum);
    velocity[i] = std::min(velocity[i], std::sqrt(
        velocity[i - 1] * velocity[i - 1] + 2.0 * allowedAcceleration * ds));
  }
  velocity.back() = std::min(velocity.back(), config.endVelocity);
  for (std::size_t i = velocity.size() - 1; i-- > 0;) {
    const double ds = samples[i + 1].distance - samples[i].distance;
    const double allowedDeceleration = std::max(0.0,
        -voltageAccelerationBounds(samples[i + 1], velocity[i + 1], config).minimum);
    velocity[i] = std::min(velocity[i], std::sqrt(
        velocity[i + 1] * velocity[i + 1] + 2.0 * allowedDeceleration * ds));
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
    states[i].direction = direction;
    if (i > 0) {
      const double ds = samples[i].distance - samples[i - 1].distance;
      const double sum = velocity[i] + velocity[i - 1];
      if (ds > 1e-9 && sum <= 1e-9)
        throw std::invalid_argument("trajectory contains an untraversable zero-speed interval");
      const double dt = sum > 1e-9 ? 2.0 * ds / sum : 0.0;
      states[i].time = states[i - 1].time + dt;
      if (dt > 1e-9) {
        states[i - 1].acceleration = direction *
            (velocity[i] - velocity[i - 1]) / dt;
      }
    }
  }
  states.back().acceleration = 0.0;
  // Conservative interval bounds also cover interpolated curvature-rate wheel
  // acceleration. A uniform time dilation preserves geometry and all zero-speed
  // stops while making speed/acceleration/voltage feasible together.
  double timeScale = 1.0;
  if (config.maxVoltage > 0) {
    const double halfTrack = config.trackWidth * .5;
    for (std::size_t i = 1; i < states.size(); ++i) {
      const auto& a = states[i-1]; const auto& b = states[i];
      const double ds = b.distance-a.distance;
      const double maxV = std::max(std::abs(a.velocity),std::abs(b.velocity));
      const double dk = ds > 1e-9 ? std::abs(b.curvature-a.curvature)/ds : 0;
      for (int side = 0; side < 2; ++side) {
        const auto& ff = side == 0 ? config.leftFeedforward : config.rightFeedforward;
        const double sign = side == 0 ? -1.0 : 1.0;
        const double factor = std::max(std::abs(1+sign*halfTrack*a.curvature),
                                       std::abs(1+sign*halfTrack*b.curvature));
        const double vVolts = ff.velocityGain * maxV * factor;
        const double aVolts = ff.accelerationGain *
            (std::abs(a.acceleration)*factor + halfTrack*dk*maxV*maxV);
        const double budget = config.maxVoltage-ff.staticGain;
        timeScale = std::max(timeScale,
            (vVolts+std::sqrt(vVolts*vVolts+4*budget*aVolts))/(2*budget));
      }
    }
  }
  if (timeScale > 1.0+1e-9) {
    if (config.startVelocity > 0 || config.endVelocity > 0)
      throw std::invalid_argument("voltage retiming cannot preserve requested rolling boundary speeds");
    for (auto& state : states) {
      state.time *= timeScale;
      state.velocity /= timeScale;
      state.angularVelocity /= timeScale;
      state.acceleration /= timeScale*timeScale;
    }
  }
  return Trajectory(std::move(states));
}

}  // namespace vantage
