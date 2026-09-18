#include "vantage/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vantage {

DifferentialDriveKinematics::DifferentialDriveKinematics(double trackWidth)
    : trackWidth_(trackWidth) {
  if (!(trackWidth > 0.0)) throw std::invalid_argument("trackWidth must be > 0");
}

WheelSpeeds DifferentialDriveKinematics::toWheelSpeeds(
    const ChassisSpeeds& speeds) const {
  const double delta = speeds.angular * trackWidth_ * 0.5;
  return {speeds.linear - delta, speeds.linear + delta};
}

ChassisSpeeds DifferentialDriveKinematics::toChassisSpeeds(
    const WheelSpeeds& speeds) const {
  return {(speeds.left + speeds.right) * 0.5,
          (speeds.right - speeds.left) / trackWidth_};
}

NonlinearPoseController::NonlinearPoseController(
    NonlinearControllerConfig config)
    : config_(config) {
  if (!std::isfinite(config.kp) || !std::isfinite(config.kd) ||
      !std::isfinite(config.minimumFeedbackSpeed) ||
      !std::isfinite(config.maxLinearCorrection) ||
      !std::isfinite(config.longitudinalScale) || config.longitudinalScale < 0.0 ||
      !std::isfinite(config.lateralScale) || config.lateralScale < 0.0 ||
      !std::isfinite(config.headingScale) || config.headingScale < 0.0 ||
      !std::isfinite(config.maxAngularCorrection) || !(config.kp > 0.0) ||
      !(config.kd > 0.0) || config.kd > 1.0 ||
      config.minimumFeedbackSpeed < 0.0 ||
      config.maxLinearCorrection < 0.0 ||
      config.maxAngularCorrection < 0.0) {
    throw std::invalid_argument(
        "controller requires finite kp > 0, 0 < kd <= 1 and nonnegative limits");
  }
}

PoseError NonlinearPoseController::error(
    const Pose2d& current, const TrajectoryState& reference) const {
  return errorInRobotFrame(current, reference.pose);
}

ChassisSpeeds NonlinearPoseController::calculate(
    const Pose2d& current, const TrajectoryState& reference,
    double lateralMultiplier) const {
  if (!std::isfinite(lateralMultiplier) || lateralMultiplier < 0.0 ||
      lateralMultiplier > 1.0) {
    throw std::invalid_argument("lateral multiplier must be in [0, 1]");
  }
  const PoseError e = error(current, reference);
  const double vRef = reference.velocity;
  const double wRef = reference.angularVelocity;
  // k uses speed magnitude, but the lateral term must retain drive direction.
  // Dropping this sign makes reverse trajectories steer away from cross-track
  // error. direction also keeps the sign at a stopped reverse endpoint.
  const double direction = vRef < -1e-9 ? -1.0 : vRef > 1e-9 ? 1.0
      : reference.direction < 0.0 ? -1.0 : 1.0;
  const double feedbackSpeed = std::max(std::abs(vRef),
                                        config_.minimumFeedbackSpeed);
  const double signedFeedbackSpeed = direction * feedbackSpeed;
  const double k = 2.0 * config_.kd *
                   std::sqrt(wRef * wRef + config_.kp *
                             feedbackSpeed * feedbackSpeed);

  const double linearCorrection = std::clamp(
      config_.longitudinalScale * k * e.longitudinal, -config_.maxLinearCorrection,
      config_.maxLinearCorrection);
  const double angularCorrection = std::clamp(
      config_.headingScale * k * e.heading + lateralMultiplier * config_.lateralScale * config_.kp * signedFeedbackSpeed *
          sinc(e.heading) * e.lateral,
      -config_.maxAngularCorrection, config_.maxAngularCorrection);
  return {vRef * std::cos(e.heading) + linearCorrection,
          wRef + angularCorrection};
}

MotorFeedforward::MotorFeedforward(FeedforwardConfig config) : config_(config) {
  if (!std::isfinite(config.staticActivationVelocity) ||
      config.staticActivationVelocity < 0.0) {
    throw std::invalid_argument("static activation speed must be finite and nonnegative");
  }
}

double MotorFeedforward::calculate(double velocity, double acceleration) const {
  double sign = 0.0;
  if (velocity > 1e-9) sign = 1.0;
  if (velocity < -1e-9) sign = -1.0;
  if (config_.staticVelocityDeadband > 0.0) {
    sign = std::clamp(velocity / config_.staticVelocityDeadband, -1.0, 1.0);
  }
  return config_.staticGain * sign + config_.velocityGain * velocity +
         config_.accelerationGain * acceleration;
}

double MotorFeedforward::calculateWithHysteresis(double velocity,
                                                double acceleration) {
  if (config_.staticActivationVelocity == 0.0) {
    return calculate(velocity, acceleration);
  }
  const double magnitude = std::abs(velocity);
  if (magnitude >= config_.staticActivationVelocity) {
    staticDirection_ = velocity > 0.0 ? 1.0 : -1.0;
  } else if (magnitude <= config_.staticActivationVelocity * 0.5 ||
             velocity * staticDirection_ <= 0.0) {
    staticDirection_ = 0.0;
  }
  return config_.staticGain * staticDirection_ +
         config_.velocityGain * velocity +
         config_.accelerationGain * acceleration;
}

VelocityPid::VelocityPid(VelocityPidConfig config) : config_(config) {}

double VelocityPid::calculate(double setpoint, double measurement, double dt,
                              double minOutput, double maxOutput) {
  if (!(dt > 0.0)) return 0.0;
  const double error = setpoint - measurement;
  const double rawDerivative = initialized_
                                   ? -(measurement - previousMeasurement_) / dt
                                   : 0.0;
  const double tau = std::max(0.0, config_.derivativeTimeConstant);
  const double alpha = tau / (tau + dt);
  filteredDerivative_ = alpha * filteredDerivative_ +
                        (1.0 - alpha) * rawDerivative;

  const double candidateIntegral = std::clamp(
      integral_ + error * dt, -std::abs(config_.integralLimit),
      std::abs(config_.integralLimit));
  const auto outputFor = [&](double integral) {
    return config_.kp * error + config_.ki * integral +
           config_.kd * filteredDerivative_;
  };
  const double candidateOutput = outputFor(candidateIntegral);
  // Conditional integration: do not accumulate farther into saturation.
  if ((candidateOutput <= maxOutput || error < 0.0) &&
      (candidateOutput >= minOutput || error > 0.0)) {
    integral_ = candidateIntegral;
  }

  previousMeasurement_ = measurement;
  initialized_ = true;
  return std::clamp(outputFor(integral_), minOutput, maxOutput);
}

void VelocityPid::reset() {
  integral_ = 0.0;
  previousMeasurement_ = 0.0;
  filteredDerivative_ = 0.0;
  initialized_ = false;
}

}  // namespace vantage
