#include "vantage/follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vantage {

TrajectoryFollower::TrajectoryFollower(FollowerConfig config)
    : config_(config),
      kinematics_(config.trackWidth),
      poseController_(config.poseController),
      leftFeedforward_(config.leftFeedforward),
      rightFeedforward_(config.rightFeedforward),
      leftPid_(config.leftVelocityPid),
      rightPid_(config.rightVelocityPid) {
  if (!(config.nominalVoltage > 0.0) || config.settleCycles == 0) {
    throw std::invalid_argument("invalid follower voltage or settle cycle count");
  }
}

void TrajectoryFollower::start(const Trajectory& trajectory, double nowSeconds) {
  if (trajectory.empty()) throw std::invalid_argument("trajectory is empty");
  trajectory_ = &trajectory;
  startTime_ = nowSeconds;
  previousTime_ = nowSeconds;
  previousSetpoint_ = {};
  status_ = FollowerStatus::kRunning;
  settledCycles_ = 0;
  leftPid_.reset();
  rightPid_.reset();
}

FollowerOutput TrajectoryFollower::update(
    double nowSeconds, const Pose2d& pose,
    const WheelSpeeds& measuredWheelSpeeds, double availableVoltage) {
  FollowerOutput output;
  output.status = status_;
  if (status_ != FollowerStatus::kRunning || trajectory_ == nullptr) return output;
  if (!std::isfinite(nowSeconds) || !std::isfinite(pose.x) ||
      !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
      !std::isfinite(measuredWheelSpeeds.left) ||
      !std::isfinite(measuredWheelSpeeds.right) ||
      !std::isfinite(availableVoltage)) {
    status_ = FollowerStatus::kDiverged;
    output.status = status_;
    return output;
  }

  const double elapsed = std::max(0.0, nowSeconds - startTime_);
  const double dt = std::clamp(nowSeconds - previousTime_, 1e-4, 0.1);
  const TrajectoryState reference = trajectory_->sample(elapsed);
  output.elapsed = elapsed;
  output.dt = dt;
  output.reference = reference;
  output.poseFeedbackActive = config_.enablePoseFeedback;
  output.velocityFeedbackActive = config_.enableVelocityFeedback;
  output.poseError = poseController_.error(pose, reference);
  const double positionError = std::hypot(output.poseError.longitudinal,
                                          output.poseError.lateral);
  if (positionError > config_.divergenceLimit) {
    status_ = FollowerStatus::kDiverged;
    output.status = status_;
    return output;
  }

  const bool timeComplete = elapsed >= trajectory_->duration();
  const bool poseSettled = positionError <= config_.positionTolerance &&
      std::abs(output.poseError.heading) <= config_.headingTolerance;
  const bool wheelsSettled =
      std::abs(measuredWheelSpeeds.left) <= config_.velocityTolerance &&
      std::abs(measuredWheelSpeeds.right) <= config_.velocityTolerance;
  const bool settleCandidate = timeComplete && poseSettled && wheelsSettled;

  // Do not kick a stopped robot back out of tolerance while confirming that it
  // is settled. Pose feedback can still request a small non-zero wheel speed
  // anywhere inside positionTolerance, and static feedforward then turns that
  // request into a full +/-kS step. Applying those commands during the settle
  // window produces an avoidable forward/reverse limit cycle.
  if (settleCandidate) {
    ++settledCycles_;
    previousSetpoint_ = {};
    previousTime_ = nowSeconds;
    leftPid_.reset();
    rightPid_.reset();
    if (settledCycles_ >= config_.settleCycles) {
      status_ = FollowerStatus::kSettled;
    } else if (elapsed > trajectory_->duration() +
                           config_.timeoutAfterTrajectory) {
      status_ = FollowerStatus::kTimedOut;
    }
    output.status = status_;
    return output;
  }
  settledCycles_ = 0;

  const ChassisSpeeds command = config_.enablePoseFeedback
      ? poseController_.calculate(pose, reference)
      : ChassisSpeeds{reference.velocity, reference.angularVelocity};
  output.chassisSetpoint = command;
  output.wheelSetpoint = kinematics_.toWheelSpeeds(command);
  const double leftAcceleration =
      (output.wheelSetpoint.left - previousSetpoint_.left) / dt;
  const double rightAcceleration =
      (output.wheelSetpoint.right - previousSetpoint_.right) / dt;
  const double voltageLimit = std::max(
      0.0, std::min(std::abs(availableVoltage), config_.nominalVoltage));
  output.voltageLimit = voltageLimit;

  const double leftFf = leftFeedforward_.calculate(
      output.wheelSetpoint.left, leftAcceleration);
  const double rightFf = rightFeedforward_.calculate(
      output.wheelSetpoint.right, rightAcceleration);
  output.leftFeedforwardVoltage = leftFf;
  output.rightFeedforwardVoltage = rightFf;
  if (config_.enableVelocityFeedback) {
    output.leftFeedbackVoltage = leftPid_.calculate(
        output.wheelSetpoint.left, measuredWheelSpeeds.left, dt,
        -voltageLimit - leftFf, voltageLimit - leftFf);
    output.rightFeedbackVoltage = rightPid_.calculate(
        output.wheelSetpoint.right, measuredWheelSpeeds.right, dt,
        -voltageLimit - rightFf, voltageLimit - rightFf);
  }
  output.leftVoltage = leftFf + output.leftFeedbackVoltage;
  output.rightVoltage = rightFf + output.rightFeedbackVoltage;
  const double peak = std::max(std::abs(output.leftVoltage),
                               std::abs(output.rightVoltage));
  if (peak > voltageLimit && peak > 0.0) {
    const double scale = voltageLimit / peak;
    output.leftVoltage *= scale;
    output.rightVoltage *= scale;
    output.saturated = true;
  }

  if (elapsed > trajectory_->duration() +
                    config_.timeoutAfterTrajectory) {
    status_ = FollowerStatus::kTimedOut;
    output.leftVoltage = 0.0;
    output.rightVoltage = 0.0;
  }

  previousSetpoint_ = output.wheelSetpoint;
  previousTime_ = nowSeconds;
  output.status = status_;
  return output;
}

void TrajectoryFollower::cancel() {
  trajectory_ = nullptr;
  status_ = FollowerStatus::kIdle;
  settledCycles_ = 0;
  leftPid_.reset();
  rightPid_.reset();
}

}  // namespace vantage
