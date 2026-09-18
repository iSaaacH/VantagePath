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
  if (!std::isfinite(config.nominalVoltage) || !(config.nominalVoltage > 0.0) || config.settleCycles == 0) {
    throw std::invalid_argument("invalid follower voltage or settle cycle count");
  }
  for (double value : {config.maxWheelVelocity, config.maxWheelAcceleration,
                      config.terminalMaxPositionError, config.terminalProgressTimeout,
                      config.positionTolerance, config.headingTolerance,
                      config.velocityTolerance, config.timeoutAfterTrajectory,
                      config.brakingLateralTaperSeconds, config.pointApproachSeconds,
                      config.pointApproachAngularBrakeSeconds}) {
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("follower limits must be finite and nonnegative");
  }
  if (config.pointApproachSeconds > 0.0) {
    for (double value : {config.pointApproachBlendSeconds, config.pointApproachTolerance,
                        config.pointApproachGuardDistance, config.pointApproachMaxCurvature,
                        config.pointApproachMaxAngularSpeed, config.pointApproachDeceleration}) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("point approach needs finite positive limits");
    }
    if (!config.stopAtProfileEnd || config.enableTerminalRecovery ||
        config.pointApproachBlendSeconds > config.pointApproachSeconds ||
        config.pointApproachAngularBrakeSeconds > config.pointApproachSeconds ||
        config.pointApproachGuardDistance < config.pointApproachTolerance ||
        config.pointApproachMaxCurvature * config.trackWidth >= 2.0)
      throw std::invalid_argument("point approach must stop at profile end without wheel reversal");
  }
  if (!std::isfinite(config.brakingLateralEndMultiplier) ||
      config.brakingLateralEndMultiplier < 0.0 ||
      config.brakingLateralEndMultiplier > 1.0) {
    throw std::invalid_argument("braking lateral multiplier must be in [0, 1]");
  }
  if (config.enableTerminalRecovery &&
      (!std::isfinite(config.terminalMaxLinearSpeed) ||
       !std::isfinite(config.terminalMaxAngularSpeed) ||
       !std::isfinite(config.terminalMaxWheelAcceleration) ||
       !(config.terminalMaxLinearSpeed > 0.0) ||
       !(config.terminalMaxAngularSpeed > 0.0) ||
       !(config.terminalMaxWheelAcceleration > 0.0) ||
       !(config.poseController.minimumFeedbackSpeed > 0.0) ||
       !std::isfinite(config.positionTolerance) || !(config.positionTolerance > 0.0))) {
    throw std::invalid_argument("terminal recovery needs positive finite speed limits");
  }
}

void TrajectoryFollower::start(const Trajectory& trajectory, double nowSeconds) {
  if (trajectory.empty()) throw std::invalid_argument("trajectory is empty");
  if (!std::isfinite(nowSeconds)) throw std::invalid_argument("start time must be finite");
  trajectory_ = &trajectory;
  startTime_ = nowSeconds;
  previousTime_ = nowSeconds;
  previousSetpoint_ = {};
  brakingLateralTaperDuration_ = 0.0;
  const auto& states = trajectory.states();
  if (config_.brakingLateralTaperSeconds > 0.0 &&
      std::abs(states.back().velocity) < 1e-9) {
    // Find the last braking segment once, not from noisy measured speed.
    // Do not taper mid-bend deceleration, startup, or a rolling endpoint.
    std::size_t firstBrake = states.size() - 1;
    while (firstBrake > 0 &&
           std::abs(states[firstBrake - 1].velocity) >
               std::abs(states[firstBrake].velocity) + 1e-9) {
      --firstBrake;
    }
    brakingLateralTaperDuration_ = std::min(config_.brakingLateralTaperSeconds,
        trajectory.duration() - states[firstBrake].time);
  }
  const auto first = trajectory.sample(0);
  previousReferenceSetpoint_ = kinematics_.toWheelSpeeds({first.velocity, first.angularVelocity});
  status_ = FollowerStatus::kRunning;
  settledCycles_ = 0;
  leftPid_.reset();
  rightPid_.reset();
  leftFeedforward_.reset();
  rightFeedforward_.reset();
  terminalPhase_ = TerminalPhase::kTracking;
  terminalDirection_ = 1.0;
  terminalProgressTime_ = nowSeconds;
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
      !std::isfinite(availableVoltage) || availableVoltage <= 0 || nowSeconds < previousTime_) {
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
  if (timeComplete && config_.stopAtProfileEnd) {
    // Return before any pose, wheel, or endpoint recovery can energize motors.
    // Keep the final reference/error for the commissioning log.
    status_ = FollowerStatus::kProfileComplete;
    output.status = status_;
    return output;
  }
  if (elapsed > trajectory_->duration() + config_.timeoutAfterTrajectory) {
    status_ = FollowerStatus::kTimedOut;
    output.status = status_;
    return output;
  }
  const bool terminalRecovery = timeComplete && config_.enablePoseFeedback &&
      config_.enableTerminalRecovery && std::abs(reference.velocity) < 1e-9 &&
      std::abs(reference.angularVelocity) < 1e-9;
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
    leftFeedforward_.reset();
    rightFeedforward_.reset();
    output.terminalPhase = terminalPhase_;
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

  // Without an explicit endpoint controller, a stationary reference is not a
  // point regulator. Do not run the cancellation-prone minimum-speed tracker.
  if (timeComplete && std::abs(reference.velocity) < 1e-9 &&
      std::abs(reference.angularVelocity) < 1e-9 && !terminalRecovery && !poseSettled) {
    status_ = FollowerStatus::kProfileComplete;
    output.status = status_;
    return output;
  }

  if (brakingLateralTaperDuration_ > 0.0) {
    const double progress = std::clamp(1.0 -
        (trajectory_->duration() - elapsed) / brakingLateralTaperDuration_, 0.0, 1.0);
    const double blend = progress * progress * (3.0 - 2.0 * progress);
    output.lateralFeedbackMultiplier = 1.0 - blend *
        (1.0 - config_.brakingLateralEndMultiplier);
  }
  ChassisSpeeds command = config_.enablePoseFeedback
      ? poseController_.calculate(pose, reference, output.lateralFeedbackMultiplier)
      : ChassisSpeeds{reference.velocity, reference.angularVelocity};
  const auto& endpoint = trajectory_->states().back();
  if (config_.enablePoseFeedback && config_.pointApproachSeconds > 0.0 &&
      std::abs(endpoint.velocity) < 1e-9 &&
      elapsed >= std::max(0.0, trajectory_->duration() - config_.pointApproachSeconds)) {
    const double start = std::max(0.0, trajectory_->duration() - config_.pointApproachSeconds);
    const double progress = std::clamp((elapsed-start)/config_.pointApproachBlendSeconds, 0.0, 1.0);
    const double blend = progress*progress*(3.0-2.0*progress);
    output.pointApproachBlend = blend;
    const auto goal = errorInRobotFrame(pose, endpoint.pose);
    const double distance = std::hypot(goal.longitudinal, goal.lateral);
    if (distance <= config_.pointApproachTolerance) {
      // Position only, not a claim of heading or measured-velocity settlement.
      status_ = FollowerStatus::kPositionComplete;
      output.status = status_;
      output.reference = endpoint;
      output.poseError = goal;
      return output;
    }
    const double direction = endpoint.direction < 0.0 ? -1.0 : 1.0;
    const double curvature = 2.0*goal.lateral/(distance*distance);
    if (direction*goal.longitudinal <= 0.0 ||
        (distance <= config_.pointApproachGuardDistance &&
         std::abs(curvature) > config_.pointApproachMaxCurvature)) {
      // Conservative model-based feasibility guard, not proof of physical
      // reachability. Never loop around or pivot for a last-inch lateral miss.
      status_ = FollowerStatus::kEndpointUnreachable;
      output.status = status_;
      output.reference = endpoint;
      output.poseError = goal;
      return output;
    }
    const double approachSpeed = std::min(
        std::abs(reference.velocity),
        std::sqrt(2.0*config_.pointApproachDeceleration*
                  std::max(0.0, distance-config_.pointApproachTolerance)));
    const double pointLinear = direction*approachSpeed;
    const double pointAngular = std::clamp(pointLinear*std::clamp(curvature,
        -config_.pointApproachMaxCurvature, config_.pointApproachMaxCurvature),
        -config_.pointApproachMaxAngularSpeed, config_.pointApproachMaxAngularSpeed);
    command.linear = (1.0-blend)*command.linear + blend*pointLinear;
    command.angular = (1.0-blend)*command.angular + blend*pointAngular;
    // Keep both wheel setpoints in the intended travel direction, including
    // during the blend. Existing wheel slew and voltage clamps still follow.
    command.linear = direction*std::max(0.0, direction*command.linear);
    double angularCeiling = config_.pointApproachMaxAngularSpeed;
    if (config_.pointApproachAngularBrakeSeconds > 0.0) {
      const double remaining = std::clamp((trajectory_->duration()-elapsed) /
          config_.pointApproachAngularBrakeSeconds, 0.0, 1.0);
      // Unlike curvature alone, this ceiling cannot stay at 45 deg/s as
      // distance shrinks. Smooth endpoints avoid a step into final braking.
      angularCeiling *= remaining*remaining*(3.0-2.0*remaining);
    }
    const double angularLimit = std::min(angularCeiling,
        std::abs(command.linear)*config_.pointApproachMaxCurvature);
    command.angular = std::clamp(command.angular,-angularLimit,angularLimit);
  }
  if (terminalRecovery) {
    const auto previousPhase = terminalPhase_;
    if (config_.terminalMaxPositionError > 0 && positionError > config_.terminalMaxPositionError) {
      status_ = FollowerStatus::kDiverged; output.status = status_; return output;
    }
    const double bearing = std::atan2(output.poseError.lateral,
                                      output.poseError.longitudinal);
    // Latch forward/reverse once per approach, avoiding sign flips when the
    // target lies almost exactly beside the robot.
    if (terminalPhase_ == TerminalPhase::kTracking) {
      terminalPhase_ = TerminalPhase::kBraking;
    }
    if ((terminalPhase_ == TerminalPhase::kBraking && wheelsSettled) ||
        (terminalPhase_ == TerminalPhase::kAlignHeading &&
         positionError > config_.positionTolerance)) {
      terminalDirection_ = std::cos(bearing) >= 0.0 ? 1.0 : -1.0;
      terminalPhase_ = TerminalPhase::kRotateToPosition;
    }
    const double approachError = wrapAngle(
        bearing - (terminalDirection_ < 0.0 ? kPi : 0.0));
    // Enter with margin so a little rotation-induced drift does not bounce
    // between position and final-heading recovery.
    if (terminalPhase_ != TerminalPhase::kBraking && positionError <= config_.positionTolerance * 0.75) {
      terminalPhase_ = TerminalPhase::kAlignHeading;
    } else if (terminalPhase_ == TerminalPhase::kRotateToPosition &&
               std::abs(approachError) < 0.12) {
      terminalPhase_ = TerminalPhase::kDriveToPosition;
    } else if (terminalPhase_ == TerminalPhase::kDriveToPosition &&
               std::abs(approachError) > 0.35) {
      terminalPhase_ = TerminalPhase::kRotateToPosition;
    }
    // Reuse the existing nonlinear gain schedule; these are bounded terminal
    // maneuvers, not a higher trajectory speed or additional tuning gain.
    const double gain = 2.0 * config_.poseController.kd *
        std::sqrt(config_.poseController.kp) *
        config_.poseController.minimumFeedbackSpeed;
    command = {};
    if (!poseSettled && terminalPhase_ != TerminalPhase::kBraking) {
      const double headingError = terminalPhase_ == TerminalPhase::kAlignHeading
          ? output.poseError.heading : approachError;
      command.angular = std::clamp(gain * headingError,
          -config_.terminalMaxAngularSpeed, config_.terminalMaxAngularSpeed);
      if (terminalPhase_ == TerminalPhase::kDriveToPosition) {
        command.linear = terminalDirection_ * std::min(
            gain * positionError, config_.terminalMaxLinearSpeed) *
            std::max(0.0, std::cos(approachError));
      }
    }
    if (previousPhase != terminalPhase_) {
      leftPid_.reset();
      rightPid_.reset();
    }
    const double progressError = terminalPhase_ == TerminalPhase::kBraking
        ? std::max(std::abs(measuredWheelSpeeds.left), std::abs(measuredWheelSpeeds.right))
        : terminalPhase_ == TerminalPhase::kDriveToPosition ? positionError
        : std::abs(terminalPhase_ == TerminalPhase::kAlignHeading
            ? output.poseError.heading : approachError);
    const double progressStep = terminalPhase_ == TerminalPhase::kDriveToPosition
        ? config_.positionTolerance * .1 : .01;
    if (previousPhase != terminalPhase_ || progressError < terminalBestError_ - progressStep) {
      terminalBestError_ = progressError;
      terminalProgressTime_ = nowSeconds;
    } else if (config_.terminalProgressTimeout > 0 &&
               nowSeconds-terminalProgressTime_ > config_.terminalProgressTimeout) {
      status_ = FollowerStatus::kDiverged; output.status = status_; return output;
    }
  }
  output.terminalPhase = terminalPhase_;
  output.chassisSetpoint = command;
  output.wheelSetpoint = kinematics_.toWheelSpeeds(command);
  const double peakSpeed = std::max(std::abs(output.wheelSetpoint.left), std::abs(output.wheelSetpoint.right));
  if (config_.maxWheelVelocity > 0 && peakSpeed > config_.maxWheelVelocity) {
    const double scale = config_.maxWheelVelocity / peakSpeed;
    output.wheelSetpoint.left *= scale; output.wheelSetpoint.right *= scale;
  }
  const double accelerationLimit = terminalRecovery ? config_.terminalMaxWheelAcceleration : config_.maxWheelAcceleration;
  if (accelerationLimit > 0) {
    // Bound the kA transient when changing from tracking to a pivot or from
    // pivoting to translation. Apply one factor to the wheel-command change.
    const double dl = output.wheelSetpoint.left - previousSetpoint_.left;
    const double dr = output.wheelSetpoint.right - previousSetpoint_.right;
    const double peakDelta = std::max(std::abs(dl), std::abs(dr));
    const double allowed = accelerationLimit * dt;
    if (peakDelta > allowed) {
      output.wheelSetpoint.left = previousSetpoint_.left + dl * allowed / peakDelta;
      output.wheelSetpoint.right = previousSetpoint_.right + dr * allowed / peakDelta;
    }
  }
  output.chassisSetpoint = kinematics_.toChassisSpeeds(output.wheelSetpoint);
  const auto nominalWheels = kinematics_.toWheelSpeeds({reference.velocity, reference.angularVelocity});
  // kA tracks planned acceleration, not the derivative of noisy pose feedback.
  // Terminal commands have no moving reference and use the bounded target slew.
  const double leftAcceleration =
      terminalRecovery ? (output.wheelSetpoint.left - previousSetpoint_.left) / dt
                       : (nominalWheels.left - previousReferenceSetpoint_.left) / dt;
  const double rightAcceleration =
      terminalRecovery ? (output.wheelSetpoint.right - previousSetpoint_.right) / dt
                       : (nominalWheels.right - previousReferenceSetpoint_.right) / dt;
  const double voltageLimit = std::max(
      0.0, std::min(std::abs(availableVoltage), config_.nominalVoltage));
  output.voltageLimit = voltageLimit;

  const double leftFf = leftFeedforward_.calculateWithHysteresis(
      output.wheelSetpoint.left, leftAcceleration);
  const double rightFf = rightFeedforward_.calculateWithHysteresis(
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
  // PID output limits may already have clipped to the remaining voltage.
  // Report that saturation even when final scaling is consequently unnecessary.
  output.saturated = std::abs(leftFf) > voltageLimit ||
      std::abs(rightFf) > voltageLimit ||
      std::abs(output.leftVoltage) >= voltageLimit - 1e-9 ||
      std::abs(output.rightVoltage) >= voltageLimit - 1e-9;
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
  previousReferenceSetpoint_ = nominalWheels;
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
  leftFeedforward_.reset();
  rightFeedforward_.reset();
  terminalPhase_ = TerminalPhase::kTracking;
}

}  // namespace vantage
