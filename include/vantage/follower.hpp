#pragma once

#include "vantage/controller.hpp"

namespace vantage {

struct FollowerConfig {
  NonlinearControllerConfig poseController;
  FeedforwardConfig leftFeedforward;
  FeedforwardConfig rightFeedforward;
  VelocityPidConfig leftVelocityPid;
  VelocityPidConfig rightVelocityPid;
  double trackWidth = 0.30;
  double nominalVoltage = 12.0;
  double positionTolerance = 0.03;
  double headingTolerance = 0.04;
  double velocityTolerance = 0.05;
  double divergenceLimit = 1.0;
  double timeoutAfterTrajectory = 1.0;
  unsigned settleCycles = 8;
  // Isolation switches for commissioning. Feedforward always remains active;
  // these independently enable the outer pose loop and inner wheel loop.
  bool enablePoseFeedback = true;
  bool enableVelocityFeedback = true;
  // Opt-in stop-at-end recovery, never applied to rolling endpoints.
  bool enableTerminalRecovery = false;
  // Commissioning only: zero output at profile end, without claiming settled.
  bool stopAtProfileEnd = false;
  double terminalMaxLinearSpeed = 0.0;
  double terminalMaxAngularSpeed = 0.0;
  double terminalMaxWheelAcceleration = 0.0;
  // Optional corrected-command limits, in the same distance units as the path.
  double maxWheelVelocity = 0.0;
  double maxWheelAcceleration = 0.0;
  // Optional tracking velocity floor (distance units/s). Reference ramp,
  // heading alignment, endpoint approach and wheel limits take priority.
  double minimumTrackingVelocity = 0.0;
  // Zero disables these additional recovery guards (legacy library clients).
  double terminalMaxPositionError = 0.0;
  double terminalProgressTimeout = 0.0;
  // Opt-in, pre-completion lateral-only taper for a stopped endpoint.
  // Limited to the final monotonic braking segment; zero duration disables.
  double brakingLateralTaperSeconds = 0.0;
  double brakingLateralEndMultiplier = 1.0;
  // Opt-in pre-profile-end point approach. Never pivots or reverses drive
  // direction to chase an endpoint; zero window retains legacy tracking.
  double pointApproachSeconds = 0.0;
  double pointApproachBlendSeconds = 0.2;
  double pointApproachTolerance = 0.35;
  double pointApproachGuardDistance = 1.0;
  double pointApproachMaxCurvature = 0.14;
  double pointApproachMaxAngularSpeed = 0.75;
  double pointApproachDeceleration = 35.0;
  // Smooth the angular-speed ceiling to zero before the profile deadline.
  // Opt-in; does not add a heading target or post-profile movement.
  double pointApproachAngularBrakeSeconds = 0.0;
};

enum class TerminalPhase { kTracking, kBraking, kRotateToPosition, kDriveToPosition,
                           kAlignHeading };
enum class FollowerStatus { kIdle, kRunning, kSettled, kTimedOut, kDiverged, kProfileComplete,
                            kPositionComplete, kEndpointUnreachable };

struct FollowerOutput {
  double leftVoltage = 0.0;
  double rightVoltage = 0.0;
  WheelSpeeds wheelSetpoint;
  PoseError poseError;
  TrajectoryState reference;
  ChassisSpeeds chassisSetpoint;
  double leftFeedforwardVoltage = 0.0;
  double rightFeedforwardVoltage = 0.0;
  double leftFeedbackVoltage = 0.0;
  double rightFeedbackVoltage = 0.0;
  double elapsed = 0.0;
  double dt = 0.0;
  double voltageLimit = 0.0;
  FollowerStatus status = FollowerStatus::kIdle;
  bool saturated = false;
  bool poseFeedbackActive = false;
  bool velocityFeedbackActive = false;
  TerminalPhase terminalPhase = TerminalPhase::kTracking;
  double lateralFeedbackMultiplier = 1.0;
  double pointApproachBlend = 0.0;
};

class TrajectoryFollower {
 public:
  explicit TrajectoryFollower(FollowerConfig config);
  void start(const Trajectory& trajectory, double nowSeconds);
  FollowerOutput update(double nowSeconds, const Pose2d& pose,
                        const WheelSpeeds& measuredWheelSpeeds,
                        double availableVoltage);
  void cancel();
  FollowerStatus status() const { return status_; }

 private:
  FollowerConfig config_;
  DifferentialDriveKinematics kinematics_;
  NonlinearPoseController poseController_;
  MotorFeedforward leftFeedforward_;
  MotorFeedforward rightFeedforward_;
  VelocityPid leftPid_;
  VelocityPid rightPid_;
  const Trajectory* trajectory_ = nullptr;
  double startTime_ = 0.0;
  double previousTime_ = 0.0;
  double brakingLateralTaperDuration_ = 0.0;
  WheelSpeeds previousSetpoint_;
  WheelSpeeds previousReferenceSetpoint_;
  FollowerStatus status_ = FollowerStatus::kIdle;
  unsigned settledCycles_ = 0;
  TerminalPhase terminalPhase_ = TerminalPhase::kTracking;
  double terminalDirection_ = 1.0;
  double terminalBestError_ = 0.0;
  double terminalProgressTime_ = 0.0;
};

}  // namespace vantage
