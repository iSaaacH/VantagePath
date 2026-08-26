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
};

enum class FollowerStatus { kIdle, kRunning, kSettled, kTimedOut, kDiverged };

struct FollowerOutput {
  double leftVoltage = 0.0;
  double rightVoltage = 0.0;
  WheelSpeeds wheelSetpoint;
  PoseError poseError;
  FollowerStatus status = FollowerStatus::kIdle;
  bool saturated = false;
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
  WheelSpeeds previousSetpoint_;
  FollowerStatus status_ = FollowerStatus::kIdle;
  unsigned settledCycles_ = 0;
};

}  // namespace vantage
