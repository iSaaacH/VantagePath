#pragma once

#include "vantage/trajectory.hpp"

namespace vantage {

struct ChassisSpeeds {
  double linear = 0.0;
  double angular = 0.0;
};

struct WheelSpeeds {
  double left = 0.0;
  double right = 0.0;
};

class DifferentialDriveKinematics {
 public:
  explicit DifferentialDriveKinematics(double trackWidth);
  WheelSpeeds toWheelSpeeds(const ChassisSpeeds& speeds) const;
  ChassisSpeeds toChassisSpeeds(const WheelSpeeds& speeds) const;
  double trackWidth() const { return trackWidth_; }

 private:
  double trackWidth_;
};

struct NonlinearControllerConfig {
  // Base gain schedule is Ramsete-style,
  // not an independent axis PID: kp is the spatial convergence coefficient
  // (traditionally "b") and kd is the damping ratio (traditionally "zeta").
  double kp = 2.0;
  double kd = 0.75;  // 0 < kd <= 1
  // Keeps terminal feedback controllable after reference velocity reaches 0.
  double minimumFeedbackSpeed = 0.10;
  double maxLinearCorrection = 0.75;
  double maxAngularCorrection = 4.0;
  // Independent correction weights; 1 preserves the original controller.
  // Keep kp/kd fixed when tuning these. Feedforward is not scaled.
  double longitudinalScale = 1.0;
  double lateralScale = 1.0;
  double headingScale = 1.0;
};

// A velocity-scheduled nonlinear unicycle tracker. Unlike a point PID, it
// closes the loop on the full time-indexed pose and preserves smooth reference
// velocity/curvature feedforward.
class NonlinearPoseController {
 public:
  explicit NonlinearPoseController(NonlinearControllerConfig config = {});
  ChassisSpeeds calculate(const Pose2d& current,
                          const TrajectoryState& reference,
                          double lateralMultiplier = 1.0) const;
  PoseError error(const Pose2d& current,
                  const TrajectoryState& reference) const;

 private:
  NonlinearControllerConfig config_;
};

struct FeedforwardConfig {
  double staticGain = 0.0;
  double velocityGain = 1.0;
  double accelerationGain = 0.0;
  // Fade static friction compensation in over this velocity magnitude.
  // Zero preserves the original sign-based compensation.
  double staticVelocityDeadband = 0.0;
  // Optional follower-only Schmitt threshold: full kS above this speed,
  // release below half of it. Zero retains the stateless fade behavior.
  double staticActivationVelocity = 0.0;
};

class MotorFeedforward {
 public:
  explicit MotorFeedforward(FeedforwardConfig config = {});
  double calculate(double velocity, double acceleration) const;
  double calculateWithHysteresis(double velocity, double acceleration);
  void reset() { staticDirection_ = 0.0; }

 private:
  FeedforwardConfig config_;
  double staticDirection_ = 0.0;
};

struct VelocityPidConfig {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integralLimit = 0.0;
  double derivativeTimeConstant = 0.02;
};

class VelocityPid {
 public:
  explicit VelocityPid(VelocityPidConfig config = {});
  double calculate(double setpoint, double measurement, double dt,
                   double minOutput, double maxOutput);
  void reset();

 private:
  VelocityPidConfig config_;
  double integral_ = 0.0;
  double previousMeasurement_ = 0.0;
  double filteredDerivative_ = 0.0;
  bool initialized_ = false;
};

}  // namespace vantage
