#include "vantage/vantage.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
int failures = 0;

void expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void near(double actual, double expected, double tolerance,
          const std::string& message) {
  expect(std::abs(actual - expected) <= tolerance,
         message + " (actual=" + std::to_string(actual) + ")");
}
}  // namespace

int main() {
  using namespace vantage;

  const FieldDimensions vexField{144.0, 144.0};
  const Pose2d gpsOrigin = cornerToVexGps({72.0, 72.0, kPi * 0.5}, vexField);
  near(gpsOrigin.x, 0.0, 1e-12, "corner frame converts GPS centre x");
  near(gpsOrigin.y, 0.0, 1e-12, "corner frame converts GPS centre y");
  near(gpsOrigin.theta, 0.0, 1e-12, "corner heading converts to GPS north");
  const Pose2d cornerRoundTrip = vexGpsToCorner(gpsOrigin, vexField);
  near(cornerRoundTrip.x, 72.0, 1e-12, "GPS conversion round trips x");
  near(cornerRoundTrip.y, 72.0, 1e-12, "GPS conversion round trips y");
  near(cornerRoundTrip.theta, kPi * 0.5, 1e-12,
       "GPS conversion round trips heading");
  const Pose2d allianceMirror = mirrorPose(
      {12.0, 24.0, 0.25}, FieldMirror::kAlliance180, vexField);
  near(allianceMirror.x, 132.0, 1e-12, "alliance mirror transforms x");
  near(allianceMirror.y, 120.0, 1e-12, "alliance mirror transforms y");
  near(allianceMirror.theta, wrapAngle(0.25 + kPi), 1e-12,
       "alliance mirror transforms heading");
  const auto reordered = reverseWaypoints(
      {{{10.0, 20.0, 0.0}, 4.0}, {{30.0, 40.0, kPi * 0.5}, 8.0}});
  near(reordered.front().pose.x, 30.0, 1e-12, "path order reverses");
  near(reordered.front().pose.theta, -kPi * 0.5, 1e-12,
       "reversed path tangent faces into path");
  near(reordered.front().tangentScale, 8.0, 1e-12,
       "reversed path keeps tangent scale");

  TrajectoryConfig config;
  config.maxVelocity = 2.0;
  config.maxAcceleration = 1.0;
  config.maxDeceleration = 1.25;
  config.maxWheelVelocity = 2.1;
  config.trackWidth = 0.4;
  config.sampleDistance = 0.03;
  // Bézier segments share the same constrained trajectory generator.
  std::vector<Waypoint> bezierWaypoints = {
      {{0, 0, 1.0}, 3.0, true, {}}, {{2, 0, -1.0}, 3.0}};
  const auto bezierLine = generateTrajectory(bezierWaypoints, config);
  near(bezierLine.length(), 2.0, 1e-9, "zero controls is a line despite endpoint headings");
  for (const auto& state : bezierLine.states()) near(state.pose.y, 0.0, 1e-12, "line stays straight");
  bezierWaypoints[0].controlPoints = {{0.2,0.2,0}, {0.4,0.6,0}, {0.7,1.0,0}, {1.3,1.0,0}, {1.6,0.6,0}, {1.8,0.2,0}};
  const auto bezierCurve = generateTrajectory(bezierWaypoints, config);
  expect(bezierCurve.length() > 2.0, "six controls bend the path");
  near(bezierCurve.states().front().pose.x, 0.0, 1e-12, "Bezier start");
  near(bezierCurve.states().back().pose.x, 2.0, 1e-12, "Bezier end");
  bool reachesArch = false;
  for (const auto& state : bezierCurve.states()) {
    reachesArch |= state.pose.y > 0.65;
    expect(std::isfinite(state.time) && std::isfinite(state.curvature), "Bezier states are finite");
    expect(std::abs(state.velocity) <= config.maxVelocity + 1e-9, "Bezier respects speed limit");
    expect(std::abs(state.velocity * state.velocity * state.curvature) <= config.maxCentripetalAcceleration + 1e-9, "Bezier respects centripetal limit");
  }
  expect(reachesArch, "six-control Bernstein arch evaluated");
  near(bezierCurve.states()[bezierCurve.states().size()/2].pose.y,
       0.765625, 0.001, "six-control curve agrees with Bernstein midpoint");
  const auto repeatedControls = generateTrajectory(
      {{{0,0,0}, 0, true, {{0,0,0}, {0,2,0}}}, {{0,2,0}, 0}}, config);
  near(repeatedControls.states().front().pose.theta, kPi/2, 1e-9,
       "repeated starting control keeps vertical tangent");
  near(repeatedControls.states().back().pose.theta, kPi/2, 1e-9,
       "repeated ending control keeps vertical tangent");
  auto backwardsConfig = config; backwardsConfig.reversed = true;
  const auto backwardsBezier = generateTrajectory(bezierWaypoints, backwardsConfig);
  near(backwardsBezier.length(), bezierCurve.length(), 1e-12, "reverse drive retains Bezier geometry");
  expect(backwardsBezier.states()[1].velocity < 0, "Bezier supports reverse drive");
  const auto reversedBezier = reverseWaypoints(bezierWaypoints);
  expect(reversedBezier[0].bezierToNext && reversedBezier[0].controlPoints.size() == 6, "reverse transfers outgoing controls");
  near(reversedBezier[0].controlPoints.front().x, 1.8, 1e-12, "reverse reorders controls");
  near(generateTrajectory(reversedBezier, config).length(), bezierCurve.length(), 1e-9, "reverse order preserves shape");
  const auto mirroredBezier = mirrorWaypoints(bezierWaypoints, FieldMirror::kLeftRight, {2,2});
  near(mirroredBezier[0].controlPoints.front().x, 1.8, 1e-12, "mirror transforms controls");
  near(generateTrajectory(mirroredBezier, config).length(), bezierCurve.length(), 1e-9, "mirror preserves length");
  auto cornerPath = bezierWaypoints;
  cornerPath[0].controlPoints.clear(); cornerPath[1].bezierToNext = true;
  cornerPath.push_back({{2,2,0}, 1.0});
  const auto cornerTrajectory = generateTrajectory(cornerPath, config);
  for (const auto& state : cornerTrajectory.states()) {
    if (std::abs(state.pose.x-2) < 1e-9 && std::abs(state.pose.y) < 1e-9)
      near(state.velocity, 0, 1e-12, "stops at independently authored segment join");
  }

  const Trajectory straight = generateTrajectory(
      {{{0, 0, 0}}, {{2, 0, 0}}}, config);
  expect(!straight.empty(), "straight trajectory generated");
  near(straight.length(), 2.0, 0.01, "straight trajectory length");
  near(straight.states().front().velocity, 0.0, 1e-12, "starts stopped");
  near(straight.states().back().velocity, 0.0, 1e-12, "ends stopped");
  for (std::size_t i = 1; i < straight.states().size(); ++i) {
    expect(straight.states()[i].time >= straight.states()[i - 1].time,
           "trajectory time is monotonic");
    expect(std::abs(straight.states()[i].velocity) <= config.maxVelocity + 1e-9,
           "trajectory respects chassis velocity");
  }

  TrajectoryConfig voltageConfig = config;
  voltageConfig.maxVelocity = 4.0;
  voltageConfig.maxWheelVelocity = 4.0;
  voltageConfig.maxAcceleration = 10.0;
  voltageConfig.maxDeceleration = 10.0;
  voltageConfig.maxVoltage = 4.0;
  voltageConfig.leftFeedforward = {0.2, 2.0, 0.5};
  voltageConfig.rightFeedforward = voltageConfig.leftFeedforward;
  const Trajectory voltageLimited = generateTrajectory(
      {{{0, 0, 0}}, {{4, 0, 0}}}, voltageConfig);
  for (const auto& state : voltageLimited.states()) {
    const double sign = state.velocity > 1e-9 ? 1.0 : 0.0;
    const double plannedVoltage = 0.2 * sign + 2.0 * state.velocity +
                                  0.5 * state.acceleration;
    expect(std::abs(plannedVoltage) <= voltageConfig.maxVoltage + 0.08,
           "time parameterization respects feedforward voltage");
  }

  const Trajectory curve = generateTrajectory(
      {{{0, 0, 0}}, {{1, 1, vantage::kPi / 2.0}}}, config);
  for (const auto& state : curve.states()) {
    const double left = state.velocity *
                        (1.0 - state.curvature * config.trackWidth * 0.5);
    const double right = state.velocity *
                         (1.0 + state.curvature * config.trackWidth * 0.5);
    expect(std::max(std::abs(left), std::abs(right)) <=
               config.maxWheelVelocity + 1e-8,
           "curve respects wheel velocity");
    expect(state.velocity * state.velocity * std::abs(state.curvature) <=
               config.maxCentripetalAcceleration + 1e-8,
           "curve respects centripetal acceleration");
  }

  config.reversed = true;
  const Trajectory reverse = generateTrajectory(
      {{{0, 0, 0}}, {{1, 0, 0}}}, config);
  expect(reverse.states()[reverse.states().size() / 2].velocity < 0,
         "reverse trajectory has negative velocity");
  near(std::abs(reverse.states().front().pose.theta), vantage::kPi, 1e-9,
       "reverse trajectory faces opposite its geometric tangent");

  DifferentialDriveKinematics kinematics(0.4);
  const WheelSpeeds wheels = kinematics.toWheelSpeeds({1.0, 2.0});
  near(wheels.left, 0.6, 1e-12, "left inverse kinematics");
  near(wheels.right, 1.4, 1e-12, "right inverse kinematics");
  near(kinematics.toChassisSpeeds(wheels).angular, 2.0, 1e-12,
       "forward kinematics round trip");

  NonlinearPoseController controller;
  TrajectoryState reference;
  reference.pose = {1, 0.2, 0};
  reference.velocity = 1.0;
  const ChassisSpeeds correction = controller.calculate({0, 0, 0}, reference);
  expect(correction.linear > 1.0, "controller corrects longitudinal lag");
  expect(correction.angular > 0.0, "controller steers toward positive lateral error");

  MotorFeedforward feedforward({0.2, 2.0, 0.5});
  near(feedforward.calculate(1.0, 2.0), 3.2, 1e-12,
       "feedforward combines static, velocity, acceleration");

  DifferentialDriveOdometry odometry;
  odometry.update(0, 0, 0);
  Pose2d pose = odometry.update(1, 1, 0);
  near(pose.x, 1.0, 1e-9, "odometry integrates forward travel");
  near(pose.y, 0.0, 1e-9, "straight odometry has no lateral travel");

  IncrementalSensorFusion encoderFusion(3, {1.0, 0.5, true});
  encoderFusion.update({0.0, 10.0, 100.0});
  near(encoderFusion.update({5.0, 15.0, 105.0}).value, 5.0, 1e-12,
       "fusion averages matching sensor deltas with different zeros");
  const FusionOutput rejectedEncoder =
      encoderFusion.update({10.0, 20.0, 205.0});
  near(rejectedEncoder.value, 10.0, 1e-12,
       "fusion rejects an encoder spike backed by peer consensus");
  expect(rejectedEncoder.contributing == 2 && encoderFusion.rejected(2),
         "fusion reports rejected encoder and contributor count");
  const double dead = std::numeric_limits<double>::infinity();
  near(encoderFusion.update({15.0, 25.0, dead}).value, 15.0, 1e-12,
       "fusion continues after one sensor disconnects");
  encoderFusion.update({20.0, 30.0, 210.0});
  near(encoderFusion.update({25.0, 35.0, 215.0}).value, 25.0, 1e-12,
       "returning sensor reseeds without injecting its missing interval");

  IncrementalSensorFusion imuFusion(2, {3.0, 0.0, true});
  imuFusion.update({0.0, 0.0});
  near(imuFusion.update({1.0, 1.2}).value, 1.1, 1e-12,
       "dual IMU fusion averages agreeing changes");
  const FusionOutput imuGlitch = imuFusion.update({2.0, 20.0});
  near(imuGlitch.value, 2.1, 1e-12,
       "dual IMU fusion follows the continuous sensor during a glitch");
  expect(imuGlitch.contributing == 1 && imuFusion.rejected(1),
         "dual IMU fusion exposes the rejected sensor");

  FollowerConfig followerConfig;
  followerConfig.trackWidth = config.trackWidth;
  followerConfig.leftFeedforward.velocityGain = 1.0;
  followerConfig.rightFeedforward.velocityGain = 1.0;
  followerConfig.divergenceLimit = 5.0;
  TrajectoryFollower follower(followerConfig);
  follower.start(straight, 0.0);
  const FollowerOutput output = follower.update(
      straight.duration() * 0.5, straight.sample(straight.duration() * 0.5).pose,
      {}, 12.0);
  expect(output.status == FollowerStatus::kRunning, "follower runs mid-path");
  expect(output.leftVoltage > 0 && output.rightVoltage > 0,
         "follower commands both wheels on straight path");

  // Once a completed trajectory is inside every tolerance, the settle window
  // must observe the stopped robot without re-energising it. Otherwise the
  // terminal pose correction and +/-kS can repeatedly kick it across the goal.
  FollowerConfig settleConfig = followerConfig;
  settleConfig.positionTolerance = 0.1;
  settleConfig.headingTolerance = 0.1;
  settleConfig.velocityTolerance = 0.1;
  settleConfig.settleCycles = 3;
  settleConfig.leftFeedforward.staticGain = 0.2;
  settleConfig.rightFeedforward.staticGain = 0.2;
  TrajectoryFollower settlingFollower(settleConfig);
  settlingFollower.start(straight, 0.0);
  const Pose2d nearEnd{straight.states().back().pose.x - 0.05,
                       straight.states().back().pose.y,
                       straight.states().back().pose.theta};
  for (unsigned cycle = 0; cycle < settleConfig.settleCycles; ++cycle) {
    const FollowerOutput settling = settlingFollower.update(
        straight.duration() + cycle * 0.01, nearEnd, {}, 12.0);
    near(settling.leftVoltage, 0.0, 1e-12,
         "settle confirmation does not re-energise left wheel");
    near(settling.rightVoltage, 0.0, 1e-12,
         "settle confirmation does not re-energise right wheel");
  }
  expect(settlingFollower.status() == FollowerStatus::kSettled,
         "settle confirmation still reaches settled state");

  // End-to-end kinematic simulation with perfect wheel-speed actuators. This
  // exercises timed sampling, nonlinear feedback, kinematics, and settling on
  // a changing-curvature S path rather than only isolated formulas.
  config.reversed = false;
  config.maxVelocity = 1.2;
  config.maxAcceleration = 1.5;
  const Trajectory sCurve = generateTrajectory(
      {{{0, 0, 0}}, {{0.8, 0.45, 0.2}}, {{1.6, 0, 0}}}, config);
  followerConfig.positionTolerance = 0.04;
  followerConfig.headingTolerance = 0.05;
  followerConfig.velocityTolerance = 0.08;
  followerConfig.settleCycles = 5;
  followerConfig.timeoutAfterTrajectory = 2.0;
  TrajectoryFollower simulatedFollower(followerConfig);
  Pose2d simulatedPose{};
  WheelSpeeds simulatedWheels{};
  simulatedFollower.start(sCurve, 0.0);
  constexpr double dt = 0.01;
  for (double time = dt; time < sCurve.duration() + 2.0; time += dt) {
    const FollowerOutput command = simulatedFollower.update(
        time, simulatedPose, simulatedWheels, 12.0);
    simulatedWheels = command.wheelSetpoint;
    const ChassisSpeeds chassis = kinematics.toChassisSpeeds(simulatedWheels);
    const double dtheta = chassis.angular * dt;
    const double localX = chassis.linear * dt * sinc(dtheta);
    const double localY = chassis.linear * dt *
        (std::abs(dtheta) < 1e-8 ? dtheta * 0.5
                                 : (1.0 - std::cos(dtheta)) / dtheta);
    const double c = std::cos(simulatedPose.theta);
    const double s = std::sin(simulatedPose.theta);
    simulatedPose.x += c * localX - s * localY;
    simulatedPose.y += s * localX + c * localY;
    simulatedPose.theta = wrapAngle(simulatedPose.theta + dtheta);
    if (simulatedFollower.status() != FollowerStatus::kRunning) break;
  }
  expect(simulatedFollower.status() == FollowerStatus::kSettled,
         "S-curve simulation reaches stable settled state");
  near(simulatedPose.x, sCurve.states().back().pose.x, 0.04,
       "S-curve endpoint x");
  near(simulatedPose.y, sCurve.states().back().pose.y, 0.04,
       "S-curve endpoint y");

  if (failures == 0) std::cout << "All VantagePath tests passed\n";
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
