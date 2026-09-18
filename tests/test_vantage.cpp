#include "vantage/vantage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
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
  // Angular constraints affect bends, not straight-line speeds. Check sampled
  // references as well as stored states: v and curvature interpolate separately.
  auto angularConfig = config;
  angularConfig.maxAngularVelocity = 0.5;
  const auto angularCurve = generateTrajectory(bezierWaypoints, angularConfig);
  expect(angularCurve.duration() > bezierCurve.duration(),
         "angular ceiling slows the bend");
  auto checkAngularCeiling = [&](const Trajectory& path, double limit) {
    for (const auto& state : path.states()) {
      expect(std::abs(state.angularVelocity) <= limit + 1e-9,
             "stored reference respects angular ceiling");
    }
    for (int i = 0; i <= 2000; ++i) {
      const auto state = path.sample(path.duration() * i / 2000.0);
      expect(std::abs(state.angularVelocity) <= limit + 1e-9,
             "interpolated reference respects angular ceiling");
    }
  };
  checkAngularCeiling(angularCurve, angularConfig.maxAngularVelocity);
  const auto angularLine = generateTrajectory(
      {{{0,0,1}, 3, true, {}}, {{2,0,-1}, 3}}, angularConfig);
  near(angularLine.duration(), bezierLine.duration(), 1e-12,
       "angular ceiling does not slow a straight line");
  angularConfig.reversed = true;
  const auto angularReverse = generateTrajectory(bezierWaypoints, angularConfig);
  checkAngularCeiling(angularReverse, angularConfig.maxAngularVelocity);
  near(angularReverse.duration(), angularCurve.duration(), 1e-12,
       "angular ceiling is direction independent");
  auto angularMirror = bezierWaypoints;
  for (auto& waypoint : angularMirror) {
    waypoint.pose.y = -waypoint.pose.y;
    for (auto& control : waypoint.controlPoints) control.y = -control.y;
  }
  angularConfig.reversed = false;
  checkAngularCeiling(generateTrajectory(angularMirror, angularConfig),
                      angularConfig.maxAngularVelocity);
  for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
    angularConfig.maxAngularVelocity = invalid;
    bool rejected = false;
    try { generateTrajectory(bezierWaypoints, angularConfig); }
    catch (const std::invalid_argument&) { rejected = true; }
    expect(rejected, "invalid angular limit is rejected");
  }

  // 4613R commissioning profile, inches/radians. Keep this route as a regression
  // for the curvature envelope and the curve-tuning timeout budget.
  TrajectoryConfig robotCurveConfig;
  robotCurveConfig.maxVelocity = robotCurveConfig.maxWheelVelocity = 52.5;
  robotCurveConfig.maxAcceleration = 35;
  robotCurveConfig.maxDeceleration = 40;
  robotCurveConfig.maxCentripetalAcceleration = 50;
  robotCurveConfig.maxAngularVelocity = 55 * kPi / 180;
  robotCurveConfig.maxVoltage = 10;
  robotCurveConfig.leftFeedforward = {1.25, 0.164743650, 0.025};
  robotCurveConfig.rightFeedforward = robotCurveConfig.leftFeedforward;
  robotCurveConfig.trackWidth = 11;
  robotCurveConfig.sampleDistance = 0.35;
  const auto robotCurve = generateTrajectory(
      {{{24.25,72,0}, 30, true, {{73.5,72.75,0}}},
       {{72.5,48.5,-0.007877}, 28.1976}}, robotCurveConfig);
  checkAngularCeiling(robotCurve, robotCurveConfig.maxAngularVelocity);
  near(robotCurve.length(), 60.2311, 0.001, "robot curve geometry is unchanged");
  expect(robotCurve.duration() + 5.0 < 10,
         "robot curve retains the full settle budget within chassis timeout");
  double peakRobotSpeed = 0;
  for (const auto& state : robotCurve.states()) {
    peakRobotSpeed = std::max(peakRobotSpeed, std::abs(state.velocity));
    if (std::abs(state.curvature) > 0.062) {
      expect(std::abs(state.velocity) < 15.5,
             "tight hook slows to approximately 15.5 inches per second");
    }
  }
  expect(peakRobotSpeed > 30, "gentle sections are not globally limited to hook speed");
  std::cout << "4613R curve: duration=" << robotCurve.duration()
            << "s peak=" << peakRobotSpeed << "in/s\n";
  FollowerConfig extendedSettleConfig;
  extendedSettleConfig.trackWidth = 11;
  extendedSettleConfig.timeoutAfterTrajectory = 5.0;
  extendedSettleConfig.positionTolerance = 1.0;
  extendedSettleConfig.headingTolerance = 2 * kPi / 180;
  extendedSettleConfig.velocityTolerance = 1.5;
  extendedSettleConfig.divergenceLimit = 36;
  extendedSettleConfig.settleCycles = 8;
  extendedSettleConfig.enableTerminalRecovery = true;
  extendedSettleConfig.terminalMaxLinearSpeed = 6;
  extendedSettleConfig.terminalMaxAngularSpeed = .9;
  extendedSettleConfig.terminalMaxWheelAcceleration = 40;
  TrajectoryFollower extendedSettling(extendedSettleConfig);
  extendedSettling.start(robotCurve, 0);
  const auto endpoint = robotCurve.states().back().pose;
  auto offTarget = endpoint;
  offTarget.x -= 3;
  offTarget.theta = wrapAngle(endpoint.theta - 14 * kPi / 180);
  expect(extendedSettling.update(robotCurve.duration() + 1.6,
             offTarget, {}, 12).status == FollowerStatus::kRunning,
         "curve remains active beyond the former 1.5-second settle deadline");
  expect(extendedSettling.update(robotCurve.duration() + 4.9,
             offTarget, {}, 12).status == FollowerStatus::kRunning,
         "stationary off-target curve gets extended time, not false success");
  const auto expired = extendedSettling.update(robotCurve.duration() + 5.01,
                                               offTarget, {}, 12);
  expect(expired.status == FollowerStatus::kTimedOut,
         "extended settling still has a finite safety timeout");
  near(expired.leftVoltage, 0, 1e-12, "timeout stops left output");
  near(expired.rightVoltage, 0, 1e-12, "timeout stops right output");
  extendedSettling.start(robotCurve, 0);
  FollowerOutput recovered;
  for (unsigned i = 0; i < extendedSettleConfig.settleCycles; ++i) {
    recovered = extendedSettling.update(robotCurve.duration() + 2 + i * 0.01,
                                         endpoint, {}, 12);
  }
  expect(recovered.status == FollowerStatus::kSettled,
         "successful recovery settles without waiting for the extended deadline");
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
  bool cornerRejected = false;
  try { generateTrajectory(cornerPath, config); }
  catch (const std::invalid_argument&) { cornerRejected = true; }
  expect(cornerRejected, "rejects sharp Bezier join without a planned turn");
  cornerPath.back().pose = {4,0,0};
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
  expect(reverse.states().back().direction < 0.0,
         "reverse trajectory preserves direction at stopped endpoint");

  DifferentialDriveKinematics kinematics(0.4);
  const WheelSpeeds wheels = kinematics.toWheelSpeeds({1.0, 2.0});
  near(wheels.left, 0.6, 1e-12, "left inverse kinematics");
  near(wheels.right, 1.4, 1e-12, "right inverse kinematics");
  near(kinematics.toChassisSpeeds(wheels).angular, 2.0, 1e-12,
       "forward kinematics round trip");

  NonlinearPoseController controller;
  // Replay a real stalled endpoint in robot-relative coordinates. The damping
  // trial strengthens the heading term without increasing the opposing lateral
  // coefficient; it is not a proof that hardware will converge.
  NonlinearControllerConfig loggedConfig;
  loggedConfig.kp = 0.01;
  loggedConfig.kd = 0.7;
  loggedConfig.minimumFeedbackSpeed = (12 - 1.25) / 0.164743650 * 0.4;
  loggedConfig.maxLinearCorrection = 18;
  loggedConfig.maxAngularCorrection = 1;
  TrajectoryState loggedEndpoint;
  loggedEndpoint.pose = {-0.154634, 2.984124, -14.056616 * kPi / 180};
  const auto oldCorrection = NonlinearPoseController(loggedConfig).calculate(
      {0,0,0}, loggedEndpoint);
  near(-oldCorrection.angular * 180 / kPi, 7.1843224, 1e-5,
       "endpoint replay reproduces logged clockwise turn request");
  loggedConfig.kd = 0.9;
  const auto tunedCorrection = NonlinearPoseController(loggedConfig).calculate(
      {0,0,0}, loggedEndpoint);
  near(-tunedCorrection.angular * 180 / kPi, 21.8600805, 1e-5,
       "damping trial strengthens heading correction at logged pose");
  expect(std::abs(tunedCorrection.angular) <= loggedConfig.maxAngularCorrection,
         "damping trial keeps angular correction bounded");
  auto mirroredEndpoint = loggedEndpoint;
  mirroredEndpoint.pose.y *= -1;
  mirroredEndpoint.pose.theta *= -1;
  const auto mirroredCorrection = NonlinearPoseController(loggedConfig).calculate(
      {0,0,0}, mirroredEndpoint);
  near(mirroredCorrection.angular, -tunedCorrection.angular, 1e-12,
       "damping trial corrects mirrored errors symmetrically");
  near(mirroredCorrection.linear, tunedCorrection.linear, 1e-12,
       "mirroring does not change longitudinal correction");
  TrajectoryState atTarget;
  const auto zeroCorrection = NonlinearPoseController(loggedConfig).calculate(
      {0,0,0}, atTarget);
  near(zeroCorrection.linear, 0, 1e-12, "no linear correction at target");
  near(zeroCorrection.angular, 0, 1e-12, "no angular correction at target");
  TrajectoryState reference;
  reference.pose = {1, 0.2, 0};
  reference.velocity = 1.0;
  const ChassisSpeeds correction = controller.calculate({0, 0, 0}, reference);
  expect(correction.linear > 1.0, "controller corrects longitudinal lag");
  expect(correction.angular > 0.0, "controller steers toward positive lateral error");

  TrajectoryState reverseReference;
  reverseReference.pose = {0, 0.2, 0};
  reverseReference.velocity = -1.0;
  reverseReference.direction = -1.0;
  const ChassisSpeeds reverseCorrection =
      controller.calculate({0, 0, 0}, reverseReference);
  expect(reverseCorrection.angular < 0.0,
         "reverse controller preserves lateral correction sign");
  reverseReference.velocity = 0.0;
  const ChassisSpeeds stoppedReverseCorrection =
      controller.calculate({0, 0, 0}, reverseReference);
  expect(stoppedReverseCorrection.angular < 0.0,
         "reverse direction survives a zero-speed endpoint");

  MotorFeedforward feedforward({0.2, 2.0, 0.5});
  near(feedforward.calculate(1.0, 2.0), 3.2, 1e-12,
       "feedforward combines static, velocity, acceleration");
  MotorFeedforward friction({1.25, 0.164743650, 0.025, 2.0, 0.25});
  near(friction.calculateWithHysteresis(0.1, 0), 0.016474365, 1e-9,
       "tiny command does not engage static compensation");
  near(friction.calculateWithHysteresis(1.220111, 0), 1.4510055395, 1e-9,
       "intentional slow motion receives full calibrated kS");
  expect(friction.calculateWithHysteresis(0.2, 0) > 1.25,
         "hysteresis retains active sign above release threshold");
  near(friction.calculateWithHysteresis(-0.1, 0), -0.016474365, 1e-9,
       "small reversal clears old sign rather than pushing wrong way");
  expect(friction.calculateWithHysteresis(-0.3, 0) < -1.25,
         "intentional reverse command gets reverse static compensation");
  near(friction.calculateWithHysteresis(0, 0), 0, 1e-12,
       "zero command never receives static compensation");
  friction.calculateWithHysteresis(0.3, 0);
  friction.reset();
  near(friction.calculateWithHysteresis(0.2, 0), 0.03294873, 1e-9,
       "new run resets friction latch");

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

  TrajectoryFollower invalidInputFollower(followerConfig);
  invalidInputFollower.start(straight, 0.0);
  const FollowerOutput invalidInput = invalidInputFollower.update(
      0.1, {std::numeric_limits<double>::quiet_NaN(), 0, 0}, {}, 12.0);
  expect(invalidInput.status == FollowerStatus::kDiverged,
         "non-finite localization fails safe instead of commanding NaN voltage");

  FollowerConfig isolatedConfig = followerConfig;
  FollowerConfig curveOnlyConfig = followerConfig;
  curveOnlyConfig.stopAtProfileEnd = true;
  TrajectoryFollower curveOnly(curveOnlyConfig);
  curveOnly.start(straight, 0.0);
  expect(curveOnly.update(straight.duration() * .5, {}, {}, 12).status ==
             FollowerStatus::kRunning, "curve-only still tracks before profile end");
  const auto curveEnd = curveOnly.update(straight.duration(), {}, {1, -1}, 12);
  expect(curveEnd.status == FollowerStatus::kProfileComplete,
         "curve-only completion does not claim settled");
  near(curveEnd.leftVoltage, 0, 1e-12, "curve-only end stops left output");
  near(curveEnd.rightVoltage, 0, 1e-12, "curve-only end stops right output");
  expect(curveEnd.terminalPhase == TerminalPhase::kTracking,
         "curve-only cannot enter recovery");
  near(curveOnly.update(straight.duration()+1, {}, {}, 12).leftVoltage,
       0, 1e-12, "curve-only stays stopped");
  isolatedConfig.enablePoseFeedback = false;
  isolatedConfig.enableVelocityFeedback = false;
  TrajectoryFollower isolatedFollower(isolatedConfig);
  isolatedFollower.start(straight, 0.0);
  const FollowerOutput isolated = isolatedFollower.update(
      straight.duration() * 0.5, {0, 1, 0}, {100, -100}, 12.0);
  expect(!isolated.poseFeedbackActive && !isolated.velocityFeedbackActive,
         "follower reports isolated feedback loops");
  near(isolated.chassisSetpoint.linear, isolated.reference.velocity, 1e-12,
       "disabled pose loop leaves reference linear speed unchanged");
  near(isolated.leftFeedbackVoltage, 0.0, 1e-12,
       "disabled wheel loop contributes no left feedback voltage");

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

  // Endpoint recovery from the logged 4.31-inch sideways / 15-degree error.
  // Simulated voltage-driven plant includes static friction and wheel inertia;
  // it is regression evidence, not validation of the real drivetrain model.
  auto recoveryConfig = extendedSettleConfig;
  recoveryConfig.poseController = {0.01, 0.9, 26.1011578, 18, 1};
  recoveryConfig.leftFeedforward = {1.25, 0.164743650, 0.025, 2, 0.25};
  recoveryConfig.rightFeedforward = recoveryConfig.leftFeedforward;
  recoveryConfig.leftVelocityPid = {0.35, 0, 0, 0, 0.02};
  recoveryConfig.rightVelocityPid = recoveryConfig.leftVelocityPid;
  recoveryConfig.enableTerminalRecovery = true;
  recoveryConfig.terminalMaxLinearSpeed = 6;
  recoveryConfig.terminalMaxAngularSpeed = 0.9;
  recoveryConfig.terminalMaxWheelAcceleration = 40;
  const DifferentialDriveKinematics recoveryKinematics(11);
  for (const double mirror : {-1.0, 1.0}) {
    // Build equivalent robot-frame error about the actual trajectory endpoint.
    Pose2d p = endpoint;
    p.theta = wrapAngle(endpoint.theta - mirror * 15.017423 * kPi / 180);
    const double ex = -0.120631, ey = mirror * 4.311939;
    p.x -= std::cos(p.theta) * ex - std::sin(p.theta) * ey;
    p.y -= std::sin(p.theta) * ex + std::cos(p.theta) * ey;
    WheelSpeeds speeds{}, lastTargets{};
    TrajectoryFollower recoveryFollower(recoveryConfig);
    recoveryFollower.start(robotCurve, 0);
    bool sawDrive = false, sawAlign = false;
    double finishTime = 0;
    for (double t = robotCurve.duration() + dt;
         t < robotCurve.duration() + 5.1; t += dt) {
      const auto out = recoveryFollower.update(t, p, speeds, 12);
      if (out.status != FollowerStatus::kRunning) { finishTime = t; break; }
      sawDrive |= out.terminalPhase == TerminalPhase::kDriveToPosition;
      sawAlign |= out.terminalPhase == TerminalPhase::kAlignHeading;
      expect(std::abs(out.leftVoltage) <= 12 && std::abs(out.rightVoltage) <= 12,
             "terminal voltage remains bounded");
      expect(std::abs(out.chassisSetpoint.linear) <= 6.000001 &&
             std::abs(out.chassisSetpoint.angular) <= 0.900001,
             "terminal maneuver obeys commissioning speed caps");
      if (std::abs(out.leftVoltage) + std::abs(out.rightVoltage) > 1e-9) {
        expect(std::abs(out.wheelSetpoint.left - lastTargets.left) <= 40*out.dt+1e-6 &&
               std::abs(out.wheelSetpoint.right - lastTargets.right) <= 40*out.dt+1e-6,
               "terminal transitions limit wheel acceleration");
      }
      lastTargets = out.wheelSetpoint;
      auto plant = [&](double v, double voltage) {
        if (std::abs(v) < 0.03 && std::abs(voltage) <= 1.25) return 0.0;
        const double sign = std::abs(v) < 0.03 ? std::copysign(1.0, voltage)
                                              : std::copysign(1.0, v);
        const double next = v + dt * (voltage - sign * 1.25 - 0.164743650 * v) / 0.025;
        return v * next < 0 && std::abs(voltage) <= 1.25 ? 0.0 : next;
      };
      speeds = {plant(speeds.left, out.leftVoltage), plant(speeds.right, out.rightVoltage)};
      const auto motion = recoveryKinematics.toChassisSpeeds(speeds);
      const double midpoint = p.theta + motion.angular * dt * 0.5;
      p.x += motion.linear * dt * std::cos(midpoint);
      p.y += motion.linear * dt * std::sin(midpoint);
      p.theta = wrapAngle(p.theta + motion.angular * dt);
    }
    std::cout << "Endpoint recovery mirror=" << mirror << " status="
              << static_cast<int>(recoveryFollower.status()) << " time="
              << finishTime - robotCurve.duration() << "s error="
              << std::hypot(p.x-endpoint.x,p.y-endpoint.y) << "in\n";
    expect(sawDrive && sawAlign, "terminal recovery closes position then aligns heading");
    expect(recoveryFollower.status() == FollowerStatus::kSettled,
           "P-only voltage/friction simulation recovers logged endpoint within deadline");
    recoveryFollower.cancel();
    const auto cancelled = recoveryFollower.update(100, p, speeds, 12);
    near(cancelled.leftVoltage, 0, 1e-12, "cancel stops terminal recovery left output");
    near(cancelled.rightVoltage, 0, 1e-12, "cancel stops terminal recovery right output");
  }

  // Isolation disables recovery as well as nonlinear feedback.
  auto noPoseConfig = recoveryConfig;
  noPoseConfig.enablePoseFeedback = false;
  noPoseConfig.enableVelocityFeedback = false;
  TrajectoryFollower noPoseRecovery(noPoseConfig);
  noPoseRecovery.start(robotCurve, 0);
  const auto noPoseOutput = noPoseRecovery.update(robotCurve.duration()+0.01, offTarget, {}, 12);
  expect(noPoseOutput.terminalPhase == TerminalPhase::kTracking,
         "pose isolation never activates terminal recovery");
  near(noPoseOutput.leftVoltage, 0, 1e-12, "isolated stopped reference has zero output");

  TrajectoryFollower stalledRecovery(recoveryConfig);
  stalledRecovery.start(robotCurve, 0);
  const auto stalledOutput = stalledRecovery.update(robotCurve.duration()+5.01, offTarget, {}, 12);
  expect(stalledOutput.status == FollowerStatus::kTimedOut,
         "terminal recovery keeps finite stall deadline");
  near(stalledOutput.leftVoltage, 0, 1e-12, "timed-out recovery stops left wheel");
  near(stalledOutput.rightVoltage, 0, 1e-12, "timed-out recovery stops right wheel");

  auto clipConfig = recoveryConfig;
  clipConfig.nominalVoltage = 0.5;
  TrajectoryFollower clippedRecovery(clipConfig);
  clippedRecovery.start(robotCurve, 0);
  const auto clippedOutput = clippedRecovery.update(robotCurve.duration()+0.01, offTarget, {}, 12);
  expect(clippedOutput.saturated, "PID-clipped feedforward saturation is visible");
  expect(std::abs(clippedOutput.leftVoltage) <= 0.500001 &&
         std::abs(clippedOutput.rightVoltage) <= 0.500001,
         "terminal recovery never bypasses voltage limit");

  if (failures == 0) std::cout << "All VantagePath tests passed\n";
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
