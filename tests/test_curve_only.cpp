// Regression tests for curve-only stopping and independent correction weights.
// These test commands, NOT a fitted robot model or predicted endpoint accuracy.
#include "vantage/vantage.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace vantage;
int failures = 0;
void check(bool ok, const char* message) {
  if (!ok) { if (failures < 20) std::cerr << "FAIL: " << message << '\n'; ++failures; }
}
bool near(double a, double b, double tolerance = 1e-9) { return std::abs(a-b) < tolerance; }

int main() {
  NonlinearControllerConfig base{.015,.9,26.1011578,18,100};
  TrajectoryState ref;
  ref.pose = {.2,.4,.1}; ref.velocity = 10; ref.angularVelocity = .2;
  const auto original = NonlinearPoseController(base).calculate({}, ref);
  const double k = 2*.9*std::sqrt(.2*.2+.015*26.1011578*26.1011578);
  check(near(original.linear,10*std::cos(.1)+k*.2), "default forward formula unchanged");
  check(near(original.angular,.2+k*.1+.015*26.1011578*sinc(.1)*.4), "default angular formula unchanged");
  auto changed = base; changed.headingScale = 2;
  auto out = NonlinearPoseController(changed).calculate({},ref);
  check(near(out.linear,original.linear) && near(out.angular-original.angular,k*.1), "heading independently adjustable");
  changed = base; changed.lateralScale = 2;
  out = NonlinearPoseController(changed).calculate({},ref);
  check(near(out.linear,original.linear) && near(out.angular-original.angular,.015*26.1011578*sinc(.1)*.4), "lateral independently adjustable");
  changed = base; changed.longitudinalScale = 2;
  out = NonlinearPoseController(changed).calculate({},ref);
  check(near(out.angular,original.angular) && near(out.linear-original.linear,k*.2), "forward independently adjustable");
  changed.headingScale = std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try { NonlinearPoseController invalid(changed); } catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "invalid correction weights rejected");

  // Historical V3 run at t=2.497: reproduce the actual tracking command,
  // then change ONLY lateralScale. Internal heading/error is CCW, logs are CW.
  base.maxAngularCorrection = 1;
  ref.pose = {.386242,-3.217250,7.901781*kPi/180};
  ref.velocity = 15.469933; ref.angularVelocity = -54.994227203*kPi/180;
  out = NonlinearPoseController(base).calculate({},ref);
  check(near(out.linear,17.643544,1e-4), "logged linear command reconstructed");
  check(near(-out.angular*180/kPi,79.462978,1e-4), "logged angular command reconstructed");
  changed=base; changed.lateralScale=1.2;
  const auto trial=NonlinearPoseController(changed).calculate({},ref);
  check(near(trial.linear,out.linear), "lateral trial does not change forward command at fixed pose");
  check(near(-trial.angular*180/kPi,93.851324,1e-4), "lateral trial fixed-state command");
  check(std::abs(trial.angular-ref.angularVelocity)<=1, "existing angular correction bound retained");
  ref.pose.y = 0;
  check(near(NonlinearPoseController(changed).calculate({},ref).angular,
             NonlinearPoseController(base).calculate({},ref).angular), "heading-only correction unchanged");
  for (double direction : {-1.0,1.0}) {
    ref.pose={0,.2,0}; ref.velocity=direction*10; ref.angularVelocity=0;
    const auto b=NonlinearPoseController(base).calculate({},ref);
    const auto c=NonlinearPoseController(changed).calculate({},ref);
    check(near(c.angular,1.2*b.angular) && c.angular*direction>0, "lateral correction retains reverse sign");
  }

  // Actual V4 scale1.20 run, then next scale1.40 command at the SAME state.
  // This is not a prediction of how the robot will move at the new setting.
  ref.pose={.471612,-3.495744,10.467671*kPi/180};
  ref.velocity=15.469933; ref.angularVelocity=-54.994227203*kPi/180;
  const auto measured=NonlinearPoseController(changed).calculate({},ref);
  check(near(measured.linear,18.045864,1e-4), "V4 logged forward command");
  check(near(-measured.angular*180/kPi,85.684141,1e-4), "V4 logged steering command");
  changed.lateralScale=1.4;
  const auto next=NonlinearPoseController(changed).calculate({},ref);
  check(near(next.linear,measured.linear), "next trial leaves fixed-state forward command unchanged");
  check(near(-next.angular*180/kPi,101.280541,1e-4), "next trial steering command");
  check(std::abs(next.angular-ref.angularVelocity)<=1, "next trial keeps correction limit");
  ref.pose.y=0;
  check(near(NonlinearPoseController(changed).calculate({},ref).angular,
             NonlinearPoseController(base).calculate({},ref).angular), "next trial heading contribution unchanged");

  // Actual lateral1.40 run, then heading1.10 at the same logged state.
  ref.pose={.412843,-3.265454,12.831498*kPi/180};
  ref.velocity=15.469752; ref.angularVelocity=-54.995184312*kPi/180;
  const auto recorded140=NonlinearPoseController(changed).calculate({},ref);
  check(near(recorded140.linear,17.563748,1e-4), "scale1.40 recorded linear command");
  check(near(-recorded140.angular*180/kPi,79.602077,1e-4), "scale1.40 recorded angular command");
  changed.headingScale=1.1;
  const auto trialHeading=NonlinearPoseController(changed).calculate({},ref);
  check(near(trialHeading.linear,recorded140.linear), "heading trial preserves fixed-state forward command");
  check(near(-trialHeading.angular*180/kPi,71.893067,1e-4), "heading trial command counterfactual");
  check(trialHeading.angular>recorded140.angular, "stronger heading correction reduces clockwise overshoot command");
  check(std::abs(trialHeading.angular-ref.angularVelocity)<=1, "heading trial correction bounded");
  ref.pose.theta=0;
  auto lateralOnly=changed; lateralOnly.headingScale=1;
  check(near(NonlinearPoseController(changed).calculate({},ref).angular,
             NonlinearPoseController(lateralOnly).calculate({},ref).angular), "heading trial leaves lateral-only correction unchanged");

  // Latest heading1.10 log: wheel tracking counterfactual at fixed velocities.
  VelocityPid priorWheel({.45,0,0,0,.02}), nextWheel({.55,0,0,0,.02});
  const double priorFeedback=priorWheel.calculate(10.400306,12.687821,.01,-12,12);
  const double nextFeedback=nextWheel.calculate(10.400306,12.687821,.01,-12,12);
  check(near(priorFeedback,-1.029382,1e-5), "recorded right wheel feedback reconstructed");
  check(near(nextFeedback,-1.25813325,1e-5), "wheel P trial feedback");
  check(near(3.545757+nextFeedback,2.28762375,1e-5), "fixed-state right output counterfactual");
  check(nextFeedback<priorFeedback, "overspeed receives stronger braking correction");

  // Measured P0.55 run; P0.65 comparison holds velocities/feedforward fixed.
  VelocityPid measuredWheel({.55,0,0,0,.02}), trialWheel({.65,0,0,0,.02});
  check(near(measuredWheel.calculate(10.063739,12.950169,.01,-12,12),
             -1.5875365,1e-5), "P0.55 recorded feedback reconstructed");
  const double trialFeedback=trialWheel.calculate(10.063739,12.950169,.01,-12,12);
  check(near(trialFeedback,-1.8761795,1e-5), "P0.65 fixed-state feedback");
  check(near(2.551160+trialFeedback,.6749805,1e-5), "P0.65 fixed-state output");

  // P0.65 hardware trial vibrated. Reduce lateral-driven CW steering without
  // weakening the opposing heading term. This is NOT a convergence prediction.
  ref.pose={.515926,-3.157753,9.302815*kPi/180};
  ref.velocity=15.469933; ref.angularVelocity=-54.994227203*kPi/180;
  const auto vibratingCommand=NonlinearPoseController(changed).calculate({},ref);
  check(near(-vibratingCommand.angular*180/kPi,92.249705,1e-4), "failed trial command replay");
  changed.lateralScale=1.0;
  const auto rollbackCommand=NonlinearPoseController(changed).calculate({},ref);
  check(rollbackCommand.angular>vibratingCommand.angular, "rollback reduces clockwise steering");
  check(near(rollbackCommand.linear,vibratingCommand.linear), "rollback preserves fixed-state forward command");
  auto weakerHeading=changed; weakerHeading.headingScale=.8;
  check(NonlinearPoseController(weakerHeading).calculate({},ref).angular<rollbackCommand.angular,
        "weakening heading would increase clockwise steering at overshoot");
  ref.pose.y=0;
  auto priorLateral=changed; priorLateral.lateralScale=1.4;
  check(near(NonlinearPoseController(priorLateral).calculate({},ref).angular,
             NonlinearPoseController(changed).calculate({},ref).angular), "rollback retains heading-only correction");

  // Latest rollback run still overshoots heading. Strengthen only heading
  // weight at this logged state, before testing the combined faster profile.
  ref.pose={.459249,-3.541708,8.120728*kPi/180};
  ref.velocity=15.469752; ref.angularVelocity=-54.995938865*kPi/180;
  const auto priorHeadingCommand=NonlinearPoseController(changed).calculate({},ref);
  check(near(-priorHeadingCommand.angular*180/kPi,80.511477,1e-4), "rollback heading command replay");
  changed.headingScale=1.4;
  const auto strongerHeadingCommand=NonlinearPoseController(changed).calculate({},ref);
  check(strongerHeadingCommand.angular>priorHeadingCommand.angular, "stronger heading reduces CW overshoot command");
  check(near(strongerHeadingCommand.linear,priorHeadingCommand.linear), "heading leaves fixed-state translation unchanged");
  ref.pose.theta=0;
  auto previousHeading=changed; previousHeading.headingScale=1.1;
  check(near(NonlinearPoseController(previousHeading).calculate({},ref).angular,
             NonlinearPoseController(changed).calculate({},ref).angular), "heading change preserves lateral-only correction");
  std::cout << "Heading trial: fixed-state CW command="
            << -strongerHeadingCommand.angular*180/kPi << "deg/s (previous80.511477)\n";

  TrajectoryConfig tc;
  tc.maxVelocity=tc.maxWheelVelocity=35;
  tc.maxAcceleration=25; tc.maxDeceleration=35;
  tc.maxCentripetalAcceleration=50; tc.maxAngularVelocity=55*kPi/180;
  tc.maxVoltage=10; tc.trackWidth=11; tc.sampleDistance=.35;
  tc.leftFeedforward=tc.rightFeedforward={1.25,.164743650,.025};
  const auto baselinePath=generateTrajectory({{{24.25,72,0},30,true,{{73.5,72.75,0}}},
                                     {{72.5,48.5,-.007877},28.1976,true,{}}},tc);
  check(std::abs(baselinePath.duration()-3.311)<.002, "baseline planned duration");
  tc.maxAcceleration=40;
  const auto path=generateTrajectory({{{24.25,72,0},30,true,{{73.5,72.75,0}}},
                                     {{72.5,48.5,-.007877},28.1976,true,{}}},tc);
  check(path.duration()<baselinePath.duration(), "higher acceleration shortens reference profile");
  check(path.sample(.5).velocity>baselinePath.sample(.5).velocity*1.4, "faster reference launch");
  check(near(path.states().front().velocity,0) && near(path.states().back().velocity,0), "profile starts and ends stopped");
  for (const auto& state : path.states()) {
    check(state.velocity<=35+1e-6, "speed ceiling unchanged");
    check(state.acceleration<=40+1e-6 && state.acceleration>=-35-1e-6, "acceleration and braking limits");
  }
  std::cout << "Acceleration trial: planned duration=" << path.duration()
            << "s, reference speed at0.5s=" << path.sample(.5).velocity << "in/s\n";
  FollowerConfig cfg;
  cfg.trackWidth=11; cfg.poseController=changed;
  cfg.leftFeedforward=cfg.rightFeedforward={1.25,.164743650,.025,2,.25};
  cfg.leftVelocityPid=cfg.rightVelocityPid={.45,0,0,0,.02};
  cfg.divergenceLimit=36; cfg.timeoutAfterTrajectory=5;
  cfg.stopAtProfileEnd=true; cfg.enableTerminalRecovery=false;
  const auto endpoint=path.states().back().pose;
  auto stopped = [](const FollowerOutput& o) {
    return o.status==FollowerStatus::kProfileComplete &&
           o.leftVoltage==0 && o.rightVoltage==0 &&
           o.wheelSetpoint.left==0 && o.wheelSetpoint.right==0 &&
           o.terminalPhase==TerminalPhase::kTracking;
  };
  TrajectoryFollower f(cfg); f.start(path,0);
  const Pose2d loggedEnd{76.336346,47.725068,wrapAngle((90+164.981516)*kPi/180)};
  auto o=f.update(path.duration()-.014,loggedEnd,{3.717577,2.302219},12);
  check(o.status==FollowerStatus::kRunning, "tracking still active before profile end");
  o=f.update(path.duration()+.001,loggedEnd,{3.717577,2.302219},12);
  check(stopped(o), "logged sideways miss stops without coordinate approach");
  check(std::hypot(o.poseError.longitudinal,o.poseError.lateral)>3.8, "remaining error preserved, not hidden as success");
  check(stopped(f.update(path.duration()+3,loggedEnd,{},12)), "cannot restart endpoint turn later");

  // 1,000 distinct endpoint-error/heading/speed cases: zero output must win
  // over any pose error or stored feedforward/PID state at profile end.
  for(int i=0;i<1000;++i) {
    Pose2d p=endpoint;
    const double a=2*kPi*(i%40)/40, d=.5+19.5*(i/40)/24;
    p.x+=d*std::cos(a); p.y+=d*std::sin(a);
    p.theta=wrapAngle(p.theta+i*.137);
    f.start(path,0);
    f.update(1,path.sample(1).pose,{8,-4},12);
    check(stopped(f.update(path.duration()+.001,p,{double(i%12),-double(i%9)},12)), "no powered endpoint maneuver");
  }
  f.cancel(); o=f.update(100,endpoint,{},12);
  check(o.status==FollowerStatus::kIdle && o.leftVoltage==0 && o.rightVoltage==0, "cancel remains zero output");
  std::cout << "Curve-only: 1000 endpoint-stop cases, logged command replay and separate-control tests; failures=" << failures << '\n';
  return failures ? 1 : 0;
}
