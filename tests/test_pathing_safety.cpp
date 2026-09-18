#include "vantage/follower.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace vantage;

FollowerConfig robotFollower() {
  FollowerConfig c;
  c.trackWidth=11; c.divergenceLimit=36;
  c.poseController={.015,.9,(12-1.25)/.164743650*.4,18,1,1,1,1.4};
  c.leftFeedforward={1.25,.164743650,.025,0,.25}; c.rightFeedforward=c.leftFeedforward;
  c.leftVelocityPid={.45,0,0,0,.02}; c.rightVelocityPid=c.leftVelocityPid;
  c.positionTolerance=1; c.headingTolerance=2*kPi/180; c.velocityTolerance=1.5;
  c.timeoutAfterTrajectory=5; c.enableTerminalRecovery=true;
  c.terminalMaxLinearSpeed=6; c.terminalMaxAngularSpeed=45*kPi/180;
  c.terminalMaxWheelAcceleration=40; c.terminalMaxPositionError=8;
  c.terminalProgressTimeout=1.5; c.maxWheelVelocity=35; c.maxWheelAcceleration=80;
  return c;
}

int main() {
  // V7/V8/V9: persistent overspeed gets bounded integral, no higher P/D. Check
  // cap, unwinding, saturation anti-windup and reset between runs.
  for (const auto gains : {VelocityPidConfig{.45,.4,0,2.5,.02},
                           VelocityPidConfig{.45,.6,0,1.6666666666666667,.02},
                           VelocityPidConfig{.45,.4,0,3.75,.02},
                           VelocityPidConfig{.45,.8,0,1.25,.02}}) {
  const double integralLimit = gains.integralLimit;
  VelocityPid wheel(gains);
  double wheelOutput=0;
  for(int i=0;i<1000;++i) wheelOutput=wheel.calculate(10,12,.01,-12,12);
  assert(std::abs(wheelOutput-(-.9-gains.ki*integralLimit))<1e-9);
  for(int i=0;i<125;++i) wheelOutput=wheel.calculate(12,10,integralLimit/250,-12,12);
  assert(std::abs(wheelOutput-.9)<1e-9);
  wheel.reset();
  for(int i=0;i<1000;++i) wheel.calculate(10,0,.01,-1,1);
  assert(std::abs(wheel.calculate(0,0,.01,-12,12))<1e-9);
  wheel.reset(); assert(std::abs(wheel.calculate(10,10,.01,-12,12))<1e-9);
  }
  // V15 builds 50% more bias in 0.5 s, with the same eventual 1 V cap.
  VelocityPid v14wheel({.45,.4,0,2.5,.02}), v15wheel({.45,.6,0,1.6666666666666667,.02});
  double v14bias=0, v15bias=0;
  for(int i=0;i<50;++i) {
    v14bias=v14wheel.calculate(10,12,.01,-12,12)+.9;
    v15bias=v15wheel.calculate(10,12,.01,-12,12)+.9;
  }
  assert(std::abs(v14bias+.4)<1e-9);
  assert(std::abs(v15bias+.6)<1e-9);
  // At the same constant overspeed, V9 builds twice the correction in 0.5 s.
  VelocityPid v7wheel({.45,.4,0,2.5,.02}), v9wheel({.45,.8,0,1.25,.02});
  double v7bias=0, v9bias=0;
  for(int i=0;i<50;++i) {
    v7bias=v7wheel.calculate(10,12,.01,-12,12)+.9;
    v9bias=v9wheel.calculate(10,12,.01,-12,12)+.9;
  }
  assert(std::abs(v7bias+.4)<1e-9 && std::abs(v9bias+.8)<1e-9);
  // V16 uses the .8/1.25 candidate: 4/3 of V15's uncapped integral response.
  assert(std::abs(v9bias-v15bias*4.0/3.0)<1e-9);
  auto cfg=robotFollower();
  // V24 angular braking changes only the terminal turn ceiling. Exercise
  // forward/reverse, both turn signs, zero-by-deadline and opt-in compatibility.
  for (double direction : {-1.0,1.0}) for (double side : {-1.0,1.0}) {
    auto c=cfg;
    c.enableTerminalRecovery=false; c.stopAtProfileEnd=true;
    c.pointApproachSeconds=.8; c.pointApproachMaxAngularSpeed=45*kPi/180;
    c.pointApproachAngularBrakeSeconds=.4;
    c.maxWheelAcceleration=0;
    TrajectoryState a,b,end;
    a.velocity=18*direction; a.direction=direction;
    b=a; b.time=1.6; b.pose={6*direction,0,0};
    end=b; end.time=2; end.pose={8*direction,0,0}; end.velocity=0;
    Trajectory path({a,b,end});
    auto old=c; old.pointApproachAngularBrakeSeconds=0;
    TrajectoryFollower f(c), legacy(old); f.start(path,0); legacy.start(path,0);
    for(double t : {1.0,1.4,1.6,1.8,1.9,1.99}) {
      const Pose2d pose{6*direction,side,0};
      const auto out=f.update(t,pose,{},12), before=legacy.update(t,pose,{},12);
      assert(out.status==FollowerStatus::kRunning);
      assert(std::abs(out.chassisSetpoint.linear-before.chassisSetpoint.linear)<1e-9);
      if(t<=1.6) assert(std::abs(out.chassisSetpoint.angular-before.chassisSetpoint.angular)<1e-9);
      else {
        const double p=(2-t)/.4;
        const double cap=45*kPi/180*p*p*(3-2*p);
        assert(std::abs(std::abs(out.chassisSetpoint.angular)-cap)<1e-9);
        assert(out.chassisSetpoint.angular*direction*side<0);
        assert(out.wheelSetpoint.left*direction>=0 && out.wheelSetpoint.right*direction>=0);
      }
    }
    const auto stopped=f.update(2,{6*direction,side,0},{3*direction,2*direction},12);
    assert(stopped.status==FollowerStatus::kProfileComplete);
    assert(stopped.leftVoltage==0 && stopped.rightVoltage==0);
    assert(f.update(2.1,{}, {},12).leftVoltage==0);
    for(double bad : {-1.0,.9,std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
      c.pointApproachAngularBrakeSeconds=bad;
      bool threw=false;
      try { TrajectoryFollower invalid(c); } catch(const std::invalid_argument&) { threw=true; }
      assert(threw);
    }
  }
  // V23 point approach: endpoint heading is irrelevant; feasibility and zero
  // output at capture/miss/deadline are explicit. No hardware simulation here.
  for (double direction : {-1.0,1.0}) {
    auto c=cfg;
    c.enableTerminalRecovery=false; c.stopAtProfileEnd=true;
    c.poseController.lateralScale=12; c.poseController.headingScale=3;
    c.pointApproachSeconds=.8; c.pointApproachBlendSeconds=.2;
    c.pointApproachTolerance=.35; c.pointApproachGuardDistance=1;
    c.pointApproachMaxCurvature=.14;
    c.pointApproachMaxAngularSpeed=45*kPi/180;
    c.pointApproachDeceleration=35;
    // Disable slew for direct command checks, then test bounded sequences below.
    c.maxWheelAcceleration=0;
    TrajectoryState a,b;
    a.velocity=4*direction; a.direction=direction;
    b=a; b.time=2; b.pose={8*direction,0,0}; b.velocity=0;
    Trajectory path({a,b});
    TrajectoryFollower f(c); f.start(path,0);
    auto legacy=c; legacy.pointApproachSeconds=0;
    TrajectoryFollower baseline(legacy); baseline.start(path,0);
    auto before=f.update(1,{4*direction,0,0},{},12);
    auto unchanged=baseline.update(1,{4*direction,0,0},{},12);
    assert(before.chassisSetpoint.angular==unchanged.chassisSetpoint.angular);
    assert(before.chassisSetpoint.linear==unchanged.chassisSetpoint.linear);
    auto approach=f.update(1.5,{6*direction,1,0},{},12);
    assert(approach.pointApproachBlend==1);
    assert(approach.status==FollowerStatus::kRunning);
    assert(approach.chassisSetpoint.angular*direction<0);
    assert(approach.wheelSetpoint.left*direction>=0);
    assert(approach.wheelSetpoint.right*direction>=0);
    assert(std::abs(approach.chassisSetpoint.angular)<=45*kPi/180);
    assert(std::abs(approach.chassisSetpoint.angular)<=
        std::abs(approach.chassisSetpoint.linear)*.14+1e-9);
    b.pose.theta=2.5;
    Trajectory otherHeading({a,b});
    TrajectoryFollower headingIgnored(c); headingIgnored.start(otherHeading,0);
    auto same=headingIgnored.update(1.5,{6*direction,1,0},{},12);
    assert(std::abs(same.chassisSetpoint.angular-approach.chassisSetpoint.angular)<1e-12);
    assert(std::abs(same.chassisSetpoint.linear-approach.chassisSetpoint.linear)<1e-12);
    auto reached=f.update(1.6,{7.8*direction,0,1.5},{},12);
    assert(reached.status==FollowerStatus::kPositionComplete);
    assert(reached.leftVoltage==0 && reached.rightVoltage==0);
    assert(f.update(1.7,{}, {},12).leftVoltage==0); // latched, no re-chase
    f.start(path,0);
    auto missed=f.update(1.5,{8.5*direction,0,0},{},12);
    assert(missed.status==FollowerStatus::kEndpointUnreachable);
    assert(missed.leftVoltage==0 && missed.rightVoltage==0);
    f.start(path,0);
    auto tooTight=f.update(1.5,{7.8*direction,.6,0},{},12);
    assert(tooTight.status==FollowerStatus::kEndpointUnreachable);
    assert(tooTight.leftVoltage==0 && tooTight.rightVoltage==0);
    f.start(path,0);
    auto deadline=f.update(2,{6*direction,0,0},{},12);
    assert(deadline.status==FollowerStatus::kProfileComplete);
    assert(deadline.leftVoltage==0 && deadline.rightVoltage==0);
    b.velocity=direction;
    Trajectory rolling({a,b}); f.start(rolling,0);
    assert(f.update(1.5,{6*direction,0,0},{},12).pointApproachBlend==0);
    c.enablePoseFeedback=false;
    TrajectoryFollower openLoop(c); openLoop.start(path,0);
    assert(openLoop.update(1.5,{8.5*direction,0,0},{},12).status==FollowerStatus::kRunning);
  }
  {
    auto c=cfg; c.pointApproachSeconds=.8;
    bool threw=false;
    try { TrajectoryFollower invalid(c); } catch(const std::invalid_argument&) { threw=true; }
    assert(threw); // no mixing with post-profile recovery
    c.enableTerminalRecovery=false; c.stopAtProfileEnd=true;
    for(double invalid : {-1.0,0.0,std::numeric_limits<double>::quiet_NaN(),.2}) {
      c.pointApproachMaxCurvature=invalid; threw=false;
      try { TrajectoryFollower bad(c); } catch(const std::invalid_argument&) { threw=true; }
      assert(threw); // .2 at 11in track width would reverse a wheel
    }
  }
  // V21: taper only the final stopping segment, with smooth endpoints.
  // Command regression, not a physical prediction of the next run.
  {
    auto c=cfg;
    c.poseController.lateralScale=8; c.poseController.headingScale=3;
    c.leftVelocityPid={.45,.6,0,1.6666666666666667,.02};
    c.rightVelocityPid=c.leftVelocityPid;
    c.enableTerminalRecovery=false; c.stopAtProfileEnd=true;
    c.brakingLateralTaperSeconds=.4; c.brakingLateralEndMultiplier=.8;
    TrajectoryState logged;
    logged.pose={.226411,-1.138137,11.096810*kPi/180};
    logged.velocity=.249539; logged.angularVelocity=-.596844556*kPi/180;
    NonlinearPoseController controller(c.poseController);
    const auto before=controller.calculate({},logged);
    const auto after=controller.calculate({},logged,.8);
    assert(std::abs(-before.angular*180/kPi-12.012888)<1e-4);
    assert(after.angular>0 && before.angular<0); // stops adding clockwise lead
    assert(std::abs(after.linear-before.linear)<1e-12);
    assert(std::abs(after.angular-logged.angularVelocity)<=1);
    auto mirrored=logged;
    mirrored.pose.y=-logged.pose.y; mirrored.pose.theta=-logged.pose.theta;
    mirrored.angularVelocity=-logged.angularVelocity;
    assert(std::abs(controller.calculate({},mirrored,.8).angular+after.angular)<1e-12);
    auto straight=logged; straight.pose={.1,0,0}; straight.angularVelocity=0;
    assert(controller.calculate({},straight,.8).angular==0);
    assert(controller.calculate({},straight,.8).linear==controller.calculate({},straight).linear);
    auto reversed=logged;
    reversed.velocity=-logged.velocity; reversed.pose.y=-logged.pose.y;
    reversed.direction=-1;
    assert(std::abs(controller.calculate({},reversed,.8).angular-after.angular)<1e-12);

    TrajectoryState a,b,d,e,z;
    a.velocity=0;
    b.time=.2; b.velocity=10;
    d.time=.5; d.velocity=5; // a mid-path slowdown, must not taper here
    e.time=.6; e.velocity=15;
    z.time=1; z.velocity=0;
    Trajectory stopping({a,b,d,e,z});
    TrajectoryFollower f(c); f.start(stopping,0);
    WheelSpeeds previous{};
    double previousMultiplier=1;
    for(int i=1;i<100;++i) {
      const auto out=f.update(i*.01,{0,1,.1},previous,12);
      const double m=out.lateralFeedbackMultiplier;
      assert(out.status==FollowerStatus::kRunning);
      assert(m>=.8 && m<=previousMultiplier+1e-12);
      assert(previousMultiplier-m<=.0075+1e-9);
      if(i<=60) assert(std::abs(m-1)<1e-12);
      if(i==80) assert(std::abs(m-.9)<1e-12);
      assert(std::abs(out.wheelSetpoint.left)<=35+1e-9);
      assert(std::abs(out.wheelSetpoint.right)<=35+1e-9);
      assert(std::abs(out.wheelSetpoint.left-previous.left)<=.8+1e-9);
      assert(std::abs(out.wheelSetpoint.right-previous.right)<=.8+1e-9);
      assert(std::isfinite(out.leftVoltage) && std::abs(out.leftVoltage)<=12);
      assert(std::isfinite(out.rightVoltage) && std::abs(out.rightVoltage)<=12);
      previous=out.wheelSetpoint; previousMultiplier=m;
    }
    const auto completed=f.update(1,{},previous,12);
    assert(completed.status==FollowerStatus::kProfileComplete);
    assert(completed.leftVoltage==0 && completed.rightVoltage==0);
    f.start(stopping,2); f.cancel();
    assert(f.update(2.1,{}, {},12).leftVoltage==0);
    z.velocity=2; Trajectory rolling({a,b,d,e,z});
    f.start(rolling,3);
    assert(f.update(3.9,{}, {},12).lateralFeedbackMultiplier==1);
    c.brakingLateralTaperSeconds=0;
    TrajectoryFollower disabled(c); disabled.start(stopping,0);
    assert(disabled.update(.9,{}, {},12).lateralFeedbackMultiplier==1);
    c.brakingLateralTaperSeconds=.4;
    e.time=.9; Trajectory shortBrake({a,b,d,e,TrajectoryState{1}});
    TrajectoryFollower shortFollower(c); shortFollower.start(shortBrake,0);
    assert(shortFollower.update(.85,{}, {},12).lateralFeedbackMultiplier==1);
    assert(std::abs(shortFollower.update(.95,{}, {},12).lateralFeedbackMultiplier-.9)<1e-12);
    for(double invalid : {-1.0,1.1,std::numeric_limits<double>::quiet_NaN()}) {
      c.brakingLateralEndMultiplier=invalid;
      bool threw=false;
      try { TrajectoryFollower bad(c); } catch(const std::invalid_argument&) { threw=true; }
      assert(threw);
    }
  }
  // V20/V22 gains, fixed V19 mid-bend error. Stronger correction is NOT a
  // physical endpoint simulation. Check native direction and all output limits.
  for (double lateralScale : {8.0,12.0}) {
    auto c=cfg;
    c.poseController.longitudinalScale=1;
    c.poseController.lateralScale=lateralScale;
    c.poseController.headingScale=3;
    c.leftVelocityPid={.45,.6,0,1.6666666666666667,.02};
    c.rightVelocityPid=c.leftVelocityPid;
    c.enableTerminalRecovery=false; c.stopAtProfileEnd=true;
    TrajectoryState a;
    a.pose={.256818,-1.655570,8.225372*kPi/180};
    a.velocity=16.817899;
    a.curvature=(-41.629810935*kPi/180)/a.velocity;
    a.angularVelocity=a.velocity*a.curvature;
    auto b=a; b.time=1;
    auto command=NonlinearPoseController(c.poseController).calculate({},a);
    assert(std::abs(-command.angular*180/kPi-98.925590448)<1e-5);
    auto mirror=a;
    mirror.pose.y=-mirror.pose.y; mirror.pose.theta=-mirror.pose.theta;
    mirror.angularVelocity=-mirror.angularVelocity;
    assert(std::abs(NonlinearPoseController(c.poseController).calculate({},mirror).angular+
                    command.angular)<1e-9);
    Trajectory fixed({a,b});
    TrajectoryFollower f(c); f.start(fixed,0);
    WheelSpeeds previous{};
    for(int i=1;i<100;++i) {
      const auto result=f.update(i*.01,{},previous,12);
      assert(result.status==FollowerStatus::kRunning);
      assert(std::abs(result.wheelSetpoint.left)<=35+1e-9);
      assert(std::abs(result.wheelSetpoint.right)<=35+1e-9);
      assert(std::abs(result.wheelSetpoint.left-previous.left)<=.8+1e-9);
      assert(std::abs(result.wheelSetpoint.right-previous.right)<=.8+1e-9);
      assert(std::isfinite(result.leftVoltage) && std::abs(result.leftVoltage)<=12);
      assert(std::isfinite(result.rightVoltage) && std::abs(result.rightVoltage)<=12);
      previous=result.wheelSetpoint;
    }
    auto completed=f.update(1.01,{},previous,12);
    assert(completed.status==FollowerStatus::kProfileComplete);
    assert(completed.leftVoltage==0 && completed.rightVoltage==0);
    f.start(fixed,2); f.cancel();
    auto aborted=f.update(2.01,{},previous,12);
    assert(aborted.leftVoltage==0 && aborted.rightVoltage==0);
  }
  // V8 t=2.498: heading and lateral terms nearly cancel despite 3.50 in miss.
  // V9 changes the ratio, not wheel-loop aggressiveness. Fixed-state replay
  // verifies commands only, NOT the future physical endpoint.
  auto tracking=cfg.poseController;
  tracking.lateralScale=2; tracking.headingScale=2.8;
  TrajectoryState logged;
  logged.pose={.535210,-3.499908,8.654157*kPi/180};
  logged.velocity=16.699069;
  logged.angularVelocity=-54.773192548*kPi/180;
  auto oldCommand=NonlinearPoseController(tracking).calculate({},logged);
  assert(std::abs(-oldCommand.angular*180/kPi-65.666148)<1e-4);
  tracking.lateralScale=4;
  auto newCommand=NonlinearPoseController(tracking).calculate({},logged);
  assert(std::abs(-newCommand.angular*180/kPi-112.068972)<1e-4);
  assert(std::abs(newCommand.linear-oldCommand.linear)<1e-9);
  assert(std::abs(newCommand.angular-logged.angularVelocity)<=1+1e-9);
  // Mirror and rotate: correction follows path error, never a global X bias.
  logged.pose.y=-logged.pose.y; logged.pose.theta=-logged.pose.theta;
  logged.angularVelocity=-logged.angularVelocity;
  auto mirrored=NonlinearPoseController(tracking).calculate({},logged);
  assert(std::abs(mirrored.angular+newCommand.angular)<1e-9);
  const double rotation=1.2;
  const auto lp=logged.pose;
  logged.pose={lp.x*std::cos(rotation)-lp.y*std::sin(rotation),
               lp.x*std::sin(rotation)+lp.y*std::cos(rotation),lp.theta+rotation};
  auto rotated=NonlinearPoseController(tracking).calculate({0,0,rotation},logged);
  assert(std::abs(rotated.angular-mirrored.angular)<1e-9);
  // V9 STARTUP1 -> V10: more early correction without increasing the late
  // right-turn demand. These are fixed-pose commands, not simulated endpoints.
  auto v10Tracking=tracking;
  v10Tracking.lateralScale=6; v10Tracking.headingScale=4.5;
  logged.pose={.288655,-.686788,2.084880*kPi/180};
  logged.velocity=31.284290;
  logged.angularVelocity=-38.708121776*kPi/180;
  auto v9Early=NonlinearPoseController(tracking).calculate({},logged);
  auto v10Early=NonlinearPoseController(v10Tracking).calculate({},logged);
  assert(v10Early.angular<v9Early.angular);
  assert(std::abs(v10Early.angular-logged.angularVelocity)<=1+1e-9);
  logged.pose={.390347,-2.501716,12.301307*kPi/180};
  logged.velocity=16.666369;
  logged.angularVelocity=-54.775848822*kPi/180;
  auto v9Late=NonlinearPoseController(tracking).calculate({},logged);
  auto v10Late=NonlinearPoseController(v10Tracking).calculate({},logged);
  assert(std::abs(-v9Late.angular*180/kPi-70.666437)<1e-4);
  assert(std::abs(-v10Late.angular*180/kPi-56.45)<.01);
  assert(std::abs(v10Late.linear-v9Late.linear)<1e-9);
  auto noPivot=cfg; noPivot.poseController=v10Tracking;
  noPivot.enableTerminalRecovery=false; noPivot.stopAtProfileEnd=true;
  TrajectoryState stopped; Trajectory stopPath({stopped});
  TrajectoryFollower noPivotFollower(noPivot); noPivotFollower.start(stopPath,0);
  auto stopOutput=noPivotFollower.update(.01,{0,4,.2},{3,2},12);
  assert(stopOutput.status==FollowerStatus::kProfileComplete);
  assert(stopOutput.leftVoltage==0 && stopOutput.rightVoltage==0);
  // V6 actual steering weights: correction doubles until the existing clamp;
  // no terminal motion even with the V5 run's substantial endpoint miss.
  auto v6=cfg; v6.poseController.lateralScale=2; v6.poseController.headingScale=2.8;
  v6.enableTerminalRecovery=false; v6.stopAtProfileEnd=true;
  TrajectoryState v6end; Trajectory v6hold({v6end});
  TrajectoryFollower v6f(v6); v6f.start(v6hold,0);
  auto v6out=v6f.update(.01,{0,4.33,.19},{3,2},12);
  assert(v6out.status==FollowerStatus::kProfileComplete);
  assert(v6out.leftVoltage==0 && v6out.rightVoltage==0);
  TrajectoryState moving; moving.velocity=20;
  const Pose2d smallError{0,.1,.01};
  auto before=NonlinearPoseController(cfg.poseController).calculate(smallError,moving);
  auto after=NonlinearPoseController(v6.poseController).calculate(smallError,moving);
  assert(std::abs(after.angular-2*before.angular)<1e-9);
  // Actual logged endpoint error, mirrored, rotated, and with target behind.
  // Ideal wheel tracking verifies controller geometry, NOT hardware accuracy.
  for (double sign : {-1.,1.}) for (double rotation : {0., 3.12}) {
    TrajectoryState end; end.time=0; end.pose={72.5,48.5,-177.638625*kPi/180};
    const auto transform=[&](Pose2d p) {
      p.y*=sign; p.theta=wrapAngle(p.theta*sign+rotation);
      return Pose2d{p.x*std::cos(rotation)-p.y*std::sin(rotation),
                    p.x*std::sin(rotation)+p.y*std::cos(rotation),p.theta};
    };
    end.pose=transform(end.pose);
    Trajectory hold({end});
    // The public log uses compass headings; construct the equivalent error in
    // the mathematical endpoint frame rather than mixing coordinate conventions.
    Pose2d pose={end.pose.x-4.323557*sign*std::sin(end.pose.theta),
          end.pose.y+4.323557*sign*std::cos(end.pose.theta),
          wrapAngle(end.pose.theta+sign*10.564732*kPi/180)};
    TrajectoryFollower f(cfg); f.start(hold,0);
    WheelSpeeds wheels{}; FollowerOutput out;
    for (int i=1;i<=501;++i) {
      out=f.update(i*.01,pose,wheels,12);
      assert(std::abs(out.wheelSetpoint.left)<=35.00001);
      assert(std::abs(out.wheelSetpoint.right)<=35.00001);
      if(out.status!=FollowerStatus::kRunning) break;
      wheels=out.wheelSetpoint;
      const double v=(wheels.left+wheels.right)/2, w=(wheels.right-wheels.left)/11;
      pose.x+=v*std::cos(pose.theta+w*.005)*.01;
      pose.y+=v*std::sin(pose.theta+w*.005)*.01;
      pose.theta=wrapAngle(pose.theta+w*.01);
    }
    std::cout<<"Endpoint mirror="<<sign<<" rotation="<<rotation<<" status="<<int(out.status)<<" time="<<out.elapsed<<"\n";
    assert(out.status==FollowerStatus::kSettled);
    assert(std::hypot(pose.x-end.pose.x,pose.y-end.pose.y)<=1);
    assert(std::abs(wrapAngle(pose.theta-end.pose.theta))<=2*kPi/180);
    assert(out.leftVoltage==0 && out.rightVoltage==0);
  }
  TrajectoryState end; Trajectory hold({end});
  TrajectoryFollower stuck(cfg); stuck.start(hold,0);
  FollowerOutput out;
  for(int i=1;i<200;++i) { out=stuck.update(i*.01,{0,4,0},{},12); if(out.status!=FollowerStatus::kRunning) break; }
  assert(out.status==FollowerStatus::kDiverged && out.leftVoltage==0 && out.rightVoltage==0);
  stuck.start(hold,0); out=stuck.update(.01,{0,9,0},{},12);
  assert(out.status==FollowerStatus::kDiverged);
  stuck.start(hold,0); out=stuck.update(.01,{0,4,0},{3,2},12);
  assert(out.terminalPhase==TerminalPhase::kBraking);
  stuck.cancel(); out=stuck.update(.02,{}, {},12); assert(out.leftVoltage==0 && out.rightVoltage==0);
  stuck.start(hold,0); out=stuck.update(.01,{}, {std::numeric_limits<double>::infinity(),0},12);
  assert(out.status==FollowerStatus::kDiverged);

  TrajectoryState a,b; a.velocity=b.velocity=20; b.time=10; b.distance=200; b.pose={200,0,0};
  Trajectory line({a,b});
  // Isolate pose correction: kA must not differentiate a 0.1in measurement step.
  cfg.maxWheelAcceleration=0;
  TrajectoryFollower clean(cfg), noisy(cfg); clean.start(line,0); noisy.start(line,0);
  clean.update(.01,{.2,0,0},{20,20},12); noisy.update(.01,{.2,0,0},{20,20},12);
  auto x=clean.update(.02,{.4,0,0},{20,20},12), y=noisy.update(.02,{.5,0,0},{20,20},12);
  assert(std::abs((y.leftFeedforwardVoltage-x.leftFeedforwardVoltage)-
      .164743650*(y.wheelSetpoint.left-x.wheelSetpoint.left))<1e-9);

  TrajectoryConfig tc; tc.maxVelocity=35; tc.maxAcceleration=40; tc.maxDeceleration=35;
  tc.maxWheelVelocity=35; tc.trackWidth=11; tc.maxVoltage=10;
  tc.leftFeedforward={1.25,.164743650,.025}; tc.rightFeedforward=tc.leftFeedforward;
  auto straight=generateTrajectory({{{0,0,0},0,true,{}},{{24,0,0},0,false,{}}},tc);
  const double t=straight.states()[1].time*.4, eps=1e-7;
  assert(std::abs((straight.sample(t+eps).distance-straight.sample(t-eps).distance)/(2*eps)-straight.sample(t).velocity)<1e-5);
  // V11 changes only the curve's planned angular ceiling. Preserve launch,
  // geometry and wheel limits; this does not predict physical tracking error.
  auto curveConfig=tc;
  curveConfig.maxCentripetalAcceleration=50;
  curveConfig.sampleDistance=.35;
  curveConfig.maxAngularVelocity=55*kPi/180;
  const std::vector<Waypoint> curvePoints={
      {{24.25,72,0},0,true,{{73.5,72.75,0}}},
      {{72.5,48.5,-.007877},0,false,{}}};
  auto normalBend=generateTrajectory(curvePoints,curveConfig);
  curveConfig.maxAngularVelocity=40*kPi/180;
  auto slowerBend=generateTrajectory(curvePoints,curveConfig);
  assert(slowerBend.duration()>normalBend.duration());
  assert(std::abs(slowerBend.sample(.2).velocity-8)<1e-6);
  assert(std::abs(slowerBend.sample(.2).velocity-normalBend.sample(.2).velocity)<1e-6);
  assert(std::abs(slowerBend.states().back().pose.x-72.5)<1e-9);
  assert(std::abs(slowerBend.states().back().pose.y-48.5)<1e-9);
  for (const auto& state:slowerBend.states()) {
    assert(std::abs(state.angularVelocity)<=40*kPi/180+1e-6);
    assert(std::abs(state.velocity-state.angularVelocity*5.5)<=35+1e-6);
    assert(std::abs(state.velocity+state.angularVelocity*5.5)<=35+1e-6);
  }
  std::cout<<"Curve planned durations: 55deg/s="<<normalBend.duration()
           <<" 40deg/s="<<slowerBend.duration()<<"; launch unchanged\n";
  // V13: recover some pace without restoring the aggressive steering gains.
  curveConfig.maxAngularVelocity=45*kPi/180;
  curveConfig.maxDeceleration=30;
  auto pacedBend=generateTrajectory(curvePoints,curveConfig);
  assert(pacedBend.duration()<slowerBend.duration());
  assert(std::abs(pacedBend.sample(.2).velocity-8)<1e-6);
  assert(std::abs(pacedBend.states().back().pose.x-72.5)<1e-9);
  assert(std::abs(pacedBend.states().back().pose.y-48.5)<1e-9);
  for (const auto& state:pacedBend.states()) {
    assert(std::abs(state.angularVelocity)<=45*kPi/180+1e-6);
    assert(state.acceleration>=-30-1e-6);
    assert(std::abs(state.velocity-state.angularVelocity*5.5)<=35+1e-6);
    assert(std::abs(state.velocity+state.angularVelocity*5.5)<=35+1e-6);
  }
  std::cout<<"V13 planned duration="<<pacedBend.duration()
           <<"; braking <=30 in/s^2, launch unchanged\n";
  bool rejected=false; tc.maxVoltage=.5;
  try { generateTrajectory({{{0,0,0},0,true,{}},{{24,0,0},0,false,{}}},tc); } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);
  std::cout<<"Pathing safety regressions passed\n";
}
