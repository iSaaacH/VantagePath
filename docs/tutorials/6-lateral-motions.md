# 6 - Lateral Motions

This tutorial generates and follows the first complete pose-to-pose trajectory.

## Generate a straight trajectory

```cpp
const auto limits = trajectoryConfig();

const vantage::Trajectory straight = vantage::generateTrajectory(
    {
        {{0.0, 0.0, 0.0}},
        {{1.0, 0.0, 0.0}},
    },
    limits);
```

Each waypoint contains a `Pose2d` and optional tangent scale. The default
tangent is calculated from neighboring waypoint distance.

## Start the follower

Keep the trajectory alive for the entire motion because the follower stores a
pointer to it:

```cpp
vantage::Trajectory activeTrajectory;
vantage::TrajectoryFollower follower(followerConfig());

void startStraight() {
  activeTrajectory = vantage::generateTrajectory(
      {{{0.0, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}},
      trajectoryConfig());
  follower.start(activeTrajectory, nowSeconds());
}
```

## Run the fixed-period loop

```cpp
void driveTick() {
  const vantage::Pose2d pose = odometry.update(
      readLeftDistance(), readRightDistance(), readHeadingRadians());

  const vantage::FollowerOutput output = follower.update(
      nowSeconds(),
      pose,
      {readLeftVelocity(), readRightVelocity()},
      readBatteryVolts());

  writeDriveVolts(output.leftVoltage, output.rightVoltage);

  if (output.status != vantage::FollowerStatus::kRunning) {
    writeDriveVolts(0.0, 0.0);
  }
}
```

Run `driveTick()` every 10–20 ms. Use a monotonic clock; do not use a wall-clock
time that can jump.

## Reverse motion

Set `reversed` before generation. Waypoint headings describe the geometric path
tangent; VantagePath rotates the robot reference by π and makes velocity
negative:

```cpp
auto reverseLimits = trajectoryConfig();
reverseLimits.reversed = true;

const auto reverse = vantage::generateTrajectory(
    {{{0.0, 0.0, vantage::kPi}}, {{-1.0, 0.0, vantage::kPi}}},
    reverseLimits);
```

Continue to [7 - Trajectory Following](7-trajectory-following.md).
