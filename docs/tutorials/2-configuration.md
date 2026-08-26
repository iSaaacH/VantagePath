# 2 - Configuration

Most path-following failures begin as configuration errors. Complete each
section before tuning gains.

## Choose one unit system

VantagePath accepts any consistent length unit. Metres are recommended for new
projects; inches are also valid.

| Quantity | Metre project | Inch project |
|---|---:|---:|
| pose | m | in |
| wheel distance | m | in |
| wheel velocity | m/s | in/s |
| track width | m | in |
| acceleration | m/s² | in/s² |
| heading | rad | rad |
| voltage | V | V |

Never pass degrees to `vantage::Pose2d::theta`.

## Measure the drivetrain

1. Measure loaded wheel diameter at the tread.
2. Push the robot forward exactly one wheel revolution.
3. Confirm both reported wheel distances are positive and close to one wheel circumference.
4. Rotate the robot several full turns.
5. Adjust effective track width until odometry reports the measured rotation.

Create a configuration function:

```cpp
vantage::TrajectoryConfig trajectoryConfig() {
  vantage::TrajectoryConfig cfg;
  cfg.trackWidth = 0.305;                 // measured metres
  cfg.maxVelocity = 1.2;                  // conservative first test
  cfg.maxAcceleration = 1.0;
  cfg.maxDeceleration = 1.2;
  cfg.maxCentripetalAcceleration = 1.0;
  cfg.maxWheelVelocity = 1.4;
  cfg.sampleDistance = 0.02;
  cfg.maxVoltage = 6.0;
  cfg.leftFeedforward = {0.35, 5.1, 0.22};
  cfg.rightFeedforward = {0.36, 5.0, 0.23};
  return cfg;
}
```

The feedforward numbers above are examples only. Tutorial 4 replaces them with
measurements from your robot.

## Configure the follower

The follower track width must match the trajectory configuration:

```cpp
vantage::FollowerConfig followerConfig() {
  vantage::FollowerConfig cfg;
  cfg.trackWidth = 0.305;
  cfg.nominalVoltage = 6.0;
  cfg.leftFeedforward = {0.35, 5.1, 0.22};
  cfg.rightFeedforward = {0.36, 5.0, 0.23};
  cfg.leftVelocityPid = {1.0, 0.0, 0.0, 0.0, 0.02};
  cfg.rightVelocityPid = cfg.leftVelocityPid;
  cfg.poseController = {2.0, 0.8, 0.1, 0.75, 4.0};
  cfg.positionTolerance = 0.03;
  cfg.headingTolerance = 2.0 * vantage::kPi / 180.0;
  cfg.velocityTolerance = 0.05;
  cfg.divergenceLimit = 0.75;
  cfg.timeoutAfterTrajectory = 1.0;
  cfg.settleCycles = 8;
  return cfg;
}
```

Create long-lived objects after the hardware objects they read:

```cpp
vantage::DifferentialDriveOdometry odometry;
vantage::TrajectoryFollower follower(followerConfig());
```

## Calibrate and seed odometry

After the IMU finishes calibration, seed cumulative wheel distance and heading:

```cpp
odometry.reset(
    {0.0, 0.0, 0.0},
    readLeftDistance(),
    readRightDistance(),
    readHeadingRadians());
```

Print the pose while pushing the disabled robot. Forward movement should
increase `x`; moving left should increase `y`; counter-clockwise rotation
should increase `theta`.

!!! danger

    Do not compensate for a reversed encoder by changing controller gains.
    Correct the sensor or motor sign first.

Continue to [3 - Driver Control](3-driver-control.md).
