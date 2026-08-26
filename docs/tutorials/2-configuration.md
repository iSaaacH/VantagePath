# 2 - Configure the Drivetrain

This page starts with raw motor degrees and ends with validated wheel distance,
heading, track width, odometry, trajectory limits, and follower configuration.
Complete it on a disabled robot before tuning any controller gains.

## What you need to measure

Write these values down before editing code:

| Measurement | Example | Your value |
| --- | ---: | ---: |
| Loaded wheel diameter | `0.06985 m` | |
| Wheel turns per encoder turn | `0.75` | |
| Physical left-to-right track width | `0.305 m` | |
| Control period | `0.010 s` | |
| Maximum commissioning voltage | `6.0 V` | |

Use metres throughout this tutorial. Inches also work, but every position,
distance, speed, acceleration, and track-width value must then use inches.
Angles are always radians inside VantagePath.

## Step 1: define wheel diameter and gearing

Measure the wheel while it carries the robot's weight. Tread compression can
make the effective diameter different from the value printed on the wheel.

```cpp
constexpr double kWheelDiameter = 0.06985;  // loaded diameter, metres

// A 36-tooth encoder/motor gear driving a 48-tooth wheel gear:
constexpr double kWheelTurnsPerEncoderTurn = 36.0 / 48.0;
```

The ratio means `wheel rotations / encoder rotations`. For direct drive, use
`1.0`. Convert cumulative encoder degrees to cumulative wheel distance:

```cpp
double wheelDistance(double encoderDegrees) {
  const double encoderTurns = encoderDegrees / 360.0;
  const double wheelTurns = encoderTurns * kWheelTurnsPerEncoderTurn;
  return wheelTurns * vantage::kPi * kWheelDiameter;
}
```

### Check the conversion

1. Mark one wheel and the floor.
2. Push the robot forward exactly 10 wheel revolutions.
3. Record left and right encoder degrees.
4. Run both readings through `wheelDistance()`.
5. Compare the result with tape-measure distance.

Both calculated distances must be positive when moving forward. Correct a
reversed motor/encoder sign in hardware configuration, not in controller gains.
If the scale is wrong, correct wheel diameter or gearing before moving on.

## Step 2: configure fused drive encoders in PROS

If every motor on one side shares an output shaft, VantagePath can fuse all of
their encoders instead of trusting one motor:

```cpp
#include <vantage/pros.hpp>

const std::vector<vantage::pros::DriveMotorSpec> leftMotorSpecs = {
    // port, real cartridge, encoder-to-common-shaft multiplier
    {-7, pros::v5::MotorGears::blue, 1.0},
    {-2, pros::v5::MotorGears::blue, 1.0},
    {-6, pros::v5::MotorGears::green, 3.0},
};

vantage::pros::FusedDrive leftDrive(
    leftMotorSpecs,
    {12.0, 0.5, true}); // absolute gate, relative gate, reject outliers
```

The third motor's encoder turns one third as far as the common shaft, so its
`toCommon` value is `3.0`. Use `1.0` when an encoder already measures the common
shaft directly. Configure the right side the same way.

Apply each physical cartridge during robot initialization:

```cpp
leftDrive.initialize();
rightDrive.initialize();
```

Read cumulative common-shaft degrees with `get_position()`, then pass that
number to `wheelDistance()`.

!!! note
    `toCommon` normalizes motors coupled to the same shaft. The separate
    `kWheelTurnsPerEncoderTurn` converts that shared shaft to wheel rotation.
    Do not accidentally combine the two ratios.

## Step 3: configure and verify heading

With two PROS inertial sensors:

```cpp
vantage::pros::FusedImu imu(
    15,   // primary port
    20,   // secondary port
    3.0); // allowed disagreement per control tick, degrees
```

Initialize them before starting the odometry task:

```cpp
imu.reset(false);
while (imu.is_calibrating()) pros::delay(10);
imu.set_data_rate(10);
```

Convert continuous degrees to radians:

```cpp
double headingRadians() {
  return imu.get_rotation() * vantage::kPi / 180.0;
}
```

Turn the robot counter-clockwise. The heading passed to VantagePath must
increase. Negate it once in `headingRadians()` if the sensor convention is the
opposite. Do not negate it again elsewhere.

## Step 4: measure effective track width

Physical wheel spacing is only a starting estimate. Tank-drive scrub changes
the width that best predicts rotation. VantagePath uses this effective track
width for wheel-speed constraints and differential-drive kinematics.

1. Put the robot on its normal field surface.
2. Record cumulative left distance `L0`, right distance `R0`, and heading `H0`.
3. Slowly rotate counter-clockwise at least five complete turns.
4. Record `L1`, `R1`, and `H1`.
5. Calculate:

```text
deltaLeft  = L1 - L0
deltaRight = R1 - R0
deltaTheta = H1 - H0             (continuous radians, do not wrap)

effectiveTrackWidth = (deltaRight - deltaLeft) / deltaTheta
```

Repeat clockwise and counter-clockwise three times. Use the median absolute
result. A trial far from the others usually means wheel slip, wrapped heading,
or an encoder-sign error.

Example:

```text
left travel  = -4.80 m
right travel =  4.78 m
heading      = 31.42 rad (five turns)
track width  = (4.78 - -4.80) / 31.42 = 0.305 m
```

## Step 5: define one shared track-width constant

The planner and follower must use the same measured value:

```cpp
constexpr double kTrackWidth = 0.305;

vantage::TrajectoryConfig trajectoryConfig() {
  vantage::TrajectoryConfig cfg;
  cfg.trackWidth = kTrackWidth;
  cfg.maxVelocity = 1.2;
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

vantage::FollowerConfig followerConfig() {
  vantage::FollowerConfig cfg;
  cfg.trackWidth = kTrackWidth;
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

These feedforward and feedback values are examples, not universal gains.
[Tutorial 4](4-tuning.md) explains how to replace them with measurements.

## Step 6: seed odometry

Create long-lived objects:

```cpp
vantage::DifferentialDriveOdometry odometry;
vantage::TrajectoryFollower follower(followerConfig());
```

After IMU calibration, seed odometry with current cumulative sensor readings:

```cpp
odometry.reset(
    {0.0, 0.0, 0.0},
    wheelDistance(leftDrive.get_position()),
    wheelDistance(rightDrive.get_position()),
    headingRadians());
```

Then update it every 10–20 ms:

```cpp
const vantage::Pose2d pose = odometry.update(
    wheelDistance(leftDrive.get_position()),
    wheelDistance(rightDrive.get_position()),
    headingRadians());
```

## Step 7: perform the disabled push test

Print `pose.x`, `pose.y`, and `pose.theta` while moving the disabled robot:

1. Push forward exactly 1 m: X should increase by approximately 1 m.
2. Pull backward to the start: X should return close to zero.
3. Rotate counter-clockwise 360°: the raw continuous IMU reading should
   increase by about `2*pi` radians, while the wrapped odometry heading returns
   close to zero and X/Y remain close to their starting values.
4. Repeat clockwise.
5. Watch `contributing()` and per-sensor `rejected()` diagnostics. Healthy
   sensors should not disappear during ordinary motion.

Do not start controller tuning until these checks pass. A controller cannot
correct incorrect units, gearing, signs, or geometry.

Continue to [3 - Driver Control](3-driver-control.md).
