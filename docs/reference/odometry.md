# Odometry

Header: `#include <vantage/odometry.hpp>`

`DifferentialDriveOdometry` combines cumulative left/right wheel distance with
a continuous gyro heading and integrates the finite SE(2) motion.

## Construct

```cpp
vantage::DifferentialDriveOdometry odometry({0.0, 0.0, 0.0});
```

The optional pose is used as the initial field pose.

## Update

```cpp
vantage::Pose2d pose = odometry.update(
    leftDistance, rightDistance, gyroHeadingRadians);
```

Pass cumulative distances, not per-tick deltas. The first update establishes
the sensor baselines and does not move the pose.

## Reset

```cpp
odometry.reset(newPose, leftDistance, rightDistance, gyroHeadingRadians);
```

Reset preserves the desired field heading by calculating an internal gyro
offset. Supply current sensor readings so the next update does not jump.

!!! note
    Wheel distances and pose coordinates can use metres, inches, or another
    unit, but every distance in the entire controller must use the same unit.

