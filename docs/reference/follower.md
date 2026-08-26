# Follower

Header: `#include <vantage/follower.hpp>`

`TrajectoryFollower` combines timed sampling, nonlinear pose control,
differential-drive kinematics, motor feedforward, wheel velocity feedback,
voltage saturation, and explicit completion states.

## `FollowerConfig`

| Field | Default | Meaning |
| --- | ---: | --- |
| `poseController` | defaults | Nonlinear pose gains and clamps |
| `leftFeedforward`, `rightFeedforward` | defaults | Identified motor models |
| `leftVelocityPid`, `rightVelocityPid` | defaults | Wheel feedback gains |
| `trackWidth` | `0.30` | Effective drive track width |
| `nominalVoltage` | `12.0` | Absolute controller voltage ceiling |
| `positionTolerance` | `0.03` | Final translational tolerance |
| `headingTolerance` | `0.04` | Final heading tolerance |
| `velocityTolerance` | `0.05` | Final measured wheel-speed tolerance |
| `divergenceLimit` | `1.0` | Immediate abort distance |
| `timeoutAfterTrajectory` | `1.0` | Extra time allowed to settle |
| `settleCycles` | `8` | Consecutive valid ticks required |

## Lifecycle

```cpp
follower.start(trajectory, nowSeconds);

auto output = follower.update(
    nowSeconds, currentPose, measuredWheelSpeeds, batteryVoltage);

leftMotor.move_voltage(output.leftVoltage * 1000.0);
rightMotor.move_voltage(output.rightVoltage * 1000.0);
```

Times must be monotonic seconds. `availableVoltage` is clamped against
`nominalVoltage`. `cancel()` returns the follower to idle and resets both wheel
PID controllers.

## Status and output

`FollowerOutput` exposes voltages, wheel setpoints, pose error, status, and a
flag indicating coupled voltage saturation.

| `FollowerStatus` | Meaning |
| --- | --- |
| `kIdle` | Never started or explicitly cancelled |
| `kRunning` | Actively following |
| `kSettled` | Time complete and all tolerances held for `settleCycles` |
| `kTimedOut` | Failed to settle before the post-trajectory deadline |
| `kDiverged` | Position error exceeded `divergenceLimit` |

Stop or brake the drive on every terminal status.

