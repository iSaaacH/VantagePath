# Control

Header: `#include <vantage/controller.hpp>`

## Differential-drive kinematics

`DifferentialDriveKinematics(trackWidth)` converts between:

- `ChassisSpeeds { linear, angular }`
- `WheelSpeeds { left, right }`

Use `toWheelSpeeds()` for motor targets and `toChassisSpeeds()` for measured
robot motion. A non-positive track width throws `std::invalid_argument`.

## Nonlinear pose controller

`NonlinearPoseController::calculate(current, reference)` returns corrected
linear and angular chassis speeds. Unlike independent point PID loops, it
tracks the full timed pose while preserving reference velocity and curvature.

| `NonlinearControllerConfig` field | Default | Meaning |
| --- | ---: | --- |
| `convergence` | `2.0` | Position correction aggressiveness |
| `damping` | `0.75` | Damping ratio; must be in `(0, 1]` |
| `minimumFeedbackSpeed` | `0.10` | Keeps terminal feedback controllable |
| `maxLinearCorrection` | `0.75` | Linear feedback clamp |
| `maxAngularCorrection` | `4.0` | Angular feedback clamp |

`error(current, reference)` returns the same robot-frame `PoseError` used by
the controller.

## Motor feedforward

`MotorFeedforward` implements:

```text
voltage = kS * sign(velocity) + kV * velocity + kA * acceleration
```

Configure it with `FeedforwardConfig { staticGain, velocityGain,
accelerationGain }` and call `calculate(velocity, acceleration)`.

## Velocity PID

`VelocityPid` provides derivative-on-measurement filtering, integral limiting,
and conditional anti-windup. Call:

```cpp
double correction = pid.calculate(
    setpoint, measurement, dtSeconds, minOutput, maxOutput);
```

It returns zero for non-positive `dt`. Call `reset()` before a new motion.

