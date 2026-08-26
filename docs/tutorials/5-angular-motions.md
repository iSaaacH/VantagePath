# 5 - Angular Motions

A differential drive can turn in place by commanding equal and opposite wheel
speeds. VantagePath intentionally leaves the motor loop in your hardware
adapter, so angular motion follows the same feedforward/PID pipeline as a path.

## Heading convention

All VantagePath headings are radians and counter-clockwise positive:

```cpp
constexpr double degreesToRadians(double degrees) {
  return degrees * vantage::kPi / 180.0;
}
```

Always wrap heading error:

```cpp
const double error = vantage::wrapAngle(targetHeading - measuredHeading);
```

Without wrapping, a request from `179°` to `-179°` incorrectly turns `358°`
instead of `2°`.

## Damped turn controller

Call this at the same fixed period as the rest of the drive:

```cpp
double turnVoltage(double targetHeading,
                   double measuredHeading,
                   double measuredAngularVelocity,
                   double maxVoltage) {
  const double error =
      vantage::wrapAngle(targetHeading - measuredHeading);
  const double volts = 7.0 * error - 0.32 * measuredAngularVelocity;
  return std::clamp(volts, -maxVoltage, maxVoltage);
}
```

Apply the result as:

```cpp
const double volts = turnVoltage(target, heading, omega, 6.0);
writeDriveVolts(-volts, volts);
```

## Settling without oscillation

Do not finish on position error alone. Require small heading error and small
angular velocity for consecutive samples:

```cpp
const bool inside =
    std::abs(error) < degreesToRadians(0.75) &&
    std::abs(omega) < degreesToRadians(3.0);
settledCycles = inside ? settledCycles + 1 : 0;

if (settledCycles >= 10) {
  writeDriveVolts(0.0, 0.0);
}
```

Avoid a forced minimum voltage near the target. It prevents the command from
approaching zero and commonly creates continuous left-right hunting.

Continue to [6 - Lateral Motions](6-lateral-motions.md).
