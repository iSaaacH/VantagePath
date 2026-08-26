# 3 - Driver Control

VantagePath owns autonomous calculations, not joystick shaping. Driver control
should cancel the follower and write the drivetrain directly.

## Arcade mixing

```cpp
void arcade(double forward, double turn, double maxVolts = 12.0) {
  follower.cancel();

  double left = forward + turn;
  double right = forward - turn;
  const double peak = std::max({1.0, std::abs(left), std::abs(right)});

  left /= peak;
  right /= peak;
  writeDriveVolts(left * maxVolts, right * maxVolts);
}
```

Inputs are normalized to `[-1, 1]`. Scaling both sides by the same value
preserves requested curvature when combined throttle and turn would saturate.

## PROS opcontrol loop

```cpp
void opcontrol() {
  while (true) {
    const double forward =
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    const double turn =
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    arcade(forward, turn);
    pros::delay(10);
  }
}
```

## Add a deadband

```cpp
double deadband(double value, double threshold = 0.05) {
  if (std::abs(value) <= threshold) return 0.0;
  return std::copysign(
      (std::abs(value) - threshold) / (1.0 - threshold), value);
}
```

Apply the deadband before `arcade`. Keep autonomous wheel control linear; do not
apply joystick curves to follower voltage.

Continue to [4 - Characterization & Tuning](4-tuning.md).
