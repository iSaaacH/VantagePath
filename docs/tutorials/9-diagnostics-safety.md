# 9 - Diagnostics & Safety

The final step is proving that the controller behaves safely when the robot or
environment is imperfect.

## Log every control tick

At minimum, log:

```text
time
reference x, y, heading, velocity, curvature
measured x, y, heading
left/right target wheel speed
left/right measured wheel speed
left/right commanded voltage
battery voltage
saturation flag
follower status
actual loop dt
```

## Handle every status

```cpp
switch (output.status) {
  case vantage::FollowerStatus::kRunning:
    writeDriveVolts(output.leftVoltage, output.rightVoltage);
    break;
  case vantage::FollowerStatus::kSettled:
    writeDriveVolts(0.0, 0.0);
    advanceAutonomous();
    break;
  case vantage::FollowerStatus::kTimedOut:
  case vantage::FollowerStatus::kDiverged:
    writeDriveVolts(0.0, 0.0);
    abortAutonomous();
    break;
  case vantage::FollowerStatus::kIdle:
    writeDriveVolts(0.0, 0.0);
    break;
}
```

## First-on-robot test order

1. Put the robot securely on blocks.
2. Confirm positive voltage and encoder signs.
3. Confirm pose axes and heading convention by hand.
4. Confirm disable and `cancel()` command zero in one tick.
5. Set a 6 V ceiling.
6. Run a short straight path in a clear field.
7. Run the same path in reverse.
8. Run a wide constant-radius curve.
9. Run an S-curve.
10. Repeat with low battery, payload, and intentional starting offsets.

## Common symptoms

| Symptom | Check first |
|---|---|
| oscillates at endpoint | forced minimum output, tolerance below sensor noise, excessive pose gain |
| snakes on a straight | heading noise/delay, track width, excessive pose `kp` |
| cuts corners | centripetal limit, wheel-speed limit, localization delay |
| both wheels miss target similarly | feedforward or battery units |
| one wheel consistently misses | that side's feedforward, gearing, friction, encoder scaling |
| immediately diverges | coordinate frame, units, starting pose |
| times out without moving | motor sign, zero voltage limit, disabled output |

You have now completed the VantagePath tutorials. Use the
[API Reference](../reference/index.md) while writing autonomous routines.
