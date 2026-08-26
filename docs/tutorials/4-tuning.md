# 4 - Characterization & Tuning

Tune from the motors outward. Pose gains cannot fix an incorrect motor model.

## 1. Collect feedforward data

Create a CSV with this exact header:

```text
voltage,velocity,acceleration
```

Run slow voltage ramps and faster step tests for one side of the drive. Record
volts, wheel velocity, and wheel acceleration using consistent units. Save the
left and right files separately as `left.csv` and `right.csv`.

Fit each side with the included tool:

```bash
python3 tools/fit_feedforward.py left.csv
python3 tools/fit_feedforward.py right.csv
```

The model is:

```text
voltage = kS * sign(velocity) + kV * velocity + kA * acceleration
```

Copy each result into both `TrajectoryConfig` and `FollowerConfig`.

## 2. Tune wheel velocity feedback

1. Set `ki = 0` and `kd = 0`.
2. Command a low wheel-speed step while the robot is secured on blocks.
3. Increase `kp` until measured speed follows without sustained oscillation.
4. Add a small `kd` only if repeatable overshoot remains.
5. Add `ki` only for persistent loaded bias after feedforward is correct.
6. If using `ki`, set a tight `integralLimit`.

Plot target and measured speed for each side. Do not tune from sound alone.

## 3. Tune trajectory limits

Start with a 6 V ceiling and approximately half measured free speed.

1. Run a straight trajectory.
2. Increase acceleration until tracking error or wheel slip rises.
3. Back acceleration down by at least 20%.
4. Repeat for deceleration.
5. Run a constant-radius curve.
6. Lower centripetal acceleration until the robot no longer scrubs or tips.

## 4. Tune pose feedback

Keep damping between `0.7` and `0.9`. Increase `convergence` gradually while
starting the robot 5–10 cm away from the planned pose. If the robot snakes on a
straight, reduce convergence and check localization delay/noise.

The heading tolerance must be larger than measured stationary IMU noise.
Otherwise the robot is mathematically prevented from settling.

## 5. Raise voltage last

Only after the earlier steps pass repeatedly should you raise `maxVoltage` and
`nominalVoltage`. Repeat low-battery and payload tests after every increase.

Continue to [5 - Angular Motions](5-angular-motions.md).
