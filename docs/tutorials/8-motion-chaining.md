# 8 - Motion Chaining

For a smooth route, generate one trajectory with intermediate waypoints instead
of starting several independent trajectories.

## The stop-and-start version

```cpp
runToPose({0.8, 0.0, 0.0});
runToPose({1.2, 0.4, 0.7});
runToPose({2.0, 0.5, 0.0});
```

Each call plans a zero end velocity, so the robot stops twice.

## The continuous version

```cpp
const auto route = vantage::generateTrajectory(
    {
        {{0.0, 0.0, 0.0}},
        {{0.8, 0.0, 0.0}},
        {{1.2, 0.4, 0.7}},
        {{2.0, 0.5, 0.0}},
    },
    trajectoryConfig());

follower.start(route, nowSeconds());
```

The planner carries nonzero velocity through intermediate waypoints while
respecting curvature, wheel, acceleration, and voltage limits.

## Coordinate mechanisms during a path

Use elapsed time or path distance from the sampled reference to trigger other
subsystems without stopping the drive:

```cpp
const double elapsed = nowSeconds() - trajectoryStartTime;
const auto reference = route.sample(elapsed);

if (reference.distance >= 0.75 && !intakeStarted) {
  intakeStarted = true;
  startIntake();
}
```

Keep mechanism work non-blocking. The drivetrain update must continue at its
fixed period.

## Replacing a running motion

Cancel before replacing the trajectory object that the follower references:

```cpp
follower.cancel();
activeTrajectory = makeNewTrajectory();
follower.start(activeTrajectory, nowSeconds());
```

Continue to [9 - Diagnostics & Safety](9-diagnostics-safety.md).
