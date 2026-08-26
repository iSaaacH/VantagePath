# 7 - Trajectory Following

Multi-waypoint trajectories are where time parameterization becomes more useful
than point-to-point control. The planner slows before curves and preserves
continuous geometry through intermediate waypoints.

## Create an S-curve

```cpp
auto limits = trajectoryConfig();

const auto sCurve = vantage::generateTrajectory(
    {
        {{0.0, 0.0, 0.0}},
        {{0.8, 0.45, 0.20}},
        {{1.6, 0.0, 0.0}},
    },
    limits);
```

## Control tangent length

The optional second `Waypoint` field changes how strongly the path leaves or
enters that waypoint:

```cpp
vantage::Waypoint start{{0.0, 0.0, 0.0}, 0.7};
vantage::Waypoint middle{{0.8, 0.45, 0.20}, 0.4};
vantage::Waypoint end{{1.6, 0.0, 0.0}, 0.7};

const auto path = vantage::generateTrajectory(
    {start, middle, end}, limits);
```

Use automatic tangents first. Override them only after inspecting curvature and
wheel-speed plots.

## Inspect the generated states

```cpp
for (const auto& state : path.states()) {
  std::printf("%.3f,%.3f,%.3f,%.3f,%.3f\n",
              state.time,
              state.pose.x,
              state.pose.y,
              state.velocity,
              state.curvature);
}
```

The generated sequence should show:

- monotonic time and distance;
- zero or configured boundary velocity;
- lower velocity at high curvature;
- no wheel speed beyond `maxWheelVelocity`;
- feedforward voltage within `maxVoltage` when that constraint is enabled.

## Handle disturbances

The follower is intended to correct small pose disturbances. If position error
exceeds `divergenceLimit`, it returns `kDiverged` and zero output. Do not set the
limit to the size of the whole field merely to hide localization problems.

Continue to [8 - Motion Chaining](8-motion-chaining.md).
