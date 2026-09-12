# Trajectory

Header: `#include <vantage/trajectory.hpp>`

## `Waypoint`

| Field | Meaning |
| --- | --- |
| `pose` | X, Y, and spline tangent heading |
| `tangentScale` | Hermite tangent magnitude; `0` selects a distance-based default |
| `bezierToNext` | `false` uses Hermite; `true` uses Bézier geometry to the next waypoint |
| `controlPoints` | Ordered `Pose2d` controls for the outgoing Bézier segment; only X/Y are used |

At least two non-overlapping waypoints are required.

## `TrajectoryConfig`

| Field | Default | Meaning |
| --- | ---: | --- |
| `maxVelocity` | `1.5` | Maximum chassis speed |
| `maxAcceleration` | `2.0` | Maximum forward acceleration |
| `maxDeceleration` | `2.5` | Maximum braking acceleration magnitude |
| `maxCentripetalAcceleration` | `2.0` | Curve-speed constraint |
| `maxWheelVelocity` | `1.8` | Speed ceiling for either wheel |
| `maxVoltage` | `0.0` | Model-based voltage ceiling; zero disables it |
| `leftFeedforward`, `rightFeedforward` | zero | `kS`, `kV`, and `kA` for voltage constraints |
| `trackWidth` | `0.30` | Effective left-to-right wheel spacing |
| `startVelocity`, `endVelocity` | `0.0` | Boundary speed magnitudes |
| `sampleDistance` | `0.025` | Geometry discretization target |
| `reversed` | `false` | Traverse the geometric path backward |

When `maxVoltage > 0`, both acceleration gains must be positive. Every limit,
dimension, and sample distance must be greater than zero. Invalid input throws
`std::invalid_argument`.

## `generateTrajectory(waypoints, config)`

Builds quintic Hermite or arbitrary-degree Bézier geometry, applies wheel, centripetal,
acceleration, deceleration, and optional voltage constraints, then returns a
time-indexed `Trajectory`. It allocates memory, so generate paths before the
autonomous control loop.

## `Trajectory`

| Member | Result |
| --- | --- |
| `empty()` | Whether the state list is empty |
| `duration()` | Final trajectory time |
| `length()` | Total path distance |
| `states()` | Const reference to every generated state |
| `sample(time)` | Interpolated state, clamped to the trajectory endpoints |

Each `TrajectoryState` contains `time`, `distance`, `pose`, `curvature`,
`velocity`, `acceleration`, and `angularVelocity`.


## Editable Bézier segments

Set `bezierToNext` on the **starting waypoint** of each Bézier segment.
An empty `controlPoints` vector makes a straight line. One control makes a
quadratic curve; six controls make a degree-seven curve. The endpoint remains
the next waypoint. Controls shape the curve; the robot does not drive through
each control point. There is no fixed control-count limit.

```cpp
std::vector<vantage::Waypoint> route = {
    {{0, 0, 0}, 0, true, {
        {0.2, 0.2, 0}, {0.4, 0.6, 0}, {0.7, 1.0, 0},
        {1.3, 1.0, 0}, {1.6, 0.6, 0}, {1.8, 0.2, 0}}},
    {{2, 0, 0}, 0}
};
auto trajectory = vantage::generateTrajectory(route, config);
```

Heading follows the Bézier tangent; `pose.theta` and `tangentScale` do not
shape Bézier segments. `config.reversed` keeps that geometry and turns the
robot's facing by 180° with negative drive velocity. Mirroring and reversing
waypoints also transform and reorder their controls.

Independent Bézier segments can meet at a corner, so the generator stops at
shared anchors whenever either adjacent segment is Bézier. Hermite-only routes
retain their existing continuous behavior. All existing speed, acceleration,
wheel, centripetal, and voltage constraints also apply to Bézier trajectories.
Studio's playback remains an approximate motion preview.

Existing two-field waypoint initializers keep working. Rebuild the library
when using the new waypoint fields; exports with control points require this
updated library.
