# Trajectory

Header: `#include <vantage/trajectory.hpp>`

## `Waypoint`

| Field | Meaning |
| --- | --- |
| `pose` | X, Y, and spline tangent heading |
| `tangentScale` | Tangent magnitude; `0` selects a distance-based default |

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

Builds C2-continuous quintic Hermite geometry, applies wheel, centripetal,
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

