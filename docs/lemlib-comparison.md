# VantagePath compared with LemLib

## The short answer

VantagePath is designed for teams that need fast, complex tank-drive routes
with predictable wheel behavior. It plans a timed state for the entire route,
slows before a difficult curve, models required motor voltage, and controls
each wheel against measured velocity. That directly addresses oscillation,
corner overshoot, and inconsistent arrival speed.

LemLib remains easier to adopt, has substantially more competition use, and
offers familiar point-motion APIs. VantagePath is not automatically better for
every robot. It is better when trajectory feasibility, per-wheel control,
diagnostics, portability, and source ownership matter more than the largest
existing user community.

This page compares VantagePath with the LemLib 0.5.6 headers and binary that
were previously installed in 4613R. Refer to the [current LemLib
documentation](https://lemlib.readthedocs.io/en/stable/) when evaluating a
different release.

## The fundamental control difference

### Geometric point tracking

A geometric follower asks, “Where is the pursuit point, and how should I steer
toward it?” Lookahead creates a tradeoff: a short lookahead hugs the path but
can steer aggressively; a long lookahead is smoother but may cut curves. Speed
and geometry are not a complete prediction of what each wheel must do at every
instant.

### Timed trajectory tracking

VantagePath asks, “At this time, what pose, velocity, acceleration, curvature,
and left/right wheel speed should the robot have?” It first generates smooth
geometry, then calculates a feasible speed profile from physical constraints.
The runtime controller corrects error around that planned motion.

```text
waypoints
  -> continuous-curvature spline
  -> wheel/curve/acceleration/voltage constraints
  -> timed pose and wheel-speed states
  -> nonlinear pose feedback
  -> per-wheel feedforward + velocity PID
  -> battery-limited paired voltages
```

## Feature-by-feature comparison

| Concern | LemLib 0.5.6 in 4613R | VantagePath |
| --- | --- | --- |
| Route reference | Geometric lookahead / point-motion target | Timed pose, curvature, velocity, and acceleration |
| Curves | Runtime steering follows path geometry | Planner lowers speed before curvature becomes difficult |
| Wheel feasibility | Chassis-level command limits | Explicit maximum speed for either differential wheel |
| Acceleration | Motion tuning and command behavior | Separate forward acceleration and braking constraints |
| Voltage model | No end-to-end identified wheel model in this integration | Per-side `kS + kV*v + kA*a`, also used during planning |
| Feedback | Lateral/angular motion PID | Nonlinear full-pose feedback plus independent wheel PID |
| Saturation | Command limiting | Battery-aware paired scaling preserves left/right ratio |
| Endpoint | Exit conditions and timeout | Planned end velocity plus pose, heading, and wheel-speed settle window |
| Fault reporting | Completion/timeout-oriented motion API | Idle, running, settled, timed out, diverged, and saturation telemetry |
| Sensor redundancy | Project-specific wrappers required | Built-in fused encoders and dual-IMU PROS adapters |
| Source ownership | Prebuilt archive used in this robot | Complete MIT-licensed source compiled with the project |
| Portability | PROS/V5-oriented | Framework-neutral C++17 core; PROS adapters are optional |
| Host testing | Robot-focused integration | Native tests for paths, constraints, control, odometry, and fusion |

## Why VantagePath should oscillate less

Oscillation is not fixed by one special gain. VantagePath removes common causes
at several layers:

1. Quintic geometry avoids an instantaneous curvature change at waypoint
   boundaries.
2. Time parameterization slows before high curvature instead of discovering
   the corner through tracking error.
3. Feedforward supplies most predicted wheel voltage, leaving feedback to
   correct smaller errors.
4. Derivative-on-measurement is filtered, reducing amplification of encoder
   noise.
5. Conditional integration stops accumulating farther into voltage saturation.
6. Pose corrections and output voltage are bounded.
7. Arrival requires consecutive settled cycles and does not force a minimum
   voltage that kicks the robot across the endpoint.

Bad measurements or excessive gains can still make any controller oscillate.
Follow the configuration and tuning tutorials in order.

## Why complex trajectories improve

On an S-curve, the outside wheel may need much more speed than the chassis
center. VantagePath evaluates both wheel speeds from local curvature and track
width, then caps chassis speed so neither wheel exceeds its limit. It also
accounts for centripetal acceleration and, when characterized, predicted motor
voltage—including wheel acceleration caused by changing curvature.

Constraints are therefore handled while planning the route, before a motor has
saturated and the robot has departed from it.

## Sensor and failure behavior

The built-in fusion engine operates on sensor deltas:

- three encoders use peer consensus before rejecting one outlier;
- two IMUs average while they agree and use continuity when they do not;
- a disconnected sensor is excluded;
- a returning sensor is reseeded without injecting a position jump;
- per-sensor alive/rejected state is available for driver alerts and logs.

The follower separately stops on divergence and distinguishes a clean settle
from a timeout. These are explicit states an autonomous routine can act on.

## Where LemLib may still be the better choice

Choose LemLib when the priority is its established ecosystem, familiar APIs,
existing team knowledge, GUI workflow, or broad field history. A simple robot
running point-to-point motions may not benefit enough from a timed trajectory
stack to justify new tuning and validation.

Choose VantagePath when you need:

- one continuous route through several waypoints;
- planned wheel, curve, acceleration, braking, and voltage feasibility;
- separate left/right characterization and telemetry;
- deterministic failure states;
- redundant encoder and IMU handling built into the library;
- source that can be inspected, changed, simulated, and host-tested.

## Migration mapping

| Previous concept | VantagePath equivalent |
| --- | --- |
| chassis pose | `DifferentialDriveOdometry::pose()` |
| `moveToPoint` / `moveToPose` | two-waypoint `generateTrajectory()` plus `TrajectoryFollower` |
| path following | multi-waypoint `generateTrajectory()` |
| drivetrain curve/turn gains | nonlinear pose configuration plus wheel velocity PID |
| motor model assumptions | measured left/right `FeedforwardConfig` |
| timeout | `kTimedOut` |
| successful arrival | `kSettled` |
| unrecoverable tracking error | `kDiverged` |
| custom fused motor/IMU wrapper | `vantage::pros::FusedDrive` / `FusedImu` |

[Configure the drivetrain](tutorials/2-configuration.md){ .md-button .md-button--primary }
[Follow the tuning procedure](tutorials/4-tuning.md){ .md-button }
