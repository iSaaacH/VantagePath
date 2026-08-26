# API reference

VantagePath is a C++17 library. Include the complete public API with:

```cpp
#include <vantage/vantage.hpp>
```

All distances, velocities, and accelerations must use one consistent unit
system. The examples use metres and seconds. Angles and angular rates are
always radians and radians per second.

## Modules

| Module | Purpose |
| --- | --- |
| [Geometry](geometry.md) | Poses, angle wrapping, interpolation, and robot-frame error |
| [Trajectory](trajectory.md) | Quintic paths and constraint-aware time parameterization |
| [Control](control.md) | Differential-drive kinematics, pose feedback, feedforward, and wheel PID |
| [Odometry](odometry.md) | Encoder and gyro integration on SE(2) |
| [Follower](follower.md) | End-to-end trajectory execution, termination, and diagnostics |

## Execution model

Generate trajectories before the time-critical autonomous period. During
autonomous, call `TrajectoryFollower::update()` from one fixed-period task and
apply both returned voltages in that same tick. The caller owns sensors, motor
I/O, timing, logging, and the autonomous state machine.

!!! warning "Lifetime requirement"
    `TrajectoryFollower::start()` stores a pointer to the supplied
    `Trajectory`. Keep that trajectory alive until the follower finishes or is
    cancelled.

