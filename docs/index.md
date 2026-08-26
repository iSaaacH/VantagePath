# VantagePath

VantagePath turns a small set of poses into a physically achievable,
time-indexed trajectory and closes the loop at both robot-pose and wheel-speed
levels. It targets differential/tank drivetrains and keeps hardware access
outside the library.

## What problem it solves

Point-to-point PID and fixed-lookahead pure pursuit answer “where should I aim?”
but not “where should the robot be at this time, at what velocity and
acceleration?” That omission makes aggressive corners, voltage sag, wheel-speed
mismatch, and precise arrival harder to manage. VantagePath carries position,
heading, curvature, velocity, acceleration, and time through the entire stack.

## Coordinate and unit contract

- `x` points forward at zero heading; `y` points left.
- heading is radians, counter-clockwise positive.
- use any internally consistent length unit. SI metres are strongly recommended.
- time is seconds, angular velocity is radians/second, output is volts.
- encoder distance and wheel velocity must use the same length unit as paths.

## Start here

1. Read [Architecture](architecture.md).
2. Characterize and tune in the exact order in [Tuning](tuning.md).
3. Implement the thin sensor/motor adapter described in [API](api.md).
4. Pass every item in the [Safety checklist](safety.md).

VantagePath is not a substitute for measuring wheel diameter, effective track
width, motor feedforward, sensor signs, or loop timing on the real robot.
