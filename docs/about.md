# About VantagePath

VantagePath exists for teams that have outgrown point-to-point PID and fixed
lookahead path following. It plans what the robot should be doing at every
instant—pose, curvature, velocity, acceleration, and wheel speed—then combines
model-based feedforward with feedback to stay on that plan.

The core is framework-neutral C++17. A project supplies sensor measurements,
time, battery voltage, and the final motor-voltage write. This keeps the same
trajectory mathematics usable in PROS, FRC, a desktop simulator, or another
embedded framework.

## Design goals

- predictable behavior at corners and endpoints;
- constraints that represent what a tank drive can physically do;
- no hidden motor or task ownership;
- measurable failure states instead of a single timeout boolean;
- host-side testing before code reaches a robot;
- a small public API students can understand and explain.

VantagePath is MIT licensed. See the
[GitHub repository](https://github.com/iSaaacH/VantagePath) for source and
release history.
