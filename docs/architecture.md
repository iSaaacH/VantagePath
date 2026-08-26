# Architecture

## 1. Continuous-curvature geometry

Adjacent waypoints are connected with quintic Hermite polynomials. Position,
first derivative, and second derivative agree at segment endpoints, avoiding
the curvature impulses that cause a tank drive to demand an instantaneous
left/right velocity change. Waypoint headings specify tangent direction;
`tangentScale` can override the automatic chord-based magnitude.

## 2. Time parameterization

Dense geometric samples receive a local speed ceiling from all constraints:

- chassis maximum velocity;
- differential wheel speed, using `v_left/right = v(1 +/- curvature*track/2)`;
- centripetal acceleration, using `v <= sqrt(a_c / |curvature|)`.
- optional left/right feedforward voltage, including the wheel acceleration
  contributed by curvature changing along the path.

A forward pass enforces acceleration and a backward pass enforces deceleration.
This is why the follower can anticipate a turn instead of reacting after
cross-track error appears.

## 3. Full-pose nonlinear feedback

The reference is sampled by elapsed time. Pose error is transformed into the
robot frame, then a velocity-scheduled nonlinear controller adds bounded
longitudinal, lateral, and heading corrections to reference linear/angular
velocity. Correction gain falls naturally near a stopped endpoint, and hard
bounds prevent a localization jump from creating an unsafe command.

A small configurable feedback-speed floor remains active at a stopped
reference. Without it, the standard nonlinear gain collapses to zero at the
endpoint and a robot that arrives with residual pose error cannot correct it.

## 4. Wheel control

Differential kinematics turns chassis velocity into independent wheel targets.
Each wheel receives identified static, velocity, and acceleration feedforward,
then a small velocity PID correction. Derivative acts on measurement and is
low-pass filtered. Integration is clamped and conditionally disabled when its
output would drive farther into saturation.

Finally, both voltages are scaled together against the lower of measured
battery voltage and nominal voltage. Preserving their ratio preserves intended
curvature under saturation.

## 5. Deterministic failure semantics

A motion only reports `kSettled` after time is complete and pose, heading, and
both wheel velocities remain inside tolerance for consecutive cycles. It
reports `kTimedOut` if convergence takes too long and `kDiverged` if pose error
exceeds a configured safety envelope. All terminal fault outputs are zero.
