# API reference

All public declarations live in `include/vantage/`; include everything with
`#include <vantage/vantage.hpp>`.

## Geometry

`Pose2d { x, y, theta }` stores a field pose. `PoseError` stores longitudinal,
lateral, and heading error in the robot frame. `wrapAngle`, `sinc`,
`errorInRobotFrame`, and `interpolate` are allocation-free helpers.

## Trajectory generation

`Waypoint { pose, tangentScale }` defines spline boundary conditions.
`TrajectoryConfig` supplies maximum chassis velocity, acceleration,
deceleration, centripetal acceleration, wheel velocity, track width, boundary
velocities, spatial sampling, and reverse direction.

Set `maxVoltage` and the left/right `DriveFeedforwardConstraint` values to make
time parameterization limit acceleration from predicted voltage. The planner
includes the additional wheel acceleration caused by changing curvature. A
zero `maxVoltage` disables this optional constraint.

`generateTrajectory(waypoints, config)` validates its inputs and returns a
`Trajectory`. Generation may allocate and should happen before a time-critical
autonomous period. `Trajectory::sample(seconds)` uses binary search and
interpolation to return a `TrajectoryState` containing time, path distance,
pose, curvature, velocity, acceleration, and angular velocity.

## Control

`DifferentialDriveKinematics` converts between `ChassisSpeeds` and
`WheelSpeeds`. `NonlinearPoseController::calculate` turns current pose and a
timed reference into corrected chassis velocity.

`MotorFeedforward` implements `kS*sign(v) + kV*v + kA*a`.
`VelocityPid` provides filtered derivative-on-measurement and conditional
integration. Call it at a known `dt` and provide the remaining output bounds.

## End-to-end follower

Construct `TrajectoryFollower` from `FollowerConfig`, call `start`, then call
`update` at a fixed 10–20 ms period. Supply monotonic seconds, current pose,
measured left/right wheel speeds, and current battery voltage. Apply the two
returned voltages in the same control tick.

Inspect `FollowerOutput.status`, `poseError`, `wheelSetpoint`, and `saturated`
for logging. On `kSettled`, `kTimedOut`, or `kDiverged`, stop/brake the drive and
advance or abort the autonomous state machine as appropriate. `cancel()` is
safe from the owner of the control loop.

## Odometry

`DifferentialDriveOdometry::update` accepts cumulative left distance,
cumulative right distance, and fused continuous gyro heading. It integrates a
finite SE(2) twist rather than applying a first-order Cartesian approximation.
Call `reset` whenever the field pose is externally established.
