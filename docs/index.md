# Documentation Home

Welcome to the VantagePath documentation. VantagePath is a C++17 trajectory
generation and control library for differential-drive robots.

If this is your first time using VantagePath, follow the numbered
[tutorials](tutorials/1-getting-started.md) in order. They begin with a blank
project and end with constrained, multi-segment autonomous trajectories,
diagnostics, and safe robot commissioning.

If VantagePath is already configured and you need a class or function, go
straight to the [API reference](reference/index.md).

!!! warning "Robot testing is required"

    Example gains are deliberately conservative. Wheel diameter, track width,
    feedforward, feedback gains, and trajectory limits must be measured on your
    own drivetrain before full-speed use.

## What VantagePath provides

- C2-continuous quintic spline generation
- velocity, acceleration, deceleration, wheel-speed, centripetal, and voltage constraints
- time-indexed nonlinear pose feedback
- independent wheel feedforward and velocity feedback
- anti-windup, derivative filtering, and battery-aware voltage desaturation
- differential-drive SE(2) odometry
- built-in fused drive-encoder and dual-IMU classes for PROS
- explicit settled, timed-out, diverged, saturated, and cancelled states
- no dependency on PROS, WPILib, an RTOS, or a particular motor vendor

[Begin: install VantagePath](tutorials/1-getting-started.md){ .md-button .md-button--primary }
[Configure your drivetrain](tutorials/2-configuration.md){ .md-button }
[Compare VantagePath with LemLib](lemlib-comparison.md){ .md-button }
[Browse the API reference](reference/index.md){ .md-button }
