# VantagePath

VantagePath is a deterministic C++17 trajectory and control library for
differential-drive robots. It is designed for small embedded controllers: no
RTOS, motor vendor, global state, or heap allocation occurs in the control
loop. The same core can run in a host simulator, PROS/VEX V5, or an FRC robot.

Instead of chasing a geometric lookahead point, VantagePath generates a timed,
continuous-curvature trajectory and tracks its pose, velocity, and curvature.
The output pipeline is:

```
waypoints -> quintic spline -> physical constraints -> timed states
          -> nonlinear pose feedback -> wheel targets
          -> feedforward + wheel PID -> battery-limited voltages
```

## Highlights

- C2-continuous quintic Hermite paths with automatic tangent selection
- forward/backward time parameterization
- chassis speed, wheel speed, acceleration, deceleration, and centripetal limits
- optional per-side feedforward voltage constraint during time parameterization
- velocity-scheduled nonlinear pose feedback with bounded corrections
- separate left/right `kS + kV*v + kA*a` feedforward and wheel velocity PID
- derivative filtering, conditional-integration anti-windup, voltage desaturation
- fused-gyro differential odometry and framework-neutral units
- built-in fault-tolerant PROS `FusedDrive` and dual-IMU `FusedImu` adapters
- settle window, timeout, cancel, saturation telemetry, and divergence abort
- host-side tests and a documented hardware adapter boundary
- browser-based path studio with draggable tangent controls, alliance mirrors,
  reverse paths, `.vpath` save/open, and C++ export

## Minimal use

```cpp
#include <vantage/vantage.hpp>

vantage::TrajectoryConfig limits;
limits.trackWidth = 0.305;        // metres
limits.maxVelocity = 1.8;         // m/s
limits.maxAcceleration = 2.2;     // m/s^2
limits.maxWheelVelocity = 2.1;    // m/s

auto trajectory = vantage::generateTrajectory({
  {{0.0, 0.0, 0.0}},
  {{1.0, 0.6, 0.7}},
  {{2.0, 1.0, 0.0}},
}, limits);

vantage::FollowerConfig gains;
gains.trackWidth = limits.trackWidth;
gains.leftFeedforward = {0.35, 5.1, 0.22};
gains.rightFeedforward = {0.36, 5.0, 0.23};
gains.leftVelocityPid.kp = gains.rightVelocityPid.kp = 1.1;
vantage::TrajectoryFollower follower(gains);
follower.start(trajectory, nowSeconds());

// Call at a fixed period (10-20 ms):
auto out = follower.update(nowSeconds(), measuredPose(), measuredWheelSpeeds(),
                           measuredBatteryVolts());
setDriveVoltage(out.leftVoltage, out.rightVoltage);
```

Follow the numbered [getting-started
tutorials](https://isaaach.github.io/VantagePath/tutorials/1-getting-started/) or
open the [API reference](https://isaaach.github.io/VantagePath/reference/).

## Path studio

Open [VantagePath Studio](https://isaaach.github.io/VantagePath/studio/) on
GitHub Pages, or launch the zero-install editor locally from the repository
root:

```sh
python3 -m http.server 4173 --directory editor
```

Then open <http://localhost:4173>. Studio uses a bottom-left corner authoring
origin and can export through the official centre-origin VEX GPS frame. See the
[field coordinate reference](docs/reference/field.md) for the exact conversion.

## Build and test

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

To vendor it without a package manager, add this repository as a Git submodule
and compile `src/*.cpp` with `include/` on the include path.

## Status

The algorithms and host tests are complete. Robot-specific gains and physical
limits must be measured on each drivetrain; no library can safely infer those.
See `docs/tuning.md` before running at full speed.

MIT licensed.
