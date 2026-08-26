# Sensor Fusion and PROS adapters

VantagePath includes a platform-neutral fusion engine and ready-to-use PROS
classes for fused drive encoders and dual inertial sensors.

## Platform-neutral fusion

Header: `#include <vantage/sensor_fusion.hpp>`

`IncrementalSensorFusion` combines cumulative readings by comparing their
per-update changes. This permits different sensor zero points and prevents a
sensor that reconnects from injecting the distance it missed.

```cpp
vantage::IncrementalSensorFusion fusion(
    3, {12.0, 0.5, true});

const vantage::FusionOutput output = fusion.update(
    {encoderA, encoderB, encoderC});
```

The three configuration values are absolute delta threshold, relative delta
threshold, and whether outlier rejection is enabled. Non-finite readings are
automatically excluded.

Fusion behavior depends on the evidence available:

- Three or more sensors reject one reading only when all peers agree without it.
- Two disagreeing sensors use continuity with the previously accepted delta.
- One live sensor continues alone.
- Zero live sensors report `available == false` and hold the last value.

`FusionOutput` contains the fused cumulative `value`, accepted `delta`, number
of `contributing` sensors, and `available`. Use `alive(index)`,
`rejected(index)`, and `enabled(index)` for diagnostics.

## Built-in PROS `FusedDrive`

Header: `#include <vantage/pros.hpp>`

```cpp
std::vector<vantage::pros::DriveMotorSpec> leftMotors = {
    {-7, pros::v5::MotorGears::blue, 1.0},
    {-2, pros::v5::MotorGears::blue, 1.0},
    {-6, pros::v5::MotorGears::green, 3.0},
};

vantage::pros::FusedDrive left(
    leftMotors,
    {12.0, 0.5, true});
```

`toCommon` converts each motor's encoder into the shared output-shaft unit
before comparison. This is essential when different cartridges or external
gearing drive the same shaft.

After PROS device startup, apply the physical cartridge configuration:

```cpp
left.initialize();
```

`get_position()` returns the fused common-shaft position. Motor voltage calls
remain ordinary `pros::MotorGroup` calls. Diagnostics and runtime controls are:

- `motorPositionDegrees(index)`
- `alive(index)`, `rejected(index)`, and `contributing()`
- `setReadingEnabled(index, enabled)` for a faulty encoder
- `setOutputEnabled(index, enabled)` for a faulty motor output
- `setOutlierRejection(enabled)` for controlled diagnosis

## Built-in PROS `FusedImu`

```cpp
vantage::pros::FusedImu imu(
    15,   // primary smart port
    20,   // secondary smart port
    3.0); // maximum per-tick disagreement in degrees
```

It is a `pros::Imu` subclass. `get_rotation()` returns continuous fused degrees
and `get_heading()` returns the same result wrapped to `[0, 360)`. `reset()`,
`is_calibrating()`, and `set_data_rate()` operate on both enabled sensors.

If the sensors agree, their changes are averaged. If they disagree, the change
closest to the previous accepted motion wins. One sensor can disconnect while
the other continues; only loss of both returns a non-finite heading.

Use `primaryAlive()`, `secondaryAlive()`, `primaryRejected()`,
`secondaryRejected()`, `disagreementDegrees()`, and `contributing()` in driver
alerts and logs.

!!! warning
    Fusion handles isolated noise, glitches, and disconnects. Two sensors that
    drift together can still be wrong. Validate heading against field landmarks
    or another independent localization source.
