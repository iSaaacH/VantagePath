# Geometry

Header: `#include <vantage/geometry.hpp>`

## `Pose2d`

```cpp
struct Pose2d {
  double x;
  double y;
  double theta;
};
```

A field pose. `theta` is in radians; zero points along positive field X and
positive rotation is counter-clockwise.

## `PoseError`

```cpp
struct PoseError {
  double longitudinal;
  double lateral;
  double heading;
};
```

An error expressed in the current robot frame. Positive longitudinal error is
forward and positive lateral error is to the robot's left.

## Helpers

### `wrapAngle(double radians)`

Returns the equivalent angle near the interval `[-pi, pi]`.

### `sinc(double x)`

Returns `sin(x) / x` with a numerically stable approximation near zero.

### `errorInRobotFrame(current, target)`

Transforms field-frame displacement into longitudinal, lateral, and wrapped
heading error relative to `current`.

### `interpolate(a, b, u)`

Interpolates position and follows the shortest wrapped angular displacement.
`u` is clamped to `[0, 1]`.

`vantage::kPi` is provided as a compile-time `double` constant.

