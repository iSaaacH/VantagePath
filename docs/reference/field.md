# Field coordinates and mirroring

VantagePath Studio authors paths in inches with `(0, 0)` at the bottom-left
inside corner of the field. Positive X points right, positive Y points up, and
headings use VantagePath's mathematical convention: zero along positive X and
positive rotation counter-clockwise.

This corner frame is an editor convention. The official VEX V5 GPS origin is
actually at the **centre** of the field, not at a corner. GPS X and Y span
approximately -1.8 m to +1.8 m, and the sensor heading uses zero toward the top
of the field with positive rotation clockwise. See the
[official VEX GPS guide](https://kb.vex.com/hc/en-us/articles/360061932711-Using-the-GPS-Sensor-with-VEX-V5).

## Convert a sensor pose

```cpp
vantage::Pose2d gpsPose{xMetres, yMetres, gpsHeadingRadians};
vantage::Pose2d editorPose = vantage::vexGpsToCorner(
    gpsPose, {3.6576, 3.6576});
```

`cornerToVexGps()` performs the inverse conversion. Both functions preserve
the supplied linear unit, so field dimensions must use that same unit.

## Mirror a route

```cpp
auto otherAlliance = vantage::mirrorWaypoints(
    waypoints, vantage::FieldMirror::kAlliance180, {144.0, 144.0});
auto otherStartingSide = vantage::mirrorWaypoints(
    waypoints, vantage::FieldMirror::kLeftRight, {144.0, 144.0});
```

`kAlliance180` rotates positions and headings by 180 degrees. The left/right
and bottom/top options are true reflections. `reverseWaypoints()` reverses the
ordered geometry, while `TrajectoryConfig::reversed` makes the robot drive the
same ordered geometry backward.

## Studio

Open [VantagePath Studio](https://isaaach.github.io/VantagePath/studio/) in a
browser. The dependency-free source is in the repository's `editor/` directory.
It supports exact VantagePath quintic previews, draggable tangent controls,
alliance overlays, all mirror modes, reverse order and reverse driving,
undo/redo, browser autosave, `.vpath` files, and C++ export.

The Override field overlay follows version 1.1 of the 2026-27 game manual's diagonal
Autonomous Line, Midfield, alliance-side starting areas, Goals, Loaders, and
Toggles. Treat the editor overlay as planning guidance; the current official
manual remains authoritative for legality.

### Field-image calibration

Studio uses the same calibrated field asset as the separate
[`iSaaacH/vex-pathing`](https://github.com/iSaaacH/vex-pathing) application. It
is an orthographic top-down render cropped at the measured foam-floor edges and
resized to 2048 × 2048. The square image is mapped directly onto coordinates
`(0, 0)` through `(144, 144)` with no drawing offset or secondary scale factor.

The overlay grid is generated independently at 24-inch intervals. Its seven
lines per axis bound six field tiles and provide a standing visual calibration
check. Goal centres in the calibrated image land within 0.37 inches of the
expected 24-inch lattice. See Studio's `NOTICE.md` for the CC BY 4.0 attribution.
