# VantagePath Studio

Serve this directory with any static web server:

```sh
python3 -m http.server 4173 --directory editor
```

Then open <http://localhost:4173>. The editor has no package install or build
step. Documents autosave in the browser and can be downloaded as `.vpath`
JSON. C++ export supports the bottom-left corner authoring frame or the official
VEX GPS centre frame.

Run the pure model tests with:

```sh
node --test editor/model.test.mjs
```

## Routes and control points

Select a route in the left menu, then select its segment. Routes can be renamed
and duplicated. The inspector groups segment geometry, drive direction, and
control points together; robot, field, mirror, and export settings expand on demand.
Each route segment also has an always-visible **Forward / Reverse** switch in the
left rail. Use **Set entire route** for a one-click route-wide direction, then
override individual segments when the route mixes forward and reverse driving.

New segments use Bézier geometry. Set **Control points** to `0` for a straight
line, `6` for six editable controls, or another nonnegative integer. Use **+ Add**
and the individual remove buttons to change the list. Drag the labelled C handles
on the field or enter their X/Y coordinates. Anchors are the segment endpoints;
controls bend the curve without adding destinations. Heading follows the curve.

Old files keep their heading/tangent spline geometry until you choose Bézier.
Switching curve types resets that segment's controls; Undo restores them.
Controls persist in autosave and `.vpath` files and follow mirror/reverse actions.
C++ exports include the full control polygon, including GPS-frame conversion,
and require the updated VantagePath library. Playback timing is an estimate.

## Fine positioning and robot start

Dragging now snaps to **¼ inch** by default. Select 1″ or 6″ in the toolbar if
needed, hold **Shift** to bypass snapping, or hold **Alt** for movement at one
fifth of the pointer speed. Grabbing the edge of a point preserves the cursor
offset. Small blue diamonds labelled **C** shape the curve; white circles labelled
**P** are route points the robot passes through. The field legend explains both.

**Robot start** is a separate document setting with X, Y and facing inputs and a
purple robot outline on the field. Drag its outline to move it. New routes begin
at this position. Changing the start does not silently move existing routes;
use **Set first route point to robot start** when you want to align one. Route
mirroring, reversal and deletion leave robot start unchanged.

Start placement is saved in `.vpath` files and autosave. Older documents initialize
it from their first route point. C++ export includes a separate `…RobotStart`
`Pose2d` for initializing localization; GPS export includes both the GPS pose
and its conversion to the trajectory frame. Playback previews the selected route,
while the purple outline remains at the document's initial placement. Drag the
yellow playback robot directly along the field route to scrub the preview; the
timeline slider remains available for fine adjustment.
