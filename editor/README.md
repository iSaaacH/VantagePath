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
