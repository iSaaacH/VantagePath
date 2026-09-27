import assert from "node:assert/strict";
import test from "node:test";
import { makeDocument, segmentPoint } from "./model.js";
import { routeSections } from "./planner.js";
import * as edits from "./route-edits.js";

const route = () => makeDocument().paths[0];
const robot = makeDocument().robot;

test("appending continues the curve smoothly in the same direction", () => {
  const path = { ...route(), segmentReversed: [false, true] };
  const { path: next, point } = edits.appendPoint(path, { x: 120, y: 60 });
  assert.equal(next.waypoints.at(-1).id, point.id);
  assert.equal(next.segmentReversed.at(-1), true, "keeps the previous drive direction");
  assert.equal(routeSections({ ...next, segmentReversed: next.segmentReversed.map(() => false) }).length, 1, "no corner at the old end point");
  assert.equal(path.waypoints.length, 3, "the original route is not changed");
});

test("appending on top of the last point is refused", () => {
  const path = route();
  const last = path.waypoints.at(-1);
  assert.match(edits.appendPoint(path, { x: last.x + 0.1, y: last.y }).error, /on top/);
});

test("appended points snap and stay on the field", () => {
  const { path } = edits.appendPoint(route(), { x: 150.3, y: 40.13 }, 0.25);
  assert.deepEqual([path.waypoints.at(-1).x, path.waypoints.at(-1).y], [144, 40.25]);
});

test("the point ahead stays a robot half-width inside the walls", () => {
  const ahead = edits.pointAhead(route(), robot);
  assert.ok(ahead.x <= 144 - robot.length / 2 && ahead.y <= 144 - robot.width / 2);
});

test("inserting on the path splits the nearest segment; far clicks don't", () => {
  const path = route();
  assert.equal(edits.insertPointOnPath(path, { x: 5, y: 140 }), null);
  const onCurve = segmentPoint(path, 1, 0.5);
  const inserted = edits.insertPointOnPath(path, { x: onCurve.x + 1, y: onCurve.y });
  assert.ok(inserted, "a click within reach of the curve inserts");
  assert.equal(inserted.index, 2);
  assert.equal(inserted.path.waypoints.length, 4);
  assert.ok(Math.hypot(inserted.point.x - onCurve.x, inserted.point.y - onCurve.y) < 1.5);
});

test("deleting an interior point joins its neighbours with a straight segment", () => {
  const path = route();
  const { path: next, nextPointId } = edits.deletePoint(path, path.waypoints[1].id);
  assert.equal(next.waypoints.length, 2);
  assert.deepEqual(next.controlPoints, [[]]);
  assert.equal(nextPointId, path.waypoints[2].id);
});

test("deleting the first point drops its segment", () => {
  const path = route();
  const { path: next } = edits.deletePoint(path, path.waypoints[0].id);
  assert.equal(next.segmentReversed.length, 1);
  assert.equal(next.controlPoints.length, 1);
});

test("control points can be added and removed", () => {
  const { path, control } = edits.addControl(route(), 0);
  assert.equal(path.controlPoints[0].length, 3);
  assert.equal(edits.deleteControl(path, 0, control.id).controlPoints[0].length, 2);
});

test("an alliance copy is rotated 180°, renamed and gets fresh ids", () => {
  const path = route();
  const copy = edits.allianceCopy(path, "red");
  assert.equal(copy.name, "Primary route (blue)");
  assert.notEqual(copy.id, path.id);
  assert.deepEqual([copy.waypoints[0].x, copy.waypoints[0].y], [126, 126]);
  assert.ok(copy.waypoints.every((point, i) => point.id !== path.waypoints[i].id));
  assert.equal(edits.allianceCopy(copy, "red").name, "Primary route (red)");
});

test("reversing a route reverses points, directions and controls", () => {
  const path = { ...route(), segmentReversed: [false, true] };
  const reversed = edits.reverseRoute(path);
  assert.equal(reversed.waypoints[0].x, 116);
  assert.deepEqual(reversed.segmentReversed, [true, false]);
  assert.deepEqual(reversed.controlPoints[0].map((p) => p.x), [112, 84]);
});

test("direction and geometry setters change only their segment", () => {
  const path = route();
  assert.deepEqual(edits.setSegmentDirection(path, 1, true).segmentReversed, [false, true]);
  assert.deepEqual(edits.setAllDirections(path, true).segmentReversed, [true, true]);
  assert.equal(edits.setSegmentGeometry(path, 0, "hermite").controlPoints[0], null);
  assert.deepEqual(edits.setSegmentGeometry(path, 0, "bezier").controlPoints[0], []);
  assert.equal(edits.withControlCount(path, 1, 0).controlPoints[1].length, 0);
});

test("mirroring keeps ids so selection survives", () => {
  const path = route();
  const mirrored = edits.mirrorPath(path, "left-right");
  assert.equal(mirrored.waypoints[0].id, path.waypoints[0].id);
  assert.equal(mirrored.waypoints[0].x, 126);
});

test("new routes start at the robot start and duplicates get new ids", () => {
  const start = { x: 130, y: 20, heading: 0 };
  const fresh = edits.newRoute(start, 2);
  assert.equal(fresh.name, "Route 2");
  assert.equal(fresh.waypoints[1].x, 94, "turns back when near the far wall");
  const copy = edits.duplicateRoute(fresh);
  assert.equal(copy.name, "Route 2 copy");
  assert.notEqual(copy.waypoints[0].id, fresh.waypoints[0].id);
});
