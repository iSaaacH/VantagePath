import assert from "node:assert/strict";
import test from "node:test";
import { cppExport, suggestedTimeoutMs } from "./export.js";
import { makeDocument, splitSegment } from "./model.js";
import { planRoute } from "./planner.js";
import { deleteMarker, deletePoint, reverseRoute } from "./route-edits.js";

test("timeouts are 1.5x the plan plus a quarter second, in 50 ms steps", () => {
  assert.equal(suggestedTimeoutMs(1), 1750);
  assert.equal(suggestedTimeoutMs(1.01), 1800);
});

test("a segment speed limit slows the plan and exports as pathSpeedScale", () => {
  const document = makeDocument();
  const base = planRoute(document.paths[0], document.robot).duration;
  document.paths[0].segmentSpeed = [30, null];
  assert.ok(planRoute(document.paths[0], document.robot).duration > base);
  assert.match(cppExport(document, "route"), /config\.pathSpeedScale = \[\]\(double percent\) \{ if \(percent < \d+\.\d{9}\) return 0\.500000000; return 1\.000000000; \};/);
});

test("markers become progress constants and chassis waits", () => {
  const document = makeDocument();
  document.paths[0].markers = [{ id: "a", name: "Intake on", segment: 1, t: 0.5 }];
  assert.match(cppExport(document, "route"), /constexpr double routeIntakeOnPercent = \d+\.\d;/);
  const chassis = cppExport(document, "route", "corner", "chassis");
  assert.match(chassis, /if \(chassis\.waitUntilProgress\(\d+\.\d\)\) \{\n {4}\/\/ Intake on/);
  assert.match(chassis, /chassis\.setPoseCorner\(18\.0000, 18\.0000, 0\.000000\)/);
});

test("the chassis export turns at corners, waits between routes and flags gaps", () => {
  const document = makeDocument();
  document.paths[0].controlPoints = [[], []];
  document.paths[0].waitAfterMs = 400;
  document.paths.push({ ...structuredClone(document.paths[0]), id: "b", name: "Second", waitAfterMs: 0 });
  const chassis = cppExport(document, "auto", "corner", "chassis");
  assert.match(chassis, /chassis\.turnToHeading\(37\.7, \d+, \{\}, false\);/);
  assert.match(chassis, /pros::delay\(400\);/);
  assert.match(chassis, /WARNING: starts at \(18\.0, 18\.0\)/);
});

test("routes that can't be planned are skipped with a comment, not a crash", () => {
  const document = makeDocument();
  document.paths[0].waypoints[1] = { ...document.paths[0].waypoints[1], x: 18, y: 18 };
  document.paths[0].controlPoints[0] = [];
  assert.match(cppExport(document, "route"), /\/\/ SKIPPED "Primary route":/);
});

test("markers follow their segment through split, delete and reverse", () => {
  const path = { ...makeDocument().paths[0], markers: [{ id: "m", name: "M", segment: 1, t: 0.8 }] };
  const split = splitSegment(path, 1, 0.5).path;
  assert.deepEqual([split.markers[0].segment, Number(split.markers[0].t.toFixed(6))], [2, 0.6]);
  const earlier = splitSegment(path, 0, 0.5).path;
  assert.equal(earlier.markers[0].segment, 2, "markers after the split shift up");
  assert.equal(reverseRoute(path).markers[0].segment, 0);
  assert.equal(Number(reverseRoute(path).markers[0].t.toFixed(6)), 0.2);
  assert.equal(deletePoint(path, path.waypoints[2].id).path.markers.length, 0, "a marker on a removed segment goes with it");
  assert.equal(deleteMarker(path, "m").markers.length, 0);
});
