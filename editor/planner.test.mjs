import assert from "node:assert/strict";
import { cppExport } from "./export.js";
import test from "node:test";
import { analyzeRoute, wallOverlap } from "./analysis.js";
import { anchorHeading, DEFAULT_ROBOT, fromDisplayHeading, historySnapshot, isHeadingDerived, makeDocument, restoreSnapshot, smoothJoin, splitSegment, toDisplayHeading } from "./model.js";
import { planRoute, poseAtTime, routeSections, timeAtDistance, turnDuration } from "./planner.js";

function corneredRoute() {
  const document = makeDocument();
  document.paths[0].controlPoints = [[], []];
  return document;
}

test("the default route is smooth and drives through P2 in one section", () => {
  const { paths: [path] } = makeDocument();
  const sections = routeSections(path);
  assert.equal(sections.length, 1);
  const plan = planRoute(path, DEFAULT_ROBOT);
  assert.equal(plan.error, null);
  assert.equal(plan.steps.filter((step) => step.type === "turn").length, 0);
});

test("a straight-leg corner splits into two sections with a point turn", () => {
  const { paths: [path] } = corneredRoute();
  const sections = routeSections(path);
  assert.equal(sections.length, 2);
  assert.ok(sections[0].endsAtKink);
  const plan = planRoute(path, DEFAULT_ROBOT);
  const turn = plan.steps.find((step) => step.type === "turn");
  assert.ok(turn, "the robot turns in place at the corner");
  assert.ok(Math.abs(turn.angle - (Math.atan2(62, 48) - Math.atan2(32, 50))) < 1e-9);
  assert.ok(plan.duration > plan.steps[0].duration + turn.duration);
});

test("preview time reflects curve slow-downs and joins, not one straight profile", () => {
  const { paths: [path] } = corneredRoute();
  const plan = planRoute(path, DEFAULT_ROBOT);
  const firstDrive = plan.steps[0];
  assert.equal(firstDrive.states[0].velocity, 0);
  assert.equal(firstDrive.states.at(-1).velocity, 0, "the robot is at rest before turning at the corner");
});

test("playback pose and time are consistent along the route", () => {
  const { paths: [path] } = corneredRoute();
  const plan = planRoute(path, DEFAULT_ROBOT);
  const start = poseAtTime(plan, 0);
  assert.deepEqual([start.x, start.y], [18, 18]);
  const end = poseAtTime(plan, plan.duration);
  assert.ok(Math.abs(end.x - 116) < 1e-9 && Math.abs(end.y - 112) < 1e-9);
  const halfway = timeAtDistance(plan, plan.length / 2);
  assert.ok(Math.abs(poseAtTime(plan, halfway).distance - plan.length / 2) < 0.05);
});

test("point turns take longer for bigger angles", () => {
  assert.equal(turnDuration(0, DEFAULT_ROBOT), 0);
  assert.ok(turnDuration(Math.PI, DEFAULT_ROBOT) > turnDuration(Math.PI / 4, DEFAULT_ROBOT));
});

test("corner checks offer a smooth-join fix that removes the stop", () => {
  const document = corneredRoute();
  const path = document.paths[0];
  const issues = analyzeRoute(path, planRoute(path, document.robot), document.robot);
  const corner = issues.find((issue) => issue.id === "corner-1");
  assert.ok(corner && corner.action === "smooth");
  const smoothed = smoothJoin(path, 1);
  assert.equal(routeSections(smoothed).length, 1, "the join is tangent-continuous after smoothing");
  assert.equal(smoothed.controlPoints[0].length, 2, "a straight leg gains two controls to bend at one end");
});

test("smoothing a Hermite join sets the shared anchor heading", () => {
  const document = makeDocument();
  const path = document.paths[0];
  path.controlPoints = [null, []];
  path.waypoints[1].heading = 0;
  assert.ok(routeSections(path).length > 1);
  assert.equal(routeSections(smoothJoin(path, 1)).length, 1);
});

test("a reversal with a heading jump gets a turn warning", () => {
  const document = corneredRoute();
  document.paths[0].segmentReversed = [false, true];
  const path = document.paths[0];
  const issues = analyzeRoute(path, planRoute(path, document.robot), document.robot);
  assert.ok(issues.some((issue) => issue.id === "cusp-1"));
});

test("tight curves and wall crossings are reported", () => {
  const document = makeDocument();
  const path = document.paths[0];
  path.waypoints[0] = { ...path.waypoints[0], x: 4, y: 4 };
  path.controlPoints[0] = [{ id: "a", x: 30, y: 4 }, { id: "b", x: 30, y: 30 }, { id: "c", x: 60, y: 30 }, { id: "d", x: 29.5, y: 5 }];
  const issues = analyzeRoute(path, planRoute(path, document.robot), document.robot);
  assert.ok(issues.some((issue) => issue.id === "wall"), "an 18 in robot centred 4 in from the wall crosses it");
  assert.ok(issues.some((issue) => issue.id === "tight-curve"));
});

test("overlapping adjacent points are an error, not an exception", () => {
  const document = makeDocument();
  const path = document.paths[0];
  path.waypoints[1] = { ...path.waypoints[1], x: 18, y: 18 };
  path.controlPoints[0] = [];
  const issues = analyzeRoute(path, planRoute(path, document.robot), document.robot);
  assert.equal(issues[0].severity, "error");
});

test("wall overlap measures the rotated footprint", () => {
  assert.equal(wallOverlap({ x: 9, y: 72, heading: 0 }, { length: 18, width: 18 }), 0);
  assert.ok(Math.abs(wallOverlap({ x: 9, y: 72, heading: Math.PI / 4 }, { length: 18, width: 18 }) - (9 * Math.SQRT2 - 9)) < 1e-9);
});

test("splitting a Bézier segment keeps its shape", () => {
  const { paths: [path] } = makeDocument();
  const before = planRoute(path, DEFAULT_ROBOT);
  const { path: split, point } = splitSegment(path, 0, 0.4);
  assert.equal(split.waypoints.length, 4);
  assert.equal(split.waypoints[1].id, point.id);
  assert.equal(split.controlPoints.length, 3);
  assert.equal(split.segmentReversed.length, 3);
  const after = planRoute(split, DEFAULT_ROBOT);
  assert.ok(Math.abs(after.length - before.length) < 1e-3, "de Casteljau split preserves the curve");
  assert.equal(routeSections(split).length, 1, "the new anchor is tangent-continuous");
});

test("anchor headings on Bézier segments come from the curve", () => {
  const { paths: [path] } = makeDocument();
  assert.ok(isHeadingDerived(path, 1));
  assert.ok(Math.abs(anchorHeading(path, 1) - Math.atan2(14, 16)) < 1e-12);
  path.controlPoints[1] = null;
  assert.equal(isHeadingDerived(path, 1), false, "a Hermite neighbour makes the heading authored");
});

test("compass and math headings convert both ways", () => {
  assert.equal(toDisplayHeading(0, "compass"), 90);
  assert.equal(toDisplayHeading(Math.PI / 2, "compass"), 0);
  assert.equal(toDisplayHeading(-Math.PI / 2, "compass"), 180);
  for (const degrees of [0, 45, 90, 179, 270, 359]) {
    assert.ok(Math.abs(toDisplayHeading(fromDisplayHeading(degrees, "compass"), "compass") - degrees) < 1e-9);
  }
  assert.ok(Math.abs(toDisplayHeading(fromDisplayHeading(-30), "math") + 30) < 1e-9);
});

test("undo snapshots ignore view preferences", () => {
  const document = makeDocument();
  const snapshot = historySnapshot(document);
  document.alliance = "blue";
  assert.equal(historySnapshot(document), snapshot, "changing the alliance view does not create an undo step");
  document.paths[0].waypoints[1].x = 70;
  const restored = restoreSnapshot(snapshot, document);
  assert.equal(restored.paths[0].waypoints[1].x, 68);
  assert.equal(restored.alliance, "blue", "restoring keeps the current view");
});

test("the export sets sample spacing, splits corners and records its format", () => {
  const code = cppExport(corneredRoute(), "route", "corner");
  assert.match(code, /config\.sampleDistance = 0\.3500;/);
  assert.match(code, /Export format 3/);
  assert.match(code, /routeSection2Trajectory/);
  assert.match(code, /turn in place \d+\.\d° to heading/);
  assert.match(cppExport(corneredRoute(), "route", "gps"), /config\.sampleDistance = 0\.008890;/);
});
