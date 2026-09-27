// Route editing operations. Each takes a path (or document) and returns a new
// one; nothing here touches the DOM or the store.

import { anchorHeading, clamp, FIELD_SIZE, mirrorWaypoint, normalizeSegmentDirections, reverseWaypoints, segmentPoint, setControlCount, splitSegment } from "./model.js";
import { segmentTangent } from "./planner.js";

export const NEW_POINT_STEP_IN = 24;
export const INSERT_HIT_IN = 2.5;
const MIN_POINT_SPACING_IN = 0.5;
const PATH_HIT_SAMPLES = 100;

function uid() { return crypto.randomUUID(); }

function copyPath(path) {
  const next = structuredClone(path);
  normalizeSegmentDirections(next);
  return next;
}

function snapTo(value, step) { return step > 0 ? Math.round(value / step) * step : value; }

/** A tile ahead of the last point in the direction it faces, inside the field. */
export function pointAhead(path, robot) {
  const last = path.waypoints.at(-1);
  if (!last) return { x: FIELD_SIZE / 2, y: FIELD_SIZE / 2 };
  const heading = path.waypoints.length > 1 ? anchorHeading(path, path.waypoints.length - 1) : last.heading;
  const margin = Math.max(robot.length, robot.width) / 2;
  const inside = (value) => clamp(value, margin, FIELD_SIZE - margin);
  const ahead = { x: inside(last.x + Math.cos(heading) * NEW_POINT_STEP_IN), y: inside(last.y + Math.sin(heading) * NEW_POINT_STEP_IN) };
  if (Math.hypot(ahead.x - last.x, ahead.y - last.y) > 1) return ahead;
  return { x: last.x + (FIELD_SIZE / 2 - last.x) / 2, y: last.y + (FIELD_SIZE / 2 - last.y) / 2 };
}

/** The segment and curve parameter nearest `at`, when within `reach` inches. */
export function nearestOnPath(path, at, reach = INSERT_HIT_IN) {
  let best = null;
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    for (let step = 1; step < PATH_HIT_SAMPLES; step += 1) {
      const t = step / PATH_HIT_SAMPLES;
      const point = segmentPoint(path, index, t);
      const distance = Math.hypot(point.x - at.x, point.y - at.y);
      if (!best || distance < best.distance) best = { index, t, distance };
    }
  }
  return best && best.distance <= reach ? best : null;
}

/**
 * Adds a point after the last one. The new segment keeps the previous drive
 * direction and leaves along the previous segment's end tangent, so the join
 * is smooth rather than a corner. Returns { path, point } or { error }.
 */
export function appendPoint(path, at, snapStep = 0) {
  const next = copyPath(path);
  const previous = next.waypoints.at(-1);
  const x = clamp(snapTo(at.x, snapStep));
  const y = clamp(snapTo(at.y, snapStep));
  if (previous && Math.hypot(x - previous.x, y - previous.y) < MIN_POINT_SPACING_IN) return { error: "That's on top of the last point" };
  const chord = previous ? Math.hypot(x - previous.x, y - previous.y) : 0;
  const point = { id: uid(), x, y, heading: previous ? Math.atan2(y - previous.y, x - previous.x) : 0, tangent: previous ? Math.max(18, chord) : 30 };
  next.waypoints.push(point);
  if (previous) {
    const segments = next.segmentReversed.length;
    const controls = [];
    if (segments > 0) {
      const tangent = segmentTangent(next, segments - 1, 1);
      controls.push({ id: uid(), x: clamp(previous.x + Math.cos(tangent) * chord / 3), y: clamp(previous.y + Math.sin(tangent) * chord / 3) });
    }
    next.segmentReversed.push(segments > 0 ? next.segmentReversed[segments - 1] : false);
    next.controlPoints.push(controls);
    next.segmentSpeed.push(null);
  }
  return { path: next, point };
}

/** Inserts a point on the curve near `at`. Returns { path, point, index } or null. */
export function insertPointOnPath(path, at) {
  const normalized = copyPath(path);
  const hit = nearestOnPath(normalized, at);
  if (!hit) return null;
  const { path: split, point } = splitSegment(normalized, hit.index, hit.t);
  return { path: split, point, index: hit.index + 1 };
}

/** Removes a route point; an interior point's neighbours join with a straight segment. */
export function deletePoint(path, pointId) {
  const next = copyPath(path);
  const index = next.waypoints.findIndex((point) => point.id === pointId);
  if (index < 0) return { path, nextPointId: null };
  next.waypoints.splice(index, 1);
  // The segment that disappears: the first one for P1, else the one after the
  // point (or before it, for the last point). Markers on it are dropped.
  const removed = index === 0 ? 0 : Math.min(index, next.segmentReversed.length - 1);
  next.segmentReversed.splice(removed, 1); next.controlPoints.splice(removed, 1); next.segmentSpeed.splice(removed, 1);
  if (index > 0 && index < next.waypoints.length) next.controlPoints[index - 1] = [];
  next.markers = next.markers.filter((marker) => marker.segment !== removed).map((marker) => (marker.segment > removed ? { ...marker, segment: marker.segment - 1 } : marker));
  return { path: next, nextPointId: next.waypoints[Math.min(index, next.waypoints.length - 1)]?.id ?? null };
}

export function deleteControl(path, segment, controlId) {
  const next = copyPath(path);
  next.controlPoints[segment] = (next.controlPoints[segment] ?? []).filter((point) => point.id !== controlId);
  return next;
}

export function addControl(path, segment) {
  const next = copyPath(path);
  setControlCount(next, segment, (next.controlPoints[segment]?.length ?? 0) + 1);
  return { path: next, control: next.controlPoints[segment]?.at(-1) ?? null };
}

export function withControlCount(path, segment, count) {
  const next = copyPath(path);
  setControlCount(next, segment, count);
  return next;
}

export function setSegmentDirection(path, segment, reversed) {
  const next = copyPath(path);
  if (segment >= 0 && segment < next.segmentReversed.length) next.segmentReversed[segment] = reversed;
  return next;
}

export function setAllDirections(path, reversed) {
  const next = copyPath(path);
  next.segmentReversed = next.segmentReversed.map(() => reversed);
  return next;
}

/** "bezier" starts an editable (straight) Bézier; "hermite" restores heading/tangent geometry. */
export function setSegmentGeometry(path, segment, mode) {
  const next = copyPath(path);
  next.controlPoints[segment] = mode === "hermite" ? null : [];
  return next;
}

export function mirrorPath(path, mode) {
  const next = copyPath(path);
  next.waypoints = next.waypoints.map((point) => mirrorWaypoint(point, mode));
  next.controlPoints = next.controlPoints.map((items) => items?.map((point) => {
    const mirrored = mirrorWaypoint({ ...point, heading: 0 }, mode);
    return { id: point.id, x: mirrored.x, y: mirrored.y };
  }) ?? null);
  return next;
}

export function reverseRoute(path) {
  const next = copyPath(path);
  next.waypoints = reverseWaypoints(next.waypoints);
  next.segmentReversed = [...next.segmentReversed].reverse();
  next.controlPoints = [...next.controlPoints].reverse().map((items) => (items ? [...items].reverse() : null));
  next.segmentSpeed = [...next.segmentSpeed].reverse();
  const last = next.segmentReversed.length - 1;
  next.markers = next.markers.map((marker) => ({ ...marker, segment: last - marker.segment, t: 1 - marker.t }));
  return next;
}

function withFreshIds(path) {
  return {
    ...path,
    id: uid(),
    markers: path.markers.map((marker) => ({ ...marker, id: uid() })),
    waypoints: path.waypoints.map((point) => ({ ...point, id: uid() })),
    controlPoints: path.controlPoints.map((items) => items?.map((point) => ({ ...point, id: uid() })) ?? null),
  };
}

export function duplicateRoute(path) {
  return { ...withFreshIds(copyPath(path)), name: `${path.name} copy` };
}

function otherAllianceName(name, viewAlliance) {
  if (/\(blue\)$/i.test(name)) return name.replace(/\(blue\)$/i, "(red)");
  if (/\(red\)$/i.test(name)) return name.replace(/\(red\)$/i, "(blue)");
  return `${name} (${viewAlliance === "blue" ? "red" : "blue"})`;
}

/** A new route rotated 180° about the field centre for the other alliance. */
export function allianceCopy(path, viewAlliance) {
  return { ...withFreshIds(mirrorPath(path, "alliance")), name: otherAllianceName(path.name, viewAlliance) };
}

/** A two-point route starting at the robot start pose. */
export function newRoute(start, number) {
  const offset = start.x > FIELD_SIZE - 36 ? -36 : 36;
  return {
    id: uid(), name: `Route ${number}`, color: "#171715", segmentReversed: [false], controlPoints: [[]], segmentSpeed: [null], markers: [], waitAfterMs: 0,
    waypoints: [
      { id: uid(), x: start.x, y: start.y, heading: start.heading, tangent: 30 },
      { id: uid(), x: clamp(start.x + offset), y: start.y, heading: start.heading, tangent: 30 },
    ],
  };
}

export { smoothJoin } from "./model.js";

export function setSegmentSpeed(path, segment, speed) {
  const next = copyPath(path);
  next.segmentSpeed[segment] = Number.isFinite(speed) && speed > 0 ? speed : null;
  return next;
}

/** Adds a named marker at curve parameter t on a segment. */
export function addMarker(path, segment, t, name) {
  const next = copyPath(path);
  const marker = { id: uid(), name: name || `Marker ${next.markers.length + 1}`, segment, t: Math.min(1, Math.max(0, t)) };
  next.markers = [...next.markers, marker];
  return { path: next, marker };
}

export function updateMarker(path, markerId, changes) {
  const next = copyPath(path);
  next.markers = next.markers.map((marker) => (marker.id === markerId ? { ...marker, ...changes } : marker));
  return copyPath(next);
}

export function deleteMarker(path, markerId) {
  const next = copyPath(path);
  next.markers = next.markers.filter((marker) => marker.id !== markerId);
  return next;
}

export function setWaitAfter(path, milliseconds) {
  const next = copyPath(path);
  next.waitAfterMs = Number.isFinite(milliseconds) && milliseconds > 0 ? Math.round(milliseconds) : 0;
  return next;
}

/** Moves a route one place earlier (-1) or later (+1) in the auton order. */
export function moveRoute(paths, pathId, delta) {
  const index = paths.findIndex((path) => path.id === pathId);
  const target = index + delta;
  if (index < 0 || target < 0 || target >= paths.length) return paths;
  const next = [...paths];
  [next[index], next[target]] = [next[target], next[index]];
  return next;
}
