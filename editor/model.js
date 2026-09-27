import { segmentNoseHeading, segmentTangent, wrapRadians } from "./planner.js";

export { wrapRadians };
export const FIELD_SIZE = 144;
export const INCH_TO_METRE = 0.0254;

export const DEFAULT_ROBOT = Object.freeze({
  length: 18,
  width: 18,
  trackWidth: 12,
  maxVelocity: 60,
  maxAcceleration: 80,
  maxDeceleration: 100,
  maxWheelVelocity: 72,
  maxCentripetalAcceleration: 80,
  startVelocity: 0,
  endVelocity: 0,
  // Matches the C++ default spacing that 4613R runs (inches). The library's own
  // 0.025 default is metres-scale and makes ~80 samples per inch.
  sampleDistance: 0.35,
});

// Document keys that are view preferences. They are saved with the file but
// kept out of undo history, so undo only ever reverts route edits.
export const VIEW_KEYS = Object.freeze(["alliance", "snap", "snapStep", "showZones"]);

export function historySnapshot(document) {
  const copy = { ...document };
  for (const key of VIEW_KEYS) delete copy[key];
  return JSON.stringify(copy);
}

export function restoreSnapshot(snapshot, currentDocument) {
  const restored = JSON.parse(snapshot);
  for (const key of VIEW_KEYS) restored[key] = currentDocument[key];
  return restored;
}

export function clamp(value, minimum = 0, maximum = FIELD_SIZE) {
  return Math.min(maximum, Math.max(minimum, value));
}

// Headings are stored as radians CCW from +X (the library's convention). Teams
// on LemLib-style chassis code think in compass degrees: 0 = +Y, clockwise.
export const HEADING_MODES = Object.freeze({
  math: { label: "0° right · 90° up (CCW)", short: "CCW from +X" },
  compass: { label: "0° up · 90° right (CW)", short: "CW from +Y" },
});

export function toDisplayHeading(radians, mode = "math") {
  const degrees = radians * 180 / Math.PI;
  if (mode !== "compass") return wrapRadians(radians) * 180 / Math.PI;
  const compass = (90 - degrees) % 360;
  return compass < 0 ? compass + 360 : compass;
}

export function fromDisplayHeading(degrees, mode = "math") {
  const math = mode === "compass" ? 90 - degrees : degrees;
  return wrapRadians(math * Math.PI / 180);
}

export function mirrorWaypoint(point, mode) {
  if (mode === "left-right") return { ...point, x: FIELD_SIZE - point.x, heading: wrapRadians(Math.PI - point.heading) };
  if (mode === "bottom-top") return { ...point, y: FIELD_SIZE - point.y, heading: wrapRadians(-point.heading) };
  // Override's same-alliance quadrants sit on opposite sides of the single
  // white x=y divider. Swapping axes stays on the selected alliance side.
  if (mode === "quadrant") return { ...point, x: point.y, y: point.x, heading: wrapRadians(Math.PI / 2 - point.heading) };
  if (mode === "alliance") return { ...point, x: FIELD_SIZE - point.x, y: FIELD_SIZE - point.y, heading: wrapRadians(point.heading + Math.PI) };
  throw new Error(`Unknown mirror mode: ${mode}`);
}

export function reverseWaypoints(points) {
  return [...points].reverse().map((point) => ({ ...point, heading: wrapRadians(point.heading + Math.PI) }));
}

export function normalizeSegmentDirections(path) {
  const count = Math.max(0, path.waypoints.length - 1);
  const legacyDirection = path.reversed === true;
  path.segmentReversed = Array.from({ length: count }, (_, index) =>
    typeof path.segmentReversed?.[index] === "boolean" ? path.segmentReversed[index] : legacyDirection
  );
  path.controlPoints = Array.from({ length: count }, (_, index) => path.controlPoints?.[index] ?? null);
  // Optional per-segment speed ceiling in in/s; null uses the robot's limit.
  path.segmentSpeed = Array.from({ length: count }, (_, index) => {
    const value = path.segmentSpeed?.[index];
    return Number.isFinite(value) && value > 0 ? value : null;
  });
  // Named event markers at curve parameter t on a segment.
  path.markers = (Array.isArray(path.markers) ? path.markers : [])
    .filter((marker) => marker && Number.isInteger(marker.segment) && marker.segment >= 0 && marker.segment < count && Number.isFinite(marker.t))
    .map((marker) => ({ id: marker.id || crypto.randomUUID(), name: String(marker.name ?? "Marker").slice(0, 40) || "Marker", segment: marker.segment, t: Math.min(1, Math.max(0, marker.t)) }));
  path.waitAfterMs = Number.isFinite(path.waitAfterMs) && path.waitAfterMs > 0 ? Math.round(path.waitAfterMs) : 0;
  delete path.reversed;
  return path.segmentReversed;
}

export function directionRuns(path) {
  normalizeSegmentDirections(path);
  if (path.waypoints.length < 2) return [];
  const runs = [];
  let start = 0;
  for (let segment = 1; segment <= path.segmentReversed.length; segment += 1) {
    if (segment === path.segmentReversed.length || path.segmentReversed[segment] !== path.segmentReversed[start]) {
      runs.push({ reversed:path.segmentReversed[start], waypoints:path.waypoints.slice(start, segment + 1), controlPoints:path.controlPoints.slice(start, segment) });
      start = segment;
    }
  }
  return runs;
}

export function cornerToGps(point) {
  return {
    ...point,
    x: (point.x - FIELD_SIZE / 2) * INCH_TO_METRE,
    y: (point.y - FIELD_SIZE / 2) * INCH_TO_METRE,
    heading: wrapRadians(Math.PI / 2 - point.heading),
    tangent: point.tangent * INCH_TO_METRE,
  };
}

// null keeps legacy Hermite geometry; [] explicitly means a straight Bézier line.
export function bezierPoint(points, t) {
  const work = points.map(({ x, y }) => ({ x, y }));
  for (let size = work.length - 1; size > 0; size -= 1) {
    for (let i = 0; i < size; i += 1) {
      work[i].x += (work[i + 1].x - work[i].x) * t;
      work[i].y += (work[i + 1].y - work[i].y) * t;
    }
  }
  return work[0];
}

export function segmentPoint(path, index, t) {
  const start = path.waypoints[index], end = path.waypoints[index + 1];
  const controls = path.controlPoints?.[index];
  return Array.isArray(controls) ? bezierPoint([start, ...controls, end], t)
    : quinticPoint(start, end, t, path.segmentReversed?.[index] === true);
}

export function setControlCount(path, index, count) {
  if (!Number.isInteger(count) || count < 0 || !path.waypoints[index + 1]) return;
  normalizeSegmentDirections(path);
  const controls = path.controlPoints[index] ?? [];
  const start = controls.at(-1) ?? path.waypoints[index];
  const end = path.waypoints[index + 1];
  const extra = count - controls.length;
  for (let i = 1; i <= extra; i += 1) {
    controls.push({ id:crypto.randomUUID(), x:start.x + (end.x-start.x)*i/(extra+1), y:start.y + (end.y-start.y)*i/(extra+1) });
  }
  controls.length = count;
  path.controlPoints[index] = controls;
}

export function estimateLength(points, subdivisions = 24, segmentReversed = [], controlPoints = []) {
  let length = 0;
  for (let segment = 0; segment + 1 < points.length; segment += 1) {
    let previous = points[segment];
    for (let step = 1; step <= subdivisions; step += 1) {
      const current = segmentPoint({ waypoints:points, segmentReversed, controlPoints }, segment, step / subdivisions);
      length += Math.hypot(current.x - previous.x, current.y - previous.y);
      previous = current;
    }
  }
  return length;
}

export function motionProfile(length, robot) {
  const distance = Math.max(0, Number(length) || 0);
  const acceleration = Math.max(0.001, robot.maxAcceleration);
  const deceleration = Math.max(0.001, robot.maxDeceleration);
  const maximum = Math.max(0.001, Math.min(robot.maxVelocity, robot.maxWheelVelocity));
  const start = Math.min(maximum, Math.max(0, robot.startVelocity));
  const end = Math.min(maximum, Math.max(0, robot.endVelocity));
  let peak = maximum;
  const accelerationDistance = Math.max(0, (peak * peak - start * start) / (2 * acceleration));
  const decelerationDistance = Math.max(0, (peak * peak - end * end) / (2 * deceleration));
  if (accelerationDistance + decelerationDistance > distance) {
    peak = Math.sqrt(Math.max(0, (2 * acceleration * deceleration * distance + deceleration * start * start + acceleration * end * end) / (acceleration + deceleration)));
  }
  peak = Math.max(peak, start, end);
  const accelerateFor = Math.max(0, (peak - start) / acceleration);
  const accelerateDistance = (start + peak) * accelerateFor / 2;
  const decelerateFor = Math.max(0, (peak - end) / deceleration);
  const decelerateDistance = (end + peak) * decelerateFor / 2;
  const cruiseDistance = Math.max(0, distance - accelerateDistance - decelerateDistance);
  const cruiseFor = cruiseDistance / peak;
  const duration = accelerateFor + cruiseFor + decelerateFor;
  return { length: distance, start, end, peak, acceleration, deceleration, accelerateFor, accelerateDistance, cruiseFor, cruiseDistance, decelerateFor, decelerateDistance, duration };
}

export function profileDistance(profile, time) {
  const t = Math.min(profile.duration, Math.max(0, Number(time) || 0));
  if (t <= profile.accelerateFor) return Math.min(profile.length, profile.start * t + profile.acceleration * t * t / 2);
  const afterAcceleration = t - profile.accelerateFor;
  if (afterAcceleration <= profile.cruiseFor) return Math.min(profile.length, profile.accelerateDistance + profile.peak * afterAcceleration);
  const brakingTime = afterAcceleration - profile.cruiseFor;
  return Math.min(profile.length, profile.accelerateDistance + profile.cruiseDistance + profile.peak * brakingTime - profile.deceleration * brakingTime * brakingTime / 2);
}

export function profileTimeAtDistance(profile, distance) {
  if (!(profile.duration > 0) || !(profile.length > 0)) return 0;
  const target = Math.min(profile.length, Math.max(0, Number(distance) || 0));
  let lower = 0;
  let upper = profile.duration;
  // The profile is monotonic, so a small binary search is simpler and less
  // error-prone than maintaining three separate inverse equations.
  for (let iteration = 0; iteration < 36; iteration += 1) {
    const middle = (lower + upper) / 2;
    if (profileDistance(profile, middle) < target) lower = middle;
    else upper = middle;
  }
  return (lower + upper) / 2;
}

export function nearestPathDistance(samples, point) {
  if (!samples?.length) return 0;
  let nearestDistance = samples[0].distance ?? 0;
  let nearestSquared = (samples[0].x - point.x) ** 2 + (samples[0].y - point.y) ** 2;
  for (let index = 1; index < samples.length; index += 1) {
    const start = samples[index - 1];
    const end = samples[index];
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const lengthSquared = dx * dx + dy * dy;
    const ratio = lengthSquared > 0
      ? Math.min(1, Math.max(0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared))
      : 0;
    const x = start.x + dx * ratio;
    const y = start.y + dy * ratio;
    const squared = (x - point.x) ** 2 + (y - point.y) ** 2;
    if (squared < nearestSquared) {
      nearestSquared = squared;
      nearestDistance = (start.distance ?? 0) + ((end.distance ?? start.distance ?? 0) - (start.distance ?? 0)) * ratio;
    }
  }
  return nearestDistance;
}

export function quinticPoint(start, end, t, reversed = false) {
  // Waypoint heading is the robot's nose direction. A reversed segment drives
  // that same nose backwards, so the geometric spline tangent points 180° from
  // the heading. Flipping both endpoint tangents makes the curve back out of a
  // waypoint (a cusp) instead of looping the robot around to face its travel.
  const flip = reversed ? -1 : 1;
  function axis(p0, velocity0, p1, velocity1) {
    const a3 = -10 * p0 - 6 * velocity0 + 10 * p1 - 4 * velocity1;
    const a4 = 15 * p0 + 8 * velocity0 - 15 * p1 + 7 * velocity1;
    const a5 = -6 * p0 - 3 * velocity0 + 6 * p1 - 3 * velocity1;
    return p0 + t * (velocity0 + t * t * (a3 + t * (a4 + t * a5)));
  }
  return {
    x: axis(start.x, flip * Math.cos(start.heading) * start.tangent, end.x, flip * Math.cos(end.heading) * end.tangent),
    y: axis(start.y, flip * Math.sin(start.heading) * start.tangent, end.y, flip * Math.sin(end.heading) * end.tangent),
  };
}

/**
 * True when no geometry uses this anchor's stored heading: every segment that
 * touches it is a Bézier curve, whose tangent comes from its control points.
 */
export function isHeadingDerived(path, index) {
  const before = index > 0 ? path.controlPoints?.[index - 1] : undefined;
  const after = index < path.waypoints.length - 1 ? path.controlPoints?.[index] : undefined;
  if (before === null || after === null) return false;
  return Array.isArray(before) || Array.isArray(after);
}

/** The nose heading the robot actually has at an anchor. */
export function anchorHeading(path, index) {
  if (!isHeadingDerived(path, index)) return path.waypoints[index].heading;
  return index < path.waypoints.length - 1
    ? segmentNoseHeading(path, index, 0)
    : segmentNoseHeading(path, index - 1, 1);
}

/** Copy of a path whose Bézier-only anchors store the heading they really have. */
export function withDerivedHeadings(path) {
  return { ...path, waypoints: path.waypoints.map((point, index) => (
    isHeadingDerived(path, index) ? { ...point, heading: anchorHeading(path, index) } : point
  )) };
}

export function robotStartPose(document) {
  // P1 on the first route is the single source of truth for initial placement.
  // robotStart remains in the document for backwards compatibility with older
  // consumers, but it must never drift away from the first route point.
  const path = document.paths[0];
  const source = path?.waypoints[0] ?? document.robotStart ?? { x:18, y:18, heading:0 };
  const heading = path?.waypoints.length > 1 ? anchorHeading(path, 0) : source.heading;
  return { x:source.x, y:source.y, heading };
}

function deCasteljauSplit(polygon, t) {
  const left = [polygon[0]];
  const right = [polygon.at(-1)];
  let level = polygon.map(({ x, y }) => ({ x, y }));
  while (level.length > 1) {
    level = level.slice(1).map((point, i) => ({ x: level[i].x + (point.x - level[i].x) * t, y: level[i].y + (point.y - level[i].y) * t }));
    left.push(level[0]);
    right.unshift(level.at(-1));
  }
  return { left, right };
}

/**
 * Inserts a route point on segment `index` at curve parameter t, keeping the
 * curve's shape. Returns the new path and the inserted point.
 */
export function splitSegment(path, index, t) {
  const next = structuredClone(path);
  normalizeSegmentDirections(next);
  const start = next.waypoints[index];
  const end = next.waypoints[index + 1];
  const controls = next.controlPoints[index];
  const position = segmentPoint(next, index, t);
  const point = { id: crypto.randomUUID(), x: position.x, y: position.y, heading: 0, tangent: 30 };
  let leftControls = null;
  let rightControls = null;
  if (Array.isArray(controls)) {
    const { left, right } = deCasteljauSplit([start, ...controls, end], t);
    const withIds = (items) => items.map(({ x, y }) => ({ id: crypto.randomUUID(), x, y }));
    leftControls = withIds(left.slice(1, -1));
    rightControls = withIds(right.slice(1, -1));
  } else {
    // Legacy Hermite: the anchor carries the curve's own direction and speed.
    const before = segmentPoint(next, index, Math.max(0, t - 1e-4));
    const after = segmentPoint(next, index, Math.min(1, t + 1e-4));
    const tangent = Math.atan2(after.y - before.y, after.x - before.x);
    point.heading = wrapRadians(next.segmentReversed[index] ? tangent + Math.PI : tangent);
    point.tangent = Math.max(1, Math.hypot(after.x - before.x, after.y - before.y) / 2e-4 / 2);
  }
  next.waypoints.splice(index + 1, 0, point);
  next.segmentReversed.splice(index + 1, 0, next.segmentReversed[index]);
  next.segmentSpeed.splice(index + 1, 0, next.segmentSpeed[index]);
  next.controlPoints.splice(index, 1, leftControls, rightControls);
  next.markers = next.markers.map((marker) => {
    if (marker.segment < index) return marker;
    if (marker.segment > index) return { ...marker, segment: marker.segment + 1 };
    return marker.t < t ? { ...marker, t: marker.t / t } : { ...marker, segment: index + 1, t: (marker.t - t) / (1 - t) };
  });
  return { path: withDerivedHeadings(next), point };
}

function unit(angle) { return { x: Math.cos(angle), y: Math.sin(angle) }; }

// Aims one end of a segment's geometry along `direction` (a geometric tangent).
function aimSegmentEnd(path, segment, end, direction) {
  const anchor = path.waypoints[end === 0 ? segment : segment + 1];
  const far = path.waypoints[end === 0 ? segment + 1 : segment];
  const controls = path.controlPoints[segment];
  const chord = Math.hypot(far.x - anchor.x, far.y - anchor.y);
  const sign = end === 0 ? 1 : -1;
  const aim = unit(direction);
  if (controls === null) {
    anchor.heading = wrapRadians(path.segmentReversed[segment] ? direction + Math.PI : direction);
    return;
  }
  if (!controls.length) {
    // A straight line needs two controls to bend at one end only.
    const near = { id: crypto.randomUUID(), x: anchor.x + sign * aim.x * chord / 3, y: anchor.y + sign * aim.y * chord / 3 };
    const away = { id: crypto.randomUUID(), x: far.x + (anchor.x - far.x) / 3, y: far.y + (anchor.y - far.y) / 3 };
    path.controlPoints[segment] = end === 0 ? [near, away] : [away, near];
    return;
  }
  const nearIndex = end === 0 ? 0 : controls.length - 1;
  const control = controls[nearIndex];
  const reach = Math.hypot(control.x - anchor.x, control.y - anchor.y) || chord / 3;
  controls[nearIndex] = { ...control, x: clamp(anchor.x + sign * aim.x * reach), y: clamp(anchor.y + sign * aim.y * reach) };
}

/**
 * Makes the join at route point `index` tangent-continuous, so the robot drives
 * through it instead of stopping to turn. Aims both sides along the average
 * direction. Returns a new path.
 */
export function smoothJoin(path, index) {
  if (index <= 0 || index >= path.waypoints.length - 1) return path;
  const next = structuredClone(path);
  normalizeSegmentDirections(next);
  const directionChange = next.segmentReversed[index - 1] !== next.segmentReversed[index];
  const incoming = segmentTangent(next, index - 1, 1) + (directionChange ? Math.PI : 0);
  const outgoing = segmentTangent(next, index, 0);
  const sum = { x: Math.cos(incoming) + Math.cos(outgoing), y: Math.sin(incoming) + Math.sin(outgoing) };
  const target = Math.hypot(sum.x, sum.y) > 1e-6 ? Math.atan2(sum.y, sum.x) : outgoing;
  aimSegmentEnd(next, index - 1, 1, target + (directionChange ? Math.PI : 0));
  aimSegmentEnd(next, index, 0, target);
  return withDerivedHeadings(next);
}

// Apply pointer movement relative to the grab position so clicking the edge of
// a marker doesn't jump its centre to the cursor. Fine mode moves at 1/5 speed.
export function dragPosition(origin, pointerStart, pointer, step = 0.25, fine = false) {
  const gain = fine ? 0.2 : 1;
  const quantize = value => clamp(step > 0 ? Math.round(value / step) * step : value);
  return { x:quantize(origin.x + (pointer.x-pointerStart.x)*gain), y:quantize(origin.y + (pointer.y-pointerStart.y)*gain) };
}

export function makeDocument() {
  const document = {
    version: DOCUMENT_VERSION,
    type: "VantagePathDocument",
    title: "Competition auto",
    game: "V5RC Override 2026-27",
    coordinateFrame: "corner-bottom-left",
    units: "inches",
    field: { width: FIELD_SIZE, height: FIELD_SIZE },
    alliance: "red",
    snap: true,
    snapStep: 0.25,
    robotStart: { x:18, y:18, heading:0 },
    showZones: true,
    robot: { ...DEFAULT_ROBOT },
    paths: [{
      id: crypto.randomUUID(), name: "Primary route", color: "#171715", segmentReversed: [false, false], segmentSpeed: [null, null], markers: [], waitAfterMs: 0,
      // Tangent-continuous at P2, so the robot drives through it.
      controlPoints: [
        [{ id: crypto.randomUUID(), x: 44, y: 18 }, { id: crypto.randomUUID(), x: 52, y: 36 }],
        [{ id: crypto.randomUUID(), x: 84, y: 64 }, { id: crypto.randomUUID(), x: 112, y: 88 }],
      ],
      waypoints: [
        { id: crypto.randomUUID(), x: 18, y: 18, heading: 0.18, tangent: 38 },
        { id: crypto.randomUUID(), x: 68, y: 50, heading: 0.82, tangent: 42 },
        { id: crypto.randomUUID(), x: 116, y: 112, heading: 1.35, tangent: 34 },
      ],
    }],
  };
  document.paths = document.paths.map(withDerivedHeadings);
  return document;
}

// Format 2 adds per-segment speed limits, event markers and route waits.
// Format 1 files open unchanged: the new fields default to "none".
export const DOCUMENT_VERSION = 2;
const ALLIANCES = ["red", "blue", "skills"];

/**
 * Checks and normalises a document from storage or a file. Returns a new,
 * current-format document; the input is not modified.
 */
export function validateDocument(input) {
  if (!input || input.type !== "VantagePathDocument" || !Number.isInteger(input.version) || input.version < 1 || !Array.isArray(input.paths)) throw new Error("This is not a supported VantagePath file.");
  if (input.version > DOCUMENT_VERSION) throw new Error(`This file was made by a newer VantagePath Studio (format ${input.version}). Update Studio to open it.`);
  const value = structuredClone(input);
  value.version = DOCUMENT_VERSION;
  value.title = typeof value.title === "string" && value.title.trim() ? value.title.trim().slice(0, 120) : "Untitled route";
  value.alliance = ALLIANCES.includes(value.alliance) ? value.alliance : "red";
  value.snap = value.snap !== false;
  value.showZones = value.showZones !== false;
  for (const path of value.paths) {
    if (!path.id || !Array.isArray(path.waypoints)) throw new Error("A path is missing its waypoint data.");
    for (const point of path.waypoints) {
      for (const key of ["x", "y", "heading", "tangent"]) if (!Number.isFinite(point[key])) throw new Error(`Waypoint ${key} must be a number.`);
      point.x = clamp(point.x); point.y = clamp(point.y); point.tangent = Math.max(1, point.tangent);
    }
    if (path.controlPoints !== undefined && !Array.isArray(path.controlPoints)) throw new Error("Invalid control point data.");
    normalizeSegmentDirections(path);
    for (const controls of path.controlPoints) {
      if (controls === null) continue;
      if (!Array.isArray(controls)) throw new Error("Invalid control point list.");
      for (const point of controls) {
        if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) throw new Error("Control coordinates must be numbers.");
        point.x = clamp(point.x); point.y = clamp(point.y);
        point.id ||= crypto.randomUUID();
      }
    }
  }
  value.robotStart = robotStartPose(value);
  for (const key of ["x", "y", "heading"]) {
    if (!Number.isFinite(value.robotStart[key])) throw new Error("Robot start coordinates and heading must be numbers.");
  }
  value.robotStart.x = clamp(value.robotStart.x); value.robotStart.y = clamp(value.robotStart.y);
  value.robotStart.heading = wrapRadians(value.robotStart.heading);
  if (![0.25, 1, 6].includes(value.snapStep)) value.snapStep = 0.25;
  value.robot = { ...DEFAULT_ROBOT, ...(value.robot ?? {}) };
  for (const key of Object.keys(DEFAULT_ROBOT)) {
    if (!Number.isFinite(value.robot[key])) value.robot[key] = DEFAULT_ROBOT[key];
  }
  value.robot.length = clamp(value.robot.length, 1, 72);
  value.robot.width = clamp(value.robot.width, 1, 72);
  value.robot.trackWidth = clamp(value.robot.trackWidth, 0.1, 72);
  for (const key of ["maxVelocity", "maxAcceleration", "maxDeceleration", "maxWheelVelocity", "maxCentripetalAcceleration"]) value.robot[key] = Math.max(0.1, value.robot[key]);
  value.robot.sampleDistance = clamp(value.robot.sampleDistance, 0.05, 2);
  value.robot.startVelocity = Math.max(0, Math.min(value.robot.startVelocity, value.robot.maxVelocity));
  value.robot.endVelocity = Math.max(0, Math.min(value.robot.endVelocity, value.robot.maxVelocity));
  return value;
}
