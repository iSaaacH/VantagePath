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
});

export function clamp(value, minimum = 0, maximum = FIELD_SIZE) {
  return Math.min(maximum, Math.max(minimum, value));
}

export function wrapRadians(angle) {
  while (angle > Math.PI) angle -= Math.PI * 2;
  while (angle <= -Math.PI) angle += Math.PI * 2;
  return angle;
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

export function robotStartPose(document) {
  // P1 on the first route is the single source of truth for initial placement.
  // robotStart remains in the document for backwards compatibility with older
  // consumers, but it must never drift away from the first route point.
  const source = document.paths[0]?.waypoints[0] ?? document.robotStart ?? { x:18, y:18, heading:0 };
  return { x:source.x, y:source.y, heading:source.heading };
}

// Apply pointer movement relative to the grab position so clicking the edge of
// a marker doesn't jump its centre to the cursor. Fine mode moves at 1/5 speed.
export function dragPosition(origin, pointerStart, pointer, step = 0.25, fine = false) {
  const gain = fine ? 0.2 : 1;
  const quantize = value => clamp(step > 0 ? Math.round(value / step) * step : value);
  return { x:quantize(origin.x + (pointer.x-pointerStart.x)*gain), y:quantize(origin.y + (pointer.y-pointerStart.y)*gain) };
}

export function makeDocument() {
  return {
    version: 1,
    type: "VantagePathDocument",
    title: "Competition auto",
    game: "V5RC Override 2026-27",
    coordinateFrame: "corner-bottom-left",
    units: "inches",
    field: { width: FIELD_SIZE, height: FIELD_SIZE },
    alliance: "red",
    snap: true,
    snapStep: 0.25,
    robotStart: { x:18, y:18, heading:0.18 },
    showZones: true,
    robot: { ...DEFAULT_ROBOT },
    paths: [{
      id: crypto.randomUUID(), name: "Primary route", color: "#171715", segmentReversed: [false, false], controlPoints: [[], []],
      waypoints: [
        { id: crypto.randomUUID(), x: 18, y: 18, heading: 0.18, tangent: 38 },
        { id: crypto.randomUUID(), x: 68, y: 50, heading: 0.82, tangent: 42 },
        { id: crypto.randomUUID(), x: 116, y: 112, heading: 1.35, tangent: 34 },
      ],
    }],
  };
}

export function validateDocument(value) {
  if (!value || value.type !== "VantagePathDocument" || value.version !== 1 || !Array.isArray(value.paths)) throw new Error("This is not a supported VantagePath file.");
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
  value.robot.startVelocity = Math.max(0, Math.min(value.robot.startVelocity, value.robot.maxVelocity));
  value.robot.endVelocity = Math.max(0, Math.min(value.robot.endVelocity, value.robot.maxVelocity));
  return value;
}

export function cppExport(document, variableName, frame = "corner") {
  const safeName = /^[A-Za-z_][A-Za-z0-9_]*$/.test(variableName) ? variableName : "generatedPath";
  const blocks = document.paths.map((path, index) => {
    const runs = directionRuns(path);
    const routeSuffix = document.paths.length === 1 ? "" : `${index + 1}`;
    const trajectoryNames = [];
    const runBlocks = runs.map((run, runIndex) => {
      const gps = frame === "gps";
      // The C++ generator treats each waypoint heading as a forward tangent and
      // only flips facing/velocity for config.reversed. To make a reversed run
      // back out along the intended curve (instead of demanding a 180° spin at
      // the direction change), rotate its authored nose headings by π here; the
      // config.reversed flag below then restores the true facing.
      const oriented = run.reversed
        ? run.waypoints.map((point) => ({ ...point, heading: wrapRadians(point.heading + Math.PI) }))
        : run.waypoints;
      const points = gps ? oriented.map(cornerToGps) : oriented;
      const unit = gps ? "metres / official GPS centre frame" : "inches / bottom-left corner frame";
      const runSuffix = runs.length === 1 ? "" : `Segment${runIndex + 1}`;
      const name = `${safeName}${routeSuffix}${runSuffix}`;
      trajectoryNames.push(`${name}Trajectory`);
      const entries = points.map((point, pointIndex) => {
        const controls = run.controlPoints[pointIndex];
        const extra = Array.isArray(controls) ? `, true, {${controls.map((control) => {
          const p = gps ? cornerToGps({ ...control, heading:0, tangent:0 }) : { ...control, heading:0 };
          return `{${p.x.toFixed(4)}, ${p.y.toFixed(4)}, ${p.heading.toFixed(6)}}`;
        }).join(", ")}}` : "";
        return `    {{${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.heading.toFixed(6)}}, ${point.tangent.toFixed(4)}${extra}}`;
      }).join(",\n");
      const distanceScale = gps ? INCH_TO_METRE : 1;
      const distance = (value) => (value * distanceScale).toFixed(4);
      const conversion = gps ? `\nconst std::vector<vantage::Waypoint> ${name} = [] {\n  auto waypoints = ${name}Gps;\n  for (auto& waypoint : waypoints) {\n    for (auto& control : waypoint.controlPoints) control = vantage::vexGpsToCorner(control, {3.6576, 3.6576});\n    waypoint.pose = vantage::vexGpsToCorner(\n        waypoint.pose, {3.6576, 3.6576});\n  }\n  return waypoints;\n}();\n` : "\n";
      return `// ${path.name}${runs.length > 1 ? ` · direction section ${runIndex + 1}` : ""} — ${unit}\nconst std::vector<vantage::Waypoint> ${name}${gps ? "Gps" : ""} = {\n${entries}\n};\n${conversion}\nconst vantage::TrajectoryConfig ${name}Config = [] {\n  vantage::TrajectoryConfig config;\n  config.trackWidth = ${distance(document.robot.trackWidth)};\n  config.maxVelocity = ${distance(document.robot.maxVelocity)};\n  config.maxAcceleration = ${distance(document.robot.maxAcceleration)};\n  config.maxDeceleration = ${distance(document.robot.maxDeceleration)};\n  config.maxCentripetalAcceleration = ${distance(document.robot.maxCentripetalAcceleration)};\n  config.maxWheelVelocity = ${distance(document.robot.maxWheelVelocity)};\n  config.startVelocity = ${distance(runIndex === 0 ? document.robot.startVelocity : 0)};\n  config.endVelocity = ${distance(runIndex === runs.length - 1 ? document.robot.endVelocity : 0)};\n  config.reversed = ${run.reversed ? "true" : "false"};\n  return config;\n}();\nconst auto ${name}Trajectory = vantage::generateTrajectory(${name}, ${name}Config);`;
    });
    const collection = runs.length > 1 ? `\n\n// Run these sections in order; direction changes require a stop.\nconst std::vector<vantage::Trajectory> ${safeName}${routeSuffix}Trajectories = { ${trajectoryNames.join(", ")} };` : "";
    return runBlocks.join("\n\n") + collection;
  });
  const start = robotStartPose(document);
  const startPose = frame === "gps" ? cornerToGps({ ...start, tangent:0 }) : start;
  const poseLiteral = `{${startPose.x.toFixed(4)}, ${startPose.y.toFixed(4)}, ${startPose.heading.toFixed(6)}}`;
  const startExport = frame === "gps"
    ? `const vantage::Pose2d ${safeName}RobotStartGps = ${poseLiteral};\nconst auto ${safeName}RobotStart = vantage::vexGpsToCorner(${safeName}RobotStartGps, {3.6576, 3.6576});`
    : `const vantage::Pose2d ${safeName}RobotStart = ${poseLiteral};`;
  return `// Generated by VantagePath Studio\n#include <vantage/vantage.hpp>\n#include <vector>\n\n// Initial robot pose. Use to initialize localization before running routes.\n${startExport}\n\n${blocks.join("\n\n")}`;
}
