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

export function cornerToGps(point) {
  return {
    ...point,
    x: (point.x - FIELD_SIZE / 2) * INCH_TO_METRE,
    y: (point.y - FIELD_SIZE / 2) * INCH_TO_METRE,
    heading: wrapRadians(Math.PI / 2 - point.heading),
    tangent: point.tangent * INCH_TO_METRE,
  };
}

export function estimateLength(points, subdivisions = 24) {
  let length = 0;
  for (let segment = 0; segment + 1 < points.length; segment += 1) {
    let previous = points[segment];
    for (let step = 1; step <= subdivisions; step += 1) {
      const current = quinticPoint(points[segment], points[segment + 1], step / subdivisions);
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

export function quinticPoint(start, end, t) {
  function axis(p0, velocity0, p1, velocity1) {
    const a3 = -10 * p0 - 6 * velocity0 + 10 * p1 - 4 * velocity1;
    const a4 = 15 * p0 + 8 * velocity0 - 15 * p1 + 7 * velocity1;
    const a5 = -6 * p0 - 3 * velocity0 + 6 * p1 - 3 * velocity1;
    return p0 + t * (velocity0 + t * t * (a3 + t * (a4 + t * a5)));
  }
  return {
    x: axis(start.x, Math.cos(start.heading) * start.tangent, end.x, Math.cos(end.heading) * end.tangent),
    y: axis(start.y, Math.sin(start.heading) * start.tangent, end.y, Math.sin(end.heading) * end.tangent),
  };
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
    showZones: true,
    robot: { ...DEFAULT_ROBOT },
    paths: [{
      id: crypto.randomUUID(), name: "Primary route", color: "#171715", reversed: false,
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
  }
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
    const gps = frame === "gps";
    const points = gps ? path.waypoints.map(cornerToGps) : path.waypoints;
    const unit = frame === "gps" ? "metres / official GPS centre frame" : "inches / bottom-left corner frame";
    const entries = points.map((point) => `    {{${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.heading.toFixed(6)}}, ${point.tangent.toFixed(4)}}`).join(",\n");
    const suffix = document.paths.length === 1 ? "" : `${index + 1}`;
    const gpsSuffix = gps ? "Gps" : "";
    const distanceScale = gps ? INCH_TO_METRE : 1;
    const distance = (value) => (value * distanceScale).toFixed(4);
    const conversion = gps ? `\nconst std::vector<vantage::Waypoint> ${safeName}${suffix} = [] {\n  auto waypoints = ${safeName}${suffix}Gps;\n  for (auto& waypoint : waypoints) {\n    waypoint.pose = vantage::vexGpsToCorner(\n        waypoint.pose, {3.6576, 3.6576});\n  }\n  return waypoints;\n}();\n` : "\n";
    return `// ${path.name} — ${unit}\nconst std::vector<vantage::Waypoint> ${safeName}${suffix}${gpsSuffix} = {\n${entries}\n};\n${conversion}\nconst vantage::TrajectoryConfig ${safeName}${suffix}Config = [] {\n  vantage::TrajectoryConfig config;\n  config.trackWidth = ${distance(document.robot.trackWidth)};\n  config.maxVelocity = ${distance(document.robot.maxVelocity)};\n  config.maxAcceleration = ${distance(document.robot.maxAcceleration)};\n  config.maxDeceleration = ${distance(document.robot.maxDeceleration)};\n  config.maxCentripetalAcceleration = ${distance(document.robot.maxCentripetalAcceleration)};\n  config.maxWheelVelocity = ${distance(document.robot.maxWheelVelocity)};\n  config.startVelocity = ${distance(document.robot.startVelocity)};\n  config.endVelocity = ${distance(document.robot.endVelocity)};\n  config.reversed = ${path.reversed ? "true" : "false"};\n  return config;\n}();\nconst auto ${safeName}${suffix}Trajectory = vantage::generateTrajectory(${safeName}${suffix}, ${safeName}${suffix}Config);`;
  });
  return `// Generated by VantagePath Studio\n#include <vantage/vantage.hpp>\n#include <vector>\n\n${blocks.join("\n\n")}`;
}
