// C++ export. Two targets:
//   "library": waypoint vectors, a TrajectoryConfig and a generated
//              vantage::Trajectory per section, plus timeouts and markers.
//   "chassis": waypoint vectors and a run function for 4613R's
//              drive::VantageChassis (followPath / turnToHeading / waits).
// A route becomes one section per stretch the robot drives without stopping
// to turn, so the C++ generator never sees a corner it would reject.

import { cornerToGps, INCH_TO_METRE, isHeadingDerived, normalizeSegmentDirections, robotStartPose, segmentPoint, toDisplayHeading, wrapRadians } from "./model.js";
import { planRoute, segmentNoseHeading, segmentTangent } from "./planner.js";

export const EXPORT_FORMAT_VERSION = 3;
export const REQUIRED_LIBRARY = "VantagePath with Bézier waypoints (newer than v0.2.0)";
export const EXPORT_TARGETS = Object.freeze({
  library: "VantagePath library · trajectories",
  chassis: "4613R VantageChassis · run function",
});

// Timeouts leave room for the robot running slower than planned.
const TIMEOUT_SCALE = 1.5;
const TIMEOUT_PAD_S = 0.25;
const TIMEOUT_STEP_MS = 50;

export function suggestedTimeoutMs(seconds) {
  return Math.ceil((seconds * TIMEOUT_SCALE + TIMEOUT_PAD_S) * 1000 / TIMEOUT_STEP_MS) * TIMEOUT_STEP_MS;
}

function compass(radians) { return toDisplayHeading(radians, "compass"); }
function formatCompass(radians) { return `${compass(radians).toFixed(1)}° compass`; }

function identifier(value) {
  const words = String(value).replace(/[^A-Za-z0-9]+/g, " ").trim().split(/\s+/).filter(Boolean);
  const camel = words.map((word, i) => (i ? word[0].toUpperCase() + word.slice(1) : word[0].toLowerCase() + word.slice(1))).join("");
  return /^[A-Za-z_]/.test(camel) ? camel : `m${camel}`;
}

function cString(value) {
  return `"${String(value).replace(/\\/g, "\\\\").replace(/"/g, '\\"')}"`;
}

// Geometry heading the C++ generator expects for one anchor inside a section.
// Hermite segments read it; Bézier segments ignore it, so write their real
// tangent to keep the literal truthful for anyone reading the code.
function exportTheta(path, globalIndex, section) {
  const point = path.waypoints[globalIndex];
  if (!isHeadingDerived(path, globalIndex)) return section.reversed ? wrapRadians(point.heading + Math.PI) : point.heading;
  return globalIndex > section.lastSegment ? segmentTangent(path, globalIndex - 1, 1) : segmentTangent(path, globalIndex, 0);
}

/** Markers on one planned drive step, as percent of that section's length. */
export function sectionMarkers(path, drive) {
  const { firstSegment, lastSegment } = drive.section;
  return path.markers.filter((marker) => marker.segment >= firstSegment && marker.segment <= lastSegment).map((marker) => {
    const at = segmentPoint(path, marker.segment, marker.t);
    let best = drive.states[0];
    let bestDistance = Infinity;
    for (const state of drive.states) {
      if (state.segmentIndex !== marker.segment) continue;
      const distance = Math.hypot(state.x - at.x, state.y - at.y);
      if (distance < bestDistance) { best = state; bestDistance = distance; }
    }
    return { ...marker, percent: drive.length > 0 ? best.distance / drive.length * 100 : 0 };
  }).sort((a, b) => a.percent - b.percent);
}

/** Everything the exporters need about one route, planned like the robot runs it. */
function routeExport(document, original, routeIndex, routeCount, safeName) {
  const path = structuredClone(original);
  normalizeSegmentDirections(path);
  const plan = planRoute(path, document.robot);
  if (plan.error) return { path, plan, error: plan.error, sections: [] };
  const drives = plan.steps.filter((step) => step.type === "drive");
  const routeSuffix = routeCount === 1 ? "" : `${routeIndex + 1}`;
  const sections = drives.map((drive, index) => {
    const turn = plan.steps[plan.steps.indexOf(drive) + 1];
    return {
      drive,
      section: drive.section,
      name: `${safeName}${routeSuffix}${drives.length === 1 ? "" : `Section${index + 1}`}`,
      markers: sectionMarkers(path, drive),
      turn: turn?.type === "turn" ? { ...turn, heading: segmentNoseHeading(path, drive.section.lastSegment + 1, 0) } : null,
    };
  });
  return { path, plan, error: null, sections, routeName: `${safeName}${routeSuffix}` };
}

function waypointEntries(path, section, gps) {
  const lastIndex = section.lastSegment + 1;
  const toFrame = (point) => (gps ? cornerToGps({ tangent: 0, ...point }) : point);
  const entries = [];
  for (let globalIndex = section.firstSegment; globalIndex <= lastIndex; globalIndex += 1) {
    const point = path.waypoints[globalIndex];
    const pose = toFrame({ ...point, heading: exportTheta(path, globalIndex, section) });
    const controls = globalIndex < lastIndex ? path.controlPoints[globalIndex] : undefined;
    const extra = Array.isArray(controls) ? `, true, {${controls.map((control) => {
      const p = toFrame({ x: control.x, y: control.y, heading: Math.PI / 2 });
      return `{${p.x.toFixed(4)}, ${p.y.toFixed(4)}, 0.000000}`;
    }).join(", ")}}` : "";
    const tangent = point.tangent * (gps ? INCH_TO_METRE : 1);
    entries.push(`    {{${pose.x.toFixed(4)}, ${pose.y.toFixed(4)}, ${pose.heading.toFixed(6)}}, ${tangent.toFixed(4)}${extra}}`);
  }
  return entries.join(",\n");
}

function gpsConversion(name) {
  return `\nconst std::vector<vantage::Waypoint> ${name} = [] {\n  auto waypoints = ${name}Gps;\n  for (auto& waypoint : waypoints) {\n    for (auto& control : waypoint.controlPoints) control = vantage::vexGpsToCorner(control, {3.6576, 3.6576});\n    waypoint.pose = vantage::vexGpsToCorner(\n        waypoint.pose, {3.6576, 3.6576});\n  }\n  return waypoints;\n}();\n`;
}

function speedScaleLambda(spec) {
  if (!spec) return "";
  const branches = spec.boundaries.map((boundary, i) => `if (percent < ${boundary.toFixed(9)}) return ${spec.scales[i].toFixed(9)};`).join(" ");
  return `\n  // Per-segment speed limits as a fraction of maxVelocity by arc-length percent.\n  config.pathSpeedScale = [](double percent) { ${branches} return ${spec.scales.at(-1).toFixed(9)}; };`;
}

function markerConstants(entry) {
  return entry.markers.map((marker) => `constexpr double ${entry.name}${identifier(marker.name).replace(/^./, (c) => c.toUpperCase())}Percent = ${marker.percent.toFixed(1)};  // marker "${marker.name.replace(/\*\//g, "")}"`).join("\n");
}

function librarySection(document, route, entry, index, gps) {
  const { path } = route;
  const scale = gps ? INCH_TO_METRE : 1;
  const distance = (value) => (value * scale).toFixed(4);
  const robot = document.robot;
  const count = route.sections.length;
  const section = entry.section;
  const unit = gps ? "metres / official GPS centre frame" : "inches / bottom-left corner frame";
  const label = `${path.name}${count > 1 ? ` · section ${index + 1} of ${count} (P${section.firstSegment + 1} → P${section.lastSegment + 2})` : ""}`;
  const config = [
    `  config.trackWidth = ${distance(robot.trackWidth)};`,
    `  config.maxVelocity = ${distance(robot.maxVelocity)};`,
    `  config.maxAcceleration = ${distance(robot.maxAcceleration)};`,
    `  config.maxDeceleration = ${distance(robot.maxDeceleration)};`,
    `  config.maxCentripetalAcceleration = ${distance(robot.maxCentripetalAcceleration)};`,
    `  config.maxWheelVelocity = ${distance(robot.maxWheelVelocity)};`,
    `  config.sampleDistance = ${(robot.sampleDistance * scale).toFixed(gps ? 6 : 4)};`,
    `  config.startVelocity = ${distance(index === 0 ? robot.startVelocity : 0)};`,
    `  config.endVelocity = ${distance(index === count - 1 ? robot.endVelocity : 0)};`,
    `  config.reversed = ${section.reversed ? "true" : "false"};`,
  ].join("\n");
  const timing = `// Planned ${entry.drive.duration.toFixed(2)} s over ${entry.drive.length.toFixed(1)} in.\nconstexpr int ${entry.name}TimeoutMs = ${suggestedTimeoutMs(entry.drive.duration)};`;
  const markers = entry.markers.length ? `\n${markerConstants(entry)}` : "";
  const turn = entry.turn
    ? `\n// Stop, then turn in place ${Math.abs(entry.turn.angle * 180 / Math.PI).toFixed(1)}° to heading ${(gps ? cornerToGps({ x: 0, y: 0, heading: entry.turn.heading, tangent: 0 }).heading : entry.turn.heading).toFixed(6)} rad (${formatCompass(entry.turn.heading)}) before the next section.\nconstexpr int ${entry.name}TurnTimeoutMs = ${suggestedTimeoutMs(entry.turn.duration)};`
    : "";
  return `// ${label} — ${unit}\nconst std::vector<vantage::Waypoint> ${entry.name}${gps ? "Gps" : ""} = {\n${waypointEntries(path, section, gps)}\n};\n${gps ? gpsConversion(entry.name) : "\n"}\nconst vantage::TrajectoryConfig ${entry.name}Config = [] {\n  vantage::TrajectoryConfig config;\n${config}${speedScaleLambda(entry.drive.speedScale)}\n  return config;\n}();\nconst auto ${entry.name}Trajectory = vantage::generateTrajectory(${entry.name}, ${entry.name}Config);\n${timing}${markers}${turn}`;
}

function libraryRoute(document, route, gps) {
  const code = route.sections.map((entry, index) => librarySection(document, route, entry, index, gps));
  const collection = route.sections.length > 1
    ? `\n\n// Run these sections in order. Each starts and ends at rest; turn in place where noted.\nconst std::vector<vantage::Trajectory> ${route.routeName}Trajectories = { ${route.sections.map((entry) => `${entry.name}Trajectory`).join(", ")} };`
    : "";
  const wait = route.path.waitAfterMs ? `\n// Then wait ${route.path.waitAfterMs} ms before the next route.\nconstexpr int ${route.routeName}WaitAfterMs = ${route.path.waitAfterMs};` : "";
  return code.join("\n\n") + collection + wait;
}

function chassisRoute(route) {
  return route.sections.map((entry) => `// ${route.path.name} · P${entry.section.firstSegment + 1} → P${entry.section.lastSegment + 2}${entry.section.reversed ? " · reverse" : ""}\nconst std::vector<vantage::Waypoint> ${entry.name} = {\n${waypointEntries(route.path, entry.section, false)}\n};`).join("\n\n");
}

function chassisRunFunction(document, routes, safeName) {
  const start = robotStartPose(document);
  const body = [`  chassis.setPoseCorner(${start.x.toFixed(4)}, ${start.y.toFixed(4)}, ${start.heading.toFixed(6)});  // ${formatCompass(start.heading)}`];
  let previousEnd = null;
  for (const route of routes) {
    const first = route.path.waypoints[0];
    body.push(`\n  // ${route.path.name}: planned ${route.plan.duration.toFixed(2)} s`);
    if (previousEnd && Math.hypot(first.x - previousEnd.x, first.y - previousEnd.y) > 0.5) {
      body.push(`  // WARNING: starts at (${first.x.toFixed(1)}, ${first.y.toFixed(1)}), not where the previous route ended (${previousEnd.x.toFixed(1)}, ${previousEnd.y.toFixed(1)}).`);
    }
    for (const entry of route.sections) {
      const timeout = suggestedTimeoutMs(entry.drive.duration);
      body.push(`  chassis.followPath(${entry.name}, ${entry.section.reversed}, ${timeout}, {}, true);`);
      for (const marker of entry.markers) {
        body.push(`  if (chassis.waitUntilProgress(${marker.percent.toFixed(1)})) {\n    // ${marker.name.replace(/\*\//g, "")}: add the mechanism action here.\n  }`);
      }
      body.push("  chassis.waitUntilDone();");
      if (entry.turn) body.push(`  chassis.turnToHeading(${compass(entry.turn.heading).toFixed(1)}, ${suggestedTimeoutMs(entry.turn.duration)}, {}, false);`);
    }
    if (route.path.waitAfterMs) body.push(`  pros::delay(${route.path.waitAfterMs});`);
    previousEnd = route.path.waypoints.at(-1);
  }
  const total = routes.reduce((sum, route) => sum + route.plan.duration + (route.path.waitAfterMs ?? 0) / 1000, 0);
  return `// Runs every route in order. Planned ${total.toFixed(2)} s with Studio's robot limits;\n// timeouts are ${TIMEOUT_SCALE}× the plan + ${TIMEOUT_PAD_S} s. Headings for turnToHeading are compass degrees.\ninline void run${safeName[0].toUpperCase()}${safeName.slice(1)}(drive::VantageChassis& chassis) {\n${body.join("\n")}\n}`;
}

export function cppExport(document, variableName, frame = "corner", target = "library") {
  const safeName = /^[A-Za-z_][A-Za-z0-9_]*$/.test(variableName) ? variableName : "generatedPath";
  const chassis = target === "chassis";
  const gps = frame === "gps" && !chassis;
  const drivable = document.paths.filter((path) => path.waypoints.length > 1);
  const routes = drivable.map((path, index) => routeExport(document, path, index, drivable.length, safeName));
  const failures = routes.filter((route) => route.error).map((route) => `// SKIPPED "${route.path.name}": ${route.error}`);
  const ok = routes.filter((route) => !route.error);
  const start = robotStartPose(document);
  const header = [
    "// Generated by VantagePath Studio",
    `// Export format ${EXPORT_FORMAT_VERSION} · ${EXPORT_TARGETS[chassis ? "chassis" : "library"]} · requires ${REQUIRED_LIBRARY}.`,
    chassis ? "// Waypoints are corner-frame inches; VantageChassis builds each trajectory from its own MotionParams." : "// Trajectories are generated during static initialisation; a bad route throws before main().",
  ].join("\n");
  const includes = chassis ? `#include "robot/chassis.h"\n#include <vantage/vantage.hpp>\n#include <vector>` : "#include <vantage/vantage.hpp>\n#include <vector>";
  const problems = failures.length ? `\n${failures.join("\n")}\n` : "";
  if (chassis) {
    return `${header}\n#pragma once\n${includes}\n${problems}\n${ok.map(chassisRoute).join("\n\n")}\n\n${chassisRunFunction(document, ok, safeName)}\n`;
  }
  const startPose = gps ? cornerToGps({ ...start, tangent: 0 }) : start;
  const poseLiteral = `{${startPose.x.toFixed(4)}, ${startPose.y.toFixed(4)}, ${startPose.heading.toFixed(6)}}`;
  const startExport = gps
    ? `const vantage::Pose2d ${safeName}RobotStartGps = ${poseLiteral};\nconst auto ${safeName}RobotStart = vantage::vexGpsToCorner(${safeName}RobotStartGps, {3.6576, 3.6576});`
    : `const vantage::Pose2d ${safeName}RobotStart = ${poseLiteral};`;
  return `${header}\n${includes}\n${problems}\n// Initial robot pose. Use to initialize localization before running routes.\n// Start facing ${formatCompass(start.heading)} (${toDisplayHeading(start.heading).toFixed(1)}° CCW from +X).\n${startExport}\n\n${ok.map((route) => libraryRoute(document, route, gps)).join("\n\n")}`;
}
