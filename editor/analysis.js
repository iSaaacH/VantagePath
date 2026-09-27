// Route checks shown in Studio: things that make the robot stop, crawl, or
// leave the field. Pure functions over a planned route (see planner.js).

import { FIELD_SIZE } from "./model.js";

const DEGREES = 180 / Math.PI;
const WALL_SAMPLE_SPACING = 0.5; // inches between footprint checks
const WALL_TOLERANCE = 0.05;
const TURN_SAMPLES = 12;

function footprintCorners(pose, robot) {
  const halfLength = robot.length / 2;
  const halfWidth = robot.width / 2;
  const c = Math.cos(pose.heading);
  const s = Math.sin(pose.heading);
  return [[halfLength, halfWidth], [halfLength, -halfWidth], [-halfLength, -halfWidth], [-halfLength, halfWidth]]
    .map(([forward, left]) => ({ x: pose.x + forward * c - left * s, y: pose.y + forward * s + left * c }));
}

/** How far (inches) the robot's rectangle pokes outside the field walls. */
export function wallOverlap(pose, robot) {
  return Math.max(0, ...footprintCorners(pose, robot).map(({ x, y }) => Math.max(-x, -y, x - FIELD_SIZE, y - FIELD_SIZE)));
}

function worstWallOverlap(plan, robot) {
  let worst = { overlap: 0 };
  const consider = (pose, waypointIndex) => {
    const overlap = wallOverlap(pose, robot);
    if (overlap > worst.overlap) worst = { overlap, pose, waypointIndex };
  };
  for (const step of plan.steps) {
    if (step.type === "turn") {
      for (let i = 0; i <= TURN_SAMPLES; i += 1) consider({ x: step.x, y: step.y, heading: step.fromHeading + step.angle * i / TURN_SAMPLES }, step.waypointIndex);
      continue;
    }
    let lastChecked = -Infinity;
    step.states.forEach((state, i) => {
      if (state.distance - lastChecked < WALL_SAMPLE_SPACING && i !== step.states.length - 1) return;
      lastChecked = state.distance;
      consider(state, state.segmentIndex);
    });
  }
  return worst;
}

function tightestCurve(plan) {
  let tightest = null;
  for (const step of plan.steps) {
    if (step.type !== "drive") continue;
    for (const state of step.states) {
      const curvature = Math.abs(state.curvature);
      if (curvature > 1e-9 && (!tightest || curvature > tightest.curvature)) tightest = { curvature, state };
    }
  }
  return tightest;
}

/**
 * Issues for one planned route, most severe first. Each issue:
 * { id, severity: "error" | "warning" | "info", title, detail, waypointIndex, x, y, action }
 */
export function analyzeRoute(path, plan, robot) {
  if (!path || path.waypoints.length < 2) return [];
  if (plan.error) {
    return [{ id: "plan-error", severity: "error", title: "Route can't be planned", detail: plan.error, waypointIndex: 0, x: path.waypoints[0].x, y: path.waypoints[0].y }];
  }
  const issues = [];
  for (const stop of plan.stops) {
    const label = `P${stop.waypointIndex + 1}`;
    if (stop.reason === "corner") {
      issues.push({
        id: `corner-${stop.waypointIndex}`, severity: "warning", action: "smooth",
        title: `Sharp corner at ${label}`,
        detail: `The robot stops and turns ${Math.abs(stop.turn * DEGREES).toFixed(1)}° in place. Smooth the join to drive through it.`,
        waypointIndex: stop.waypointIndex, x: stop.x, y: stop.y,
      });
    } else if (stop.reason === "direction-change" && Math.abs(stop.turn) > 1e-3) {
      issues.push({
        id: `cusp-${stop.waypointIndex}`, severity: "warning", action: "smooth",
        title: `Turn in place at ${label}`,
        detail: `Changing direction here also needs a ${Math.abs(stop.turn * DEGREES).toFixed(1)}° point turn. Smooth the join for a clean reversal.`,
        waypointIndex: stop.waypointIndex, x: stop.x, y: stop.y,
      });
    }
  }
  const tightest = tightestCurve(plan);
  if (tightest) {
    const radius = 1 / tightest.curvature;
    const halfTrack = robot.trackWidth / 2;
    if (radius < halfTrack) {
      issues.push({
        id: "tight-curve", severity: "warning",
        title: `Curve tighter than the robot can arc`,
        detail: `Radius ${radius.toFixed(2)} in near P${tightest.state.segmentIndex + 1}–P${tightest.state.segmentIndex + 2} is under half the track width (${halfTrack.toFixed(1)} in). The inner wheel reverses and the robot crawls at ${tightest.state.velocity.toFixed(1)} in/s.`,
        waypointIndex: tightest.state.segmentIndex, x: tightest.state.x, y: tightest.state.y,
      });
    }
  }
  const wall = worstWallOverlap(plan, robot);
  if (wall.overlap > WALL_TOLERANCE) {
    issues.push({
      id: "wall", severity: "warning",
      title: "Robot leaves the field",
      detail: `The ${robot.length}×${robot.width} in footprint crosses a wall by ${wall.overlap.toFixed(1)} in near P${wall.waypointIndex + 1}.`,
      waypointIndex: wall.waypointIndex, x: wall.pose.x, y: wall.pose.y,
    });
  }
  const joinStops = plan.stops.filter((stop) => stop.reason === "bezier-join").length;
  if (joinStops) {
    issues.push({
      id: "bezier-stops", severity: "info",
      title: `${joinStops} stop${joinStops === 1 ? "" : "s"} at Bézier joins`,
      detail: "The library brings the robot to rest wherever two Bézier segments meet, even when the join is smooth.",
      waypointIndex: plan.stops.find((stop) => stop.reason === "bezier-join").waypointIndex,
    });
  }
  const rank = { error: 0, warning: 1, info: 2 };
  return issues.sort((a, b) => rank[a.severity] - rank[b.severity]);
}
