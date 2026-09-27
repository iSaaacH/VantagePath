// SVG markup for the field: floor, alliance zones, routes, markers, checks.
// Field inches map to SVG units with y flipped (SVG y grows downwards).

import { FIELD_SIZE, normalizeSegmentDirections, segmentPoint } from "../model.js";
import { escapeHtml } from "./html.js";

const CURVE_SAMPLES = 60;
const HANDLE_SCALE = 5;

const svgY = (y) => FIELD_SIZE - y;

export function fieldMarkup(doc) {
  // The field image is cropped to exactly the 144 x 144 inch foam Floor. Its
  // edges map directly to SVG 0..144, and this independent 24-inch grid must
  // continue to align with its six tile rows and columns.
  const grid = Array.from({ length: 7 }, (_, index) => index * 24).map((value) => `<path class="tile-grid" d="M${value} 0V144M0 ${value}H144"/>`).join("");
  const redVisible = doc.showZones && doc.alliance === "red" ? "" : " zone-hidden";
  const blueVisible = doc.showZones && doc.alliance === "blue" ? "" : " zone-hidden";
  return `<rect class="field-floor" width="144" height="144"/>
    <image class="field-image" href="assets/override-field.webp" x="0" y="0" width="144" height="144" preserveAspectRatio="none"/>
    ${grid}
    <polygon class="alliance-zone-red${redVisible}" points="0,0 48,72 72,96 144,144 0,144"/>
    <polygon class="alliance-zone-blue${blueVisible}" points="0,0 144,0 144,144 96,72 72,48"/>
    <rect class="field-wall" width="144" height="144"/>
    <text class="anchor-label axis-label" x="2.8" y="141">0,0</text><text class="anchor-label axis-label" x="134" y="141">+X</text><text class="anchor-label axis-label" x="2.8" y="7">+Y</text>`;
}

function curveData(path, index) {
  let d = `M${path.waypoints[index].x} ${svgY(path.waypoints[index].y)}`;
  for (let step = 1; step <= CURVE_SAMPLES; step += 1) {
    const sample = segmentPoint(path, index, step / CURVE_SAMPLES);
    d += ` L${sample.x.toFixed(3)} ${svgY(sample.y).toFixed(3)}`;
  }
  return d;
}

/** A recorded odometry trace drawn over the field. */
export function logOverlayMarkup(overlay) {
  if (!overlay?.points?.length) return "";
  const points = overlay.points.map((point) => `${point.x.toFixed(2)},${svgY(point.y).toFixed(2)}`).join(" ");
  const start = overlay.points[0];
  const end = overlay.points.at(-1);
  return `<g class="log-overlay" aria-hidden="true"><polyline points="${points}"/><circle cx="${start.x}" cy="${svgY(start.y)}" r="1.2"/><rect x="${end.x - 1.1}" y="${svgY(end.y) - 1.1}" width="2.2" height="2.2"/></g>`;
}

/** Other routes, faint and clickable so switching routes is one tap. */
export function inactiveRoutesMarkup(doc, activeId) {
  return doc.paths.filter((path) => path.id !== activeId && path.waypoints.length > 1).map((path) => {
    normalizeSegmentDirections(path);
    const d = path.waypoints.slice(0, -1).map((_, index) => curveData(path, index)).join(" ");
    return `<g class="inactive-route" data-path-id="${path.id}"><title>${escapeHtml(path.name)} · click to edit</title><path class="inactive-hit" d="${d}"/><path class="inactive-curve" d="${d}"/></g>`;
  }).join("");
}

function robotMarkup(path, robot) {
  const start = path.waypoints[0];
  return `<g id="playback-robot" class="playback-robot" data-playback-marker="true" transform="translate(${start.x} ${svgY(start.y)}) rotate(${-start.heading * 180 / Math.PI})">
    <title>Preview robot · drag along the route to scrub</title>
    <rect class="robot-box" x="${-robot.length / 2}" y="${-robot.width / 2}" width="${robot.length}" height="${robot.width}" rx="1"/>
    <line class="robot-nose" x1="${robot.length * .12}" y1="0" x2="${robot.length * .43}" y2="0"/>
  </g>`;
}

function controlsMarkup(path, selection) {
  const items = path.controlPoints?.[selection.segment];
  if (!items) return "";
  const segment = selection.segment;
  const polygon = [path.waypoints[segment], ...items, path.waypoints[segment + 1]];
  return `<polyline class="control-line bezier-guide" fill="none" points="${polygon.map((p) => `${p.x},${svgY(p.y)}`).join(" ")}"/>`
    + items.map((p, i) => `<g class="control-marker ${p.id === selection.controlId ? "selected" : ""}" data-control-id="${p.id}" transform="translate(${p.x} ${svgY(p.y)})"><title>Control point C${i + 1}: bends the curve</title><circle class="marker-hit" r="3.2"/><path class="control-diamond" d="M0 -1.8 1.8 0 0 1.8 -1.8 0Z"/><text class="anchor-label control-label" x="2.6" y="-2.6">C${i + 1}</text></g>`).join("");
}

function anchorsMarkup(path, selection) {
  return path.waypoints.map((point, index) => {
    const selected = !selection.controlId && point.id === selection.pointId;
    const hermite = (index > 0 && path.controlPoints[index - 1] === null) || (index < path.waypoints.length - 1 && path.controlPoints[index] === null);
    const handleX = point.x + Math.cos(point.heading) * point.tangent / HANDLE_SCALE;
    const handleY = point.y + Math.sin(point.heading) * point.tangent / HANDLE_SCALE;
    const handle = hermite ? `<line class="control-line" x1="${point.x}" y1="${svgY(point.y)}" x2="${handleX}" y2="${svgY(handleY)}"/>
      <circle class="handle" data-handle-id="${point.id}" cx="${handleX}" cy="${svgY(handleY)}" r="2"><title>Heading and tangent handle for P${index + 1}</title></circle>` : "";
    return `<g>${handle}
      <circle class="anchor-hit" data-point-id="${point.id}" cx="${point.x}" cy="${svgY(point.y)}" r="3.6"/>
      <circle class="anchor ${selected ? "selected" : ""}${index === 0 ? " is-start" : ""}" data-point-id="${point.id}" cx="${point.x}" cy="${svgY(point.y)}" r="2.1"><title>Route point P${index + 1}</title></circle>
      <text class="anchor-label" x="${point.x + 3.8}" y="${svgY(point.y) - 3}">P${index + 1}</text></g>`;
  }).join("");
}

function checksMarkup(plan, issues) {
  const flagged = new Set(issues.filter((issue) => issue.action === "smooth").map((issue) => issue.waypointIndex));
  const stops = plan.stops.map((stop) => {
    const warn = flagged.has(stop.waypointIndex);
    const why = stop.reason === "corner" ? "sharp corner: stops and turns in place" : stop.reason === "direction-change" ? "changes drive direction" : "Bézier join: the library stops here";
    return `<g class="stop-marker${warn ? " is-warning" : ""}" transform="translate(${stop.x} ${svgY(stop.y)})"><title>Robot stops at P${stop.waypointIndex + 1} · ${why}</title><path d="M-1.4 -3.4h2.8l2 2v2.8l-2 2h-2.8l-2-2v-2.8z"/></g>`;
  }).join("");
  const spots = issues.filter((issue) => !issue.action && Number.isFinite(issue.x)).map((issue) =>
    `<g class="issue-marker is-${issue.severity}" transform="translate(${issue.x} ${svgY(issue.y)})"><title>${escapeHtml(issue.title)}: ${escapeHtml(issue.detail)}</title><circle r="4.2"/><text y="1.3">!</text></g>`).join("");
  return `<g class="checks-layer">${spots}${stops}</g>`;
}

const FOOTPRINT_SPACING_IN = 8;

// Faint robot outlines along the planned route, to see the space it sweeps.
function footprintMarkup(plan, robot) {
  const ghosts = [];
  const ghost = (x, y, heading) => ghosts.push(`<rect class="footprint" x="${-robot.length / 2}" y="${-robot.width / 2}" width="${robot.length}" height="${robot.width}" transform="translate(${x.toFixed(2)} ${svgY(y).toFixed(2)}) rotate(${(-heading * 180 / Math.PI).toFixed(1)})"/>`);
  for (const step of plan.steps) {
    if (step.type === "turn") { ghost(step.x, step.y, step.fromHeading + step.angle); continue; }
    let next = 0;
    for (const state of step.states) {
      if (state.distance + 1e-9 < next) continue;
      ghost(state.x, state.y, state.heading);
      next = state.distance + FOOTPRINT_SPACING_IN;
    }
    const last = step.states.at(-1);
    ghost(last.x, last.y, last.heading);
  }
  return `<g class="footprint-layer" aria-hidden="true">${ghosts.join("")}</g>`;
}

function eventMarkersMarkup(path, selection) {
  return (path.markers ?? []).map((marker) => {
    const at = segmentPoint(path, marker.segment, marker.t);
    const selected = marker.id === selection.markerId;
    return `<g class="event-marker${selected ? " selected" : ""}" data-marker-id="${marker.id}" transform="translate(${at.x} ${svgY(at.y)})"><title>Marker “${escapeHtml(marker.name)}” · drag along the path</title><circle class="marker-hit" r="3.2"/><path class="event-flag" d="M0 0V-5.2L3.4 -4.1 0 -3"/><circle class="event-dot" r="0.9"/><text class="anchor-label event-label" x="3.8" y="-4.2">${escapeHtml(marker.name)}</text></g>`;
  }).join("");
}

/**
 * The active route. Route and control points sit above the preview robot so
 * they stay selectable when the robot is parked on a point.
 */
export function activeRouteMarkup(path, { robot, selection, plan, issues, showFootprint = false }) {
  if (!path?.waypoints.length) return "";
  normalizeSegmentDirections(path);
  const curves = path.waypoints.slice(0, -1).map((_, index) => {
    const d = curveData(path, index);
    const current = index === selection.segment ? " is-current" : "";
    return `<path class="path-shadow" d="${d}"/><path class="path-curve${path.segmentReversed[index] ? " reversed" : ""}${current}" data-segment-hit="${index}" d="${d}"/>`;
  }).join("");
  const footprint = showFootprint && !plan.error ? footprintMarkup(plan, robot) : "";
  return `${footprint}${curves}${checksMarkup(plan, issues)}${robotMarkup(path, robot)}${controlsMarkup(path, selection)}${eventMarkersMarkup(path, selection)}${anchorsMarkup(path, selection)}`;
}
