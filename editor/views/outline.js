// Left panel: robot start and the route outline (routes → points and segments).

import { anchorHeading, isHeadingDerived, toDisplayHeading } from "../model.js";
import { escapeHtml, fixed, plural } from "./html.js";

function headingText(radians, mode) {
  return `${toDisplayHeading(radians, mode).toFixed(1)}°`;
}

export function robotStartMarkup(doc, prefs, isSelected) {
  const path = doc.paths[0];
  const point = path?.waypoints[0];
  if (!point) return `<p class="muted">Add a route point to place the robot.</p>`;
  const derived = path.waypoints.length > 1 && isHeadingDerived(path, 0);
  const heading = path.waypoints.length > 1 ? anchorHeading(path, 0) : point.heading;
  return `<div class="start-card${isSelected ? " is-selected" : ""}">
    <div class="start-head"><span class="start-badge">P1</span><h2>Robot start</h2><button class="text-button" data-action="select-start">Select</button></div>
    <div class="field-trio">
      <label><span>X in</span><input data-start="x" type="number" min="0" max="144" step="0.25" value="${fixed(point.x)}" /></label>
      <label><span>Y in</span><input data-start="y" type="number" min="0" max="144" step="0.25" value="${fixed(point.y)}" /></label>
      <label title="${derived ? "Set by the first Bézier segment: move its first control point to change it." : ""}"><span>Facing °</span><input data-start="heading" type="number" step="1" value="${toDisplayHeading(heading, prefs.headingMode).toFixed(1)}" ${derived ? "disabled" : ""} /></label>
    </div>
    <p class="hint">${derived ? "Facing follows the first curve. " : ""}P1 of the first route is the robot start.</p>
  </div>`;
}

function segmentRow(path, index, { selection, planStep }) {
  const reversed = path.segmentReversed[index];
  const controls = path.controlPoints[index];
  const kind = controls === null ? "Legacy spline" : controls.length ? `Bézier · ${plural(controls.length, "control")}` : "Straight";
  const current = index === selection.segment ? " is-current" : "";
  return `<li class="outline-segment${current}">
    <button class="outline-row" data-select-segment="${index}" aria-pressed="${index === selection.segment}">
      <span class="seg-arrow" aria-hidden="true">${reversed ? "↑" : "↓"}</span>
      <span class="outline-main">Segment ${index + 1}<small>${kind}${planStep ? ` · ${fixed(planStep, 1)} in` : ""}</small></span>
    </button>
    <button class="direction-chip${reversed ? " is-reverse" : ""}" data-toggle-direction="${index}" aria-label="Segment ${index + 1} drives ${reversed ? "in reverse" : "forward"}. Switch direction" title="Switch drive direction">${reversed ? "← Rev" : "→ Fwd"}</button>
  </li>`;
}

function pointRow(path, index, { selection, prefs, flagged }) {
  const point = path.waypoints[index];
  const heading = path.waypoints.length > 1 ? anchorHeading(path, index) : point.heading;
  const selected = !selection.controlId && point.id === selection.pointId;
  const flag = flagged.get(index);
  return `<li class="outline-point${selected ? " is-selected" : ""}">
    <button class="outline-row" data-select-point-id="${point.id}" aria-pressed="${selected}">
      <span class="point-badge">P${index + 1}</span>
      <span class="outline-main"><span class="mono">${fixed(point.x, 1)}, ${fixed(point.y, 1)}</span><small>${headingText(heading, prefs.headingMode)}${isHeadingDerived(path, index) ? " · from curve" : ""}</small></span>
      ${flag ? `<span class="row-flag is-${flag.severity}" title="${escapeHtml(flag.title)}">!</span>` : ""}
    </button>
  </li>`;
}

// Total planned time of every route plus waits, against the period length.
export function budgetMarkup(doc, routeDurations) {
  const limit = doc.alliance === "skills" ? 60 : 15;
  const total = doc.paths.reduce((sum, path) => sum + (routeDurations.get(path.id) ?? 0) + (path.waitAfterMs ?? 0) / 1000, 0);
  const fraction = Math.min(1, total / limit);
  const over = total > limit;
  return `<div class="budget${over ? " is-over" : ""}" title="All routes in order, including waits, planned with Studio's robot limits">
    <div class="budget-text"><span>${doc.alliance === "skills" ? "Skills" : "Autonomous"} budget</span><b>${total.toFixed(1)} / ${limit} s</b></div>
    <div class="budget-bar"><i style="width:${(fraction * 100).toFixed(1)}%"></i></div>
  </div>`;
}

/** The route list. The active route expands into alternating points and segments. */
export function routeTreeMarkup(doc, { activePathId, selection, prefs, plan, issues }) {
  const flagged = new Map(issues.filter((issue) => issue.severity !== "info" && Number.isInteger(issue.waypointIndex)).map((issue) => [issue.waypointIndex, issue]));
  return doc.paths.map((path, routeIndex) => {
    const active = path.id === activePathId;
    const segments = Math.max(0, path.waypoints.length - 1);
    const summary = active && !plan.error && segments ? `${fixed(plan.duration, 2)} s · ${plural(plan.stops.length, "stop")}` : `${plural(segments, "segment")}`;
    const head = `<button class="route-head" data-path-id="${path.id}" aria-expanded="${active}">
      <span class="route-index">${String(routeIndex + 1).padStart(2, "0")}</span>
      <span class="outline-main"><strong>${escapeHtml(path.name)}</strong><small>${summary}</small></span>
      <span class="route-caret" aria-hidden="true">${active ? "▾" : "▸"}</span>
    </button>`;
    if (!active) return `<li class="route">${head}</li>`;
    const segmentLengths = new Map();
    for (const step of plan.steps ?? []) {
      if (step.type !== "drive") continue;
      let previous = 0;
      for (const state of step.states) {
        segmentLengths.set(state.segmentIndex, (segmentLengths.get(state.segmentIndex) ?? 0) + state.distance - previous);
        previous = state.distance;
      }
    }
    const rows = path.waypoints.flatMap((_, index) => {
      const items = [pointRow(path, index, { selection, prefs, flagged })];
      if (index < segments) items.push(segmentRow(path, index, { selection, planStep: segmentLengths.get(index) }));
      return items;
    }).join("");
    return `<li class="route is-active">${head}
      <div class="route-tools">
        <label class="sr-only" for="route-name">Route name</label>
        <input id="route-name" data-edit="route-name" value="${escapeHtml(path.name)}" aria-label="Route name" />
        <div class="route-tool-buttons"><button data-action="all-forward" title="Drive every segment forward">All →</button><button data-action="all-reverse" title="Drive every segment in reverse">All ←</button><button data-action="duplicate-path">Duplicate</button><button data-action="route-up" aria-label="Run this route earlier" title="Run earlier" ${routeIndex === 0 ? "disabled" : ""}>↑</button><button data-action="route-down" aria-label="Run this route later" title="Run later" ${routeIndex === doc.paths.length - 1 ? "disabled" : ""}>↓</button></div>
        <label class="inline-field"><span>Wait after route</span><input data-edit="wait-after" type="number" min="0" step="50" value="${path.waitAfterMs ?? 0}" /><span>ms</span></label>
      </div>
      <ol class="outline-list">${rows}</ol>
      <button class="add-row" data-action="add-point">＋ Add point after P${path.waypoints.length}</button>
    </li>`;
  }).join("");
}
