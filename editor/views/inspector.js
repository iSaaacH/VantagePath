// Right panel: route checks, then fields for whatever is selected.

import { anchorHeading, isHeadingDerived, toDisplayHeading } from "../model.js";
import { suggestedTimeoutMs } from "../export.js";
import { escapeHtml, fixed, plural } from "./html.js";

export function checksMarkup(issues) {
  const counts = ["error", "warning", "info"].map((severity) => issues.filter((issue) => issue.severity === severity).length);
  const summary = [counts[0] && plural(counts[0], "error"), counts[1] && plural(counts[1], "warning"), counts[2] && plural(counts[2], "note")].filter(Boolean);
  const items = issues.length
    ? issues.map((issue) => `<li class="check is-${issue.severity}">
        <button class="check-body" data-check-point="${issue.waypointIndex}"><b>${escapeHtml(issue.title)}</b><span>${escapeHtml(issue.detail)}</span></button>
        ${issue.action === "smooth" ? `<button class="check-fix" data-smooth-index="${issue.waypointIndex}">Smooth join</button>` : ""}
      </li>`).join("")
    : `<li class="check is-ok"><span>No point turns, wall hits or impossible curves. Planned with the same rules as the robot.</span></li>`;
  return `<div class="panel-label"><span>Route checks</span><small id="checks-count">${summary.length ? summary.join(" · ") : "all clear"}</small></div>
    <ul id="checks-list" class="checks-list">${items}</ul>`;
}

function pointPanel(path, index, prefs) {
  const point = path.waypoints[index];
  const derived = path.waypoints.length > 1 && isHeadingDerived(path, index);
  const heading = path.waypoints.length > 1 ? anchorHeading(path, index) : point.heading;
  const interior = index > 0 && index < path.waypoints.length - 1;
  const convention = prefs.headingMode === "compass" ? "CW from +Y" : "CCW from +X";
  const headingFields = derived
    ? `<div class="readout-row"><span>Heading</span><b>${toDisplayHeading(heading, prefs.headingMode).toFixed(1)}°</b><small>set by the curve · ${convention}</small></div>`
    : `<div class="field-pair">
        <label><span>Heading ° <em>${convention}</em></span><input data-edit="point-heading" type="number" step="1" value="${toDisplayHeading(point.heading, prefs.headingMode).toFixed(1)}" /></label>
        <label><span>Tangent in</span><input data-edit="point-tangent" type="number" min="1" step="1" value="${fixed(point.tangent, 1)}" /></label>
      </div>`;
  return `<div class="panel-label"><span>Route point P${index + 1}</span><small>${index === 0 ? "route start" : index === path.waypoints.length - 1 ? "route end" : "robot drives through"}</small></div>
    <div class="field-pair">
      <label><span>X in</span><input data-edit="point-x" type="number" min="0" max="144" step="0.25" value="${fixed(point.x)}" /></label>
      <label><span>Y in</span><input data-edit="point-y" type="number" min="0" max="144" step="0.25" value="${fixed(point.y)}" /></label>
    </div>
    ${headingFields}
    <div class="button-row">
      ${interior ? `<button data-smooth-index="${index}" title="Make the curve pass straight through this point">Smooth join</button>` : ""}
      <button class="danger" data-action="delete-point">Delete P${index + 1}</button>
    </div>`;
}

function controlPanel(path, segment, controlId) {
  const controls = path.controlPoints[segment] ?? [];
  const index = controls.findIndex((point) => point.id === controlId);
  const control = controls[index];
  if (!control) return "";
  return `<div class="panel-label"><span>Control C${index + 1}</span><small>bends segment ${segment + 1}</small></div>
    <div class="field-pair">
      <label><span>X in</span><input data-edit="control-x" data-control-index="${index}" type="number" min="0" max="144" step="0.25" value="${fixed(control.x)}" /></label>
      <label><span>Y in</span><input data-edit="control-y" data-control-index="${index}" type="number" min="0" max="144" step="0.25" value="${fixed(control.y)}" /></label>
    </div>
    <p class="hint">Control points shape the curve. The robot does not drive through them.</p>
    <div class="button-row"><button class="danger" data-remove-control="${index}">Delete C${index + 1}</button></div>`;
}

function segmentStats(plan, segment) {
  let length = 0; let time = 0; let peak = 0; let curvature = 0; let sectionTime = 0;
  for (const step of plan.steps ?? []) {
    if (step.type !== "drive") continue;
    if (segment >= step.section.firstSegment && segment <= step.section.lastSegment) sectionTime = step.duration;
    step.states.forEach((state, i) => {
      if (state.segmentIndex !== segment) return;
      peak = Math.max(peak, Math.abs(state.velocity));
      curvature = Math.max(curvature, Math.abs(state.curvature));
      if (i === 0) return;
      const previous = step.states[i - 1];
      length += state.distance - previous.distance;
      time += state.time - previous.time;
    });
  }
  return { length, time, peak, radius: curvature > 1e-9 ? 1 / curvature : Infinity, sectionTime, sectionTimeoutMs: sectionTime ? suggestedTimeoutMs(sectionTime) : 0 };
}

function markersMarkup(path, segment, selectedMarkerId) {
  const markers = path.markers.filter((marker) => marker.segment === segment);
  const rows = markers.map((marker) => `<div class="marker-row${marker.id === selectedMarkerId ? " is-selected" : ""}">
      <label><span class="sr-only">Marker name</span><input data-edit="marker-name" data-marker-id="${marker.id}" value="${escapeHtml(marker.name)}" maxlength="40" /></label>
      <label title="Position along this segment"><span class="sr-only">Position along segment, percent</span><input data-edit="marker-t" data-marker-id="${marker.id}" type="number" min="0" max="100" step="1" value="${(marker.t * 100).toFixed(0)}" /></label>
      <button data-remove-marker="${marker.id}" aria-label="Remove marker ${escapeHtml(marker.name)}">×</button>
    </div>`).join("");
  return `<div class="control-toolbar"><span>${markers.length ? plural(markers.length, "event marker") : "No event markers"}</span><button data-action="add-marker">＋ Marker</button></div>
    ${markers.length ? `<div class="marker-head" aria-hidden="true"><span>Name</span><span>% of segment</span></div>` : ""}
    <div class="marker-list">${rows}</div>`;
}

function segmentPanel(path, segment, selection, plan) {
  if (path.waypoints.length < 2) return `<p class="hint">Add a second point to make a segment.</p>`;
  const controls = path.controlPoints[segment];
  const legacy = controls === null;
  const reversed = path.segmentReversed[segment];
  const stats = plan.error ? null : segmentStats(plan, segment);
  const rows = (controls ?? []).map((point, i) => `<div class="control-row ${point.id === selection.controlId ? "is-selected" : ""}">
      <button data-select-control="${point.id}" aria-label="Select control point ${i + 1}">C${i + 1}</button>
      <label><span class="sr-only">Control ${i + 1} X</span><input type="number" min="0" max="144" step="0.25" data-edit="control-x" data-control-index="${i}" value="${fixed(point.x)}" /></label>
      <label><span class="sr-only">Control ${i + 1} Y</span><input type="number" min="0" max="144" step="0.25" data-edit="control-y" data-control-index="${i}" value="${fixed(point.y)}" /></label>
      <button data-remove-control="${i}" aria-label="Remove control point ${i + 1}">×</button>
    </div>`).join("");
  return `<div class="panel-label"><span>Segment ${segment + 1} · P${segment + 1} → P${segment + 2}</span></div>
    ${stats ? `<dl class="stat-grid">
      <div><dt>Length</dt><dd>${fixed(stats.length, 1)}<small> in</small></dd></div>
      <div><dt>Drive time</dt><dd>${fixed(stats.time, 2)}<small> s</small></dd></div>
      <div><dt>Top speed</dt><dd>${fixed(stats.peak, 0)}<small> in/s</small></dd></div>
      <div><dt>Tightest</dt><dd>${Number.isFinite(stats.radius) ? `${fixed(stats.radius, 1)}<small> in R</small>` : "straight"}</dd></div>
    </dl>${stats.sectionTimeoutMs ? `<p class="hint">Section timeout ${stats.sectionTimeoutMs} ms (planned ${fixed(stats.sectionTime, 2)} s × 1.5 + 0.25 s).</p>` : ""}` : ""}
    <div class="segmented" role="group" aria-label="Drive direction">
      <button data-set-direction="false" class="${reversed ? "" : "is-active"}" aria-pressed="${!reversed}">→ Forward</button>
      <button data-set-direction="true" class="${reversed ? "is-active" : ""}" aria-pressed="${reversed}">← Reverse</button>
    </div>
    <label class="stacked"><span>Speed limit in/s <em>blank = robot max</em></span><input data-edit="segment-speed" type="number" min="1" step="1" placeholder="robot max" value="${path.segmentSpeed[segment] ?? ""}" /></label>
    <label class="stacked"><span>Curve type</span><select data-edit="geometry-mode">
      <option value="bezier" ${legacy ? "" : "selected"}>Bézier · control points</option>
      <option value="hermite" ${legacy ? "selected" : ""}>Legacy · heading and tangent</option>
    </select></label>
    ${legacy ? `<p class="hint">Shaped by the headings and tangents of P${segment + 1} and P${segment + 2}. Drag their yellow handles.</p>` : `
      <div class="control-toolbar"><span>${controls.length ? plural(controls.length, "control point") : "Straight line"}</span><button data-action="add-control">＋ Control</button></div>
      <div class="control-list">${rows}</div>`}
    ${markersMarkup(path, segment, selection.markerId)}`;
}

/** Selected thing first (point or control), then its segment. */
export function inspectorMarkup(path, { selection, prefs, plan }) {
  if (!path) return "";
  const index = path.waypoints.findIndex((point) => point.id === selection.pointId);
  const top = selection.controlId ? controlPanel(path, selection.segment, selection.controlId) : index >= 0 ? pointPanel(path, index, prefs) : "";
  return `${top ? `<section class="panel">${top}</section>` : ""}<section class="panel">${segmentPanel(path, selection.segment, selection, plan)}</section>`;
}
