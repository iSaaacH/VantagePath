import { analyzeRoute } from "./analysis.js";
import { anchorHeading, dragPosition, FIELD_SIZE, clamp, cppExport, fromDisplayHeading, historySnapshot, isHeadingDerived, makeDocument, mirrorWaypoint, nearestPathDistance, normalizeSegmentDirections, restoreSnapshot, robotStartPose, segmentPoint, setControlCount, smoothJoin, splitSegment, reverseWaypoints, toDisplayHeading, validateDocument, withDerivedHeadings, wrapRadians } from "./model.js";
import { planRoute, poseAtTime, routeSamples, segmentTangent, timeAtDistance } from "./planner.js";

const STORAGE_KEY = "vantagepath-studio-v1";
const PREFS_KEY = "vantagepath-studio-prefs";
const NUDGE_COALESCE_MS = 800;
const LONG_PRESS_MS = 550;
const LONG_PRESS_SLOP_IN = 1.5;
const INSERT_HIT_IN = 2.5;
const DELETE_CONFIRM_MS = 3000;
const NEW_POINT_STEP_IN = 24;
const svg = document.querySelector("#field");
const stage = document.querySelector("#field-stage");
const fileInput = document.querySelector("#file-input");
const pathList = document.querySelector("#path-list");
const waypointList = document.querySelector("#waypoint-list");
const controlPanel = document.querySelector("#control-panel");
let selectedSegment = 0;
let selectedControlId = null;
const segmentList = document.querySelector("#segment-list");
const toast = document.querySelector("#toast");
const titleNode = document.querySelector("#document-title");
const pointInputs = {
  x: document.querySelector("#point-x"), y: document.querySelector("#point-y"),
  heading: document.querySelector("#point-heading"), tangent: document.querySelector("#point-tangent"),
};
const robotInputs = [...document.querySelectorAll("[data-robot]")];
const scrubber = document.querySelector("#path-scrubber");

let startupMessage = "";
let documentState = loadLocal();
let prefs = loadPrefs();
let activePathId = documentState.paths[0]?.id ?? null;
let selectedPointId = documentState.paths[0]?.waypoints[0]?.id ?? null;
let history = [historySnapshot(documentState)];
let historyIndex = 0;
let lastCoalesce = { key: null, at: 0 };
let planCache = { key: "", plan: null, issues: [] };
let saveState = { ok: true, at: null };
let deleteArmedUntil = 0;
let longPress = null;
let drag = null;
let toastTimer = 0;
let playback = { playing:false, time:0, startedAt:0, startedFrom:0, frame:0 };

function activePath() { return documentState.paths.find((path) => path.id === activePathId) ?? documentState.paths[0]; }
function selectedPoint() { return activePath()?.waypoints.find((point) => point.id === selectedPointId) ?? null; }
function escapeHtml(value) { return String(value).replace(/[&<>'"]/g, (character) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[character])); }
function uid() { return crypto.randomUUID(); }
function snap(value, bypass = false) { return documentState.snap && !bypass ? Math.round(value / documentState.snapStep) * documentState.snapStep : value; }

function readStorage(key) {
  try { return localStorage.getItem(key); } catch { return null; }
}

function loadLocal() {
  const raw = readStorage(STORAGE_KEY);
  if (raw === null) return makeDocument();
  try {
    return validateDocument(JSON.parse(raw));
  } catch (error) {
    // Keep the unreadable copy so the next autosave can't destroy it.
    const backupKey = `${STORAGE_KEY}-unreadable-${Date.now()}`;
    try { localStorage.setItem(backupKey, raw); } catch { /* storage full or blocked */ }
    startupMessage = `Your saved route couldn't be read (${error instanceof Error ? error.message : "unknown error"}). A copy was kept as "${backupKey}"; starting fresh.`;
    return makeDocument();
  }
}

function loadPrefs() {
  try {
    const stored = JSON.parse(readStorage(PREFS_KEY) ?? "{}");
    return { headingMode: stored.headingMode === "compass" ? "compass" : "math" };
  } catch { return { headingMode: "math" }; }
}

function savePrefs() {
  try { localStorage.setItem(PREFS_KEY, JSON.stringify(prefs)); } catch { /* preferences are optional */ }
}

function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(documentState));
    saveState = { ok: true, at: new Date() };
  } catch (error) {
    saveState = { ok: false, at: new Date(), reason: error instanceof Error ? error.message : "storage unavailable" };
  }
  renderSaveState();
}

function renderSaveState() {
  const node = document.querySelector("#save-state");
  if (!node) return;
  node.classList.toggle("is-error", !saveState.ok);
  const time = saveState.at ? ` ${saveState.at.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}` : "";
  node.querySelector("span").textContent = saveState.ok ? `Saved in browser${time}` : "Not saved — use Save .vpath";
  node.title = saveState.ok ? "Changes autosave to this browser. Save .vpath to keep a file." : `Autosave failed: ${saveState.reason}`;
}

// Route edits go through here. View preferences (alliance, snap, zones) use
// setView() instead so undo never flips them.
function commit(message, { coalesce = null } = {}) {
  stopPlayback(true);
  documentState.paths = documentState.paths.map(withDerivedHeadings);
  documentState.robotStart = robotStartPose(documentState);
  const serial = historySnapshot(documentState);
  const now = performance.now();
  const merge = coalesce && lastCoalesce.key === coalesce && now - lastCoalesce.at < NUDGE_COALESCE_MS && historyIndex > 0;
  if (history[historyIndex] !== serial) {
    if (merge) history[historyIndex] = serial;
    else {
      history = history.slice(0, historyIndex + 1);
      history.push(serial);
      historyIndex += 1;
    }
  }
  lastCoalesce = { key: coalesce, at: now };
  persist(); render();
  if (message) notify(message);
}

function setView(key, value) {
  documentState[key] = value;
  persist(); render();
}

function headingText(radians) { return `${toDisplayHeading(radians, prefs.headingMode).toFixed(1)}°`; }

// The planned route (planner.js) for the active path, recomputed only when the
// path or robot changes. Timing and stops match the exported C++ trajectories.
function routePlan() {
  const path = activePath();
  const key = JSON.stringify([path, documentState.robot]);
  if (planCache.key !== key) {
    const plan = planRoute(path, documentState.robot);
    planCache = { key, plan, issues: analyzeRoute(path, plan, documentState.robot) };
  }
  return planCache;
}

function updatePlaybackUi() {
  const { plan } = routePlan();
  playback.time = Math.min(playback.time, plan.duration);
  const progress = plan.duration > 0 ? playback.time / plan.duration : 0;
  scrubber.value = String(Math.round(progress * 1000));
  const pose = poseAtTime(plan, playback.time);
  const speed = pose ? ` · ${Math.abs(pose.velocity).toFixed(0)} in/s` : "";
  document.querySelector("#playback-time").textContent = `${playback.time.toFixed(1)} / ${plan.duration.toFixed(1)} s${speed}`;
  const button = document.querySelector("#play-path");
  button.textContent = playback.playing ? "❚❚" : "▶";
  button.setAttribute("aria-label", playback.playing ? "Pause path" : "Play full path");
  const heading = pose?.heading ?? documentState.robotStart.heading;
  document.querySelector("#heading-readout").textContent = `θ ${headingText(heading)}`;
  const robot = document.querySelector("#playback-robot");
  if (robot) {
    robot.style.display = plan.steps.length ? "" : "none";
    robot.classList.toggle("is-dragging", drag?.type === "playback");
  }
  if (pose && robot) robot.setAttribute("transform", `translate(${pose.x} ${FIELD_SIZE - pose.y}) rotate(${-pose.heading * 180 / Math.PI})`);
}

function stopPlayback(reset = false) {
  if (playback.frame) cancelAnimationFrame(playback.frame);
  playback.frame = 0; playback.playing = false;
  if (reset) playback.time = 0;
  if (document.querySelector("#play-path")) updatePlaybackUi();
}

function playbackFrame(now) {
  if (!playback.playing) return;
  const duration = routePlan().plan.duration;
  playback.time = Math.min(duration, playback.startedFrom + (now - playback.startedAt) / 1000);
  if (playback.time >= duration) playback.playing = false;
  updatePlaybackUi();
  if (playback.playing) playback.frame = requestAnimationFrame(playbackFrame);
  else playback.frame = 0;
}

function togglePlayback() {
  const { plan } = routePlan();
  const duration = plan.duration;
  if (plan.error) return notify(`Can't play: ${plan.error}`);
  if (!(duration > 0)) return notify("Add at least two waypoints to play the path");
  if (playback.playing) return stopPlayback(false);
  if (playback.time >= duration) playback.time = 0;
  playback.playing = true; playback.startedAt = performance.now(); playback.startedFrom = playback.time;
  updatePlaybackUi(); playback.frame = requestAnimationFrame(playbackFrame);
}

function restore(index) {
  if (index < 0 || index >= history.length) return;
  selectedControlId = null;
  stopPlayback(true);
  historyIndex = index;
  lastCoalesce = { key: null, at: 0 };
  documentState = restoreSnapshot(history[index], documentState);
  if (!documentState.paths.some((path) => path.id === activePathId)) activePathId = documentState.paths[0]?.id ?? null;
  if (!activePath()?.waypoints.some((point) => point.id === selectedPointId)) selectedPointId = activePath()?.waypoints[0]?.id ?? null;
  persist(); render();
}

function notify(message) {
  toast.textContent = message;
  toast.classList.add("is-visible");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("is-visible"), 2200);
}

function fieldMarkup() {
  // The field image is cropped to exactly the 144 x 144 inch foam Floor. Its
  // edges map directly to SVG 0..144, and this independent 24-inch grid must
  // continue to align with its six tile rows and columns.
  const grid = Array.from({ length: 7 }, (_, index) => index * 24).map((value) => `<path class="tile-grid" d="M${value} 0V144M0 ${value}H144"/>`).join("");
  const redVisible = documentState.showZones && documentState.alliance === "red" ? "" : " zone-hidden";
  const blueVisible = documentState.showZones && documentState.alliance === "blue" ? "" : " zone-hidden";
  return `<rect class="field-floor" width="144" height="144"/>
    <image class="field-image" href="assets/override-field.webp" x="0" y="0" width="144" height="144" preserveAspectRatio="none"/>
    ${grid}
    <polygon class="alliance-zone-red${redVisible}" points="0,0 48,72 72,96 144,144 0,144"/>
    <polygon class="alliance-zone-blue${blueVisible}" points="0,0 144,0 144,144 96,72 72,48"/>
    <rect class="field-wall" width="144" height="144"/>
    <text class="anchor-label" x="2.8" y="141">0,0</text><text class="anchor-label" x="134" y="141">+X</text><text class="anchor-label" x="2.8" y="7">+Y</text>`;
}

function pathMarkup(path) {
  if (!path?.waypoints.length) return "";
  normalizeSegmentDirections(path);
  const curves = [];
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    let d = `M${path.waypoints[index].x} ${FIELD_SIZE - path.waypoints[index].y}`;
    for (let step = 1; step <= 60; step += 1) {
      const sample = segmentPoint(path, index, step / 60);
      d += ` L${sample.x} ${FIELD_SIZE - sample.y}`;
    }
    curves.push(`<path class="path-shadow" d="${d}"/><path class="path-curve ${path.segmentReversed[index] ? "reversed" : ""}" d="${d}"/>`);
  }
  const bezierControls = (path.controlPoints ?? []).map((items, segment) => {
    if (!items || segment !== selectedSegment) return "";
    const polygon = [path.waypoints[segment], ...items, path.waypoints[segment + 1]];
    return `<polyline class="control-line bezier-guide" fill="none" points="${polygon.map(p => `${p.x},${FIELD_SIZE-p.y}`).join(" ")}"/>` + items.map((p, i) => `<g class="control-marker ${p.id === selectedControlId ? "selected" : ""}" data-control-id="${p.id}" transform="translate(${p.x} ${FIELD_SIZE-p.y})"><title>Control point C${i+1}: bends the curve</title><circle class="marker-hit" r="3"/><path class="control-diamond" d="M0 -1.6 1.6 0 0 1.6 -1.6 0Z"/><text class="anchor-label control-label" x="2.5" y="-2.5">C${i+1}</text></g>`).join("");
  }).join("");
  const controls = path.waypoints.map((point, index) => {
    const handleX = point.x + Math.cos(point.heading) * point.tangent / 5;
    const handleY = point.y + Math.sin(point.heading) * point.tangent / 5;
    const selected = !selectedControlId && point.id === selectedPointId;
    const legacy = (index > 0 && path.controlPoints[index-1] === null) || (index < path.waypoints.length-1 && path.controlPoints[index] === null);
    return `<g>${legacy ? `<line class="control-line" x1="${point.x}" y1="${FIELD_SIZE - point.y}" x2="${handleX}" y2="${FIELD_SIZE - handleY}"/>
      <circle class="handle" data-handle-id="${point.id}" cx="${handleX}" cy="${FIELD_SIZE - handleY}" r="2"/>` : ""}
      <circle class="anchor ${selected ? "selected" : ""}" data-point-id="${point.id}" cx="${point.x}" cy="${FIELD_SIZE - point.y}" r="2.1"/>
      <text class="anchor-label" x="${point.x + 3.8}" y="${FIELD_SIZE - point.y - 3}">P${index + 1}</text></g>`;
  }).join("");
  const robot = documentState.robot;
  const robotMarkup = `<g id="playback-robot" class="playback-robot" data-playback-marker="true" transform="translate(${path.waypoints[0].x} ${FIELD_SIZE - path.waypoints[0].y}) rotate(${-path.waypoints[0].heading * 180 / Math.PI})">
    <rect class="robot-box" x="${-robot.length / 2}" y="${-robot.width / 2}" width="${robot.length}" height="${robot.width}" rx="1"/>
    <line class="robot-nose" x1="${robot.length * .12}" y1="0" x2="${robot.length * .43}" y2="0"/>
  </g>`;
  // Keep route/control points above the player so they remain selectable when
  // the playback robot is parked directly on a point.
  return `${curves.join("")}${checksMarkup()}${robotMarkup}${bezierControls}${controls}`;
}

// Stops and problems drawn on the field, under the editable markers.
function checksMarkup() {
  const { plan, issues } = routePlan();
  const flagged = new Set(issues.filter((issue) => issue.action === "smooth").map((issue) => issue.waypointIndex));
  const stops = plan.stops.map((stop) => {
    const warn = flagged.has(stop.waypointIndex);
    const why = stop.reason === "corner" ? "sharp corner: stops and turns in place" : stop.reason === "direction-change" ? "changes drive direction" : "Bézier join: the library stops here";
    return `<g class="stop-marker${warn ? " is-warning" : ""}" transform="translate(${stop.x} ${FIELD_SIZE - stop.y})"><title>Robot stops at P${stop.waypointIndex + 1} · ${why}</title><path d="M-1.4 -3.4h2.8l2 2v2.8l-2 2h-2.8l-2-2v-2.8z"/></g>`;
  }).join("");
  const spots = issues.filter((issue) => !issue.action && Number.isFinite(issue.x)).map((issue) =>
    `<g class="issue-marker is-${issue.severity}" transform="translate(${issue.x} ${FIELD_SIZE - issue.y})"><title>${escapeHtml(issue.title)}: ${escapeHtml(issue.detail)}</title><circle r="4.2"/><text y="1.3">!</text></g>`).join("");
  return `<g class="checks-layer">${spots}${stops}</g>`;
}

function renderChecks() {
  const { issues } = routePlan();
  const plural = (count, word) => `${count} ${word}${count === 1 ? "" : "s"}`;
  const counts = ["error", "warning", "info"].map((severity) => issues.filter((issue) => issue.severity === severity).length);
  const summary = [counts[0] && plural(counts[0], "error"), counts[1] && plural(counts[1], "warning"), counts[2] && plural(counts[2], "note")].filter(Boolean);
  document.querySelector("#checks-count").textContent = summary.length ? summary.join(" · ") : "all clear";
  document.querySelector("#checks-list").innerHTML = issues.length
    ? issues.map((issue) => `<li class="check is-${issue.severity}">
        <button class="check-body" data-check-point="${issue.waypointIndex}"><b>${escapeHtml(issue.title)}</b><span>${escapeHtml(issue.detail)}</span></button>
        ${issue.action === "smooth" ? `<button class="check-fix" data-smooth-index="${issue.waypointIndex}">Smooth join</button>` : ""}
      </li>`).join("")
    : `<li class="check is-ok"><span>No stops to turn, walls or impossible curves. Planned with the same rules as the robot.</span></li>`;
}

function render() {
  const path = activePath();
  if (path) normalizeSegmentDirections(path);
  selectedSegment = Math.max(0, Math.min(selectedSegment, (path?.waypoints.length ?? 1)-2));
  svg.innerHTML = fieldMarkup() + pathMarkup(path);
  const start = robotStartPose(documentState);
  document.querySelectorAll("[data-start]").forEach(input => { const key = input.dataset.start; input.value = (key === "heading" ? toDisplayHeading(start.heading, prefs.headingMode) : start[key]).toFixed(2); });
  const startDerived = Boolean(documentState.paths[0]) && documentState.paths[0].waypoints.length > 1 && isHeadingDerived(documentState.paths[0], 0);
  const startHeadingInput = document.querySelector("#start-heading");
  startHeadingInput.disabled = startDerived;
  startHeadingInput.title = startDerived ? "Set by the first Bézier segment. Move its first control point to change it." : "";
  document.querySelector("#heading-mode").value = prefs.headingMode;
  const headingLabel = prefs.headingMode === "compass" ? "0° up · 90° right" : "0° right · 90° up";
  document.querySelectorAll("[data-heading-label]").forEach((node) => { node.textContent = startDerived ? `Facing follows the curve (${headingLabel})` : headingLabel; });
  document.querySelectorAll("[data-heading-short]").forEach((node) => { node.textContent = prefs.headingMode === "compass" ? "(CW from +Y)" : "(CCW from +X)"; });
  const primaryPoint = documentState.paths[0]?.waypoints[0];
  document.querySelector("#robot-start-section").classList.toggle("is-selected", activePathId === documentState.paths[0]?.id && selectedPointId === primaryPoint?.id);
  titleNode.textContent = documentState.title;
  pathList.innerHTML = documentState.paths.map((item, index) => `<div class="path-item ${item.id === activePathId ? "is-active" : ""}">
    <button class="path-select" data-path-id="${item.id}" aria-label="Edit ${escapeHtml(item.name)}"><i class="path-swatch" style="background:${escapeHtml(item.color)}"></i><span><strong>${escapeHtml(item.name)}</strong><small>${Math.max(0,item.waypoints.length-1)} segments · ${(item.controlPoints ?? []).flat().filter(Boolean).length} controls</small></span><b>${String(index + 1).padStart(2,"0")}</b></button>
    <div class="path-direction-status"><span>${item.segmentReversed.some(Boolean) ? (item.segmentReversed.every(Boolean) ? "All reverse" : "Mixed direction") : "All forward"}</span><b>⇄</b></div>
  </div>`).join("");
  const nameInput = document.querySelector("#route-name");
  nameInput.value = path?.name ?? ""; nameInput.disabled = !path;
  document.querySelector("#route-count").textContent = `${documentState.paths.length} routes`;
  renderControls(path);
  const point = selectedPoint();
  document.querySelector("#waypoint-inspector").hidden = Boolean(selectedControlId);
  document.querySelector("#waypoint-inspector").style.opacity = point ? "1" : ".4";
  Object.values(pointInputs).forEach((input) => { input.disabled = !point; });
  if (point) {
    pointInputs.x.value = point.x.toFixed(2); pointInputs.y.value = point.y.toFixed(2);
    pointInputs.heading.value = toDisplayHeading(point.heading, prefs.headingMode).toFixed(1); pointInputs.tangent.value = point.tangent.toFixed(1);
    const index = path.waypoints.findIndex((candidate) => candidate.id === point.id);
    document.querySelector("#waypoint-label").textContent = `Route point P${index+1}`;
    document.querySelector("#selection-index").textContent = `P${String(index + 1).padStart(2,"0")}`;
  }
  if (selectedControlId) document.querySelector("#selection-index").textContent = `Control C${(path?.controlPoints[selectedSegment] ?? []).findIndex(p => p.id === selectedControlId)+1}`;
  document.querySelector("#waypoint-count").textContent = `${path?.waypoints.length ?? 0} ${(path?.waypoints.length ?? 0) === 1 ? "point" : "points"}`;
  waypointList.innerHTML = (path?.waypoints ?? []).map((item, index) => {
    const heading = path.waypoints.length > 1 ? anchorHeading(path, index) : item.heading;
    return `<div class="waypoint-row ${item.id === selectedPointId ? "is-selected" : ""}">
    <button class="waypoint-select" data-select-point-id="${item.id}" aria-label="Select anchor ${index + 1}">P${String(index + 1).padStart(2,"0")}</button>
    <label><span class="visually-hidden">Anchor ${index + 1} X coordinate in inches</span><input data-coordinate-point-id="${item.id}" data-coordinate="x" type="number" min="0" max="144" step="0.25" value="${item.x.toFixed(2)}" /></label>
    <label><span class="visually-hidden">Anchor ${index + 1} Y coordinate in inches</span><input data-coordinate-point-id="${item.id}" data-coordinate="y" type="number" min="0" max="144" step="0.25" value="${item.y.toFixed(2)}" /></label>
    <output class="waypoint-heading" aria-label="Route point ${index + 1} heading in degrees" title="Robot heading at P${index + 1}${isHeadingDerived(path, index) ? " (set by the curve)" : ""} · ${headingLabel}">${headingText(heading)}</output>
  </div>`;
  }).join("");
  segmentList.innerHTML = (path?.segmentReversed ?? []).map((reversed, index) => `<div class="segment-direction ${index === selectedSegment ? "is-current" : ""}">
    <button class="segment-select-button" data-segment-index="${index}" aria-pressed="${index === selectedSegment}">
      <span><b>Segment ${index+1} · P${index+1} → P${index+2}</b><small>${path.controlPoints[index] === null ? "Legacy spline" : `${path.controlPoints[index].length} controls`}</small></span>
    </button>
    <div class="segment-direction-toggle" role="group" aria-label="Drive direction for segment ${index+1}">
      <button class="${reversed ? "" : "is-active"}" data-direction-index="${index}" data-direction-value="false" aria-pressed="${!reversed}">→ Forward</button>
      <button class="${reversed ? "is-active" : ""}" data-direction-index="${index}" data-direction-value="true" aria-pressed="${reversed}">← Reverse</button>
    </div>
  </div>`).join("");
  const { plan } = routePlan();
  const stopCount = plan.stops.length;
  document.querySelector("#path-summary").textContent = plan.error
    ? `${path?.waypoints.length ?? 0} anchors · can't plan`
    : `${path?.waypoints.length ?? 0} anchors · ${plan.length.toFixed(1)} in · ${plan.duration.toFixed(2)} s · ${stopCount} stop${stopCount === 1 ? "" : "s"}`;
  renderChecks();
  const deleteButton = document.querySelector("#delete-path");
  deleteButton.textContent = performance.now() < deleteArmedUntil ? `Click again to delete “${path?.name ?? "route"}”` : "Delete active route";
  deleteButton.classList.toggle("is-armed", performance.now() < deleteArmedUntil);
  const directions = path?.segmentReversed ?? [];
  document.querySelector("#direction-summary").textContent = directions.some(Boolean) ? (directions.every(Boolean) ? "All reverse" : "Mixed") : "All forward";
  document.querySelector("#snap-step").value = String(documentState.snapStep);
  document.querySelector("#snap-toggle").checked = documentState.snap;
  document.querySelector("#zone-toggle").checked = documentState.showZones;
  robotInputs.forEach((input) => { input.value = Number(documentState.robot[input.dataset.robot]).toFixed(1); });
  document.querySelectorAll("[data-alliance]").forEach((button) => button.classList.toggle("is-active", button.dataset.alliance === documentState.alliance));
  document.querySelector('[data-action="undo"]').disabled = historyIndex === 0;
  document.querySelector('[data-action="redo"]').disabled = historyIndex === history.length - 1;
  renderSaveState();
  updatePlaybackUi();
}

function renderControls(path) {
  const segments = path?.segmentReversed ?? [];
  const controls = path?.controlPoints?.[selectedSegment];
  const legacy = controls === null;
  document.querySelector("#segment-select").innerHTML = segments.map((_, i) => `<option value="${i}" ${i === selectedSegment ? "selected" : ""}>Segment ${i+1} · P${i+1} → P${i+2}</option>`).join("");
  document.querySelector("#segment-select").disabled = !segments.length;
  document.querySelector("#segment-direction").value = String(segments[selectedSegment] ?? false);
  document.querySelector("#segment-direction").disabled = !segments.length;
  document.querySelector("#geometry-mode").value = legacy ? "hermite" : "bezier";
  document.querySelector("#geometry-mode").disabled = !segments.length;
  document.querySelector("#control-count").value = controls?.length ?? 0;
  document.querySelector("#control-count").disabled = !segments.length || legacy;
  document.querySelector('[data-action="add-control"]').disabled = !segments.length;
  document.querySelector("#control-help").textContent = !segments.length ? "Add two route points to create a segment." : legacy ? "Saved spline geometry. Choose Bézier to edit individual control points." : controls.length ? "Drag a blue C diamond, or edit its X / Y below." : "Straight line between route points. Add control points to bend it.";
  controlPanel.innerHTML = (controls ?? []).map((point, i) => `<div class="control-row ${point.id === selectedControlId ? "is-selected" : ""}">
    <button data-select-control="${point.id}" aria-label="Select control point ${i+1}">C${i+1}</button>
    <label><span class="visually-hidden">Control ${i+1} X</span><input type="number" min="0" max="144" step="0.1" data-control-coordinate="x" data-control-index="${i}" value="${point.x.toFixed(2)}" /></label>
    <label><span class="visually-hidden">Control ${i+1} Y</span><input type="number" min="0" max="144" step="0.1" data-control-coordinate="y" data-control-index="${i}" value="${point.y.toFixed(2)}" /></label>
    <button data-remove-control="${i}" aria-label="Remove control point ${i+1}">×</button>
  </div>`).join("");
  const pointIndex = path?.waypoints.findIndex(p => p.id === selectedPointId) ?? -1;
  const usesHermite = pointIndex >= 0 && ((pointIndex > 0 && path.controlPoints[pointIndex-1] === null) || path.controlPoints[pointIndex] === null);
  document.querySelector("#heading-fields").hidden = !usesHermite;
}

document.querySelector("#route-name").addEventListener("change", event => {
  if (activePath()) { activePath().name = event.target.value.trim() || "Untitled route"; commit("Route renamed"); }
});
document.querySelector("#segment-select").addEventListener("change", event => { selectedSegment = Number(event.target.value); selectedControlId = null; render(); });
document.querySelector("#segment-direction").addEventListener("change", event => {
  activePath().segmentReversed[selectedSegment] = event.target.value === "true"; commit("Drive direction updated");
});
document.querySelector("#geometry-mode").addEventListener("change", event => {
  const path = activePath(); if (!path) return;
  path.controlPoints[selectedSegment] = event.target.value === "hermite" ? null : [];
  selectedControlId = null; commit("Segment geometry updated");
});
document.querySelector("#control-count").addEventListener("change", event => {
  const count = Number(event.target.value);
  if (!event.target.value || !Number.isInteger(count) || count < 0) return render();
  setControlCount(activePath(), selectedSegment, count); selectedControlId = null; commit("Control point count updated");
});
controlPanel.addEventListener("change", event => {
  const input = event.target.closest("[data-control-coordinate]"); if (!input) return;
  const value = Number(input.value); if (!input.value || !Number.isFinite(value)) return render();
  const point = activePath().controlPoints[selectedSegment][Number(input.dataset.controlIndex)];
  point[input.dataset.controlCoordinate] = clamp(value); selectedControlId = point.id; commit("Control point updated");
});

function svgCoordinates(event) {
  const point = svg.createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
  const local = point.matrixTransform(svg.getScreenCTM().inverse());
  return { x: clamp(local.x), y: clamp(FIELD_SIZE - local.y) };
}

// Where the toolbar/A key puts a new point: a tile ahead of the last point in
// the direction it faces, pulled back inside the field.
function pointAhead(path) {
  const last = path.waypoints.at(-1);
  if (!last) return { x: 72, y: 72 };
  const heading = path.waypoints.length > 1 ? anchorHeading(path, path.waypoints.length - 1) : last.heading;
  const margin = Math.max(documentState.robot.length, documentState.robot.width) / 2;
  const inside = (value) => clamp(value, margin, FIELD_SIZE - margin);
  const ahead = { x: inside(last.x + Math.cos(heading) * NEW_POINT_STEP_IN), y: inside(last.y + Math.sin(heading) * NEW_POINT_STEP_IN) };
  if (Math.hypot(ahead.x - last.x, ahead.y - last.y) > 1) return ahead;
  return { x: last.x + (72 - last.x) / 2, y: last.y + (72 - last.y) / 2 };
}

// The segment and curve parameter nearest `at`, if it is close to the drawn path.
function nearestOnPath(path, at) {
  let best = null;
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    for (let step = 1; step < 100; step += 1) {
      const t = step / 100;
      const point = segmentPoint(path, index, t);
      const distance = Math.hypot(point.x - at.x, point.y - at.y);
      if (!best || distance < best.distance) best = { index, t, distance };
    }
  }
  return best && best.distance <= INSERT_HIT_IN ? best : null;
}

// Double-click / long-press: insert on the path when aimed at it, else append.
function addPointAt(at) {
  const path = activePath();
  if (!path) return;
  normalizeSegmentDirections(path);
  const hit = nearestOnPath(path, at);
  if (!hit) return addPoint(at);
  const { path: split, point } = splitSegment(path, hit.index, hit.t);
  Object.assign(path, split);
  selectedPointId = point.id; selectedControlId = null; selectedSegment = hit.index + 1;
  commit(`Route point inserted as P${hit.index + 2}`);
}

function addPoint(at = pointAhead(activePath() ?? { waypoints: [] })) {
  const path = activePath();
  if (!path) return;
  selectedControlId = null;
  const previous = path.waypoints.at(-1);
  const heading = previous ? Math.atan2(at.y - previous.y, at.x - previous.x) : 0;
  const point = { id: uid(), x: snap(at.x), y: snap(at.y), heading, tangent: previous ? Math.max(18, Math.hypot(at.x - previous.x, at.y - previous.y)) : 30 };
  if (previous && Math.hypot(point.x - previous.x, point.y - previous.y) < 0.5) return notify("That's on top of the last point");
  path.waypoints.push(point);
  if (previous) {
    const segments = path.segmentReversed.length;
    // Carry on from the previous segment: same drive direction, and a first
    // control along its end tangent so the new join is smooth, not a corner.
    const reversed = segments > 0 ? path.segmentReversed[segments - 1] : false;
    const controls = [];
    if (segments > 0) {
      const tangent = segmentTangent(path, segments - 1, 1);
      const reach = Math.hypot(point.x - previous.x, point.y - previous.y) / 3;
      controls.push({ id: uid(), x: clamp(previous.x + Math.cos(tangent) * reach), y: clamp(previous.y + Math.sin(tangent) * reach) });
    }
    path.segmentReversed.push(reversed); path.controlPoints.push(controls);
  }
  selectedPointId = point.id; commit("Waypoint added");
}

function deletePoint() {
  const path = activePath();
  if (selectedControlId && path) {
    const items = path.controlPoints[selectedSegment];
    const index = items?.findIndex(p => p.id === selectedControlId) ?? -1;
    if (index >= 0) { items.splice(index,1); selectedControlId = null; return commit("Control point deleted"); }
  }
  if (!path || !selectedPointId) return;
  const index = path.waypoints.findIndex((point) => point.id === selectedPointId);
  if (index < 0) return;
  path.waypoints.splice(index, 1);
  if (index === 0) { path.segmentReversed.shift(); path.controlPoints.shift(); }
  else {
    const removed = Math.min(index, path.segmentReversed.length - 1);
    path.segmentReversed.splice(removed, 1); path.controlPoints.splice(removed, 1);
    // Deleting an interior anchor joins its neighbours with a new straight segment.
    if (index < path.waypoints.length) path.controlPoints[index-1] = [];
  }
  selectedPointId = path.waypoints[Math.min(index, path.waypoints.length - 1)]?.id ?? null;
  commit("Waypoint deleted");
}

function transformPath(mode) {
  const path = activePath(); if (!path) return;
  path.waypoints = path.waypoints.map((point) => mirrorWaypoint(point, mode));
  path.controlPoints = path.controlPoints.map(items => items?.map(p => { const mirrored = mirrorWaypoint({ ...p, heading:0 }, mode); return { id:p.id, x:mirrored.x, y:mirrored.y }; }) ?? null);
  commit(mode === "quadrant" ? "Mirrored to the opposite same-alliance quadrant" : "Path mirrored");
}

function mirroredAllianceCopy() {
  const original = activePath(); if (!original) return;
  const copy = structuredClone(original);
  copy.id = uid();
  copy.name = /\(blue\)$/i.test(copy.name) ? copy.name.replace(/\(blue\)$/i, "(red)") : /\(red\)$/i.test(copy.name) ? copy.name.replace(/\(red\)$/i, "(blue)") : `${copy.name} (${documentState.alliance === "blue" ? "red" : "blue"})`;
  copy.waypoints = copy.waypoints.map((point) => ({ ...mirrorWaypoint(point, "alliance"), id: uid() }));
  copy.controlPoints = copy.controlPoints.map((items) => items?.map((p) => { const mirrored = mirrorWaypoint({ ...p, heading:0 }, "alliance"); return { id:uid(), x:mirrored.x, y:mirrored.y }; }) ?? null);
  documentState.paths.push(copy); activePathId = copy.id; selectedPointId = copy.waypoints[0]?.id ?? null; selectedControlId = null;
  commit(`Created “${copy.name}” for the other alliance`);
}

function smoothAt(index) {
  const path = activePath(); if (!path) return;
  Object.assign(path, smoothJoin(path, index));
  selectedPointId = path.waypoints[index]?.id ?? selectedPointId; selectedControlId = null;
  commit(`Join at P${index + 1} smoothed`);
}

function nudgeSelection(dx, dy) {
  const path = activePath(); if (!path) return;
  const target = selectedControlId
    ? path.controlPoints[selectedSegment]?.find((p) => p.id === selectedControlId)
    : selectedPoint();
  if (!target) return;
  target.x = clamp(target.x + dx); target.y = clamp(target.y + dy);
  commit(null, { coalesce: `nudge:${selectedControlId ?? selectedPointId}` });
}

function newPath() {
  selectedSegment = 0; selectedControlId = null;
  const number = documentState.paths.length + 1;
  const path = { id:uid(), name:`Route ${number}`, color:"#171715", segmentReversed:[false], controlPoints:[[]], waypoints:[
    { id:uid(), ...documentState.robotStart, tangent:30 },
    { id:uid(), x:clamp(documentState.robotStart.x + (documentState.robotStart.x > 108 ? -36 : 36)), y:documentState.robotStart.y, heading:documentState.robotStart.heading, tangent:30 }
  ] };
  documentState.paths.push(path); activePathId = path.id; selectedPointId = path.waypoints[0].id; commit("New path created");
}

function download(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const link = Object.assign(document.createElement("a"), { href:url, download:name }); link.click();
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

function safeFileName(value) { return value.trim().toLowerCase().replace(/[^a-z0-9]+/g,"-").replace(/^-|-$/g,"") || "vantage-path"; }

function runAction(action) {
  if (action === "select-start") {
    const primary = documentState.paths[0], point = primary?.waypoints[0];
    if (!point) return notify("Add a route point first");
    activePathId = primary.id; selectedPointId = point.id; selectedSegment = 0; selectedControlId = null;
    stopPlayback(true); render(); return;
  }
  if (action === "duplicate-path") {
    const original = activePath(); if (!original) return;
    const copy = structuredClone(original); copy.id = uid(); copy.name += " copy";
    copy.waypoints.forEach(p => p.id = uid()); copy.controlPoints.forEach(items => items?.forEach(p => p.id = uid()));
    documentState.paths.push(copy); activePathId = copy.id; selectedPointId = copy.waypoints[0]?.id; selectedControlId = null;
    return commit("Route duplicated");
  }
  if (action === "add-control") {
    const path = activePath(); if (!path) return;
    setControlCount(path, selectedSegment, (path.controlPoints[selectedSegment]?.length ?? 0)+1);
    selectedControlId = path.controlPoints[selectedSegment]?.at(-1)?.id;
    return commit("Control point added");
  }
  if (action === "new-path") return newPath();
  if (action === "add-point") return addPoint();
  if (action === "delete-point") return deletePoint();
  if (action === "undo") return restore(historyIndex - 1);
  if (action === "redo") return restore(historyIndex + 1);
  if (action === "mirror-alliance-copy") return mirroredAllianceCopy();
  if (action === "mirror-quadrant") return transformPath("quadrant");
  if (action === "mirror-left-right") return transformPath("left-right");
  if (action === "mirror-bottom-top") return transformPath("bottom-top");
  if (action === "reverse-order") { const path=activePath(); if(path){path.waypoints=reverseWaypoints(path.waypoints);path.segmentReversed.reverse();path.controlPoints.reverse().forEach(items => items?.reverse());commit("Path order reversed");} return; }
  if (action === "all-forward" || action === "all-reverse") {
    const path = activePath(); if (!path) return;
    normalizeSegmentDirections(path);
    path.segmentReversed.fill(action === "all-reverse");
    commit(action === "all-reverse" ? "Entire route set to reverse" : "Entire route set to forward"); return;
  }
  if (action === "play-path") return togglePlayback();
  if (action === "delete-path") {
    if (!activePath()) return;
    if (performance.now() > deleteArmedUntil) {
      deleteArmedUntil = performance.now() + DELETE_CONFIRM_MS;
      render();
      setTimeout(render, DELETE_CONFIRM_MS + 50);
      return;
    }
    deleteArmedUntil = 0;
    documentState.paths = documentState.paths.filter((path) => path.id !== activePathId);
    if (!documentState.paths.length) { newPath(); return; }
    activePathId = documentState.paths[0].id; selectedPointId = activePath().waypoints[0]?.id ?? null; commit("Path deleted"); return;
  }
  if (action === "open") return fileInput.click();
  if (action === "save") { download(`${safeFileName(documentState.title)}.vpath`, JSON.stringify(documentState,null,2), "application/json"); notify("VantagePath file saved"); return; }
  if (action === "export") {
    const frame = document.querySelector("#export-frame").value;
    const variable = document.querySelector("#variable-name").value;
    download(`${safeFileName(documentState.title)}.hpp`, cppExport(documentState, variable, frame), "text/x-c++hdr"); notify("C++ trajectory exported"); return;
  }
  if (action === "home") notify("VantagePath Studio · editor ready");
}

document.addEventListener("click", (event) => {
  const action = event.target.closest("[data-action]")?.dataset.action;
  if (action) runAction(action);
  const remove = event.target.closest("[data-remove-control]");
  if (remove) { activePath().controlPoints[selectedSegment].splice(Number(remove.dataset.removeControl),1); selectedControlId = null; commit("Control point removed"); }
  const control = event.target.closest("[data-select-control]");
  if (control) { selectedControlId = control.dataset.selectControl; render(); }
  const pathButton = event.target.closest("[data-path-id]");
  if (pathButton) { stopPlayback(true); activePathId = pathButton.dataset.pathId; selectedSegment = 0; selectedControlId = null; selectedPointId = activePath()?.waypoints[0]?.id ?? null; render(); }
  const segmentButton = event.target.closest("[data-segment-index]");
  if (segmentButton) {
    const path = activePath(); const index = Number(segmentButton.dataset.segmentIndex);
    if (path?.segmentReversed[index] !== undefined) { selectedSegment = index; selectedControlId = null; render(); }
  }
  const directionButton = event.target.closest("[data-direction-index]");
  if (directionButton) {
    const path = activePath(); const index = Number(directionButton.dataset.directionIndex);
    if (path?.segmentReversed[index] !== undefined) {
      selectedSegment = index; selectedControlId = null;
      path.segmentReversed[index] = directionButton.dataset.directionValue === "true";
      commit(`Segment ${index+1} set to ${path.segmentReversed[index] ? "reverse" : "forward"}`);
    }
  }
  const pointButton = event.target.closest("[data-select-point-id]");
  if (pointButton) { selectedControlId = null; selectedPointId = pointButton.dataset.selectPointId; stopPlayback(true); render(); }
  const alliance = event.target.closest("[data-alliance]")?.dataset.alliance;
  if (alliance) { setView("alliance", alliance); notify(`${alliance[0].toUpperCase()+alliance.slice(1)} field view selected`); }
  const smooth = event.target.closest("[data-smooth-index]");
  if (smooth) smoothAt(Number(smooth.dataset.smoothIndex));
  const check = event.target.closest("[data-check-point]");
  if (check) {
    const point = activePath()?.waypoints[Number(check.dataset.checkPoint)];
    if (point) { selectedPointId = point.id; selectedControlId = null; stopPlayback(true); render(); }
  }
});

waypointList.addEventListener("change", (event) => {
  const input = event.target.closest("[data-coordinate-point-id]");
  if (!input) return;
  const point = activePath()?.waypoints.find((item) => item.id === input.dataset.coordinatePointId);
  const value = Number(input.value);
  if (!point || !Number.isFinite(value)) return render();
  point[input.dataset.coordinate] = clamp(value);
  selectedControlId = null;
  selectedPointId = point.id;
  commit("Waypoint coordinates updated");
});

svg.addEventListener("pointerdown", event => {
  const target = event.target.closest("[data-point-id],[data-handle-id],[data-control-id],[data-playback-marker]");
  if (!target) return;
  if (target.dataset.playbackMarker) {
    stopPlayback(false);
    const { plan } = routePlan();
    const samples = routeSamples(plan);
    if (samples.length < 2 || !(plan.duration > 0)) return;
    drag = { type:"playback", pointerId:event.pointerId, samples, plan };
    svg.setPointerCapture(event.pointerId);
    playback.time = timeAtDistance(plan, nearestPathDistance(samples, svgCoordinates(event)));
    updatePlaybackUi(); event.preventDefault(); return;
  }
  stopPlayback(true);
  const { pointId, handleId, controlId } = target.dataset;
  selectedControlId = controlId ?? null;
  if (pointId || handleId) selectedPointId = pointId ?? handleId;
  const point = controlId ? activePath().controlPoints[selectedSegment].find(p => p.id === controlId) : selectedPoint();
  const type = controlId ? "control" : handleId ? "handle" : "point";
  const origin = type === "handle" ? { x:point.x+Math.cos(point.heading)*point.tangent/5, y:point.y+Math.sin(point.heading)*point.tangent/5 } : { x:point.x, y:point.y };
  drag = { type, point, pointerId:event.pointerId, pointerStart:svgCoordinates(event), origin, before:JSON.stringify(documentState) };
  svg.setPointerCapture(event.pointerId); render(); event.preventDefault();
});

svg.addEventListener("pointermove", event => {
  const at = svgCoordinates(event);
  document.querySelector("#coordinate-readout").textContent = `X ${at.x.toFixed(2)} · Y ${at.y.toFixed(2)}`;
  if (!drag || drag.pointerId !== event.pointerId) return;
  if (drag.type === "playback") {
    playback.time = timeAtDistance(drag.plan, nearestPathDistance(drag.samples, at));
    updatePlaybackUi(); return;
  }
  const step = documentState.snap && !event.shiftKey ? documentState.snapStep : 0;
  const moved = dragPosition(drag.origin, drag.pointerStart, at, step, event.altKey);
  const point = drag.point;
  if (drag.type === "handle") {
    point.heading = wrapRadians(Math.atan2(moved.y-point.y,moved.x-point.x));
    point.tangent = Math.max(3,Math.hypot(moved.x-point.x,moved.y-point.y)*5);
  } else { point.x = moved.x; point.y = moved.y; }
  render();
});

svg.addEventListener("pointerup", event => {
  if (drag?.pointerId !== event.pointerId) return;
  if (drag.type === "playback") { drag = null; updatePlaybackUi(); return; }
  const primaryPoint = documentState.paths[0]?.waypoints[0];
  const message = drag.type === "control" ? "Control point updated" : drag.point === primaryPoint ? "Robot start / P1 updated" : "Route point updated";
  drag = null; commit(message);
});
svg.addEventListener("pointercancel", () => {
  if (!drag) return;
  if (drag.type === "playback") { drag = null; updatePlaybackUi(); return; }
  documentState = JSON.parse(drag.before); drag = null; render();
});
const MARKER_SELECTOR = "[data-point-id],[data-handle-id],[data-control-id],[data-playback-marker]";
svg.addEventListener("dblclick", (event) => { if (!event.target.closest(MARKER_SELECTOR)) addPointAt(svgCoordinates(event)); });

// Touch has no double-click: press and hold on empty field to add a point.
svg.addEventListener("pointerdown", (event) => {
  if (event.pointerType === "mouse" || event.target.closest(MARKER_SELECTOR)) return;
  const at = svgCoordinates(event);
  clearTimeout(longPress?.timer);
  longPress = { pointerId: event.pointerId, at, timer: setTimeout(() => { longPress = null; addPointAt(at); }, LONG_PRESS_MS) };
});
const cancelLongPress = (event) => {
  if (!longPress || longPress.pointerId !== event.pointerId) return;
  if (event.type === "pointermove") {
    const at = svgCoordinates(event);
    if (Math.hypot(at.x - longPress.at.x, at.y - longPress.at.y) < LONG_PRESS_SLOP_IN) return;
  }
  clearTimeout(longPress.timer); longPress = null;
};
["pointermove", "pointerup", "pointercancel"].forEach((type) => svg.addEventListener(type, cancelLongPress));

for (const [key,input] of Object.entries(pointInputs)) input.addEventListener("change", () => {
  const point=selectedPoint(); if(!point) return;
  const value=Number(input.value); if(!Number.isFinite(value)) return render();
  if(key === "heading") point.heading=fromDisplayHeading(value, prefs.headingMode);
  else if(key === "tangent") point.tangent=Math.max(1,value);
  else point[key]=clamp(value);
  commit("Waypoint values updated");
});

robotInputs.forEach((input) => input.addEventListener("change", () => {
  const value = Number(input.value);
  if (!Number.isFinite(value)) return render();
  documentState.robot[input.dataset.robot] = value;
  validateDocument(documentState);
  commit("Robot setup updated");
}));

scrubber.addEventListener("input", () => {
  stopPlayback(false);
  playback.time = routePlan().plan.duration * Number(scrubber.value) / 1000;
  updatePlaybackUi();
});

document.querySelectorAll("[data-start]").forEach(input => input.addEventListener("change", () => {
  const value = Number(input.value); if (!input.value || !Number.isFinite(value)) return render();
  const point = documentState.paths[0]?.waypoints[0]; if (!point) return notify("Add a route point first");
  const key = input.dataset.start;
  point[key] = key === "heading" ? fromDisplayHeading(value, prefs.headingMode) : clamp(value);
  activePathId = documentState.paths[0].id; selectedPointId = point.id; selectedControlId = null;
  commit("Robot start / P1 updated");
}));
document.querySelector("#snap-step").addEventListener("change", event => setView("snapStep", Number(event.target.value)));
document.querySelector("#snap-toggle").addEventListener("change", (event) => setView("snap", event.target.checked));
document.querySelector("#zone-toggle").addEventListener("change", (event) => setView("showZones", event.target.checked));
document.querySelector("#heading-mode").addEventListener("change", (event) => {
  prefs = { ...prefs, headingMode: event.target.value === "compass" ? "compass" : "math" };
  savePrefs(); render();
});
titleNode.addEventListener("blur", () => { documentState.title=titleNode.textContent.trim() || "Untitled path";commit(); });
titleNode.addEventListener("keydown", (event) => { if(event.key === "Enter"){event.preventDefault();titleNode.blur();} });
fileInput.addEventListener("change", async () => {
  try {
    if (!fileInput.files.length) return;
    const loaded=validateDocument(JSON.parse(await fileInput.files[0].text()));
    stopPlayback(true); selectedSegment = 0; selectedControlId = null; documentState=loaded;
    activePathId=loaded.paths[0]?.id??null;selectedPointId=loaded.paths[0]?.waypoints[0]?.id??null;history=[historySnapshot(loaded)];historyIndex=0;persist();render();notify("VantagePath file opened");
  } catch(error) { notify(error instanceof Error ? error.message : "Could not open that file"); }
  fileInput.value="";
});

window.addEventListener("keydown", (event) => {
  const editing=/INPUT|SELECT/.test(event.target.tagName)||event.target.isContentEditable;if(editing)return;
  const command=event.metaKey||event.ctrlKey;
  if(command&&event.key.toLowerCase()==="z"){event.preventDefault();restore(historyIndex+(event.shiftKey?1:-1));}
  else if(event.key==="Delete"||event.key==="Backspace"){event.preventDefault();deletePoint();}
  else if(event.key.toLowerCase()==="a"&&!command){event.preventDefault();addPoint();}
  else if(event.key.startsWith("Arrow")&&!command){
    event.preventDefault();
    const step = event.altKey ? 0.25 : event.shiftKey ? 6 : 1;
    const [dx, dy] = { ArrowLeft:[-step,0], ArrowRight:[step,0], ArrowUp:[0,step], ArrowDown:[0,-step] }[event.key];
    nudgeSelection(dx, dy);
  }
});

render();
if (startupMessage) notify(startupMessage);
