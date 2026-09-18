import { dragPosition, FIELD_SIZE, clamp, cppExport, estimateLength, makeDocument, mirrorWaypoint, motionProfile, nearestPathDistance, normalizeSegmentDirections, profileDistance, profileTimeAtDistance, robotStartPose, segmentPoint, setControlCount, reverseWaypoints, validateDocument, wrapRadians } from "./model.js";

const STORAGE_KEY = "vantagepath-studio-v1";
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

let documentState = loadLocal();
let activePathId = documentState.paths[0]?.id ?? null;
let selectedPointId = documentState.paths[0]?.waypoints[0]?.id ?? null;
let history = [JSON.stringify(documentState)];
let historyIndex = 0;
let drag = null;
let toastTimer = 0;
let playback = { playing:false, time:0, startedAt:0, startedFrom:0, frame:0 };

function activePath() { return documentState.paths.find((path) => path.id === activePathId) ?? documentState.paths[0]; }
function selectedPoint() { return activePath()?.waypoints.find((point) => point.id === selectedPointId) ?? null; }
function escapeHtml(value) { return String(value).replace(/[&<>'"]/g, (character) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[character])); }
function uid() { return crypto.randomUUID(); }
function snap(value, bypass = false) { return documentState.snap && !bypass ? Math.round(value / documentState.snapStep) * documentState.snapStep : value; }

function loadLocal() {
  try { return validateDocument(JSON.parse(localStorage.getItem(STORAGE_KEY))) } catch { return makeDocument(); }
}

function persist() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(documentState));
}

function commit(message) {
  stopPlayback(true);
  documentState.robotStart = robotStartPose(documentState);
  const serial = JSON.stringify(documentState);
  if (history[historyIndex] !== serial) {
    history = history.slice(0, historyIndex + 1);
    history.push(serial);
    historyIndex += 1;
  }
  persist(); render();
  if (message) notify(message);
}

function pathSamples(path, subdivisions = 50) {
  if (!path?.waypoints.length) return [];
  normalizeSegmentDirections(path);
  const samples = [{ x:path.waypoints[0].x, y:path.waypoints[0].y, distance:0, heading:path.waypoints[0].heading, segmentIndex:0 }];
  if (path.waypoints.length > 1 && Array.isArray(path.controlPoints[0])) {
    const next = segmentPoint(path, 0, 0.0001);
    samples[0].heading = Math.atan2(next.y-samples[0].y, next.x-samples[0].x);
  }
  let distance = 0;
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    let previous = samples.at(-1);
    for (let step = 1; step <= subdivisions; step += 1) {
      const point = segmentPoint(path, index, step / subdivisions);
      const segment = Math.hypot(point.x - previous.x, point.y - previous.y);
      distance += segment;
      const heading = segment > 1e-6 ? Math.atan2(point.y - previous.y, point.x - previous.x) : previous.heading;
      const sample = { ...point, distance, heading, segmentIndex:index };
      samples.push(sample); previous = sample;
    }
  }
  return samples;
}

function playbackData() {
  const samples = pathSamples(activePath());
  return { samples, profile:motionProfile(samples.at(-1)?.distance ?? 0, documentState.robot) };
}

function poseAtDistance(samples, distance, path) {
  if (!samples.length) return null;
  let upperIndex = samples.findIndex((sample) => sample.distance >= distance);
  if (upperIndex < 0) upperIndex = samples.length - 1;
  const upper = samples[upperIndex];
  const lower = samples[Math.max(0, upperIndex - 1)];
  const span = upper.distance - lower.distance;
  const ratio = span > 1e-6 ? (distance - lower.distance) / span : 0;
  const waypoints = path?.waypoints ?? [];
  // The nose follows the direction of travel, flipped 180° on reversed segments
  // (the robot backs along the curve). At the exact anchors the authored nose
  // wins so the display never snaps at a forward/reverse cusp.
  let heading = upper.heading;
  if (path?.segmentReversed?.[upper.segmentIndex]) heading = wrapRadians(heading + Math.PI);
  if (upperIndex <= 0 && !Array.isArray(path?.controlPoints?.[0])) heading = waypoints[0]?.heading ?? heading;
  else if (upperIndex === samples.length - 1 && !Array.isArray(path?.controlPoints?.at(-1))) heading = waypoints.at(-1)?.heading ?? heading;
  return { x:lower.x + (upper.x - lower.x) * ratio, y:lower.y + (upper.y - lower.y) * ratio, heading };
}

function updatePlaybackUi() {
  const { samples, profile } = playbackData();
  playback.time = Math.min(playback.time, profile.duration);
  const progress = profile.duration > 0 ? playback.time / profile.duration : 0;
  scrubber.value = String(Math.round(progress * 1000));
  document.querySelector("#playback-time").textContent = `${playback.time.toFixed(1)} / ${profile.duration.toFixed(1)} s`;
  const button = document.querySelector("#play-path");
  button.textContent = playback.playing ? "❚❚" : "▶";
  button.setAttribute("aria-label", playback.playing ? "Pause path" : "Play full path");
  const pose = poseAtDistance(samples, profileDistance(profile, playback.time), activePath());
  const robot = document.querySelector("#playback-robot");
  if (robot) {
    robot.style.display = samples.length > 1 ? "" : "none";
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
  const duration = playbackData().profile.duration;
  playback.time = Math.min(duration, playback.startedFrom + (now - playback.startedAt) / 1000);
  if (playback.time >= duration) playback.playing = false;
  updatePlaybackUi();
  if (playback.playing) playback.frame = requestAnimationFrame(playbackFrame);
  else playback.frame = 0;
}

function togglePlayback() {
  const duration = playbackData().profile.duration;
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
  documentState = JSON.parse(history[index]);
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
  return `${curves.join("")}${robotMarkup}${bezierControls}${controls}`;
}

function render() {
  const path = activePath();
  if (path) normalizeSegmentDirections(path);
  selectedSegment = Math.max(0, Math.min(selectedSegment, (path?.waypoints.length ?? 1)-2));
  svg.innerHTML = fieldMarkup() + pathMarkup(path);
  const start = robotStartPose(documentState);
  document.querySelectorAll("[data-start]").forEach(input => { const key = input.dataset.start; input.value = (key === "heading" ? start.heading * 180 / Math.PI : start[key]).toFixed(2); });
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
    pointInputs.heading.value = (point.heading * 180 / Math.PI).toFixed(1); pointInputs.tangent.value = point.tangent.toFixed(1);
    const index = path.waypoints.findIndex((candidate) => candidate.id === point.id);
    document.querySelector("#waypoint-label").textContent = `Route point P${index+1}`;
    document.querySelector("#selection-index").textContent = `P${String(index + 1).padStart(2,"0")}`;
  }
  if (selectedControlId) document.querySelector("#selection-index").textContent = `Control C${(path?.controlPoints[selectedSegment] ?? []).findIndex(p => p.id === selectedControlId)+1}`;
  document.querySelector("#waypoint-count").textContent = `${path?.waypoints.length ?? 0} ${(path?.waypoints.length ?? 0) === 1 ? "point" : "points"}`;
  waypointList.innerHTML = (path?.waypoints ?? []).map((item, index) => `<div class="waypoint-row ${item.id === selectedPointId ? "is-selected" : ""}">
    <button class="waypoint-select" data-select-point-id="${item.id}" aria-label="Select anchor ${index + 1}">P${String(index + 1).padStart(2,"0")}</button>
    <label><span class="visually-hidden">Anchor ${index + 1} X coordinate in inches</span><input data-coordinate-point-id="${item.id}" data-coordinate="x" type="number" min="0" max="144" step="0.25" value="${item.x.toFixed(2)}" /></label>
    <label><span class="visually-hidden">Anchor ${index + 1} Y coordinate in inches</span><input data-coordinate-point-id="${item.id}" data-coordinate="y" type="number" min="0" max="144" step="0.25" value="${item.y.toFixed(2)}" /></label>
  </div>`).join("");
  segmentList.innerHTML = (path?.segmentReversed ?? []).map((reversed, index) => `<div class="segment-direction ${index === selectedSegment ? "is-current" : ""}">
    <button class="segment-select-button" data-segment-index="${index}" aria-pressed="${index === selectedSegment}">
      <span><b>Segment ${index+1} · P${index+1} → P${index+2}</b><small>${path.controlPoints[index] === null ? "Legacy spline" : `${path.controlPoints[index].length} controls`}</small></span>
    </button>
    <div class="segment-direction-toggle" role="group" aria-label="Drive direction for segment ${index+1}">
      <button class="${reversed ? "" : "is-active"}" data-direction-index="${index}" data-direction-value="false" aria-pressed="${!reversed}">→ Forward</button>
      <button class="${reversed ? "is-active" : ""}" data-direction-index="${index}" data-direction-value="true" aria-pressed="${reversed}">← Reverse</button>
    </div>
  </div>`).join("");
  document.querySelector("#path-summary").textContent = `${path?.waypoints.length ?? 0} anchors · ${estimateLength(path?.waypoints ?? [], 24, path?.segmentReversed ?? [], path?.controlPoints ?? []).toFixed(1)} in`;
  const directions = path?.segmentReversed ?? [];
  document.querySelector("#direction-summary").textContent = directions.some(Boolean) ? (directions.every(Boolean) ? "All reverse" : "Mixed") : "All forward";
  document.querySelector("#snap-step").value = String(documentState.snapStep);
  document.querySelector("#snap-toggle").checked = documentState.snap;
  document.querySelector("#zone-toggle").checked = documentState.showZones;
  robotInputs.forEach((input) => { input.value = Number(documentState.robot[input.dataset.robot]).toFixed(1); });
  document.querySelectorAll("[data-alliance]").forEach((button) => button.classList.toggle("is-active", button.dataset.alliance === documentState.alliance));
  document.querySelector('[data-action="undo"]').disabled = historyIndex === 0;
  document.querySelector('[data-action="redo"]').disabled = historyIndex === history.length - 1;
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

function addPoint(at = { x: 72, y: 72 }) {
  const path = activePath();
  if (!path) return;
  selectedControlId = null;
  const previous = path.waypoints.at(-1);
  const heading = previous ? Math.atan2(at.y - previous.y, at.x - previous.x) : 0;
  const point = { id: uid(), x: snap(at.x), y: snap(at.y), heading, tangent: previous ? Math.max(18, Math.hypot(at.x - previous.x, at.y - previous.y)) : 30 };
  path.waypoints.push(point); if (previous) { path.segmentReversed.push(false); path.controlPoints.push([]); } selectedPointId = point.id; commit("Waypoint added");
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
  if (action === "select-tool") { notify("Select tool active"); return; }
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
  if (alliance) { documentState.alliance = alliance; commit(`${alliance[0].toUpperCase()+alliance.slice(1)} field view selected`); }
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
    const { samples, profile } = playbackData();
    if (samples.length < 2 || !(profile.duration > 0)) return;
    drag = { type:"playback", pointerId:event.pointerId, samples, profile };
    svg.setPointerCapture(event.pointerId);
    playback.time = profileTimeAtDistance(profile, nearestPathDistance(samples, svgCoordinates(event)));
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
    playback.time = profileTimeAtDistance(drag.profile, nearestPathDistance(drag.samples, at));
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
svg.addEventListener("dblclick", (event) => { if (!event.target.closest("[data-point-id],[data-handle-id],[data-control-id],[data-playback-marker]")) addPoint(svgCoordinates(event)); });

for (const [key,input] of Object.entries(pointInputs)) input.addEventListener("change", () => {
  const point=selectedPoint(); if(!point) return;
  const value=Number(input.value); if(!Number.isFinite(value)) return render();
  if(key === "heading") point.heading=wrapRadians(value*Math.PI/180);
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
  playback.time = playbackData().profile.duration * Number(scrubber.value) / 1000;
  updatePlaybackUi();
});

document.querySelectorAll("[data-start]").forEach(input => input.addEventListener("change", () => {
  const value = Number(input.value); if (!input.value || !Number.isFinite(value)) return render();
  const point = documentState.paths[0]?.waypoints[0]; if (!point) return notify("Add a route point first");
  const key = input.dataset.start;
  point[key] = key === "heading" ? wrapRadians(value*Math.PI/180) : clamp(value);
  activePathId = documentState.paths[0].id; selectedPointId = point.id; selectedControlId = null;
  commit("Robot start / P1 updated");
}));
document.querySelector("#snap-step").addEventListener("change", event => { documentState.snapStep = Number(event.target.value); commit(); });

document.querySelector("#snap-toggle").addEventListener("change", (event) => { documentState.snap=event.target.checked;commit(); });
document.querySelector("#zone-toggle").addEventListener("change", (event) => { documentState.showZones=event.target.checked;commit(); });
titleNode.addEventListener("blur", () => { documentState.title=titleNode.textContent.trim() || "Untitled path";commit(); });
titleNode.addEventListener("keydown", (event) => { if(event.key === "Enter"){event.preventDefault();titleNode.blur();} });
fileInput.addEventListener("change", async () => {
  try {
    if (!fileInput.files.length) return;
    const loaded=validateDocument(JSON.parse(await fileInput.files[0].text()));
    stopPlayback(true); selectedSegment = 0; selectedControlId = null; documentState=loaded;
    activePathId=loaded.paths[0]?.id??null;selectedPointId=loaded.paths[0]?.waypoints[0]?.id??null;history=[JSON.stringify(loaded)];historyIndex=0;persist();render();notify("VantagePath file opened");
  } catch(error) { notify(error instanceof Error ? error.message : "Could not open that file"); }
  fileInput.value="";
});

window.addEventListener("keydown", (event) => {
  const editing=/INPUT|SELECT/.test(event.target.tagName)||event.target.isContentEditable;if(editing)return;
  const command=event.metaKey||event.ctrlKey;
  if(command&&event.key.toLowerCase()==="z"){event.preventDefault();restore(historyIndex+(event.shiftKey?1:-1));}
  else if(event.key==="Delete"||event.key==="Backspace"){event.preventDefault();deletePoint();}
  else if(event.key.toLowerCase()==="a"){event.preventDefault();addPoint();}
});

render();
