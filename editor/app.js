// VantagePath Studio: wires the DOM to the store, the route edits and the views.

import { cppExport, sectionMarkers } from "./export.js";
import { clamp, dragPosition, FIELD_SIZE, fromDisplayHeading, nearestPathDistance, normalizeSegmentDirections, robotStartPose, segmentPoint, toDisplayHeading, validateDocument, wrapRadians } from "./model.js";
import { poseAtTime, routeSamples, timeAtDistance } from "./planner.js";
import * as edits from "./route-edits.js";
import { activePath, canRedo, canUndo, commit, initStore, persist, replaceDocument, restore, routeDurations, routePlan, select, selectedControl, selectedMarker, selectedPoint, setActivePath, setPrefs, setView, state } from "./store.js";
import { activeRouteMarkup, fieldMarkup, inactiveRoutesMarkup, logOverlayMarkup } from "./views/field.js";
import { deviationFromPlan, parseLogCsv } from "./log-overlay.js";
import { checksMarkup, inspectorMarkup } from "./views/inspector.js";
import { budgetMarkup, robotStartMarkup, routeTreeMarkup } from "./views/outline.js";
import { GRAPH_VIEWBOX, playheadX, speedGraphMarkup } from "./views/timeline.js";

const LONG_PRESS_MS = 550;
const LONG_PRESS_SLOP_IN = 1.5;
const DOUBLE_CLICK_MS = 400;
const DOUBLE_CLICK_SLOP_IN = 2;
const DELETE_CONFIRM_MS = 3000;
const TOAST_MS = 2600;
const HANDLE_SCALE = 5;
const MARKER_SELECTOR = "[data-point-id],[data-handle-id],[data-control-id],[data-marker-id],[data-playback-marker]";
const MARKER_SNAP_SAMPLES = 200;

const $ = (selector) => document.querySelector(selector);
const svg = $("#field");
const graph = $("#speed-graph");
const scrubber = $("#path-scrubber");
const fileInput = $("#file-input");
const titleNode = $("#document-title");

let drag = null;
let longPress = null;
let deleteArmedUntil = 0;
let toastTimer = 0;
let logOverlay = null;
const playback = { playing: false, time: 0, startedAt: 0, startedFrom: 0, frame: 0 };

// ---------------------------------------------------------------- rendering

function notify(message) {
  const toast = $("#toast");
  toast.textContent = message;
  toast.classList.add("is-visible");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("is-visible"), TOAST_MS);
}

function headingText(radians) { return `${toDisplayHeading(radians, state.prefs.headingMode).toFixed(1)}°`; }

function renderField() {
  const path = activePath();
  const { plan, issues } = routePlan();
  svg.innerHTML = fieldMarkup(state.doc) + inactiveRoutesMarkup(state.doc, path?.id) + logOverlayMarkup(logOverlay)
    + activeRouteMarkup(path, { robot: state.doc.robot, selection: state.selection, plan, issues, showFootprint: state.prefs.showFootprint });
  updatePlaybackUi();
}

function renderSaveState() {
  const node = $("#save-state");
  const { ok, at, reason } = state.save;
  node.classList.toggle("is-error", !ok);
  const time = at ? ` ${at.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}` : "";
  node.querySelector("span").textContent = ok ? `Saved${time}` : "Not saved";
  node.title = ok ? "Changes autosave to this browser. Use Save to keep a .vpath file." : `Autosave failed: ${reason}. Use Save to keep a .vpath file.`;
}

// Re-rendering a panel replaces its inputs; keep focus where it was.
function renderPreservingFocus(container, markup) {
  const active = document.activeElement;
  const key = active && container.contains(active) ? active.dataset.edit ?? active.dataset.start ?? null : null;
  const index = active?.dataset?.controlIndex;
  container.innerHTML = markup;
  if (!key) return;
  const selector = `[data-edit="${key}"]${index !== undefined ? `[data-control-index="${index}"]` : ""}, [data-start="${key}"]`;
  container.querySelector(selector)?.focus();
}

function render() {
  const path = activePath();
  if (path) normalizeSegmentDirections(path);
  select({});
  const { plan, issues } = routePlan();
  renderField();
  const primary = state.doc.paths[0]?.waypoints[0];
  const startSelected = state.activePathId === state.doc.paths[0]?.id && state.selection.pointId === primary?.id && !state.selection.controlId;
  $("#route-budget").innerHTML = budgetMarkup(state.doc, routeDurations());
  renderPreservingFocus($("#robot-start"), robotStartMarkup(state.doc, state.prefs, startSelected));
  renderPreservingFocus($("#route-tree"), routeTreeMarkup(state.doc, { activePathId: state.activePathId, selection: state.selection, prefs: state.prefs, plan, issues }));
  $("#checks").innerHTML = checksMarkup(issues);
  renderPreservingFocus($("#selection-panels"), inspectorMarkup(path, { selection: state.selection, prefs: state.prefs, plan }));
  const { svg: graphSvg, peak } = speedGraphMarkup(plan, markerTimes(path, plan));
  graph.setAttribute("viewBox", GRAPH_VIEWBOX);
  graph.innerHTML = graphSvg;
  $("#graph-peak").textContent = peak ? `${peak.toFixed(0)} in/s` : "";
  $("#path-summary").textContent = !path || path.waypoints.length < 2 ? "Add two points to plan a route"
    : plan.error ? `Can't plan: ${plan.error}`
    : `${plan.length.toFixed(1)} in · ${plan.duration.toFixed(2)} s · ${plan.stops.length} stop${plan.stops.length === 1 ? "" : "s"}`;
  if (document.activeElement !== titleNode) titleNode.textContent = state.doc.title;
  $("#snap-step").value = String(state.doc.snapStep);
  $("#snap-toggle").checked = state.doc.snap;
  $("#zone-toggle").checked = state.doc.showZones;
  $("#footprint-toggle").checked = state.prefs.showFootprint;
  $("#heading-mode").value = state.prefs.headingMode;
  document.querySelectorAll("[data-robot]").forEach((input) => {
    if (document.activeElement !== input) input.value = Number(state.doc.robot[input.dataset.robot]).toFixed(input.dataset.robot === "sampleDistance" ? 2 : 1);
  });
  document.querySelectorAll("[data-alliance]").forEach((button) => {
    const active = button.dataset.alliance === state.doc.alliance;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", String(active));
  });
  document.querySelectorAll('[data-action="undo"]').forEach((button) => { button.disabled = !canUndo(); });
  document.querySelectorAll('[data-action="redo"]').forEach((button) => { button.disabled = !canRedo(); });
  const armed = performance.now() < deleteArmedUntil;
  $("#delete-path").textContent = armed ? `Click again to delete “${path?.name ?? "route"}”` : "Delete route";
  $("#delete-path").classList.toggle("is-armed", armed);
  renderSaveState();
  if ($("#export").open) renderExportPreview();
}

// When each event marker fires along the planned route, for the speed graph.
function markerTimes(path, plan) {
  if (!path || plan.error) return [];
  return plan.steps.filter((step) => step.type === "drive").flatMap((drive) => sectionMarkers(path, drive).map((marker) => {
    const target = marker.percent / 100 * drive.length;
    const state = drive.states.find((candidate) => candidate.distance >= target) ?? drive.states.at(-1);
    return { name: marker.name, time: drive.startTime + state.time };
  }));
}

// ----------------------------------------------------------------- playback

function updatePlaybackUi() {
  const { plan } = routePlan();
  playback.time = Math.min(playback.time, plan.duration);
  scrubber.value = String(Math.round((plan.duration > 0 ? playback.time / plan.duration : 0) * 1000));
  const pose = poseAtTime(plan, playback.time);
  const speed = pose ? ` · ${Math.abs(pose.velocity).toFixed(0)} in/s` : "";
  $("#playback-time").textContent = `${playback.time.toFixed(2)} / ${plan.duration.toFixed(2)} s${speed}`;
  const button = $("#play-path");
  button.textContent = playback.playing ? "❚❚" : "▶";
  button.setAttribute("aria-label", playback.playing ? "Pause preview" : "Play route");
  $("#heading-readout").textContent = `θ ${headingText(pose?.heading ?? robotStartPose(state.doc).heading)}`;
  const playhead = $("#graph-playhead");
  if (playhead) { const x = playheadX(plan, playback.time); playhead.setAttribute("x1", x); playhead.setAttribute("x2", x); }
  const robot = $("#playback-robot");
  if (!robot) return;
  robot.style.display = plan.steps.length ? "" : "none";
  robot.classList.toggle("is-dragging", drag?.type === "playback");
  if (pose) robot.setAttribute("transform", `translate(${pose.x} ${FIELD_SIZE - pose.y}) rotate(${-pose.heading * 180 / Math.PI})`);
}

function stopPlayback(reset = false) {
  if (playback.frame) cancelAnimationFrame(playback.frame);
  playback.frame = 0; playback.playing = false;
  if (reset) playback.time = 0;
}

function playbackFrame(now) {
  if (!playback.playing) return;
  const { duration } = routePlan().plan;
  playback.time = Math.min(duration, playback.startedFrom + (now - playback.startedAt) / 1000);
  if (playback.time >= duration) playback.playing = false;
  updatePlaybackUi();
  playback.frame = playback.playing ? requestAnimationFrame(playbackFrame) : 0;
}

function togglePlayback() {
  const { plan } = routePlan();
  if (plan.error) return notify(`Can't play: ${plan.error}`);
  if (!(plan.duration > 0)) return notify("Add at least two points to play the route");
  if (playback.playing) { stopPlayback(false); return updatePlaybackUi(); }
  if (playback.time >= plan.duration) playback.time = 0;
  Object.assign(playback, { playing: true, startedAt: performance.now(), startedFrom: playback.time });
  updatePlaybackUi();
  playback.frame = requestAnimationFrame(playbackFrame);
}

// -------------------------------------------------------------------- edits

// Replaces the active route with an edited copy and records it.
function applyPath(next, message, options) {
  const path = activePath();
  if (!path || !next) return;
  stopPlayback(true);
  state.doc.paths = state.doc.paths.map((candidate) => (candidate.id === path.id ? next : candidate));
  commit(message, options);
}

function snapStep() { return state.doc.snap ? state.doc.snapStep : 0; }

function addPoint(at) {
  const path = activePath(); if (!path) return;
  const result = edits.appendPoint(path, at ?? edits.pointAhead(path, state.doc.robot), snapStep());
  if (result.error) return notify(result.error);
  select({ pointId: result.point.id, controlId: null, segment: Math.max(0, result.path.waypoints.length - 2) });
  applyPath(result.path, `Added P${result.path.waypoints.length}`);
}

function addPointAt(at) {
  const path = activePath(); if (!path) return;
  const inserted = edits.insertPointOnPath(path, at);
  if (!inserted) return addPoint(at);
  select({ pointId: inserted.point.id, controlId: null, segment: inserted.index });
  applyPath(inserted.path, `Inserted P${inserted.index + 1}`);
}

function deleteSelection() {
  const path = activePath(); if (!path) return;
  const { controlId, segment, pointId } = state.selection;
  if (controlId) {
    select({ controlId: null });
    return applyPath(edits.deleteControl(path, segment, controlId), "Control point deleted");
  }
  if (!pointId) return;
  if (path.waypoints.length <= 1) return notify("A route needs at least one point");
  const { path: next, nextPointId } = edits.deletePoint(path, pointId);
  select({ pointId: nextPointId });
  applyPath(next, "Route point deleted");
}

function nudgeSelection(dx, dy) {
  const target = selectedControl() ?? selectedPoint();
  if (!target) return;
  stopPlayback(true);
  target.x = clamp(target.x + dx); target.y = clamp(target.y + dy);
  commit(null, { coalesce: `nudge:${state.selection.controlId ?? state.selection.pointId}` });
}

function addRoute(route, message) {
  stopPlayback(true);
  state.doc.paths = [...state.doc.paths, route];
  setActivePath(route.id);
  commit(message);
}

function newRoute() {
  addRoute(edits.newRoute(robotStartPose(state.doc), state.doc.paths.length + 1), "New route created");
}

function deleteRoute() {
  const path = activePath(); if (!path) return;
  if (performance.now() > deleteArmedUntil) {
    deleteArmedUntil = performance.now() + DELETE_CONFIRM_MS;
    render();
    setTimeout(render, DELETE_CONFIRM_MS + 50);
    return;
  }
  deleteArmedUntil = 0;
  stopPlayback(true);
  state.doc.paths = state.doc.paths.filter((candidate) => candidate.id !== path.id);
  if (!state.doc.paths.length) return newRoute();
  setActivePath(state.doc.paths[0].id);
  commit(`Deleted “${path.name}”`);
}

function renderLogStatus() {
  $("#clear-log").disabled = !logOverlay;
  if (!logOverlay) { $("#log-status").textContent = ""; return; }
  const deviation = deviationFromPlan(logOverlay.points, routeSamples(routePlan().plan));
  $("#log-status").textContent = `${logOverlay.name}: ${logOverlay.points.length} points${logOverlay.skipped ? `, ${logOverlay.skipped} rows skipped` : ""}.`
    + (deviation ? ` Off the active route by up to ${deviation.worst.toFixed(1)} in (average ${deviation.average.toFixed(1)} in).` : "");
}

function moveActiveRoute(delta) {
  const path = activePath(); if (!path) return;
  const moved = edits.moveRoute(state.doc.paths, path.id, delta);
  if (moved === state.doc.paths) return;
  state.doc.paths = moved;
  commit(delta < 0 ? `“${path.name}” now runs earlier` : `“${path.name}” now runs later`);
}

function smoothAt(index) {
  const path = activePath(); if (!path) return;
  select({ pointId: path.waypoints[index]?.id ?? state.selection.pointId, controlId: null });
  applyPath(edits.smoothJoin(path, index), `Smoothed the join at P${index + 1}`);
}

// ------------------------------------------------------------------ export

function exportCode() {
  const target = $("#export-target").value;
  $("#export-frame").disabled = target === "chassis";
  return cppExport(state.doc, $("#variable-name").value, $("#export-frame").value, target);
}

function renderExportPreview() {
  const code = exportCode();
  $("#export-preview").textContent = code;
  const sections = (code.match(/Trajectory = vantage::generateTrajectory/g) ?? []).length;
  const valid = /^[A-Za-z_][A-Za-z0-9_]*$/.test($("#variable-name").value);
  const chassis = $("#export-target").value === "chassis";
  if (chassis) {
    const follows = (code.match(/chassis\.followPath\(/g) ?? []).length;
    $("#export-note").textContent = `${follows} followPath call${follows === 1 ? "" : "s"} with suggested timeouts, point turns, marker waits and route waits. Paste into your auton and fill in the marker actions. Corner frame only.`;
    return;
  }
  $("#export-note").textContent = `${sections} trajector${sections === 1 ? "y" : "ies"} across ${state.doc.paths.length} route${state.doc.paths.length === 1 ? "" : "s"}. `
    + (valid ? "Corners and direction changes become separate sections that start and end at rest." : "The variable name isn't a valid C++ identifier, so “generatedPath” is used.");
}

function download(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], { type }));
  const link = Object.assign(document.createElement("a"), { href: url, download: name });
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

function safeFileName(value) { return value.trim().toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-|-$/g, "") || "vantage-path"; }

async function copyExport() {
  try {
    await navigator.clipboard.writeText(exportCode());
    notify("C++ copied to the clipboard");
  } catch {
    notify("Couldn't reach the clipboard. Select the code and copy it, or download the .hpp");
  }
}

// ------------------------------------------------------------------- sheets

function setSheet(name, open) {
  document.querySelectorAll("[data-sheet]").forEach((button) => {
    const isOpen = open && button.dataset.sheet === name;
    button.setAttribute("aria-expanded", String(isOpen));
    $(`#${button.dataset.sheet}`).classList.toggle("is-open", isOpen);
  });
  $(".sheet-backdrop").hidden = !open;
}

// ------------------------------------------------------------------ actions

const ACTIONS = {
  "select-start": () => {
    const primary = state.doc.paths[0];
    if (!primary?.waypoints[0]) return notify("Add a route point first");
    state.activePathId = primary.id;
    select({ pointId: primary.waypoints[0].id, controlId: null, segment: 0 });
    stopPlayback(true);
    render();
  },
  "add-point": () => addPoint(),
  "delete-point": deleteSelection,
  "add-control": () => {
    const path = activePath(); if (!path) return;
    const { path: next, control } = edits.addControl(path, state.selection.segment);
    select({ controlId: control?.id ?? null });
    applyPath(next, "Control point added");
  },
  "new-path": newRoute,
  "duplicate-path": () => { const path = activePath(); if (path) addRoute(edits.duplicateRoute(path), "Route duplicated"); },
  "mirror-alliance-copy": () => {
    const path = activePath(); if (!path) return;
    const copy = edits.allianceCopy(path, state.doc.alliance);
    addRoute(copy, `Created “${copy.name}” for the other alliance`);
  },
  "mirror-quadrant": () => applyPath(edits.mirrorPath(activePath(), "quadrant"), "Mirrored to the opposite same-alliance quadrant"),
  "mirror-left-right": () => applyPath(edits.mirrorPath(activePath(), "left-right"), "Mirrored left to right"),
  "mirror-bottom-top": () => applyPath(edits.mirrorPath(activePath(), "bottom-top"), "Mirrored bottom to top"),
  "reverse-order": () => applyPath(edits.reverseRoute(activePath()), "Route order reversed"),
  "all-forward": () => applyPath(edits.setAllDirections(activePath(), false), "Every segment drives forward"),
  "all-reverse": () => applyPath(edits.setAllDirections(activePath(), true), "Every segment drives in reverse"),
  "delete-path": deleteRoute,
  undo: () => { stopPlayback(true); restore(state.historyIndex - 1); },
  redo: () => { stopPlayback(true); restore(state.historyIndex + 1); },
  "play-path": togglePlayback,
  open: () => fileInput.click(),
  save: () => { download(`${safeFileName(state.doc.title)}.vpath`, JSON.stringify(state.doc, null, 2), "application/json"); notify("Saved a .vpath file"); },
  "open-settings": () => $("#settings").showModal(),
  "open-export": () => { renderExportPreview(); $("#export").showModal(); },
  "copy-export": copyExport,
  "download-export": () => { download(`${safeFileName(state.doc.title)}.hpp`, exportCode(), "text/x-c++hdr"); notify("C++ header downloaded"); },
  "add-marker": () => {
    const path = activePath(); if (!path || path.waypoints.length < 2) return notify("Add a segment first");
    const { path: next, marker } = edits.addMarker(path, state.selection.segment, 0.5);
    select({ markerId: marker.id, controlId: null });
    applyPath(next, `Added “${marker.name}”. Drag it along the path.`);
  },
  "load-log": () => $("#log-input").click(),
  "clear-log": () => { logOverlay = null; renderLogStatus(); renderField(); },
  "route-up": () => moveActiveRoute(-1),
  "route-down": () => moveActiveRoute(1),
};

function selectPointById(id) {
  const path = activePath();
  const index = path.waypoints.findIndex((candidate) => candidate.id === id);
  if (index < 0) return;
  select({ pointId: id, controlId: null, segment: Math.min(index, path.waypoints.length - 2) });
  stopPlayback(true);
  render();
}

// Clicks on buttons and rows in the panels, routed by their data attributes.
const CLICKS = [
  ["[data-sheet]", (node) => setSheet(node.dataset.sheet, node.getAttribute("aria-expanded") !== "true")],
  ["[data-close-sheet]", () => setSheet(null, false)],
  ["[data-alliance]", (node) => setView("alliance", node.dataset.alliance)],
  ["[data-smooth-index]", (node) => smoothAt(Number(node.dataset.smoothIndex))],
  ["[data-select-point-id]", (node) => selectPointById(node.dataset.selectPointId)],
  ["[data-select-segment]", (node) => { select({ segment: Number(node.dataset.selectSegment), controlId: null }); render(); }],
  ["[data-toggle-direction]", (node) => {
    const index = Number(node.dataset.toggleDirection);
    select({ segment: index });
    applyPath(edits.setSegmentDirection(activePath(), index, !activePath().segmentReversed[index]), `Segment ${index + 1} direction switched`);
  }],
  ["[data-set-direction]", (node) => applyPath(edits.setSegmentDirection(activePath(), state.selection.segment, node.dataset.setDirection === "true"), "Drive direction updated")],
  ["[data-select-control]", (node) => { select({ controlId: node.dataset.selectControl }); render(); }],
  ["[data-remove-control]", (node) => {
    const id = activePath().controlPoints[state.selection.segment]?.[Number(node.dataset.removeControl)]?.id;
    select({ controlId: null });
    applyPath(edits.deleteControl(activePath(), state.selection.segment, id), "Control point removed");
  }],
  ["[data-remove-marker]", (node) => { select({ markerId: null }); applyPath(edits.deleteMarker(activePath(), node.dataset.removeMarker), "Marker removed"); }],
  ["[data-check-point]", (node) => { const point = activePath()?.waypoints[Number(node.dataset.checkPoint)]; if (point) selectPointById(point.id); }],
];

document.addEventListener("click", (event) => {
  const action = event.target.closest("[data-action]")?.dataset.action;
  if (action && ACTIONS[action]) return ACTIONS[action]();
  if (!event.target.closest("svg")) {
    const route = event.target.closest("[data-path-id]");
    if (route) { stopPlayback(true); setActivePath(route.dataset.pathId); return render(); }
  }
  for (const [selector, handler] of CLICKS) {
    const node = event.target.closest(selector);
    if (node) return handler(node);
  }
});

// Inputs in the outline, inspector and robot-start card.
function numberFrom(input) {
  const value = Number(input.value);
  return input.value !== "" && Number.isFinite(value) ? value : null;
}

function editSelected(input, apply) {
  const point = selectedPoint();
  const value = numberFrom(input);
  if (!point || value === null) return null;
  return apply(point, value);
}

function editControl(input, path, axis) {
  const control = path.controlPoints[state.selection.segment]?.[Number(input.dataset.controlIndex)];
  const value = numberFrom(input);
  if (!control || value === null) return null;
  control[axis] = clamp(value);
  select({ controlId: control.id });
  return "Control point moved";
}

const EDITS = {
  "route-name": (input, path) => { path.name = input.value.trim() || "Untitled route"; return "Route renamed"; },
  "point-x": (input) => editSelected(input, (point, value) => { point.x = clamp(value); return "Point moved"; }),
  "point-y": (input) => editSelected(input, (point, value) => { point.y = clamp(value); return "Point moved"; }),
  "point-heading": (input) => editSelected(input, (point, value) => { point.heading = fromDisplayHeading(value, state.prefs.headingMode); return "Heading updated"; }),
  "point-tangent": (input) => editSelected(input, (point, value) => { point.tangent = Math.max(1, value); return "Tangent updated"; }),
  "wait-after": (input, path) => { const value = numberFrom(input); if (value === null) return null; path.waitAfterMs = Math.max(0, Math.round(value)); return "Wait updated"; },
  "segment-speed": (input, path) => { const value = numberFrom(input); path.segmentSpeed[state.selection.segment] = value !== null && value > 0 ? value : null; return value ? `Segment ${state.selection.segment + 1} limited to ${value} in/s` : "Speed limit cleared"; },
  "marker-name": (input, path) => { const marker = path.markers.find((m) => m.id === input.dataset.markerId); if (!marker) return null; marker.name = input.value.trim().slice(0, 40) || "Marker"; return "Marker renamed"; },
  "marker-t": (input, path) => { const marker = path.markers.find((m) => m.id === input.dataset.markerId); const value = numberFrom(input); if (!marker || value === null) return null; marker.t = clamp(value / 100, 0, 1); return "Marker moved"; },
  "control-x": (input, path) => editControl(input, path, "x"),
  "control-y": (input, path) => editControl(input, path, "y"),
};

document.addEventListener("change", (event) => {
  const input = event.target;
  const path = activePath();
  if (input.dataset.edit === "geometry-mode") return applyPath(edits.setSegmentGeometry(path, state.selection.segment, input.value), "Curve type changed");
  if (input.dataset.edit && EDITS[input.dataset.edit] && path) {
    stopPlayback(true);
    const message = EDITS[input.dataset.edit](input, path);
    return message ? commit(message) : render();
  }
  if (input.dataset.start) {
    const point = state.doc.paths[0]?.waypoints[0];
    const value = numberFrom(input);
    if (!point || value === null) return render();
    point[input.dataset.start] = input.dataset.start === "heading" ? fromDisplayHeading(value, state.prefs.headingMode) : clamp(value);
    state.activePathId = state.doc.paths[0].id;
    select({ pointId: point.id, controlId: null });
    stopPlayback(true);
    return commit("Robot start updated");
  }
  if (input.dataset.robot) {
    const value = numberFrom(input);
    if (value === null) return render();
    state.doc = validateDocument({ ...state.doc, robot: { ...state.doc.robot, [input.dataset.robot]: value } });
    stopPlayback(true);
    commit("Robot setup updated");
  }
});

$("#snap-step").addEventListener("change", (event) => setView("snapStep", Number(event.target.value)));
$("#snap-toggle").addEventListener("change", (event) => setView("snap", event.target.checked));
$("#zone-toggle").addEventListener("change", (event) => setView("showZones", event.target.checked));
$("#heading-mode").addEventListener("change", (event) => setPrefs({ headingMode: event.target.value === "compass" ? "compass" : "math" }));
$("#export-frame").addEventListener("change", renderExportPreview);
$("#export-target").addEventListener("change", renderExportPreview);
$("#footprint-toggle").addEventListener("change", (event) => setPrefs({ showFootprint: event.target.checked }));
$("#variable-name").addEventListener("input", renderExportPreview);
titleNode.addEventListener("blur", () => {
  const title = titleNode.textContent.trim() || "Untitled route";
  if (title !== state.doc.title) { state.doc.title = title; commit("Title updated"); }
});
titleNode.addEventListener("keydown", (event) => { if (event.key === "Enter") { event.preventDefault(); titleNode.blur(); } });

$("#log-input").addEventListener("change", async (event) => {
  const input = event.target;
  try {
    if (!input.files.length) return;
    const file = input.files[0];
    const { points, skipped } = parseLogCsv(await file.text());
    logOverlay = { name: file.name, points, skipped };
    renderLogStatus(); renderField();
    notify(`Overlaid ${points.length} logged points`);
  } catch (error) {
    notify(error instanceof Error ? error.message : "Couldn't read that log");
  } finally {
    input.value = "";
  }
});

fileInput.addEventListener("change", async () => {
  try {
    if (!fileInput.files.length) return;
    const loaded = validateDocument(JSON.parse(await fileInput.files[0].text()));
    stopPlayback(true);
    replaceDocument(loaded);
    persist(); render();
    notify("Opened the .vpath file");
  } catch (error) {
    notify(error instanceof Error ? error.message : "Couldn't open that file");
  } finally {
    fileInput.value = "";
  }
});

// -------------------------------------------------------------- field input

function svgCoordinates(event) {
  const point = svg.createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
  const local = point.matrixTransform(svg.getScreenCTM().inverse());
  return { x: clamp(local.x), y: clamp(FIELD_SIZE - local.y) };
}

function startPlaybackDrag(event) {
  stopPlayback(false);
  const { plan } = routePlan();
  const samples = routeSamples(plan);
  if (samples.length < 2 || !(plan.duration > 0)) return;
  drag = { type: "playback", pointerId: event.pointerId, samples, plan };
  svg.setPointerCapture(event.pointerId);
  playback.time = timeAtDistance(plan, nearestPathDistance(samples, svgCoordinates(event)));
  updatePlaybackUi();
}

function startEventMarkerDrag(event, markerId) {
  select({ markerId, controlId: null });
  const marker = selectedMarker();
  if (!marker) return;
  select({ segment: marker.segment, markerId });
  // Markers are rebuilt on render, so the drag tracks the id, not the object.
  drag = { type: "event", markerId, segment: marker.segment, name: marker.name, pointerId: event.pointerId, before: JSON.stringify(state.doc), moved: false };
  svg.setPointerCapture(event.pointerId);
  render();
}

// Curve parameter on `segment` nearest the pointer.
function nearestT(path, segment, at) {
  let best = { t: 0, distance: Infinity };
  for (let i = 0; i <= MARKER_SNAP_SAMPLES; i += 1) {
    const t = i / MARKER_SNAP_SAMPLES;
    const point = segmentPoint(path, segment, t);
    const distance = Math.hypot(point.x - at.x, point.y - at.y);
    if (distance < best.distance) best = { t, distance };
  }
  return best.t;
}

function startMarkerDrag(event, target) {
  stopPlayback(true);
  const path = activePath();
  if (target.dataset.markerId) return startEventMarkerDrag(event, target.dataset.markerId);
  const { pointId, handleId, controlId } = target.dataset;
  if (controlId) select({ controlId });
  else {
    const id = pointId ?? handleId;
    const index = path.waypoints.findIndex((point) => point.id === id);
    select({ pointId: id, controlId: null, segment: Math.min(index, path.waypoints.length - 2) });
  }
  const point = controlId ? selectedControl() : selectedPoint();
  if (!point) return;
  const type = controlId ? "control" : handleId ? "handle" : "point";
  const origin = type === "handle"
    ? { x: point.x + Math.cos(point.heading) * point.tangent / HANDLE_SCALE, y: point.y + Math.sin(point.heading) * point.tangent / HANDLE_SCALE }
    : { x: point.x, y: point.y };
  drag = { type, point, pointerId: event.pointerId, pointerStart: svgCoordinates(event), origin, before: JSON.stringify(state.doc), moved: false };
  svg.setPointerCapture(event.pointerId);
  render();
}

svg.addEventListener("pointerdown", (event) => {
  const target = event.target.closest(MARKER_SELECTOR);
  if (target) {
    event.preventDefault();
    return target.dataset.playbackMarker ? startPlaybackDrag(event) : startMarkerDrag(event, target);
  }
  const route = event.target.closest("[data-path-id]");
  if (route) { stopPlayback(true); setActivePath(route.dataset.pathId); return render(); }
  const segmentHit = event.target.closest("[data-segment-hit]");
  if (segmentHit) { select({ segment: Number(segmentHit.dataset.segmentHit), controlId: null }); render(); }
});

svg.addEventListener("pointermove", (event) => {
  const at = svgCoordinates(event);
  $("#coordinate-readout").textContent = `X ${at.x.toFixed(2)} · Y ${at.y.toFixed(2)}`;
  if (!drag || drag.pointerId !== event.pointerId) return;
  if (drag.type === "playback") {
    playback.time = timeAtDistance(drag.plan, nearestPathDistance(drag.samples, at));
    return updatePlaybackUi();
  }
  if (drag.type === "event") {
    const marker = activePath()?.markers.find((candidate) => candidate.id === drag.markerId);
    if (!marker) return;
    marker.t = nearestT(activePath(), drag.segment, at);
    drag.moved = true;
    return renderField();
  }
  const step = state.doc.snap && !event.shiftKey ? state.doc.snapStep : 0;
  const moved = dragPosition(drag.origin, drag.pointerStart, at, step, event.altKey);
  const point = drag.point;
  if (drag.type === "handle") {
    point.heading = wrapRadians(Math.atan2(moved.y - point.y, moved.x - point.x));
    point.tangent = Math.max(3, Math.hypot(moved.x - point.x, moved.y - point.y) * HANDLE_SCALE);
  } else { point.x = moved.x; point.y = moved.y; }
  drag.moved = true;
  // Only the field changes during a drag; panels update on release.
  renderField();
});

function endDrag(event) {
  if (!drag || drag.pointerId !== event.pointerId) return;
  const finished = drag;
  drag = null;
  if (finished.type === "playback") return updatePlaybackUi();
  if (!finished.moved) return render();
  const primary = state.doc.paths[0]?.waypoints[0];
  if (finished.type === "event") return commit(`Marker “${finished.name}” moved`);
  commit(finished.type === "control" ? "Control point moved" : finished.point === primary ? "Robot start moved" : "Route point moved");
}
svg.addEventListener("pointerup", endDrag);
svg.addEventListener("pointercancel", (event) => {
  if (!drag || drag.pointerId !== event.pointerId) return;
  if (drag.type !== "playback") state.doc = JSON.parse(drag.before);
  drag = null;
  render();
});

// Double-click to add, detected from pointer events because a first click on a
// segment re-renders the SVG, which stops the browser's own dblclick firing.
let lastTap = null;
svg.addEventListener("pointerdown", (event) => {
  if (event.pointerType !== "mouse" || event.button !== 0 || event.target.closest(MARKER_SELECTOR)) return;
  const at = svgCoordinates(event);
  const now = performance.now();
  if (lastTap && now - lastTap.time < DOUBLE_CLICK_MS && Math.hypot(at.x - lastTap.at.x, at.y - lastTap.at.y) < DOUBLE_CLICK_SLOP_IN) {
    lastTap = null;
    return addPointAt(at);
  }
  lastTap = { time: now, at };
});

// Touch has no double-click: press and hold on the field to add a point.
svg.addEventListener("pointerdown", (event) => {
  if (event.pointerType === "mouse" || event.target.closest(MARKER_SELECTOR)) return;
  const at = svgCoordinates(event);
  clearTimeout(longPress?.timer);
  longPress = { pointerId: event.pointerId, at, timer: setTimeout(() => { longPress = null; addPointAt(at); }, LONG_PRESS_MS) };
});
function cancelLongPress(event) {
  if (!longPress || longPress.pointerId !== event.pointerId) return;
  if (event.type === "pointermove") {
    const at = svgCoordinates(event);
    if (Math.hypot(at.x - longPress.at.x, at.y - longPress.at.y) < LONG_PRESS_SLOP_IN) return;
  }
  clearTimeout(longPress.timer); longPress = null;
}
["pointermove", "pointerup", "pointercancel"].forEach((type) => svg.addEventListener(type, cancelLongPress));

// Scrub from the slider or by pressing on the speed graph.
scrubber.addEventListener("input", () => {
  stopPlayback(false);
  playback.time = routePlan().plan.duration * Number(scrubber.value) / 1000;
  updatePlaybackUi();
});
function scrubGraph(event) {
  const box = graph.getBoundingClientRect();
  stopPlayback(false);
  playback.time = routePlan().plan.duration * clamp((event.clientX - box.left) / box.width, 0, 1);
  updatePlaybackUi();
}
graph.addEventListener("pointerdown", (event) => { graph.setPointerCapture(event.pointerId); scrubGraph(event); });
graph.addEventListener("pointermove", (event) => { if (graph.hasPointerCapture(event.pointerId)) scrubGraph(event); });

// ----------------------------------------------------------------- keyboard

const NUDGES = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] };

window.addEventListener("keydown", (event) => {
  const editing = /INPUT|SELECT|TEXTAREA/.test(event.target.tagName) || event.target.isContentEditable;
  if (editing || document.querySelector("dialog[open]")) return;
  const command = event.metaKey || event.ctrlKey;
  const key = event.key.toLowerCase();
  if (command && key === "z") { event.preventDefault(); ACTIONS[event.shiftKey ? "redo" : "undo"](); }
  else if (command && key === "y") { event.preventDefault(); ACTIONS.redo(); }
  else if (event.key === "Delete" || event.key === "Backspace") { event.preventDefault(); deleteSelection(); }
  else if (key === "a" && !command) { event.preventDefault(); addPoint(); }
  else if (event.key === " " && !command && !event.target.closest("button")) { event.preventDefault(); togglePlayback(); }
  else if (event.key === "Escape") setSheet(null, false);
  else if (NUDGES[event.key] && !command) {
    event.preventDefault();
    const step = event.altKey ? 0.25 : event.shiftKey ? 6 : 1;
    nudgeSelection(NUDGES[event.key][0] * step, NUDGES[event.key][1] * step);
  }
});

// Close a <dialog> by clicking its backdrop.
document.querySelectorAll("dialog").forEach((dialog) => dialog.addEventListener("click", (event) => {
  if (event.target === dialog) dialog.close();
}));

initStore({ render, notify });
render();
if (state.startupMessage) notify(state.startupMessage);
