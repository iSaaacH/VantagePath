import { FIELD_SIZE, clamp, cppExport, estimateLength, makeDocument, mirrorWaypoint, motionProfile, profileDistance, quinticPoint, reverseWaypoints, validateDocument, wrapRadians } from "./model.js";

const STORAGE_KEY = "vantagepath-studio-v1";
const svg = document.querySelector("#field");
const stage = document.querySelector("#field-stage");
const fileInput = document.querySelector("#file-input");
const pathList = document.querySelector("#path-list");
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
function snap(value, bypass = false) { return documentState.snap && !bypass ? Math.round(value / 6) * 6 : value; }

function loadLocal() {
  try { return validateDocument(JSON.parse(localStorage.getItem(STORAGE_KEY))) } catch { return makeDocument(); }
}

function persist() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(documentState));
}

function commit(message) {
  stopPlayback(true);
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
  const samples = [{ x:path.waypoints[0].x, y:path.waypoints[0].y, distance:0, heading:path.waypoints[0].heading }];
  let distance = 0;
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    let previous = samples.at(-1);
    for (let step = 1; step <= subdivisions; step += 1) {
      const point = quinticPoint(path.waypoints[index], path.waypoints[index + 1], step / subdivisions);
      const segment = Math.hypot(point.x - previous.x, point.y - previous.y);
      distance += segment;
      const heading = segment > 1e-6 ? Math.atan2(point.y - previous.y, point.x - previous.x) : previous.heading;
      const sample = { ...point, distance, heading };
      samples.push(sample); previous = sample;
    }
  }
  return samples;
}

function playbackData() {
  const samples = pathSamples(activePath());
  return { samples, profile:motionProfile(samples.at(-1)?.distance ?? 0, documentState.robot) };
}

function poseAtDistance(samples, distance, reversed = false) {
  if (!samples.length) return null;
  let upperIndex = samples.findIndex((sample) => sample.distance >= distance);
  if (upperIndex < 0) upperIndex = samples.length - 1;
  const upper = samples[upperIndex];
  const lower = samples[Math.max(0, upperIndex - 1)];
  const span = upper.distance - lower.distance;
  const ratio = span > 1e-6 ? (distance - lower.distance) / span : 0;
  let heading = upper.heading;
  if (upperIndex === samples.length - 1) heading = activePath()?.waypoints.at(-1)?.heading ?? heading;
  if (reversed) heading = wrapRadians(heading + Math.PI);
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
  const pose = poseAtDistance(samples, profileDistance(profile, playback.time), activePath()?.reversed);
  const robot = document.querySelector("#playback-robot");
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
  let d = `M${path.waypoints[0].x} ${FIELD_SIZE - path.waypoints[0].y}`;
  for (let index = 0; index + 1 < path.waypoints.length; index += 1) {
    for (let step = 1; step <= 30; step += 1) {
      const sample = quinticPoint(path.waypoints[index], path.waypoints[index + 1], step / 30);
      d += ` L${sample.x} ${FIELD_SIZE - sample.y}`;
    }
  }
  const controls = path.waypoints.map((point, index) => {
    const handleX = point.x + Math.cos(point.heading) * point.tangent / 5;
    const handleY = point.y + Math.sin(point.heading) * point.tangent / 5;
    const selected = point.id === selectedPointId;
    return `<g><line class="control-line" x1="${point.x}" y1="${FIELD_SIZE - point.y}" x2="${handleX}" y2="${FIELD_SIZE - handleY}"/>
      <circle class="handle" data-handle-id="${point.id}" cx="${handleX}" cy="${FIELD_SIZE - handleY}" r="2"/>
      <circle class="anchor ${selected ? "selected" : ""}" data-point-id="${point.id}" cx="${point.x}" cy="${FIELD_SIZE - point.y}" r="2.8"/>
      <text class="anchor-label" x="${point.x + 3.8}" y="${FIELD_SIZE - point.y - 3}">${String(index + 1).padStart(2,"0")}</text></g>`;
  }).join("");
  const robot = documentState.robot;
  const robotMarkup = `<g id="playback-robot" transform="translate(${path.waypoints[0].x} ${FIELD_SIZE - path.waypoints[0].y}) rotate(${-(path.waypoints[0].heading + (path.reversed ? Math.PI : 0)) * 180 / Math.PI})">
    <rect class="robot-box" x="${-robot.length / 2}" y="${-robot.width / 2}" width="${robot.length}" height="${robot.width}" rx="1"/>
    <line class="robot-nose" x1="${robot.length * .12}" y1="0" x2="${robot.length * .43}" y2="0"/>
  </g>`;
  return `${d ? `<path class="path-shadow" d="${d}"/><path class="path-curve ${path.reversed ? "reversed" : ""}" d="${d}"/>` : ""}${controls}${robotMarkup}`;
}

function render() {
  const path = activePath();
  svg.innerHTML = fieldMarkup() + pathMarkup(path);
  titleNode.textContent = documentState.title;
  pathList.innerHTML = documentState.paths.map((item, index) => `<button class="path-item ${item.id === activePathId ? "is-active" : ""}" data-path-id="${item.id}"><i class="path-swatch" style="background:${escapeHtml(item.color)}"></i><span><strong>${escapeHtml(item.name)}</strong><small>${item.waypoints.length} anchors · ${item.reversed ? "reverse drive" : "forward"}</small></span><b>${String(index + 1).padStart(2,"0")}</b></button>`).join("");
  const point = selectedPoint();
  document.querySelector("#waypoint-inspector").style.opacity = point ? "1" : ".4";
  Object.values(pointInputs).forEach((input) => { input.disabled = !point; });
  if (point) {
    pointInputs.x.value = point.x.toFixed(1); pointInputs.y.value = point.y.toFixed(1);
    pointInputs.heading.value = (point.heading * 180 / Math.PI).toFixed(1); pointInputs.tangent.value = point.tangent.toFixed(1);
    const index = path.waypoints.findIndex((candidate) => candidate.id === point.id);
    document.querySelector("#waypoint-label").textContent = `Anchor ${String(index + 1).padStart(2,"0")}`;
    document.querySelector("#selection-index").textContent = `P${String(index + 1).padStart(2,"0")}`;
  }
  document.querySelector("#path-summary").textContent = `${path?.waypoints.length ?? 0} anchors · ${estimateLength(path?.waypoints ?? []).toFixed(1)} in`;
  document.querySelector("#drive-direction").value = path?.reversed ? "reverse" : "forward";
  document.querySelector("#snap-toggle").checked = documentState.snap;
  document.querySelector("#zone-toggle").checked = documentState.showZones;
  robotInputs.forEach((input) => { input.value = Number(documentState.robot[input.dataset.robot]).toFixed(1); });
  document.querySelectorAll("[data-alliance]").forEach((button) => button.classList.toggle("is-active", button.dataset.alliance === documentState.alliance));
  document.querySelector('[data-action="undo"]').disabled = historyIndex === 0;
  document.querySelector('[data-action="redo"]').disabled = historyIndex === history.length - 1;
  updatePlaybackUi();
}

function svgCoordinates(event) {
  const point = svg.createSVGPoint(); point.x = event.clientX; point.y = event.clientY;
  const local = point.matrixTransform(svg.getScreenCTM().inverse());
  return { x: clamp(local.x), y: clamp(FIELD_SIZE - local.y) };
}

function addPoint(at = { x: 72, y: 72 }) {
  const path = activePath();
  if (!path) return;
  const previous = path.waypoints.at(-1);
  const heading = previous ? Math.atan2(at.y - previous.y, at.x - previous.x) : 0;
  const point = { id: uid(), x: snap(at.x), y: snap(at.y), heading, tangent: previous ? Math.max(18, Math.hypot(at.x - previous.x, at.y - previous.y)) : 30 };
  path.waypoints.push(point); selectedPointId = point.id; commit("Waypoint added");
}

function deletePoint() {
  const path = activePath();
  if (!path || !selectedPointId) return;
  const index = path.waypoints.findIndex((point) => point.id === selectedPointId);
  if (index < 0) return;
  path.waypoints.splice(index, 1);
  selectedPointId = path.waypoints[Math.min(index, path.waypoints.length - 1)]?.id ?? null;
  commit("Waypoint deleted");
}

function transformPath(mode) {
  const path = activePath(); if (!path) return;
  path.waypoints = path.waypoints.map((point) => mirrorWaypoint(point, mode));
  commit(mode === "quadrant" ? "Mirrored to the opposite same-alliance quadrant" : "Path mirrored");
}

function newPath() {
  const number = documentState.paths.length + 1;
  const path = { id:uid(), name:`Route ${number}`, color:"#171715", reversed:false, waypoints:[
    { id:uid(), x:18, y:18, heading:0, tangent:30 }, { id:uid(), x:54, y:24, heading:.2, tangent:30 }
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
  if (action === "select-tool") { notify("Select tool active"); return; }
  if (action === "new-path") return newPath();
  if (action === "add-point") return addPoint();
  if (action === "delete-point") return deletePoint();
  if (action === "undo") return restore(historyIndex - 1);
  if (action === "redo") return restore(historyIndex + 1);
  if (action === "mirror-quadrant") return transformPath("quadrant");
  if (action === "mirror-left-right") return transformPath("left-right");
  if (action === "mirror-bottom-top") return transformPath("bottom-top");
  if (action === "reverse-order") { const path=activePath(); if(path){path.waypoints=reverseWaypoints(path.waypoints);commit("Path order reversed");} return; }
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
  const pathButton = event.target.closest("[data-path-id]");
  if (pathButton) { stopPlayback(true); activePathId = pathButton.dataset.pathId; selectedPointId = activePath()?.waypoints[0]?.id ?? null; render(); }
  const alliance = event.target.closest("[data-alliance]")?.dataset.alliance;
  if (alliance) { documentState.alliance = alliance; commit(`${alliance[0].toUpperCase()+alliance.slice(1)} field view selected`); }
});

svg.addEventListener("pointerdown", (event) => {
  const pointId = event.target.dataset.pointId;
  const handleId = event.target.dataset.handleId;
  if (!pointId && !handleId) return;
  stopPlayback(true);
  selectedPointId = pointId ?? handleId;
  drag = { type:pointId ? "point" : "handle", pointId:selectedPointId, pointerId:event.pointerId };
  svg.setPointerCapture(event.pointerId); render(); event.preventDefault();
});

svg.addEventListener("pointermove", (event) => {
  const at = svgCoordinates(event);
  document.querySelector("#coordinate-readout").textContent = `X ${at.x.toFixed(1)} · Y ${at.y.toFixed(1)}`;
  if (!drag) return;
  const point = activePath()?.waypoints.find((candidate) => candidate.id === drag.pointId); if (!point) return;
  if (drag.type === "point") { point.x = clamp(snap(at.x,event.shiftKey)); point.y = clamp(snap(at.y,event.shiftKey)); }
  else { point.heading = wrapRadians(Math.atan2(at.y-point.y,at.x-point.x)); point.tangent = Math.max(3,Math.hypot(at.x-point.x,at.y-point.y)*5); }
  render();
});

svg.addEventListener("pointerup", (event) => { if (drag?.pointerId === event.pointerId) { drag=null; commit("Path control updated"); } });
svg.addEventListener("pointercancel", () => { drag=null; });
svg.addEventListener("dblclick", (event) => { if (!event.target.closest("[data-point-id],[data-handle-id]")) addPoint(svgCoordinates(event)); });

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

document.querySelector("#snap-toggle").addEventListener("change", (event) => { documentState.snap=event.target.checked;commit(); });
document.querySelector("#zone-toggle").addEventListener("change", (event) => { documentState.showZones=event.target.checked;commit(); });
document.querySelector("#drive-direction").addEventListener("change", (event) => { const path=activePath();if(path){path.reversed=event.target.value === "reverse";commit(`Drive direction set to ${event.target.value}`);} });
titleNode.addEventListener("blur", () => { documentState.title=titleNode.textContent.trim() || "Untitled path";commit(); });
titleNode.addEventListener("keydown", (event) => { if(event.key === "Enter"){event.preventDefault();titleNode.blur();} });
fileInput.addEventListener("change", async () => {
  try {
    const loaded=validateDocument(JSON.parse(await fileInput.files[0].text())); documentState=loaded;
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
