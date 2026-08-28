import { FIELD_SIZE, clamp, cppExport, estimateLength, makeDocument, mirrorWaypoint, quinticPoint, reverseWaypoints, validateDocument, wrapRadians } from "./model.js";

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

let documentState = loadLocal();
let activePathId = documentState.paths[0]?.id ?? null;
let selectedPointId = documentState.paths[0]?.waypoints[0]?.id ?? null;
let history = [JSON.stringify(documentState)];
let historyIndex = 0;
let drag = null;
let toastTimer = 0;

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
  const serial = JSON.stringify(documentState);
  if (history[historyIndex] !== serial) {
    history = history.slice(0, historyIndex + 1);
    history.push(serial);
    historyIndex += 1;
  }
  persist(); render();
  if (message) notify(message);
}

function restore(index) {
  if (index < 0 || index >= history.length) return;
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
      ${index === 0 ? `<rect class="robot-box" x="${point.x - 9}" y="${FIELD_SIZE - point.y - 9}" width="18" height="18" transform="rotate(${-point.heading * 180 / Math.PI} ${point.x} ${FIELD_SIZE - point.y})"/>` : ""}
      <circle class="anchor ${selected ? "selected" : ""}" data-point-id="${point.id}" cx="${point.x}" cy="${FIELD_SIZE - point.y}" r="2.8"/>
      <text class="anchor-label" x="${point.x + 3.8}" y="${FIELD_SIZE - point.y - 3}">${String(index + 1).padStart(2,"0")}</text></g>`;
  }).join("");
  return `${d ? `<path class="path-shadow" d="${d}"/><path class="path-curve ${path.reversed ? "reversed" : ""}" d="${d}"/>` : ""}${controls}`;
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
  document.querySelectorAll("[data-alliance]").forEach((button) => button.classList.toggle("is-active", button.dataset.alliance === documentState.alliance));
  document.querySelector('[data-action="undo"]').disabled = historyIndex === 0;
  document.querySelector('[data-action="redo"]').disabled = historyIndex === history.length - 1;
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
  commit(mode === "alliance" ? "Moved to opposite alliance side" : "Path mirrored");
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
  if (action === "mirror-alliance") return transformPath("alliance");
  if (action === "mirror-left-right") return transformPath("left-right");
  if (action === "mirror-bottom-top") return transformPath("bottom-top");
  if (action === "reverse-order") { const path=activePath(); if(path){path.waypoints=reverseWaypoints(path.waypoints);commit("Path order reversed");} return; }
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
  if (pathButton) { activePathId = pathButton.dataset.pathId; selectedPointId = activePath()?.waypoints[0]?.id ?? null; render(); }
  const alliance = event.target.closest("[data-alliance]")?.dataset.alliance;
  if (alliance) { documentState.alliance = alliance; commit(`${alliance[0].toUpperCase()+alliance.slice(1)} field view selected`); }
});

svg.addEventListener("pointerdown", (event) => {
  const pointId = event.target.dataset.pointId;
  const handleId = event.target.dataset.handleId;
  if (!pointId && !handleId) return;
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
