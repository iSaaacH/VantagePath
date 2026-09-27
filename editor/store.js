// Studio state: the document, what is selected, undo history and autosave.
// Views read `state`; edits go through commit() (undoable) or setView()
// (view preferences, never undone).

import { analyzeRoute } from "./analysis.js";
import { historySnapshot, makeDocument, normalizeSegmentDirections, restoreSnapshot, robotStartPose, validateDocument, withDerivedHeadings } from "./model.js";
import { planRoute } from "./planner.js";

export const STORAGE_KEY = "vantagepath-studio-v1";
const PREFS_KEY = "vantagepath-studio-prefs";
const NUDGE_COALESCE_MS = 800;

const hooks = { render: () => {}, notify: () => {} };

export const state = {
  doc: null,
  prefs: { headingMode: "math" },
  activePathId: null,
  selection: { pointId: null, controlId: null, segment: 0 },
  history: [],
  historyIndex: 0,
  lastCoalesce: { key: null, at: 0 },
  save: { ok: true, at: null, reason: "" },
  startupMessage: "",
};

let planCache = { key: "", plan: null, issues: [] };

function readStorage(key) {
  try { return localStorage.getItem(key); } catch { return null; }
}

function loadDocument() {
  const raw = readStorage(STORAGE_KEY);
  if (raw === null) return makeDocument();
  try {
    return validateDocument(JSON.parse(raw));
  } catch (error) {
    // Keep the unreadable copy so the next autosave can't destroy it.
    const backupKey = `${STORAGE_KEY}-unreadable-${Date.now()}`;
    try { localStorage.setItem(backupKey, raw); } catch { /* storage full or blocked */ }
    state.startupMessage = `Your saved route couldn't be read (${error instanceof Error ? error.message : "unknown error"}). A copy was kept as "${backupKey}"; starting fresh.`;
    return makeDocument();
  }
}

function loadPrefs() {
  try {
    const stored = JSON.parse(readStorage(PREFS_KEY) ?? "{}");
    return { headingMode: stored.headingMode === "compass" ? "compass" : "math" };
  } catch { return { headingMode: "math" }; }
}

export function initStore({ render, notify }) {
  hooks.render = render;
  hooks.notify = notify;
  state.doc = loadDocument();
  state.prefs = loadPrefs();
  replaceDocument(state.doc);
}

/** Loads a whole new document (startup or Open) with fresh history. */
export function replaceDocument(document) {
  state.doc = document;
  state.activePathId = document.paths[0]?.id ?? null;
  state.selection = { pointId: document.paths[0]?.waypoints[0]?.id ?? null, controlId: null, segment: 0 };
  state.history = [historySnapshot(document)];
  state.historyIndex = 0;
  state.lastCoalesce = { key: null, at: 0 };
}

export function setPrefs(changes) {
  state.prefs = { ...state.prefs, ...changes };
  try { localStorage.setItem(PREFS_KEY, JSON.stringify(state.prefs)); } catch { /* preferences are optional */ }
  hooks.render();
}

export function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(state.doc));
    state.save = { ok: true, at: new Date(), reason: "" };
  } catch (error) {
    state.save = { ok: false, at: new Date(), reason: error instanceof Error ? error.message : "storage unavailable" };
  }
}

export function activePath() {
  return state.doc.paths.find((path) => path.id === state.activePathId) ?? state.doc.paths[0];
}

export function selectedPoint() {
  return activePath()?.waypoints.find((point) => point.id === state.selection.pointId) ?? null;
}

export function selectedControl() {
  const { controlId, segment } = state.selection;
  return controlId ? activePath()?.controlPoints?.[segment]?.find((point) => point.id === controlId) ?? null : null;
}

export function select(changes) {
  state.selection = { ...state.selection, ...changes };
  const path = activePath();
  const lastSegment = Math.max(0, (path?.waypoints.length ?? 1) - 2);
  state.selection.segment = Math.max(0, Math.min(state.selection.segment, lastSegment));
}

export function setActivePath(id) {
  const path = state.doc.paths.find((candidate) => candidate.id === id);
  if (!path) return;
  state.activePathId = id;
  select({ pointId: path.waypoints[0]?.id ?? null, controlId: null, segment: 0 });
}

/** The active route planned exactly as the robot will run it, with checks. */
export function routePlan(path = activePath()) {
  const key = JSON.stringify([path, state.doc.robot]);
  if (planCache.key === key) return planCache;
  if (path) normalizeSegmentDirections(path);
  const plan = planRoute(path, state.doc.robot);
  planCache = { key, plan, issues: analyzeRoute(path, plan, state.doc.robot) };
  return planCache;
}

/**
 * Records an undoable route edit. `coalesce` merges a quick burst of edits
 * with the same key (arrow-key nudges) into one undo step.
 */
export function commit(message, { coalesce = null } = {}) {
  state.doc.paths = state.doc.paths.map(withDerivedHeadings);
  state.doc.robotStart = robotStartPose(state.doc);
  const serial = historySnapshot(state.doc);
  const now = performance.now();
  const merge = coalesce && state.lastCoalesce.key === coalesce && now - state.lastCoalesce.at < NUDGE_COALESCE_MS && state.historyIndex > 0;
  if (state.history[state.historyIndex] !== serial) {
    if (merge) state.history[state.historyIndex] = serial;
    else {
      state.history = [...state.history.slice(0, state.historyIndex + 1), serial];
      state.historyIndex += 1;
    }
  }
  state.lastCoalesce = { key: coalesce, at: now };
  persist();
  hooks.render();
  if (message) hooks.notify(message);
}

/** Changes a saved view preference without an undo step. */
export function setView(key, value) {
  state.doc[key] = value;
  persist();
  hooks.render();
}

export function canUndo() { return state.historyIndex > 0; }
export function canRedo() { return state.historyIndex < state.history.length - 1; }

export function restore(index) {
  if (index < 0 || index >= state.history.length) return false;
  state.historyIndex = index;
  state.lastCoalesce = { key: null, at: 0 };
  state.doc = restoreSnapshot(state.history[index], state.doc);
  if (!state.doc.paths.some((path) => path.id === state.activePathId)) state.activePathId = state.doc.paths[0]?.id ?? null;
  const path = activePath();
  if (!path?.waypoints.some((point) => point.id === state.selection.pointId)) {
    select({ pointId: path?.waypoints[0]?.id ?? null, controlId: null });
  } else if (state.selection.controlId && !selectedControl()) {
    select({ controlId: null });
  }
  persist();
  hooks.render();
  return true;
}
