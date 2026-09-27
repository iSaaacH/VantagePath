// Small helpers shared by the view modules.

const ENTITIES = { "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" };

export function escapeHtml(value) {
  return String(value).replace(/[&<>'"]/g, (character) => ENTITIES[character]);
}

export function plural(count, word) {
  return `${count} ${word}${count === 1 ? "" : "s"}`;
}

export function fixed(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}
