// Odometry log overlay: parse a CSV the robot recorded and compare it with
// the planned route. Columns named x and y (any case, optional unit suffix
// like x_in) are used; otherwise the first two numeric columns. Corner-frame
// inches, the same frame Studio edits in.

const MAX_POINTS = 20000;
const MAX_BYTES = 5 * 1024 * 1024;

function splitRow(line) {
  return line.split(/[,;\t]/).map((cell) => cell.trim());
}

function columnIndex(header, axis) {
  return header.findIndex((name) => new RegExp(`^${axis}(?:[_ (\\[].*)?$`, "i").test(name));
}

/** Returns { points: [{x, y}], skipped } or throws with a readable message. */
export function parseLogCsv(text) {
  if (typeof text !== "string" || !text.trim()) throw new Error("The log file is empty.");
  if (text.length > MAX_BYTES) throw new Error("The log file is over 5 MB.");
  const lines = text.split(/\r?\n/).filter((line) => line.trim() && !line.trim().startsWith("#"));
  const first = splitRow(lines[0]);
  const hasHeader = first.some((cell) => cell !== "" && !Number.isFinite(Number(cell)));
  let xColumn = 0;
  let yColumn = 1;
  if (hasHeader) {
    const x = columnIndex(first, "x");
    const y = columnIndex(first, "y");
    if (x >= 0 && y >= 0) { xColumn = x; yColumn = y; }
  }
  const points = [];
  let skipped = 0;
  for (const line of lines.slice(hasHeader ? 1 : 0)) {
    const cells = splitRow(line);
    const x = Number(cells[xColumn]);
    const y = Number(cells[yColumn]);
    if (!Number.isFinite(x) || !Number.isFinite(y) || cells[xColumn] === "" || cells[yColumn] === "") { skipped += 1; continue; }
    points.push({ x, y });
    if (points.length >= MAX_POINTS) break;
  }
  if (points.length < 2) throw new Error("No x,y rows found. Use columns named x and y, in inches from the bottom-left corner.");
  return { points, skipped };
}

/** Largest and average distance from each log point to the planned path. */
export function deviationFromPlan(points, routeSamples) {
  if (!routeSamples.length || !points.length) return null;
  let worst = 0;
  let sum = 0;
  for (const point of points) {
    let best = Infinity;
    for (let i = 1; i < routeSamples.length; i += 1) {
      const a = routeSamples[i - 1];
      const b = routeSamples[i];
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const lengthSquared = dx * dx + dy * dy;
      const t = lengthSquared > 0 ? Math.min(1, Math.max(0, ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared)) : 0;
      best = Math.min(best, Math.hypot(a.x + dx * t - point.x, a.y + dy * t - point.y));
    }
    worst = Math.max(worst, best);
    sum += best;
  }
  return { worst, average: sum / points.length };
}
