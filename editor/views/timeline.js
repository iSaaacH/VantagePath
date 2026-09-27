// Speed-over-time graph for the planned route. Drawn in a 1000 x 100 box that
// stretches to fit; labels live in HTML so they don't stretch with it.

const WIDTH = 1000;
const HEIGHT = 100;
const TOP_PAD = 8;

export function speedGraphMarkup(plan) {
  if (!plan || plan.error || !(plan.duration > 0)) return { svg: "", peak: 0 };
  const drives = plan.steps.filter((step) => step.type === "drive");
  const peak = Math.max(1, ...drives.flatMap((step) => step.states.map((state) => Math.abs(state.velocity))));
  const x = (time) => (time / plan.duration * WIDTH).toFixed(2);
  const y = (speed) => (HEIGHT - Math.abs(speed) / peak * (HEIGHT - TOP_PAD)).toFixed(2);
  const turns = plan.steps.filter((step) => step.type === "turn").map((step) =>
    `<rect class="graph-turn" x="${x(step.startTime)}" y="0" width="${Math.max(1, step.duration / plan.duration * WIDTH).toFixed(2)}" height="${HEIGHT}"><title>Turn in place at P${step.waypointIndex + 1}</title></rect>`).join("");
  const areas = drives.map((step) => {
    const points = step.states.map((state) => `${x(step.startTime + state.time)},${y(state.velocity)}`);
    const first = x(step.startTime);
    const last = x(step.startTime + step.duration);
    return `<polygon class="graph-area${step.section.reversed ? " is-reverse" : ""}" points="${first},${HEIGHT} ${points.join(" ")} ${last},${HEIGHT}"/><polyline class="graph-line" points="${points.join(" ")}"/>`;
  }).join("");
  const stops = drives.flatMap((step) => step.states.filter((state, i) => i > 0 && i < step.states.length - 1 && state.velocity === 0)
    .map((state) => `<line class="graph-stop" x1="${x(step.startTime + state.time)}" x2="${x(step.startTime + state.time)}" y1="${HEIGHT - 14}" y2="${HEIGHT}"/>`)).join("");
  const svg = `${turns}${areas}${stops}<line id="graph-playhead" class="graph-playhead" x1="0" x2="0" y1="0" y2="${HEIGHT}"/>`;
  return { svg, peak };
}

export function playheadX(plan, time) {
  return plan?.duration > 0 ? (time / plan.duration * WIDTH).toFixed(2) : "0";
}

export const GRAPH_VIEWBOX = `0 0 ${WIDTH} ${HEIGHT}`;
