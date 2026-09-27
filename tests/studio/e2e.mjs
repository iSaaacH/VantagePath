// End-to-end browser test for VantagePath Studio: loads the editor, drives
// the main flows (edit, insert, drag, nudge + undo, checks, markers, speed
// limits, waits, exports, log overlay, delete) and checks layouts from
// desktop to phone width with no console errors.
//
// Usage: npm --prefix tests/studio ci && npx --prefix tests/studio playwright install chromium
//        node tests/studio/e2e.mjs            (PW_CHANNEL=chrome uses installed Chrome)
import { chromium } from "playwright";
import assert from "node:assert/strict";
import { createReadStream, mkdtempSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { createServer } from "node:http";
import { tmpdir } from "node:os";
import { extname, join, normalize, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const editorRoot = resolve(fileURLToPath(new URL("../../editor", import.meta.url)));
const OUT = mkdtempSync(join(tmpdir(), "vantage-studio-e2e-")) + "/";
const TYPES = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".webp": "image/webp", ".mjs": "text/javascript" };

// Serves editor/ read-only on a random local port.
const server = createServer((request, response) => {
  const path = normalize(decodeURIComponent(new URL(request.url, "http://x").pathname)).replace(/^([/\\])+/, "");
  const file = resolve(editorRoot, path || "index.html");
  if (!file.startsWith(editorRoot)) { response.writeHead(403).end(); return; }
  try {
    if (!statSync(file).isFile()) throw new Error("not a file");
    response.writeHead(200, { "content-type": TYPES[extname(file)] ?? "application/octet-stream" });
    createReadStream(file).pipe(response);
  } catch { response.writeHead(404).end(); }
});
await new Promise((ready) => server.listen(0, "127.0.0.1", ready));
const APP = `http://127.0.0.1:${server.address().port}/`;
writeFileSync(`${OUT}run.csv`, "time_ms,x_in,y_in\n0,126,126\n100,110,120\n200,95,105\n300,80,95\n");

const browser = await chromium.launch({ headless: true, ...(process.env.PW_CHANNEL ? { channel: process.env.PW_CHANNEL } : {}) });
const errors = [];
const page = await browser.newPage({ viewport: { width: 1440, height: 900 }, acceptDownloads: true });
page.on("pageerror", (e) => errors.push(`pageerror: ${e.message}`));
page.on("console", (m) => { if (m.type() === "error") errors.push(`console: ${m.text()}`); });
const text = (sel) => page.locator(sel).first().innerText();
const stored = () => page.evaluate(() => JSON.parse(localStorage.getItem("vantagepath-studio-v1")));

await page.goto(APP); await page.evaluate(() => localStorage.clear()); await page.goto(APP);
await page.waitForSelector("#field .anchor");
await page.screenshot({ path: `${OUT}p2-desktop.png` });
console.log("summary:", await text("#path-summary"), "| checks:", await text("#checks-count"));
assert.match(await text("#path-summary"), /\d+\.\d in · \d+\.\d\d s · 1 stop/);
assert.equal(await page.locator(".outline-point").count(), 3);
assert.equal(await page.locator(".outline-segment").count(), 2);

const box = await page.locator("#field").boundingBox();
console.log("field box", Math.round(box.width), "x", Math.round(box.height));
// Field is drawn with preserveAspectRatio meet: compute the square.
const side = Math.min(box.width, box.height);
const ox = box.x + (box.width - side) / 2, oy = box.y + (box.height - side) / 2;
const toScreen = (x, y) => ({ x: ox + (x + 9) / 162 * side, y: oy + (144 - y + 9) / 162 * side });

// Heading switch + derived start heading.
assert.equal(await page.inputValue('[data-start="heading"]'), "0.0");
await page.selectOption("#heading-mode", "compass");
assert.equal(await page.inputValue('[data-start="heading"]'), "90.0");
assert.ok(await page.locator('[data-start="heading"]').isDisabled());

// Alliance view does not create undo steps.
await page.click('[data-alliance="blue"]');
assert.ok(await page.locator('[data-action="undo"]').first().isDisabled());

// Insert on curve.
const onCurve = await page.evaluate(async () => { const { segmentPoint } = await import("./model.js"); const d = JSON.parse(localStorage.getItem("vantagepath-studio-v1")); return segmentPoint(d.paths[0], 1, 0.5); });
const at = toScreen(onCurve.x, onCurve.y);
await page.mouse.dblclick(at.x, at.y);
assert.equal(await page.locator(".outline-point").count(), 4);

// Drag a point on the field.
const p4 = (await stored()).paths[0].waypoints[3];
const from = toScreen(p4.x, p4.y);
await page.mouse.move(from.x, from.y); await page.mouse.down();
await page.mouse.move(from.x - 30, from.y + 20, { steps: 5 }); await page.mouse.up();
const moved = (await stored()).paths[0].waypoints[3];
assert.ok(moved.x < p4.x, "dragging moves the point");

// Arrow nudges coalesce.
const before = (await stored()).paths[0].waypoints[3];
for (const k of ["ArrowRight", "ArrowRight", "ArrowUp"]) await page.keyboard.press(k);
const after = (await stored()).paths[0].waypoints[3];
assert.equal(after.x, before.x + 2); assert.equal(after.y, before.y + 1);
await page.keyboard.press("Meta+z");
const undone = (await stored()).paths[0].waypoints[3];
assert.equal(undone.x, before.x); assert.equal(undone.y, before.y);

// Inspector edit keeps working.
await page.fill('[data-edit="point-x"]', "100");
await page.press('[data-edit="point-x"]', "Enter");
await page.locator('[data-edit="point-x"]').blur();
assert.equal((await stored()).paths[0].waypoints[3].x, 100);

// Append keeps the route free of corners.
await page.keyboard.press("a");
assert.doesNotMatch(await text("#checks-list"), /Sharp corner/);

// Segment direction chip.
await page.click('[data-toggle-direction="0"]');
assert.equal((await stored()).paths[0].segmentReversed[0], true);
await page.keyboard.press("Meta+z");

// Speed graph scrub moves playback.
const g = await page.locator("#speed-graph").boundingBox();
await page.mouse.click(g.x + g.width * 0.5, g.y + g.height / 2);
assert.doesNotMatch(await text("#playback-time"), /^0\.00 \//);

// Settings drawer: alliance copy.
await page.click('[data-action="open-settings"]');
await page.click('[data-action="mirror-alliance-copy"]');
await page.keyboard.press("Escape");
assert.equal(await page.locator(".route").count(), 2);

// Export dialog with preview and download.
await page.click('[data-action="open-export"]');
assert.match(await text("#export-preview"), /sampleDistance = 0\.3500/);
const [download] = await Promise.all([page.waitForEvent("download"), page.click('[data-action="download-export"]')]);
assert.match(readFileSync(await download.path(), "utf8"), /Export format 3/);
await page.screenshot({ path: `${OUT}p2-export.png` });
await page.keyboard.press("Escape");

// Chassis export target.
await page.click('[data-action="open-export"]');
await page.selectOption("#export-target", "chassis");
assert.match(await text("#export-preview"), /inline void runCompetitionAuto\(drive::VantageChassis& chassis\)/);
assert.ok(await page.locator("#export-frame").isDisabled());
await page.selectOption("#export-target", "library");
await page.keyboard.press("Escape");

// Phase 3: speed limit slows the plan.
await page.click('[data-select-segment="0"]');
const beforeSpeed = await text("#path-summary");
await page.fill('[data-edit="segment-speed"]', "20");
await page.locator('[data-edit="segment-speed"]').blur();
const afterSpeed = await text("#path-summary");
console.log("speed limit:", beforeSpeed, "->", afterSpeed);
assert.notEqual(afterSpeed, beforeSpeed);

// Markers: add, rename, see on graph, drag along the path.
await page.click('[data-action="add-marker"]');
assert.equal(await page.locator(".event-marker").count(), 1);
await page.fill('[data-edit="marker-name"]', "Intake on");
await page.locator('[data-edit="marker-name"]').blur();
assert.equal((await stored()).paths.find((p) => p.markers?.length).markers[0].name, "Intake on");
assert.equal(await page.locator(".graph-marker").count(), 1);
const markerBox = await page.locator(".event-marker .event-dot").boundingBox();
const t0 = (await stored()).paths.find((p) => p.markers?.length).markers[0].t;
await page.mouse.move(markerBox.x + markerBox.width / 2, markerBox.y + markerBox.height / 2);
await page.mouse.down(); await page.mouse.move(markerBox.x + 40, markerBox.y - 25, { steps: 6 }); await page.mouse.up();
const t1 = (await stored()).paths.find((p) => p.markers?.length).markers[0].t;
console.log("marker t:", t0, "->", t1);
assert.notEqual(t1, t0);

// Waits count toward the budget; routes reorder.
const budget0 = await text(".budget-text b");
await page.fill('[data-edit="wait-after"]', "2000");
await page.locator('[data-edit="wait-after"]').blur();
const budget1 = await text(".budget-text b");
console.log("budget:", budget0, "->", budget1);
assert.notEqual(budget0, budget1);
const names0 = (await stored()).paths.map((p) => p.name);
await page.click('[data-action="route-up"]');
const names1 = (await stored()).paths.map((p) => p.name);
assert.deepEqual(names1, [names0[1], names0[0]]);

// Footprint toggle draws ghosts.
await page.check("#footprint-toggle");
assert.ok(await page.locator(".footprint").count() > 3);
await page.screenshot({ path: `${OUT}p3-desktop.png` });

// Odometry log overlay.
await page.click('[data-action="open-settings"]');
await page.setInputFiles("#log-input", `${OUT}run.csv`);
await page.waitForFunction(() => document.querySelector("#log-status").textContent.length > 0);
assert.match(await text("#log-status"), /run\.csv: 4 points\. Off the active route by up to \d+\.\d in/);
await page.keyboard.press("Escape");
assert.equal(await page.locator(".log-overlay polyline").count(), 1);

// Two-step route delete.
await page.click("#delete-path");
assert.match(await text("#delete-path"), /Click again/i);
await page.click("#delete-path");
assert.equal(await page.locator(".route").count(), 1);

// Corner + Smooth join.
await page.evaluate(async () => { const { makeDocument } = await import("./model.js"); const d = makeDocument(); d.paths[0].controlPoints = [[], []]; localStorage.setItem("vantagepath-studio-v1", JSON.stringify(d)); });
await page.goto(APP); await page.waitForSelector("#field .anchor");
assert.match(await text("#checks-list"), /Sharp corner at P2/);
await page.screenshot({ path: `${OUT}p2-corner.png` });
await page.click('.check-fix[data-smooth-index="1"]');
assert.doesNotMatch(await text("#checks-list"), /Sharp corner/);

// Laptop: routes sheet toggles.
await page.setViewportSize({ width: 1100, height: 800 });
assert.ok(await page.locator('[data-sheet="outline"]').isVisible());
await page.click('[data-sheet="outline"]');
await page.waitForTimeout(300);
await page.screenshot({ path: `${OUT}p2-laptop-sheet.png` });
await page.keyboard.press("Escape");

// Tablet and phone: no horizontal scroll; inspector sheet opens.
for (const [w, h] of [[820, 1180], [390, 844]]) {
  await page.setViewportSize({ width: w, height: h });
  await page.waitForTimeout(250);
  const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
  assert.equal(overflow, 0, `no horizontal scroll at ${w}px`);
  await page.screenshot({ path: `${OUT}p2-${w}.png` });
}
await page.click('[data-sheet="inspector"]');
await page.waitForTimeout(300);
await page.screenshot({ path: `${OUT}p2-390-inspector.png` });

// Small text and small targets audit at desktop.
await page.setViewportSize({ width: 1440, height: 900 });
await page.keyboard.press("Escape");
const audit = await page.evaluate(() => {
  const visible = (el) => { const r = el.getBoundingClientRect(); return r.width > 0 && r.height > 0 && getComputedStyle(el).visibility !== "hidden"; };
  const texts = [...document.querySelectorAll("body *:not(svg *)")].filter((el) => visible(el) && [...el.childNodes].some((n) => n.nodeType === 3 && n.textContent.trim()));
  const small = texts.filter((el) => parseFloat(getComputedStyle(el).fontSize) < 11).map((el) => el.className || el.tagName);
  const controls = [...document.querySelectorAll("button, input, select")].filter(visible);
  const tiny = controls.filter((el) => el.getBoundingClientRect().height < 32).map((el) => el.outerHTML.slice(0, 70));
  return { texts: texts.length, small, controls: controls.length, tiny };
});
console.log(`text <11px: ${audit.small.length}/${audit.texts}`, audit.small.slice(0, 5));
console.log(`controls <32px: ${audit.tiny.length}/${audit.controls}`, audit.tiny.slice(0, 5));

console.log(errors.length ? `ERRORS:\n${errors.join("\n")}` : "no console errors");
await browser.close();
server.close();
assert.equal(errors.length, 0);
if (!process.env.KEEP_SCREENSHOTS) rmSync(OUT, { recursive: true, force: true });
else console.log(`screenshots in ${OUT}`);
console.log("Studio e2e OK");
