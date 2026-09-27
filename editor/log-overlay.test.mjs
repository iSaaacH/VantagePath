import assert from "node:assert/strict";
import test from "node:test";
import { deviationFromPlan, parseLogCsv } from "./log-overlay.js";

test("named x/y columns are found in any order, with unit suffixes", () => {
  const { points } = parseLogCsv("time_ms,y_in,x_in\n0,18,20\n10,19,21\n");
  assert.deepEqual(points, [{ x: 20, y: 18 }, { x: 21, y: 19 }]);
});

test("headerless files use the first two columns and skip bad rows", () => {
  const { points, skipped } = parseLogCsv("1,2\n# comment\n3,4\nnope,5\n6,7");
  assert.equal(points.length, 3);
  assert.equal(skipped, 1);
});

test("files without two usable rows are rejected with a readable message", () => {
  assert.throws(() => parseLogCsv(""), /empty/);
  assert.throws(() => parseLogCsv("a,b\nx,y"), /No x,y rows/);
});

test("deviation measures distance to the planned path", () => {
  const plan = [{ x: 0, y: 0 }, { x: 10, y: 0 }];
  const result = deviationFromPlan([{ x: 5, y: 2 }, { x: 8, y: 0 }], plan);
  assert.equal(result.worst, 2);
  assert.equal(result.average, 1);
});
