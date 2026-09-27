// Cross-checks VantagePath Studio against the C++ library it exports for:
//   1. every exported header compiles and generates without throwing, and
//   2. the JS planner's per-section duration and length match generateTrajectory.
//
// Usage: node tests/studio/crosscheck.mjs   (needs a C++17 compiler; set CXX to choose one)

import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { mkdtempSync, rmSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { cppExport } from "../../editor/export.js";
import { makeDocument, normalizeSegmentDirections, validateDocument } from "../../editor/model.js";
import { planRoute } from "../../editor/planner.js";

const root = resolve(fileURLToPath(new URL("../..", import.meta.url)));
const compiler = process.env.CXX || "c++";
const DURATION_TOLERANCE = 1e-6;

function fixtures() {
  const smooth = makeDocument();

  // The pre-2026-09 Studio default: straight Bézier legs meeting at corners.
  // The C++ generator rejects these joins, so the export must split them.
  const cornered = makeDocument();
  cornered.paths[0].controlPoints = [[], []];

  const hermiteMixed = makeDocument();
  hermiteMixed.paths[0].controlPoints = [null, null];
  hermiteMixed.paths[0].segmentReversed = [false, true];

  const reversedCurve = makeDocument();
  reversedCurve.paths[0].segmentReversed = [true, true];

  const mixedGeometry = makeDocument();
  mixedGeometry.paths[0].controlPoints[1] = null;
  mixedGeometry.robot = { ...mixedGeometry.robot, startVelocity: 20, endVelocity: 10 };

  const twoRoutes = makeDocument();
  twoRoutes.paths.push({ ...structuredClone(cornered.paths[0]), id: "second", name: "Second" });

  // Per-segment speed limits go through config.pathSpeedScale.
  const speedLimited = makeDocument();
  speedLimited.paths[0].segmentSpeed = [25, null];
  const speedCornered = makeDocument();
  speedCornered.paths[0].controlPoints = [[], []];
  speedCornered.paths[0].segmentSpeed = [null, 18];
  speedCornered.paths[0].markers = [{ id: "m", name: "Intake on", segment: 1, t: 0.4 }];
  speedCornered.paths[0].waitAfterMs = 250;

  return { smooth, cornered, hermiteMixed, reversedCurve, mixedGeometry, twoRoutes, speedLimited, speedCornered };
}

function jsSections(document, variable) {
  return document.paths.filter((path) => path.waypoints.length > 1).flatMap((path, index, list) => {
    normalizeSegmentDirections(path);
    const plan = planRoute(path, document.robot);
    assert.equal(plan.error, null, `${variable}: JS planner failed: ${plan.error}`);
    const drives = plan.steps.filter((step) => step.type === "drive");
    const suffix = list.length === 1 ? "" : `${index + 1}`;
    return drives.map((drive, sectionIndex) => ({
      name: `${variable}${suffix}${drives.length === 1 ? "" : `Section${sectionIndex + 1}`}Trajectory`,
      duration: drive.duration,
      length: drive.length,
    }));
  });
}

function main() {
  const work = mkdtempSync(join(tmpdir(), "vantage-studio-"));
  try {
    const cases = Object.entries(fixtures()).map(([key, document]) => {
      const variable = `route_${key}`;
      const valid = validateDocument(structuredClone(document));
      return { key, variable, header: cppExport(valid, variable, "corner"), sections: jsSections(valid, variable) };
    });
    const includes = cases.map(({ key, header }) => {
      writeFileSync(join(work, `${key}.hpp`), header);
      return `#include "${key}.hpp"`;
    }).join("\n");
    const prints = cases.flatMap(({ sections }) => sections.map(({ name }) =>
      `  std::printf("${name} %.9f %.9f\\n", ${name}.duration(), ${name}.length());`)).join("\n");
    writeFileSync(join(work, "main.cpp"), `#include <cstdio>\n${includes}\nint main() {\n${prints}\n}\n`);
    const binary = join(work, "crosscheck");
    execFileSync(compiler, ["-std=c++17", "-O1", `-I${join(root, "include")}`, `-I${work}`, join(work, "main.cpp"),
      join(root, "src", "trajectory.cpp"), join(root, "src", "field.cpp"), "-o", binary], { stdio: "inherit" });
    // The VantageChassis run function compiles against the stub chassis API.
    const chassisHeaders = cases.map(({ key, variable }) => {
      const document = validateDocument(structuredClone(fixtures()[key]));
      writeFileSync(join(work, `${key}_chassis.hpp`), cppExport(document, `${variable}Run`, "corner", "chassis"));
      return `#include "${key}_chassis.hpp"`;
    }).join("\n");
    writeFileSync(join(work, "chassis.cpp"), `${chassisHeaders}\nint main() { drive::VantageChassis chassis; ${cases.map(({ variable }) => `run${variable[0].toUpperCase()}${variable.slice(1)}Run(chassis);`).join(" ")} }\n`);
    execFileSync(compiler, ["-std=c++17", "-fsyntax-only", `-I${join(root, "include")}`, `-I${join(root, "tests", "studio", "stub")}`, `-I${work}`, join(work, "chassis.cpp")], { stdio: "inherit" });
    console.log("ok  VantageChassis run functions compile against the chassis API");
    // Static initialisation throws (and aborts) if any exported route is rejected.
    const output = execFileSync(binary, { encoding: "utf8" });
    const native = new Map(output.trim().split("\n").map((line) => {
      const [name, duration, length] = line.split(" ");
      return [name, { duration: Number(duration), length: Number(length) }];
    }));
    for (const { key, sections } of cases) {
      for (const section of sections) {
        const cpp = native.get(section.name);
        assert.ok(cpp, `${key}: ${section.name} missing from native output`);
        assert.ok(Math.abs(cpp.duration - section.duration) < DURATION_TOLERANCE * Math.max(1, cpp.duration),
          `${key}: ${section.name} duration JS ${section.duration} vs C++ ${cpp.duration}`);
        assert.ok(Math.abs(cpp.length - section.length) < 1e-6 * Math.max(1, cpp.length),
          `${key}: ${section.name} length JS ${section.length} vs C++ ${cpp.length}`);
      }
      console.log(`ok  ${key}: ${sections.length} section(s) match the C++ generator`);
    }
  } finally {
    rmSync(work, { recursive: true, force: true });
  }
}

main();
