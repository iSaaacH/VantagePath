import assert from "node:assert/strict";
import { FIELD_SIZE, cornerToGps, cppExport, directionRuns, estimateLength, makeDocument, mirrorWaypoint, motionProfile, profileDistance, reverseWaypoints, validateDocument } from "./model.js";

assert.equal(FIELD_SIZE, 144, "the playable floor is exactly 144 inches");
assert.deepEqual(
  mirrorWaypoint({ x:0, y:0, heading:0, tangent:10 }, "alliance"),
  { x:144, y:144, heading:Math.PI, tangent:10 },
  "alliance mirroring uses the full 0..144 field bounds",
);

const centre = cornerToGps({ x:72, y:72, heading:Math.PI/2, tangent:10 });
assert.equal(centre.x,0); assert.equal(centre.y,0); assert.ok(Math.abs(centre.heading)<1e-12);
const mirrored = mirrorWaypoint({ x:12,y:24,heading:0.25,tangent:30 },"alliance");
assert.equal(mirrored.x,132); assert.equal(mirrored.y,120);
const quadrant = mirrorWaypoint({ x:24,y:48,heading:0,tangent:30 },"quadrant");
assert.equal(quadrant.x,48); assert.equal(quadrant.y,24); assert.equal(quadrant.heading,Math.PI/2);
const reversed = reverseWaypoints([{id:"a",x:1,y:2,heading:0,tangent:3},{id:"b",x:4,y:5,heading:1,tangent:6}]);
assert.equal(reversed[0].id,"b"); assert.equal(reversed[0].tangent,6);
assert.ok(estimateLength([{x:0,y:0,heading:0,tangent:10},{x:10,y:0,heading:0,tangent:10}])-10<1e-8);
const document = makeDocument();
assert.match(cppExport(document,"competitionAuto","gps"),/official GPS centre frame/);
assert.match(cppExport(document,"competitionAuto","gps"),/vexGpsToCorner/);
assert.match(cppExport(document,"competitionAuto","corner"),/config\.maxVelocity = 60\.0000/);
assert.match(cppExport(document,"competitionAuto","gps"),/config\.maxVelocity = 1\.5240/);
assert.match(cppExport(document,"bad-name","corner"),/generatedPath/);
document.paths[0].segmentReversed = [false, true];
assert.deepEqual(directionRuns(document.paths[0]).map((run) => run.reversed), [false, true]);
const mixedExport = cppExport(document,"competitionAuto","corner");
assert.match(mixedExport,/competitionAutoSegment1Config[\s\S]*config\.reversed = false/);
assert.match(mixedExport,/competitionAutoSegment2Config[\s\S]*config\.reversed = true/);
assert.match(mixedExport,/competitionAutoTrajectories/, "mixed direction sections are exported in execution order");
const profile = motionProfile(120, document.robot);
assert.ok(profile.duration > 0); assert.equal(profileDistance(profile, profile.duration), 120);
const legacy = makeDocument(); delete legacy.robot; delete legacy.paths[0].segmentReversed; legacy.paths[0].reversed = true;
assert.equal(validateDocument(legacy).robot.length, 18);
assert.deepEqual(validateDocument(legacy).paths[0].segmentReversed, [true, true], "legacy route direction migrates to every segment");
console.log("All VantagePath Studio model tests passed");
