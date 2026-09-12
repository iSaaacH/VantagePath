import assert from "node:assert/strict";
import { bezierPoint, segmentPoint, setControlCount, FIELD_SIZE, cornerToGps, cppExport, directionRuns, estimateLength, makeDocument, mirrorWaypoint, motionProfile, profileDistance, quinticPoint, reverseWaypoints, validateDocument, wrapRadians } from "./model.js";

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
// A reversed segment must back out of a waypoint: with the nose pointing "up"
// (heading = +y) toward a goal above it, driving in reverse to a point below
// should trace downward, not loop up-and-around.
const goal = { x:72, y:96, heading:Math.PI / 2, tangent:24 };
const behind = { x:72, y:48, heading:Math.PI / 2, tangent:24 };
const forwardStep = quinticPoint(goal, behind, 0.1, false);
const reverseStep = quinticPoint(goal, behind, 0.1, true);
assert.ok(forwardStep.y > goal.y, "forward tangent leaves the goal along the nose direction");
assert.ok(reverseStep.y < goal.y, "reversed tangent backs out opposite the nose direction");

// A reversed export run rotates its authored nose headings by π so the forward
// geometry the C++ generator builds becomes the intended back-out curve.
const reverseDoc = makeDocument();
reverseDoc.paths[0].segmentReversed = [false, true];
const reverseExport = cppExport(reverseDoc, "competitionAuto", "corner");
const flippedHeading = wrapRadians(reverseDoc.paths[0].waypoints[2].heading + Math.PI).toFixed(6);
assert.match(reverseExport, new RegExp(`${flippedHeading}}`), "reversed run emits π-rotated headings");
assert.match(reverseExport, /Segment2[\s\S]*config\.reversed = true/, "reversed run still flags config.reversed");

const profile = motionProfile(120, document.robot);
assert.ok(profile.duration > 0); assert.equal(profileDistance(profile, profile.duration), 120);
const legacy = makeDocument(); delete legacy.robot; delete legacy.paths[0].segmentReversed; legacy.paths[0].reversed = true;
assert.equal(validateDocument(legacy).robot.length, 18);
assert.deepEqual(validateDocument(legacy).paths[0].segmentReversed, [true, true], "legacy route direction migrates to every segment");
console.log("All VantagePath Studio model tests passed");

const bezierDoc = makeDocument();
const route = bezierDoc.paths[0];
setControlCount(route, 0, 0);
assert.deepEqual(segmentPoint(route, 0, 0.5), {x:43, y:34}, "zero controls produces the straight midpoint regardless of headings");
setControlCount(route, 0, 6);
assert.equal(route.controlPoints[0].length, 6);
route.controlPoints[0][2].y = 100;
const middle = segmentPoint(route, 0, 0.5);
assert.ok(middle.y > 34, "six controls affect the actual geometry");
assert.deepEqual(segmentPoint(route, 0, 0), {x:18, y:18});
assert.deepEqual(segmentPoint(route, 0, 1), {x:68, y:50});
assert.deepEqual(bezierPoint([{x:0,y:0},{x:1,y:2},{x:2,y:0}], 0.5), {x:1,y:1});
assert.deepEqual(validateDocument(JSON.parse(JSON.stringify(bezierDoc))).paths[0].controlPoints, route.controlPoints);
assert.match(cppExport(bezierDoc, "curve"), /true, \{\{25\.1429/);
assert.match(cppExport(bezierDoc, "curve", "gps"), /control = vantage::vexGpsToCorner/);
const oldDoc = makeDocument(); delete oldDoc.paths[0].controlPoints;
assert.deepEqual(validateDocument(oldDoc).paths[0].controlPoints, [null, null], "old files retain Hermite geometry");
route.controlPoints[0][0].x = NaN;
assert.throws(() => validateDocument(bezierDoc), /Control coordinates/);
