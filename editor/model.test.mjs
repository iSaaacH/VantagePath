import assert from "node:assert/strict";
import { dragPosition, bezierPoint, segmentPoint, setControlCount, DEFAULT_ROBOT, FIELD_SIZE, cornerToGps, cppExport, directionRuns, estimateLength, makeDocument, mirrorWaypoint, motionProfile, nearestPathDistance, profileDistance, profileTimeAtDistance, quinticPoint, reverseWaypoints, validateDocument, wrapRadians } from "./model.js";

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

// Start placement persists independently of route geometry, including old files.
const startDocument = makeDocument();
const routeBeforeStartEdit = structuredClone(startDocument.paths);
startDocument.robotStart = {x:30.25,y:48.5,heading:Math.PI/2};
assert.deepEqual(startDocument.paths,routeBeforeStartEdit);
const startReloaded = validateDocument(JSON.parse(JSON.stringify(startDocument)));
assert.deepEqual(startReloaded.robotStart,startDocument.robotStart);
assert.equal(startReloaded.snapStep,0.25);
const startLegacy = makeDocument(); delete startLegacy.robotStart; delete startLegacy.snapStep;
startLegacy.paths[0].waypoints[0].x = 37;
assert.equal(validateDocument(startLegacy).robotStart.x,37);
assert.equal(startLegacy.snapStep,0.25);
startLegacy.paths[0].waypoints[0].x = 60;
assert.equal(startLegacy.robotStart.x,37,'migrated start is a separate object');
assert.match(cppExport(startDocument,'autoRoute'), /const vantage::Pose2d autoRouteRobotStart = \{30\.2500, 48\.5000, 1\.570796\}/);
assert.match(cppExport(startDocument,'autoRoute','gps'), /autoRouteRobotStartGps = \{-1\.0604, -0\.5969, 0\.000000\}/);
assert.match(cppExport(startDocument,'autoRoute','gps'), /vantage::vexGpsToCorner\(autoRouteRobotStartGps/);
startDocument.robotStart.x = Infinity;
assert.throws(()=>validateDocument(startDocument),/Robot start/);

// A short pointer movement stays short, preserves the grab offset and supports fine mode.
assert.deepEqual(dragPosition({x:20,y:30},{x:21,y:31},{x:22,y:32}),{x:21,y:31});
assert.deepEqual(dragPosition({x:20,y:30},{x:21,y:31},{x:26,y:36},0.25,true),{x:21,y:31});
assert.deepEqual(dragPosition({x:20,y:30},{x:21,y:31},{x:21.1,y:31.1},0),{x:20.1,y:30.1});
assert.deepEqual(dragPosition({x:143,y:1},{x:0,y:0},{x:5,y:-5}),{x:144,y:0});

// Playback dragging projects the pointer onto the sampled route and preserves
// the velocity profile when converting route distance back into preview time.
const dragSamples = [{x:0,y:0,distance:0},{x:10,y:0,distance:10},{x:10,y:10,distance:20}];
assert.equal(nearestPathDistance(dragSamples,{x:6,y:3}),6);
assert.equal(nearestPathDistance(dragSamples,{x:13,y:7}),17);
const dragProfile = motionProfile(120, DEFAULT_ROBOT);
for (const distance of [0, 12, 60, 119.5, 120]) {
  assert.ok(Math.abs(profileDistance(dragProfile, profileTimeAtDistance(dragProfile,distance))-distance) < 1e-6);
}
