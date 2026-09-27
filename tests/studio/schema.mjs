// Checks that documents Studio writes match editor/vpath.schema.json.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import Ajv2020 from "ajv/dist/2020.js";
import { makeDocument, validateDocument } from "../../editor/model.js";
import * as edits from "../../editor/route-edits.js";

const schema = JSON.parse(readFileSync(new URL("../../editor/vpath.schema.json", import.meta.url)));
const validate = new Ajv2020({ allErrors: true }).compile(schema);

function check(name, document) {
  const ok = validate(JSON.parse(JSON.stringify(document)));
  assert.ok(ok, `${name}: ${JSON.stringify(validate.errors, null, 2)}`);
  console.log(`ok  ${name} matches vpath.schema.json`);
}

const fresh = makeDocument();
check("new document", fresh);

const busy = validateDocument(makeDocument());
const route = busy.paths[0];
let path = edits.appendPoint(route, { x: 120, y: 30 }).path;
path = edits.setSegmentSpeed(path, 0, 24);
path = edits.addMarker(path, 1, 0.3, "Intake on").path;
path = edits.setWaitAfter(path, 400);
path = edits.setSegmentGeometry(path, 2, "hermite");
busy.paths = [path, edits.allianceCopy(path, "red")];
check("edited document with markers, speeds and waits", validateDocument(busy));

const legacy = makeDocument();
legacy.version = 1;
delete legacy.paths[0].segmentSpeed; delete legacy.paths[0].markers; delete legacy.paths[0].waitAfterMs;
check("format 1 file after migration", validateDocument(legacy));

assert.equal(validate({ ...fresh, version: 1 }), false, "the schema describes format 2 only");
console.log("ok  the schema rejects format 1 as written");
