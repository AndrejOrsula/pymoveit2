// Independent source parsing guards Python parameter names, kinds and defaults.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { SIGNATURES } from '../../src/api/signatures.js';

function splitParameters(text) {
  const fields = []; let depth = 0, quote = null, start = 0;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (quote) { if (c === '\\') i++; else if (c === quote) quote = null; continue; }
    if (c === "'" || c === '"') quote = c;
    else if ('([{'.includes(c)) depth++;
    else if (')]}'.includes(c)) depth--;
    else if (c === ',' && depth === 0) { fields.push(text.slice(start, i).trim()); start = i + 1; }
  }
  fields.push(text.slice(start).trim()); return fields.filter(Boolean);
}
function defaultValue(text) {
  const constants = { None: null, True: true, False: false, DEFAULT_NODE_NAME: 'pymoveit2', DEFAULT_DESCRIPTION_NODE_NAME: 'move_group', DEFAULT_TIMEOUT_SEC: 10, DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC: 3, DEFAULT_JOINT_STATE_TIMEOUT_SEC: 10, 'CollisionObject.ADD': 0, 'tuple()': [] };
  if (Object.hasOwn(constants, text)) return constants[text];
  if (text.startsWith('(')) return splitParameters(text.slice(1, -1)).map(defaultValue);
  return JSON.parse(text);
}
function sourceSpec(source, method) {
  const match = source.match(new RegExp(`(?:^|\\n)    def ${method}\\(([\\s\\S]*?)\\)\\s*(?:->[^:]+)?:`));
  assert.ok(match, `source method ${method}`);
  let kind = 'positional'; const params = [];
  for (const field of splitParameters(match[1])) {
    if (field === '*') { kind = 'keyword_only'; continue; }
    const name = field.split(/[:=]/)[0].trim();
    if (['self', 'cls'].includes(name)) continue;
    const eq = field.indexOf('=');
    params.push({ name: name.replace(/^\*+/, ''), kind: name.startsWith('**') ? 'var_keyword' : name.startsWith('*') ? 'var_positional' : kind, default: name.startsWith('**') ? {} : name.startsWith('*') ? [] : eq === -1 ? { __required__: true } : defaultValue(field.slice(eq + 1).replace(/\s+/g, ' ').trim()) });
  }
  return params;
}
test('every exposed method parameter matches its repository Python definition', () => {
  for (const [cls, file] of Object.entries({ RobotSession: 'session', MoveIt2: 'moveit2', GripperInterface: 'gripper_interface', MoveIt2Servo: 'moveit2_servo', RobotDescription: 'robot_description' })) {
    const text = readFileSync(new URL(`../../../pymoveit2/${file}.py`, import.meta.url), 'utf8');
    const start = text.indexOf(`class ${cls}`), tail = text.slice(start), next = tail.slice(1).search(/^class /m);
    const source = next === -1 ? tail : tail.slice(0, next + 1);
    for (const [method, spec] of Object.entries(SIGNATURES[cls].methods)) assert.deepEqual(spec.params.map(({ name, kind, default: value }) => ({ name, kind, default: value })), sourceSpec(source, method), `${cls}.${method}`);
  }
});
