// Primitive distances, sphere covers, allowed collisions and attachment checks.
import test from 'node:test';
import assert from 'node:assert/strict';
import { PANDA as model } from '../../src/core/panda.js';
import { fk } from '../../src/core/kinematics.js';
import { checkState, primitiveDistance, objectSpheres } from '../../src/core/collision.js';
import { MoveGroupSim } from '../../src/core/sim.js';
import { mat4FromPose, mat4TransformPoint, norm, sub } from '../../src/core/math.js';

const object = (type, dimensions) => ({ id: 'shape', type, dimensions, pose: { position: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] }, attached: null, allowed: false });
test('all pinned SRDF named states are self collision free', () => {
  for (const [name, q] of Object.entries(model.named_states.panda_arm)) assert.deepEqual(checkState(model, q, 0.035, []), { valid: true, contacts: [] }, name);
});
test('exact distances to box, sphere, capped cylinder and finite cone', () => {
  assert.equal(primitiveDistance([2, 0, 0], object('box', [2, 2, 2])), 1);
  assert.equal(primitiveDistance([2, 0, 0], object('sphere', [1])), 1);
  assert.ok(Math.abs(primitiveDistance([2, 0, 2], object('cylinder', [2, 1])) - Math.SQRT2) < 1e-12);
  assert.equal(primitiveDistance([0, 0, 2], object('cone', [2, 1])), 1);
  assert.ok(primitiveDistance([0, 0, 0], object('cone', [2, 1])) < 0);
  const rotated = { ...object('box', [2, 1, 1]), pose: { position: [1, 0, 0], quat_xyzw: [0, 0, Math.SQRT1_2, Math.SQRT1_2] } };
  assert.ok(Math.abs(primitiveDistance([1, 2, 0], rotated) - 1) < 1e-12);
});
test('object on a link collides, allowed objects are ignored and mesh scales', () => {
  const q = model.named_states.panda_arm.ready, box = { ...object('box', [0.1, 0.1, 0.1]), pose: fk(model, q) };
  assert.equal(checkState(model, q, 0.035, [box]).valid, false);
  box.allowed = true; assert.equal(checkState(model, q, 0.035, [box]).valid, true);
  const a = objectSpheres({ ...object('mesh', []), scale: [1, 1, 1] }), b = objectSpheres({ ...object('mesh', []), scale: [2, 2, 2] });
  assert.equal(b[0][3], a[0][3] * 2);
});
test('attachment follows the hand, ignores touch links, and collides with world', () => {
  const sim = new MoveGroupSim(), pose = fk(model, sim.q, 'panda_hand_tcp');
  sim.applyCollisionObject({ ...object('sphere', [0.01]), id: 'held', pose });
  sim.attachObject('held', 'panda_hand', Object.keys(model.collision_spheres));
  assert.equal(checkState(model, sim.q, sim.fingers, sim.getObjects()).valid, true);
  sim.q[0] += 0.3; const held = sim.getObjects()[0];
  const expected = mat4TransformPoint(mat4FromPose(fk(model, sim.q, 'panda_hand')), [0, 0, 0.1034]);
  assert.ok(norm(sub(held.pose.position, expected)) < 1e-10);
  const world = { ...object('sphere', [0.02]), id: 'world', pose: held.pose };
  assert.ok(checkState(model, sim.q, sim.fingers, [held, world]).contacts.some(([a, b]) => a === 'held' && b === 'world'));
  sim.detachObject('held'); assert.equal(sim.getObjects()[0].attached, null);
});
