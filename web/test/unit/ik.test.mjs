// Reachability, partial goals, joint limits and collision-aware IK regressions.
import test from 'node:test';
import assert from 'node:assert/strict';
import { PANDA as model } from '../../src/core/panda.js';
import { fk } from '../../src/core/kinematics.js';
import { solveIK } from '../../src/core/ik.js';
import { createRng, norm, sub, quatAngle, clamp } from '../../src/core/math.js';

test('random reachable FK/IK round trips respect limits', () => {
  const rng = createRng(290);
  for (let k = 0; k < 20; k++) {
    const q = model.joints.map(j => rng.uniform(j.lower + 0.15, j.upper - 0.15));
    const seed = q.map((x, i) => clamp(x + rng.uniform(-0.25, 0.25), model.joints[i].lower, model.joints[i].upper));
    const target = fk(model, q), solution = solveIK(model, target, { seed, rng });
    assert.ok(solution, `state ${k} reachable`);
    solution.forEach((x, i) => assert.ok(x >= model.joints[i].lower && x <= model.joints[i].upper));
    const actual = fk(model, solution);
    assert.ok(norm(sub(actual.position, target.position)) <= 0.0011);
    assert.ok(quatAngle(actual.quat_xyzw, target.quat_xyzw) <= 0.0018);
  }
});
test('position-only, orientation-only, TCP, seed preference and validity rejection', () => {
  const seed = model.named_states.panda_arm.ready;
  for (const link of ['panda_link8', 'panda_hand', 'panda_hand_tcp']) {
    const target = fk(model, seed, link);
    assert.deepEqual(solveIK(model, target, { seed, target_link: link }), seed);
    assert.ok(solveIK(model, { position: target.position }, { seed, target_link: link }));
    assert.ok(solveIK(model, { quat_xyzw: target.quat_xyzw }, { seed, target_link: link, tolerance_orientation: [0.001, 0.001, 0.001] }));
    assert.equal(solveIK(model, target, { seed, target_link: link, isValid: () => false, restarts: 1 }), null);
  }
  assert.equal(solveIK(model, { position: [2, 0, 0] }), null);
});
