// Independent modified-DH and finite-difference checks of the Panda chain.
import test from 'node:test';
import assert from 'node:assert/strict';
import { PANDA as model } from '../../src/core/panda.js';
import { fk, jacobian, linkTransforms } from '../../src/core/kinematics.js';
import { createRng, norm, sub, quatMultiply, quatInverse, rotationVector } from '../../src/core/math.js';

// Independent row-major modified DH: Rx(alpha) Tx(a) Rz(theta) Tz(d).
function dh(q) {
  const a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088];
  const d = [0.333, 0, 0.316, 0, 0.384, 0, 0];
  const alpha = [0, -Math.PI / 2, Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2, Math.PI / 2];
  const multiply = (x, y) => x.map((r, i) => r.map((_, j) => r.reduce((sum, v, k) => sum + v * y[k][j], 0)));
  let t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  for (let i = 0; i < 7; i++) {
    const c = Math.cos(q[i]), s = Math.sin(q[i]), ca = Math.cos(alpha[i]), sa = Math.sin(alpha[i]);
    t = multiply(t, [[c, -s, 0, a[i]], [s * ca, c * ca, -sa, -d[i] * sa], [s * sa, c * sa, ca, d[i] * ca], [0, 0, 0, 1]]);
  }
  return multiply(t, [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.107], [0, 0, 0, 1]]);
}
test('FK matches published modified DH at 40 random joint states', () => {
  const rng = createRng(71);
  for (let i = 0; i < 40; i++) {
    const q = model.joints.map(j => rng.uniform(j.lower, j.upper)), expected = dh(q), actual = linkTransforms(model, q).panda_link8;
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) assert.ok(Math.abs(actual[c * 4 + r] - expected[r][c]) < 4e-11);
  }
});
test('Jacobian agrees with linear and world angular finite differences', () => {
  const rng = createRng(82), eps = 1e-6;
  for (let k = 0; k < 8; k++) {
    const q = model.joints.map(j => rng.uniform(j.lower + 0.1, j.upper - 0.1));
    for (const link of ['panda_link8', 'panda_hand', 'panda_hand_tcp']) {
      const p = fk(model, q, link), J = jacobian(model, q, link);
      for (let i = 0; i < 7; i++) {
        const next = [...q]; next[i] += eps; const b = fk(model, next, link);
        const actual = [...sub(b.position, p.position), ...rotationVector(quatMultiply(b.quat_xyzw, quatInverse(p.quat_xyzw)))].map(x => x / eps);
        assert.ok(norm(sub(actual, J.map(row => row[i]))) < 2e-6);
      }
    }
  }
});
test('finger translations mirror in hand y and right mesh rotates by pi', () => {
  const t = linkTransforms(model, model.named_states.panda_arm.ready, 0.03);
  assert.ok(norm(sub(Array.from(t.panda_leftfinger.slice(12, 15)), Array.from(t.panda_rightfinger.slice(12, 15)))) - 0.06 < 1e-10);
  assert.ok(Math.abs(t.panda_leftfinger[0] + t.panda_rightfinger[0]) < 1e-10);
  assert.ok(t.panda_hand_tcp);
});
