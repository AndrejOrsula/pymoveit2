// Quaternion, matrix and deterministic RNG algebra regressions.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as m from '../../src/core/math.js';

const near = (a, b, tolerance = 1e-10) => assert.ok(m.norm(m.sub(a, b)) < tolerance, `${a} != ${b}`);
test('fixed XYZ convention, quaternion inverse, rotation and shortest slerp', () => {
  const q = m.quatFromRpy(0.3, -0.4, 1.1);
  near(m.rpyFromQuat(q), [0.3, -0.4, 1.1]);
  const intrinsic = m.quatMultiply(m.quatMultiply(m.quatFromRpy(0.3, 0, 0), m.quatFromRpy(0, -0.4, 0)), m.quatFromRpy(0, 0, 1.1));
  near(m.eulerXYZFromQuat(intrinsic), [0.3, -0.4, 1.1]);
  near(m.quatMultiply(q, m.quatInverse(q)), [0, 0, 0, 1]);
  near(m.quatRotate(m.quatFromRpy(0, 0, Math.PI / 2), [1, 0, 0]), [0, 1, 0]);
  assert.ok(m.quatAngle(q, m.scale(q, -1)) < 1e-7);
  assert.ok(m.quatAngle(q, m.quatSlerp(q, m.scale(q, -1), 0.5)) < 1e-7);
  near(m.quatNormalize([0, 0, 0, 0]), [0, 0, 0, 1]);
});
test('column-major transforms round trip at trace-positive and trace-negative rotations', () => {
  const rng = m.createRng(91);
  for (let i = 0; i < 50; i++) {
    const pose = { position: [rng.next(), rng.next(), rng.next()], quat_xyzw: m.quatFromRpy(rng.uniform(-3, 3), rng.uniform(-1, 1), rng.uniform(-3, 3)) };
    const matrix = m.mat4FromPose(pose), actual = m.poseFromMat4(matrix);
    assert.ok(matrix instanceof Float64Array); assert.equal(matrix.length, 16);
    near(pose.position, actual.position);
    assert.ok(m.quatAngle(pose.quat_xyzw, actual.quat_xyzw) < 1e-7);
    near(m.mat4TransformPoint(m.mat4Multiply(matrix, m.mat4InvertRigid(matrix)), [1, 2, 3]), [1, 2, 3]);
    near(m.mat4TransformPoint(matrix, [1, 0, 0]), m.add(pose.position, m.quatRotate(pose.quat_xyzw, [1, 0, 0])));
  }
});
test('mulberry32 determinism, range, normal distribution and restorable state', () => {
  const a = m.createRng(123), b = m.createRng(123);
  for (let i = 0; i < 100; i++) { const n = a.next(); assert.equal(n, b.next()); assert.ok(n >= 0 && n < 1); }
  const saved = a.state, n = a.next(); a.state = saved; assert.equal(a.next(), n);
  assert.ok(a.uniform(5, 6) >= 5); assert.ok(Number.isFinite(a.normal()));
});
