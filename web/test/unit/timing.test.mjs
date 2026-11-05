// Continuous sampling and velocity/acceleration limits for timed joint paths.
import test from 'node:test';
import assert from 'node:assert/strict';
import { PANDA as model } from '../../src/core/panda.js';
import { timeParameterize, sampleTrajectory, synchronizedTrapezoid, cartesianProfile } from '../../src/core/timing.js';
import { norm, sub } from '../../src/core/math.js';

function checkLimits(trajectory, vs, as) {
  assert.ok(trajectory.points[0].velocities.every(v => v === 0));
  assert.ok(trajectory.points.at(-1).velocities.every(v => v === 0));
  const dt = 0.001, duration = trajectory.points.at(-1).time_from_start;
  let previous = sampleTrajectory(trajectory, 0);
  for (let t = dt; t <= duration; t += dt) {
    const actual = sampleTrajectory(trajectory, t);
    for (let i = 0; i < 7; i++) {
      assert.ok(Math.abs(actual.velocities[i]) <= model.joints[i].max_velocity * vs * 1.05 + 1e-6, `velocity joint ${i}`);
      assert.ok(Math.abs(actual.velocities[i] - previous.velocities[i]) / dt <= model.joints[i].max_acceleration * as * 1.05 + 1e-6, `acceleration joint ${i}`);
    }
    assert.ok(norm(sub(actual.positions, previous.positions)) < 0.01);
    previous = actual;
  }
}
test('arc-length parameterization respects scaled limits through corners', () => {
  const q = model.named_states.panda_arm.ready, b = q.map((x, i) => x + (i % 2 ? 0.25 : 0.4)), c = q.map((x, i) => x + (i % 2 ? -0.1 : 0.7));
  const trajectory = timeParameterize(model, [q, b, c], 0.3, 0.4);
  checkLimits(trajectory, 0.3, 0.4);
  assert.deepEqual(trajectory.points.at(-1).positions, c);
  for (const point of trajectory.points.slice(0, -1)) assert.ok(Math.abs(point.time_from_start * 10 - Math.round(point.time_from_start * 10)) < 1e-9);
});
test('small paths do not stall and zero scaling falls back to full limits', () => {
  const q = model.named_states.panda_arm.ready, b = [...q]; b[0] += 0.001;
  const trajectory = timeParameterize(model, [q, b], 0, 0);
  assert.ok(trajectory.points.at(-1).time_from_start < 0.2); checkLimits(trajectory, 1, 1);
});
test('Pilz PTP synchronizes all moving joints and Cartesian limits set duration', () => {
  const q = model.named_states.panda_arm.ready, g = q.map((x, i) => x + 0.1 * (i + 1));
  const trajectory = synchronizedTrapezoid(model, q, g, 0.5, 0.5); checkLimits(trajectory, 0.5, 0.5);
  const mid = sampleTrajectory(trajectory, trajectory.points.at(-1).time_from_start / 2).positions;
  for (let i = 1; i < 7; i++) assert.ok(Math.abs((mid[i] - q[i]) / (g[i] - q[i]) - (mid[0] - q[0]) / (g[0] - q[0])) < 1e-10);
  const profile = cartesianProfile({ position: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] }, { position: [1, 0, 0], quat_xyzw: [0, 0, 0, 1] });
  assert.ok(profile.duration > 1); assert.deepEqual(profile.at(profile.duration), [1, 0, 0]);
});
