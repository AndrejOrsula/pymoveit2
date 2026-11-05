// Collision-free search, Cartesian fractions, Pilz and path-constraint regressions.
import test from 'node:test';
import assert from 'node:assert/strict';
import { PANDA as model } from '../../src/core/panda.js';
import { plan, satisfiesConstraints } from '../../src/core/planning.js';
import { fk } from '../../src/core/kinematics.js';
import { solveIK } from '../../src/core/ik.js';
import { checkState } from '../../src/core/collision.js';
import { sampleTrajectory } from '../../src/core/timing.js';
import { createRng, lerp, norm, sub, dot } from '../../src/core/math.js';

const start = model.named_states.panda_arm.ready;
const context = (objects = [], seed = 12) => ({ q: start, fingers: 0.035, objects, rng: createRng(seed) });
const sphere = (id, position, radius = 0.025) => ({ id, type: 'sphere', dimensions: [radius], pose: { position, quat_xyzw: [0, 0, 0, 1] }, attached: null, allowed: false });
function samples(trajectory, step = 0.03) {
  const points = [], duration = trajectory.points.at(-1).time_from_start;
  for (let t = 0; t < duration; t += step) points.push(sampleTrajectory(trajectory, t).positions);
  points.push(trajectory.points.at(-1).positions); return points;
}
function assertStraight(trajectory, a, b) {
  const delta = sub(b, a), length2 = dot(delta, delta);
  for (const q of samples(trajectory, 0.01)) {
    const p = fk(model, q).position, u = dot(sub(p, a), delta) / length2;
    assert.ok(norm(sub(p, lerp(a, b, u))) < 0.001, 'Cartesian path within 1 mm');
  }
}
test('free joint goal, malformed/invalid goals, collision and unreachable failures', () => {
  const goal = [...start]; goal[0] += 0.2;
  const r = plan(model, { joint_goal: { positions: goal } }, context());
  assert.equal(r.error_code, 'SUCCESS'); assert.deepEqual(r.debug.goal_state, goal);
  assert.equal(plan(model, { joint_goal: { positions: [1, 2] } }, context()).error_code, 'GOAL_STATE_INVALID');
  assert.equal(plan(model, { group_name: 'absent', joint_goal: { positions: goal } }, context()).error_code, 'INVALID_GROUP_NAME');
  assert.equal(plan(model, { pose_goal: { position: [2, 0, 0] } }, context()).error_code, 'NO_IK_SOLUTION');
  const far = [...start]; far[0] += 1;
  assert.equal(plan(model, { joint_goal: { positions: far } }, context([sphere('goal', fk(model, far).position)])).error_code, 'GOAL_IN_COLLISION');
  assert.equal(plan(model, { joint_goal: { positions: far } }, context([sphere('start', fk(model, start).position)])).error_code, 'START_STATE_IN_COLLISION');
});
test('obstructed straight path finds a densely collision-free RRT path and is deterministic', () => {
  const goal = [...start]; goal[0] += 1;
  const mid = lerp(start, goal, 0.5), obstacle = sphere('block', fk(model, mid).position), request = { joint_goal: { positions: goal }, allowed_planning_time: 3 };
  assert.equal(checkState(model, mid, 0.035, [obstacle]).valid, false);
  const a = plan(model, request, context([obstacle])), b = plan(model, request, context([obstacle]));
  assert.equal(a.error_code, 'SUCCESS'); assert.deepEqual(a, b);
  assert.ok(a.debug.tree.length > 0 && a.debug.tree.length <= 2000); assert.ok(a.debug.ee_path.length <= 400);
  for (const q of samples(a.trajectory, 0.015)) assert.equal(checkState(model, q, 0.035, [obstacle]).valid, true);
});
test('Cartesian straight path and partial obstacle fraction; zero fraction remains SUCCESS', () => {
  const first = fk(model, start), goal = { position: [first.position[0] + 0.18, 0, first.position[2] - 0.04], quat_xyzw: first.quat_xyzw };
  const request = { pose_goal: goal, cartesian: true, cartesian_max_step: 0.0025, cartesian_avoid_collisions: true };
  const r = plan(model, request, context());
  assert.equal(r.error_code, 'SUCCESS'); assert.equal(r.fraction, 1); assertStraight(r.trajectory, first.position, goal.position);
  const obstacle = sphere('block', [first.position[0] + 0.13, 0, first.position[2] - 0.03]);
  const partial = plan(model, request, context([obstacle]));
  assert.equal(partial.error_code, 'SUCCESS'); assert.ok(partial.fraction > 0 && partial.fraction < 1);
  const impossible = { ...request, pose_goal: { position: [2, 0, 0], quat_xyzw: first.quat_xyzw }, cartesian_max_step: 2 };
  const zero = plan(model, impossible, context()); assert.equal(zero.error_code, 'SUCCESS'); assert.equal(zero.fraction, 0); assert.equal(zero.trajectory.points.length, 0);
});
test('Pilz LIN stays straight and fails at obstacles; PTP starts and stops together', () => {
  const first = fk(model, start), pose_goal = { position: [first.position[0] + 0.18, 0, first.position[2] - 0.04], quat_xyzw: first.quat_xyzw };
  const request = { pose_goal, pipeline_id: 'pilz_industrial_motion_planner', planner_id: 'LIN' };
  const r = plan(model, request, context()); assert.equal(r.error_code, 'SUCCESS'); assertStraight(r.trajectory, first.position, pose_goal.position);
  const obstacle = sphere('block', [first.position[0] + 0.13, 0, first.position[2] - 0.03]);
  assert.equal(plan(model, request, context([obstacle])).error_code, 'PLANNING_FAILED');
  const goal = start.map((x, i) => x + (i === 3 ? -0.1 : 0.1));
  const ptp = plan(model, { pipeline_id: request.pipeline_id, planner_id: 'PTP', joint_goal: { positions: goal } }, context());
  assert.equal(ptp.error_code, 'SUCCESS'); assert.ok(ptp.trajectory.points[0].velocities.every(v => v === 0)); assert.ok(ptp.trajectory.points.at(-1).velocities.every(v => v === 0));
});
test('orientation constraints remain satisfied on projected and timed paths', () => {
  const initial = fk(model, start), goal = solveIK(model, { position: [0.42, 0.05, 0.52], quat_xyzw: initial.quat_xyzw }, { seed: start, tolerance_orientation: 0.00001 });
  assert.ok(goal);
  const constraints = { orientation: [{ quat_xyzw: initial.quat_xyzw, link: model.end_effector_name, tolerance: [0.004, 0.004, 0.004], parameterization: 1 }] };
  const r = plan(model, { joint_goal: { positions: goal }, path_constraints: constraints }, context());
  assert.equal(r.error_code, 'SUCCESS', r.detail);
  for (const q of samples(r.trajectory, 0.01)) assert.equal(satisfiesConstraints(model, q, constraints), true);
  const bad = { joint: [{ joint_name: 'panda_joint1', position: 2, tolerance: 0.01 }] };
  assert.equal(plan(model, { joint_goal: { positions: goal }, path_constraints: bad }, context()).error_code, 'START_STATE_VIOLATES_PATH_CONSTRAINTS');
});
test('a valid joint goal inside a broad orientation region remains the planned endpoint', () => {
  const initial = fk(model, start), goal = [...start]; goal[0] = -0.1;
  const constraints = { orientation: [{ quat_xyzw: initial.quat_xyzw, link: model.end_effector_name, tolerance: [0.1, 0.1, 0.1], parameterization: 1 }] };
  assert.equal(satisfiesConstraints(model, goal, constraints), true);
  const r = plan(model, { joint_goal: { positions: goal }, path_constraints: constraints }, context());
  assert.equal(r.error_code, 'SUCCESS', r.detail);
  assert.deepEqual(r.debug.goal_state, goal);
  assert.deepEqual(r.trajectory.points.at(-1).positions, goal);
});
