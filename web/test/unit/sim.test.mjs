// Controller concurrency, scene invalidation, servo timeout and grasp regressions.
import test from 'node:test';
import assert from 'node:assert/strict';
import { MoveGroupSim } from '../../src/core/sim.js';
import { fk } from '../../src/core/kinematics.js';
import { minimumDistance } from '../../src/core/collision.js';
import { norm, sub } from '../../src/core/math.js';

function run(sim, until, max = 4000) { for (let i = 0; i < max && !until(); i++) sim.step(1 / 120); assert.ok(until(), 'simulation settled'); }
const gripperTrajectory = sim => ({ joint_names: sim.model.fingers.joint_names, points: [{ positions: [sim.fingers, sim.fingers], velocities: [0, 0], accelerations: [0, 0], time_from_start: 0 }, { positions: [0, 0], velocities: [0, 0], accelerations: [0, 0], time_from_start: 0.35 }] });
test('arm executes to goal, hand executes concurrently, stale starts abort and cancellation is tracked', () => {
  const sim = new MoveGroupSim(), events = []; sim.on('execution', e => events.push(e));
  const goal = [...sim.q]; goal[0] += 0.5;
  const r = sim.plan({ joint_goal: { positions: goal } }); assert.equal(r.error_code, 'SUCCESS');
  sim.execute(r.trajectory); sim.execute(gripperTrajectory(sim), 'gripper'); assert.ok(sim.isBusy('arm') && sim.isBusy('gripper'));
  run(sim, () => !sim.isBusy()); assert.ok(norm(sub(sim.q, goal)) < 1e-9); assert.equal(sim.fingers, 0);
  assert.equal(events.filter(e => e.status === 'succeeded').length, 2);
  sim.execute(r.trajectory); assert.equal(events.at(-1).status, 'aborted'); assert.equal(events.at(-1).error_code, 'INVALID_MOTION_PLAN');
  const back = sim.plan({ joint_goal: { positions: sim.model.named_states.panda_arm.ready } }); sim.execute(back.trajectory); sim.step(0.01); assert.equal(sim.cancel(), true); assert.equal(events.at(-1).status, 'canceled'); assert.equal(sim.cancel(), false);
});
test('an obstacle in the remaining path aborts within one 50 ms tick', () => {
  const sim = new MoveGroupSim(), events = []; sim.on('execution', e => events.push(e));
  const goal = [...sim.q]; goal[0] += 0.8; const plan = sim.plan({ joint_goal: { positions: goal } });
  sim.execute(plan.trajectory); sim.step(0.02);
  sim.applyCollisionObject({ id: 'new', type: 'sphere', dimensions: [0.04], pose: fk(sim.model, goal), attached: null, allowed: false }); sim.step(0.05);
  assert.equal(events.at(-1).status, 'aborted'); assert.equal(events.at(-1).error_code, 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'); assert.equal(sim.isBusy('arm'), false);
});
test('controller honors named joint order and rejects nonfinite trajectories', () => {
  const sim = new MoveGroupSim(), goal = [...sim.q]; goal[0] += 0.2;
  const result = sim.plan({ joint_goal: { positions: goal } });
  const reversed = { joint_names: [...result.trajectory.joint_names].reverse(), points: result.trajectory.points.map(p => ({ ...p, positions: [...p.positions].reverse(), velocities: [...p.velocities].reverse(), accelerations: [...p.accelerations].reverse() })) };
  sim.execute(reversed); run(sim, () => !sim.isBusy()); assert.ok(norm(sub(sim.q, goal)) < 1e-10);
  const events = []; sim.on('execution', e => events.push(e)); const invalid = structuredClone(result.trajectory); invalid.points[0].positions[0] = NaN;
  sim.execute(invalid); assert.equal(events.at(-1).status, 'aborted'); assert.ok(sim.q.every(Number.isFinite));
});
test('servo twist uses speed units, expires after 0.1 s and transforms frames', () => {
  const sim = new MoveGroupSim(), before = fk(sim.model, sim.q).position;
  sim.servoEnable(); sim.servoTwist([0.05, 0, 0], [0, 0, 0]);
  sim.step(0); assert.ok(sim.getState().velocities.every(Number.isFinite));
  for (let i = 0; i < 24; i++) sim.step(1 / 120);
  const moved = fk(sim.model, sim.q).position; assert.ok(moved[0] - before[0] > 0.004 && moved[0] - before[0] < 0.0055);
  const stopped = [...sim.q]; sim.step(0.05); assert.deepEqual(sim.q, stopped); assert.equal(sim.isBusy(), false);
  assert.equal(sim.servoTwist([0, 0, 0.01], [0, 0, 0], 'panda_hand'), true);
  sim.step(0.01); assert.ok(norm(sub(sim.q, stopped)) > 0);
  sim.servoDisable(); assert.equal(sim.servoJog(['panda_joint1'], [0.1]), false);
});
test('servo halts before an obstacle and rejects commands during arm execution', () => {
  const sim = new MoveGroupSim(), p = fk(sim.model, sim.q).position;
  sim.applyCollisionObject({ id: 'near', type: 'sphere', dimensions: [0.02], pose: { position: [p[0] + 0.13, 0, p[2]], quat_xyzw: [0, 0, 0, 1] }, attached: null, allowed: false });
  sim.servoEnable();
  for (let i = 0; i < 300; i++) { sim.servoTwist([0.2, 0, 0], [0, 0, 0]); sim.step(0.01); }
  assert.ok(minimumDistance(sim.model, sim.q, sim.fingers, sim.getObjects()) >= 0.018);
  assert.ok(sim.getLog().some(l => l.node === 'servo_node'));
  sim.clearScene(); const goal = [...sim.q]; goal[0] += 0.2; const r = sim.plan({ joint_goal: { positions: goal } });
  sim.execute(r.trajectory); assert.equal(sim.servoTwist([0.1, 0, 0], [0, 0, 0]), false);
});
test('gripper stops on a cube at its half width', () => {
  for (const allowed of [false, true]) {
    const sim = new MoveGroupSim(), tcp = fk(sim.model, sim.q, 'panda_hand_tcp');
    sim.applyCollisionObject({ id: 'cube', type: 'box', dimensions: [0.04, 0.04, 0.04], pose: tcp, attached: null, allowed });
    sim.execute(gripperTrajectory(sim), 'gripper'); run(sim, () => !sim.isBusy('gripper'));
    assert.ok(Math.abs(sim.fingers - 0.02) < 1e-10);
  }
});
test('snapshot restores RNG, servo, attachments and active controller deterministically', () => {
  const sim = new MoveGroupSim({ seed: 8 }), goal = [...sim.q]; goal[0] += 0.5;
  const r = sim.plan({ joint_goal: { positions: goal } }); sim.execute(r.trajectory); sim.step(0.02);
  const snapshot = JSON.parse(JSON.stringify(sim.snapshot())); const other = new MoveGroupSim(); other.reset(snapshot);
  for (let i = 0; i < 10; i++) { sim.step(0.01); other.step(0.01); }
  assert.deepEqual(other.getState(), sim.getState()); assert.equal(other.rng.next(), sim.rng.next());
  sim.writeLog('info', 'move_group', 'test'); for (let i = 0; i < 510; i++) sim.writeLog('info', 'move_group', `${i}`); assert.equal(sim.getLog().length, 500);
});
