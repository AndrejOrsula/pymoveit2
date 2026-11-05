// The OpenMANIPULATOR-X model: an independent planar check of its chain, the
// link2 frame pymoveit2 derives for it, and its position-only kinematics.
import test from 'node:test';
import assert from 'node:assert/strict';
import { OPEN_MANIPULATOR_X as model } from '../../src/core/open_manipulator_x.js';
import { PANDA } from '../../src/core/panda.js';
import { robotById, robotFromSearch } from '../../src/core/robots.js';
import { fk, jacobian } from '../../src/core/kinematics.js';
import { MoveGroupSim } from '../../src/core/sim.js';
import { createSession } from '../../src/api/index.js';
import { expand } from '../../src/console/commander.js';
import { createRng, norm, sub, quatMultiply, quatInverse, rotationVector } from '../../src/core/math.js';

// joint1 turns the arm about a vertical axis through joint2, at world
// (0, 0.1143, 0.0845); joints 2 to 4 pitch it down in its own plane, which
// faces -y at joint1 = 0.
function planarLink5(q) {
  const [q1, q2, q3] = q, b = q2 + q3;
  const forward = 0.024 * Math.cos(q2) + 0.128 * Math.sin(q2) + 0.124 * Math.cos(b);
  const up = -0.024 * Math.sin(q2) + 0.128 * Math.cos(q2) - 0.124 * Math.sin(b);
  return [forward * Math.sin(q1), 0.1143 - forward * Math.cos(q1), 0.0845 + up];
}
const session = () => createSession(new MoveGroupSim({ model }), { driver: 'sync' });

test('FK matches a planar model of the chain at 40 random joint states', () => {
  const rng = createRng(5);
  for (let i = 0; i < 40; i++) {
    const q = model.joints.map(j => rng.uniform(j.lower, j.upper));
    // The URDF's 1.5708 quarter turns are 4e-6 rad off; that bounds the error.
    assert.ok(norm(sub(fk(model, q, 'link5').position, planarLink5(q))) < 1e-5, JSON.stringify(q));
  }
  assert.ok(norm(sub(fk(model, model.named_states.arm.init, 'link5').position, [0, -0.0337, 0.2125])) < 1e-4);
});

test('Jacobian agrees with finite differences at link5 and end_effector_link', () => {
  const rng = createRng(6), eps = 1e-6;
  for (let k = 0; k < 8; k++) {
    const q = model.joints.map(j => rng.uniform(j.lower + 0.1, j.upper - 0.1));
    for (const link of model.ik_links) {
      const p = fk(model, q, link), J = jacobian(model, q, link);
      for (let i = 0; i < 4; i++) {
        const next = [...q]; next[i] += eps; const b = fk(model, next, link);
        const actual = [...sub(b.position, p.position), ...rotationVector(quatMultiply(b.quat_xyzw, quatInverse(p.quat_xyzw)))].map(x => x / eps);
        assert.ok(norm(sub(actual, J.map(row => row[i]))) < 2e-6);
      }
    }
  }
});

test('compute_fk and compute_ik work in link2, which turns with joint1', () => {
  const s = session();
  const home = s.invoke('robot.compute_fk');
  assert.equal(home.header.frame_id, 'link2');
  assert.equal(s.invoke('robot.move_to_configuration', [[1.0, -1.0, 0.7, 0.3]]), true);
  const turned = s.invoke('robot.compute_fk');
  const xyz = (pose) => ['x', 'y', 'z'].map(k => pose.pose.position[k]);
  assert.ok(norm(sub(xyz(turned), xyz(home))) < 1e-4, 'the same position in link2');
  const solution = s.invoke('robot.compute_ik', [[0.15, 0.0, 0.1], [0, 0, 0, 1]]);
  assert.equal(s.invoke('robot.move_to_configuration', [solution.position]), true);
  assert.ok(norm(sub(xyz(s.invoke('robot.compute_fk')), [0.15, 0, 0.1])) < 0.002);
});

test('pose goals need a free orientation, and frame_id world is fixed', () => {
  const s = session();
  assert.equal(s.invoke('robot.move_to_pose', [[0.1, -0.05, 0.1], [0, 0, 0, 1]], { frame_id: 'world' }), false);
  assert.match(s.invoke('robot.last_failure'), /^NO_IK_SOLUTION/);
  assert.equal(s.invoke('robot.move_to_pose', [[0.1, -0.05, 0.1], [0, 0, 0, 1]], { frame_id: 'world', tolerance_orientation: 3.1416 }), true);
  assert.ok(norm(sub(fk(model, s.sim.q, 'link5').position, [0.1, -0.05, 0.1])) < 0.002);
  s.invoke('robot.add_collision_box', ['cube', [0.02, 0.02, 0.02]], { position: [0, -0.1, 0.03], quat_xyzw: [0, 0, 0, 1], frame_id: 'world' });
  assert.deepEqual(s.sim.getObjects()[0].pose.position, [0, -0.1, 0.03]);
});

test('description groups, states and gripper follow RobotDescription', () => {
  const s = session();
  assert.deepEqual(s.getattr('robot.description.group_names'), ['arm', 'gripper']);
  assert.deepEqual(s.invoke('robot.description.moveit2_kwargs'), { joint_names: ['joint1', 'joint2', 'joint3', 'joint4'], base_link_name: 'link2', end_effector_name: 'link5', group_name: 'arm' });
  assert.deepEqual(s.invoke('robot.description.moveit2_gripper_kwargs'), { gripper_joint_names: ['gripper_left_joint'], open_gripper_joint_positions: [0.019], closed_gripper_joint_positions: [-0.01], gripper_group_name: 'gripper' });
  assert.deepEqual(s.invoke('robot.description.group', ['gripper']).passive_joint_names, ['gripper_right_joint']);
  assert.deepEqual(s.invoke('robot.description.joint_positions'), [0, -1, 0.7, 0.3]);
  assert.throws(() => s.invoke('robot.description.joint_positions', ['ready']), /no state 'ready' \(known: \['home', 'init'\]\)/);
  assert.equal(s.invoke('robot.gripper.close'), true); assert.equal(s.invoke('robot.gripper.wait_until_executed'), true);
  assert.equal(s.getattr('robot.gripper.is_closed'), true);
  assert.equal(s.getattr('robot.servo.frame_id'), 'link2');
});

test('robots resolve from the query string, and snapshots stay with their robot', () => {
  assert.equal(robotFromSearch('?robot=open_manipulator_x'), model);
  assert.equal(robotFromSearch('?robot=toString'), PANDA);
  assert.equal(robotById(undefined), PANDA);
  const snapshot = new MoveGroupSim().snapshot(), sim = new MoveGroupSim({ model });
  const q = model.named_states.arm.init;
  sim.execute({ joint_names: model.joint_names, points: [{ positions: sim.q, time_from_start: 0 }, { positions: q, time_from_start: 2 }] });
  assert.throws(() => sim.reset(snapshot), RangeError);
  assert.ok(sim.isBusy('arm'), 'a refused snapshot leaves the motion running');
});

test('commander names world for goals and objects of this arm', () => {
  const context = { state: () => ({ ee: fk(model, model.named_states.arm.home) }), model, random: () => 0.5 };
  assert.deepEqual(expand('add box b 0 -0.1 0.05', context).python, ["robot.add_collision_box('b', [0.1, 0.1, 0.1], position=[0.0, -0.1, 0.05], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id='world')"]);
  assert.deepEqual(expand('go home', context).python, ['robot.move_to_configuration()']);
  assert.match(expand('pose 0 -0.05 0.12', context).python[0], /frame_id='world', tolerance_orientation=3\.1416\)$/);
});
