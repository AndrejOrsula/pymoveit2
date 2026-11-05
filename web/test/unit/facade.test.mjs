// Python API call binding, lifecycle, code generation and both wait drivers.
import test from 'node:test';
import assert from 'node:assert/strict';
import { MoveGroupSim } from '../../src/core/sim.js';
import { createSession, SIGNATURES, bindArguments, pyRepr, formatCall, buildScript, PyError } from '../../src/api/index.js';
import { norm, sub, mat4FromPose } from '../../src/core/math.js';
import { armOperation, jointConstraint, orientationConstraint, positionConstraint } from '../../src/api/arm.js';
import { runSync } from '../../src/api/drivers.js';

const session = () => createSession(new MoveGroupSim(), { driver: 'sync' });
test('README flow returns Python booleans, ends at requested pose and returns to SRDF ready', () => {
  const s = session(), calls = [], ros = []; s.on('call', c => calls.push(c)); s.on('ros', r => ros.push(r));
  assert.equal(s.invoke('robot.move_to_pose', [[0.4, 0, 0.4], [1, 0, 0, 0]]), true);
  assert.ok(norm(sub(s.sim.getState().ee.position, [0.4, 0, 0.4])) < 0.001);
  assert.equal(s.invoke('robot.gripper.close'), true); assert.equal(s.invoke('robot.move_to_configuration'), true);
  assert.deepEqual(s.sim.q, s.sim.model.named_states.panda_arm.ready); assert.equal(s.sim.fingers, 0);
  assert.equal(s.invoke('robot.move_to_pose', [[2, 0, 0], [1, 0, 0, 0]]), false);
  assert.ok(s.invoke('robot.last_failure').startsWith('NO_IK_SOLUTION: The kinematics solver found no solution for that pose.'));
  assert.ok(ros.some(r => r.name === '/plan_kinematic_path' && r.type === 'moveit_msgs/srv/GetMotionPlan'));
  assert.ok(ros.some(r => r.name === '/execute_trajectory' && r.type === 'moveit_msgs/action/ExecuteTrajectory'));
  assert.equal(calls[0].status, 'started'); assert.equal(calls[1].status, 'returned'); assert.ok(calls[1].ros.length >= 2);
});
test('arm submits without waiting; cancellation, timeouts and one-consumption waits match Python', () => {
  const s = session(), goal = [...s.sim.q]; goal[0] += 0.5;
  assert.equal(s.invoke('robot.arm.move_to_configuration', [goal]), true); assert.equal(s.sim.time, 0);
  assert.equal(s.invoke('robot.query_state').name, 'EXECUTING');
  assert.equal(s.invoke('robot.wait_until_executed', [], { timeout_sec: 0.01 }), false); assert.ok(s.sim.isBusy('arm'));
  assert.equal(s.invoke('robot.cancel_execution'), true); assert.ok(s.invoke('robot.last_failure').startsWith('PREEMPTED:'));
  assert.equal(s.invoke('robot.arm.move_to_configuration', [goal]), true); assert.equal(s.invoke('robot.wait_until_executed'), true);
  assert.equal(s.invoke('robot.wait_until_executed'), false); assert.equal(s.invoke('robot.last_failure'), null); assert.equal(s.getattr('robot.motion_succeeded'), true);
});
test('CPython missing, duplicate, unexpected, keyword-only and excess argument messages', () => {
  const arm = SIGNATURES.MoveIt2.methods, robot = SIGNATURES.RobotSession.methods;
  const expect = (fn, message) => assert.throws(fn, e => e instanceof PyError && e.type === 'TypeError' && e.message === message);
  expect(() => bindArguments('MoveIt2.move_to_configuration', arm.move_to_configuration), "move_to_configuration() missing 1 required positional argument: 'joint_positions'");
  expect(() => bindArguments('MoveIt2.move_to_pose', arm.move_to_pose, [], { foo: 1 }), "move_to_pose() got an unexpected keyword argument 'foo'");
  expect(() => bindArguments('RobotSession.move_to_pose', robot.move_to_pose, [1, 2, 3]), 'move_to_pose() takes from 1 to 3 positional arguments but 4 were given');
  expect(() => bindArguments('RobotSession.move_to_pose', robot.move_to_pose, [1, 2, 3], { wait: false }), 'move_to_pose() takes from 1 to 3 positional arguments but 4 positional arguments (and 1 keyword-only argument) were given');
  expect(() => bindArguments('RobotSession.move_to_pose', robot.move_to_pose, [1], { position: 2 }), "move_to_pose() got multiple values for argument 'position'");
  const bound = bindArguments('RobotSession.move_to_pose', robot.move_to_pose, [], { cartesian: true }); assert.deepEqual(bound.kwargs, { cartesian: true }); assert.equal(bound.wait, true);
});
test('session assignment shadows forwarded properties and emits the exact pitfall hint', () => {
  const s = session(), hints = []; s.on('hint', h => hints.push(h));
  s.setattr('robot.max_velocity', 0.2); assert.equal(s.getattr('robot.max_velocity'), 0.2); assert.equal(s.getattr('robot.arm.max_velocity'), 0); assert.ok(hints[0].text.includes('robot.arm.max_velocity'));
  s.setattr('robot.arm.max_velocity', '0.3'); assert.equal(s.getattr('robot.arm.max_velocity'), 0.3);
  assert.throws(() => s.setattr('robot.arm.max_velocity', 'fast'), e => e.type === 'ValueError' && e.message === "could not convert string to float: 'fast'");
  assert.throws(() => s.setattr('robot.servo.linear_speed', NaN), e => e.type === 'ValueError' && e.message === '`linear_speed` must be finite.');
  assert.throws(() => s.setattr('robot.arm.joint_names', []), e => e.type === 'AttributeError');
  assert.throws(() => s.invoke('robot.move_to_pose', [], { position: [1, 2] }), e => e.type === 'ValueError');
  assert.ok(s.members('robot').some(m => m.name === 'move_to_pose' && m.python_signature === 'move_to_pose(position=None, quat_xyzw=None, *, wait=True, timeout_sec=None, **kwargs) -> bool'));
});
test('compute_ik ignores constraints on unknown joints and links, as MoveIt does', () => {
  // MoveIt's JointConstraint, PositionConstraint and OrientationConstraint fail configure() without
  // a joint or link model and then decide() satisfied; /compute_ik never checks configure().
  const s = session(), pose = s.invoke('robot.compute_fk');
  const ik = constraints => s.invoke('robot.compute_ik', [], { position: pose.pose.position, quat_xyzw: pose.pose.orientation, constraints });
  const flipped = [0, 0, 1, 0];
  const unknown = [
    { joint_constraints: [{ __class__: 'JointConstraint', joint_name: 'no_such_joint', position: 3, tolerance_above: 0.01, tolerance_below: 0.01, weight: 1 }] },
    { orientation_constraints: [s.invoke('robot.create_orientation_constraint', [flipped], { target_link: 'no_such_link' })] },
    { position_constraints: [s.invoke('robot.create_position_constraint', [[5, 5, 5]], { target_link: 'no_such_link' })] },
  ];
  for (const constraints of unknown) assert.equal(ik(constraints)?.__class__, 'JointState', JSON.stringify(constraints));
  assert.equal(ik({ orientation_constraints: [s.invoke('robot.create_orientation_constraint', [flipped])] }), null);
});
test('compute_ik converts joint, orientation and position constraints from their messages', () => {
  const s = session(), pose = s.invoke('robot.compute_fk'), ee = s.sim.model.end_effector_name;
  const ik = constraints => s.invoke('robot.compute_ik', [], { position: pose.pose.position, quat_xyzw: pose.pose.orientation, constraints });
  const region = (primitive, primitive_poses, extra = {}) => ({ __class__: 'PositionConstraint', header: { frame_id: s.sim.model.base_link_name }, link_name: ee, constraint_region: { primitives: [primitive], primitive_poses }, weight: 1, ...extra });
  const box = { type: 1, dimensions: [0.05, 0.05, 0.05] }, here = [{ position: pose.pose.position, orientation: pose.pose.orientation }];
  const solved = [
    { joint_constraints: s.invoke('robot.create_joint_constraints', [[...s.sim.q]]) },
    { joint: [{ joint_name: s.sim.model.joint_names[0], position: s.sim.q[0], tolerance: 0.01 }] },
    { orientation_constraints: [s.invoke('robot.create_orientation_constraint', [pose.pose.orientation], { tolerance: 0.01, parameterization: 1 })] },
    { position_constraints: [s.invoke('robot.create_position_constraint', [pose.pose.position], { tolerance: 0.01 })] },
    { position_constraints: [region(box, here)] },
    { position_constraints: [region(box, [{ position: pose.pose.position }])] },
    { position_constraints: [{ ...region(box, here), header: undefined }] },
  ];
  for (const constraints of solved) assert.equal(ik(constraints)?.__class__, 'JointState', JSON.stringify(constraints));
  const unsolved = [
    { joint: [{ joint_name: s.sim.model.joint_names[0], position: 10, tolerance: 0.01 }] },
    { position_constraints: [s.invoke('robot.create_position_constraint', [[5, 5, 5]])] },
    { position_constraints: [region(box, here, { target_point_offset: [0, 0, 0.5] })] },
    { position_constraints: [region(box, [])] },
    { orientation: [{ quat_xyzw: [0, 0, 1, 0], tolerance: 0.01 }] },
    { position: [{ position: [5, 5, 5], tolerance: 0.01 }] },
    { position_constraints: [region(box, here, { constraint_region: undefined })] },
    { position_constraints: [s.invoke('robot.create_position_constraint', [pose.pose.position], { frame_id: 'no_such_frame' })] },
    { orientation_constraints: [s.invoke('robot.create_orientation_constraint', [pose.pose.orientation], { frame_id: 'no_such_frame' })] },
  ];
  for (const constraints of unsolved) assert.equal(ik(constraints), null, JSON.stringify(constraints));
});
test('constraint messages default missing tolerances, parameterization and region orientation', () => {
  const identity = mat4FromPose({ position: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] });
  assert.deepEqual(jointConstraint({ joint_name: 'j', position: 1 }), { joint_name: 'j', position: 1, tolerance: 0, tolerance_above: 0, tolerance_below: 0 });
  assert.deepEqual(jointConstraint({ joint_name: 'j', position: 1, tolerance_above: 0.2, tolerance_below: 0.1 }), { joint_name: 'j', position: 1, tolerance: 0.1, tolerance_above: 0.2, tolerance_below: 0.1 });
  const orientation = orientationConstraint({ link_name: 'l', orientation: [0, 0, 0, 1] }, identity);
  assert.deepEqual([orientation.tolerance, orientation.parameterization, orientation.link], [[0, 0, 0], 0, 'l']);
  const region = positionConstraint({ link_name: 'l', constraint_region: { primitives: [{ type: 1, dimensions: [1, 2, 3] }], primitive_poses: [{ position: [1, 2, 3] }] } }, identity);
  assert.deepEqual(region, { link: 'l', regions: [{ type: 'box', dimensions: [1, 2, 3], pose: { position: [1, 2, 3], quat_xyzw: [0, 0, 0, 1] } }], target_point_offset: [0, 0, 0] });
});
test('kinematics messages, immediate plan future, scene transport and attachment defaults', () => {
  const s = session(), ros = []; s.on('ros', r => ros.push(r));
  const pose = s.invoke('robot.compute_fk'); assert.equal(pose.__class__, 'PoseStamped');
  assert.equal(s.invoke('robot.compute_fk', [], { fk_link_names: ['panda_hand', 'panda_link8'] }).length, 2);
  const q = s.invoke('robot.compute_ik', [], { position: pose.pose.position, quat_xyzw: pose.pose.orientation, start_joint_state: s.getattr('robot.joint_state') }); assert.equal(q.__class__, 'JointState');
  const constraints = { joint_constraints: s.invoke('robot.create_joint_constraints', [[...s.sim.q]]) };
  assert.equal(s.invoke('robot.compute_ik', [], { position: pose.pose.position, quat_xyzw: pose.pose.orientation, constraints }).__class__, 'JointState');
  const future = s.invoke('robot.plan_async', [], { joint_positions: [...s.sim.q] }); assert.deepEqual(Object.keys(future).sort(), ['__class__', 'done', 'id']); assert.equal(future.done, true);
  const trajectory = s.invoke('robot.get_trajectory', [future]); assert.equal(trajectory.__class__, 'JointTrajectory'); assert.equal(typeof trajectory.points[0].time_from_start.sec, 'number');
  s.invoke('robot.add_collision_box', [], { id: 'box', size: [0.1, 0.1, 0.1], position: [1, 1, 1], quat_xyzw: [0, 0, 0, 1] });
  s.invoke('robot.attach_collision_object', ['box']); const attached = s.sim.getObjects()[0].attached; assert.equal(attached.link, 'panda_link8'); assert.deepEqual(attached.touch_links, []);
  const allowed = s.invoke('robot.allow_collisions', ['box', true]); assert.equal(s.invoke('robot.process_allow_collision_future', [allowed]), true);
  const clear = s.invoke('robot.clear_all_collision_objects'); assert.equal(s.invoke('robot.process_clear_all_collision_objects_future', [clear]), true); assert.equal(s.sim.getObjects().length, 0);
  assert.ok(ros.some(r => r.name === '/attached_collision_object' && r.type === 'moveit_msgs/msg/AttachedCollisionObject'));
  assert.throws(() => s.invoke('robot.add_collision_mesh', ['anything.stl', 'bad'], { position: [0, 0, 0], quat_xyzw: [0, 0, 0, 1] }), e => e.type === 'ValueError' && e.message.includes('examples/assets/suzanne.stl'));
});
test('plan timeout matches Python finite-float validation through session and arm facades', () => {
  const s = session(), paths = ['robot.plan', 'robot.arm.plan'], joint_positions = [...s.sim.q];
  for (const path of paths) {
    for (const timeout_sec of [true, '1', NaN, Infinity, -Infinity]) {
      const message = typeof timeout_sec === 'number' ? '`timeout_sec` must be finite.' : '`timeout_sec` must be a finite number.';
      assert.throws(
        () => s.invoke(path, [], { joint_positions, timeout_sec }),
        e => e instanceof PyError && e.type === 'ValueError' && e.message === message,
        `${path} should reject timeout_sec=${String(timeout_sec)}`,
      );
    }
    assert.equal(s.invoke(path, [], { joint_positions, timeout_sec: null }).__class__, 'JointTrajectory');
    assert.equal(s.invoke(path, [], { joint_positions, timeout_sec: 1 }).__class__, 'JointTrajectory');
    for (const timeout_sec of [0, -1]) {
      assert.equal(s.invoke(path, [], { joint_positions, timeout_sec }), null);
      assert.equal(s.planFailure, 'planning timed out; raise `timeout_sec`, or `allowed_planning_time` when the planner itself needs longer');
    }
  }
  const pose = { position: [0.4, 0, 0.4], quat_xyzw: [1, 0, 0, 0] };
  const motions = [
    ['robot.move_to_pose', [], pose],
    ['robot.arm.move_to_pose', [], pose],
    ['robot.move_to_configuration', [joint_positions], {}],
    ['robot.arm.move_to_configuration', [joint_positions], {}],
  ];
  for (const [path, args, kwargs] of motions) {
    for (const timeout_sec of [true, '1', NaN, Infinity, -Infinity]) {
      const message = typeof timeout_sec === 'number' ? '`timeout_sec` must be finite.' : '`timeout_sec` must be a finite number.';
      assert.throws(
        () => s.invoke(path, args, { ...kwargs, timeout_sec }),
        e => e instanceof PyError && e.type === 'ValueError' && e.message === message,
        `${path} should reject timeout_sec=${String(timeout_sec)}`,
      );
    }
  }
});
test('plan_async exposes completed futures through session and arm facades', () => {
  const s = session(), joint_positions = [...s.sim.q];
  for (const path of ['robot.plan_async', 'robot.arm.plan_async']) {
    const future = s.invoke(path, [], { joint_positions });
    assert.deepEqual(Object.keys(future).sort(), ['__class__', 'done', 'id']);
    assert.equal(future.done, true);
    assert.equal(s.invoke('robot.get_trajectory', [future]).__class__, 'JointTrajectory');
  }
});
test('expired planning deadlines consume stored goals after validation, like Python', () => {
  const s = session(), q = [...s.sim.q], pathConstraint = { position: [], orientation: [], joint: [] };
  const prime = () => {
    s.invoke('robot.set_joint_goal', [q]);
    s.invoke('robot.set_path_joint_constraint', [[q[0]], ['panda_joint1']], { tolerance: 0.05 });
  };
  const assertConsumed = () => {
    assert.deepEqual(s.goals, [{}]);
    assert.deepEqual(s.pathConstraints, pathConstraint);
  };
  prime();
  assert.equal(s.invoke('robot.plan', [], { timeout_sec: 0 }), null);
  assertConsumed();
  prime();
  assert.equal(runSync(s.sim, armOperation(s, 'plan_async', { timeout_sec: 0 }, null)), null);
  assertConsumed();
  for (const path of ['robot.move_to_configuration', 'robot.arm.move_to_configuration']) {
    prime();
    assert.equal(s.invoke(path, [q], { timeout_sec: 0 }), false);
    assertConsumed();
  }
  prime();
  assert.equal(s.invoke('robot.move_to_pose', [], { position: [0.4, 0, 0.4], quat_xyzw: [1, 0, 0, 0], timeout_sec: 0 }), false);
  assertConsumed();

  for (const timeout_sec of [true, '1']) {
    prime();
    const goals = structuredClone(s.goals), constraints = structuredClone(s.pathConstraints);
    assert.throws(() => s.invoke('robot.plan', [], { timeout_sec }), e => e instanceof PyError && e.type === 'ValueError');
    assert.deepEqual(s.goals, goals);
    assert.deepEqual(s.pathConstraints, constraints);
  }
});
test('description defaults and gripper/servo behavior', () => {
  const s = session();
  assert.deepEqual(s.invoke('robot.description.joint_positions'), s.sim.q);
  assert.equal(s.invoke('robot.description.moveit2_kwargs').group_name, 'panda_arm');
  assert.deepEqual(s.invoke('robot.description.moveit2_gripper_kwargs').open_gripper_joint_positions, [0.035, 0.035]);
  assert.deepEqual(s.getattr('robot.description.group_names'), ['panda_arm', 'hand', 'panda_arm_hand']);
  assert.deepEqual(s.invoke('robot.description.group', ['panda_arm_hand']).passive_joint_names, ['panda_finger_joint2']);
  assert.equal(s.getattr('robot.gripper.is_open'), true); assert.equal(s.invoke('robot.gripper.open', [], { skip_if_noop: true }), false);
  assert.equal(s.invoke('robot.gripper.open'), true); assert.equal(s.invoke('robot.gripper.wait_until_executed'), true);
  assert.equal(s.invoke('robot.gripper.close'), true); assert.equal(s.invoke('robot.gripper.wait_until_executed'), true); assert.equal(s.getattr('robot.gripper.is_closed'), true);
  s.sim.fingers = 0.02; assert.equal(s.invoke('robot.gripper.toggle'), false);
  assert.equal(s.invoke('robot.servo', [[0.01, 0, 0]]), true); assert.equal(s.getattr('robot.servo.active_command_type'), 1);
  s.sleep(0.2); assert.equal(s.sim.isBusy(), false); assert.equal(s.invoke('robot.servo.disable'), true); assert.equal(s.getattr('robot.servo.is_enabled'), false);
});
test('OR goal alternatives and full-pose move validation follow Python builders', () => {
  const s = session();
  assert.equal(s.invoke('robot.execute', [null]), false); assert.equal(s.invoke('robot.last_failure'), 'There is no trajectory to execute.');
  assert.throws(() => s.invoke('robot.move_to_pose', [[0.4, 0, 0.4]]), e => e.type === 'ValueError' && e.message === 'Either `pose` or `position` and `quat_xyzw` must be specified!');
  s.invoke('robot.set_pose_goal', [], { position: [2, 0, 0], quat_xyzw: [1, 0, 0, 0] });
  s.invoke('robot.create_new_goal_constraint'); s.invoke('robot.set_joint_goal', [[...s.sim.q]]);
  assert.equal(s.invoke('robot.plan').__class__, 'JointTrajectory');
  s.invoke('robot.clear_goal_constraints');
  const current = s.invoke('robot.compute_fk');
  s.invoke('robot.set_pose_goal', [current]); s.invoke('robot.set_joint_goal', [[...s.sim.q]]);
  assert.equal(s.invoke('robot.plan').__class__, 'JointTrajectory');
  s.invoke('robot.set_pose_goal', [current]); s.invoke('robot.set_joint_goal', [[...s.sim.q].map((x, i) => i === 0 ? x + 1 : x)]);
  assert.equal(s.invoke('robot.plan'), null);
});
test('each plan clears goal and path constraints, as pymoveit2 does', () => {
  const s = session(), q = [...s.sim.q], bend = (i, d) => q.map((x, j) => j === i ? x + d : x);
  s.invoke('robot.set_path_joint_constraint', [[q[0]], ['panda_joint1']], { tolerance: 0.05 });
  assert.equal(s.invoke('robot.move_to_configuration', [bend(1, 0.2)]), true);
  assert.equal(s.invoke('robot.move_to_configuration', [[q[0] + 0.5, q[1] + 0.2, ...q.slice(2)]]), true);
  s.invoke('robot.set_joint_goal', [q]); assert.equal(s.invoke('robot.plan').__class__, 'JointTrajectory');
  assert.throws(() => s.invoke('robot.plan'), e => e.type === 'ValueError' && e.message === 'Planning requires a goal constraint.');
});
test('a replaced live wait retains the canceled operation outcome', async () => {
  const sim = new MoveGroupSim(), s = createSession(sim, { driver: 'live' }), firstGoal = [...sim.q], secondGoal = [...sim.q];
  firstGoal[0] += 0.6; secondGoal[0] -= 0.2;
  const first = s.invoke('robot.move_to_configuration', [firstGoal]); sim.step(0.01);
  const second = s.invoke('robot.move_to_configuration', [secondGoal]);
  for (let i = 0; i < 1000 && sim.isBusy('arm'); i++) sim.step(1 / 120);
  assert.equal(await first, false); assert.equal(await second, true);
});
test('Python repr, message constructors, float hints and complete scripts', () => {
  assert.equal(pyRepr([true, false, null]), '[True, False, None]'); assert.equal(pyRepr([1, 2], 'tuple[float, float]'), '(1.0, 2.0)');
  assert.equal(pyRepr("a'b\\c\n"), "'a\\'b\\\\c\\n'");
  assert.equal(formatCall('robot.move_to_pose', [], { position: [0.4, 0, 0.4], quat_xyzw: [1, 0, 0, 0] }), 'robot.move_to_pose(position=[0.4, 0.0, 0.4], quat_xyzw=[1.0, 0.0, 0.0, 0.0])');
  assert.equal(formatCall('robot.arm.max_velocity', [], {}), 'robot.arm.max_velocity()');
  const repr = pyRepr(session().invoke('robot.compute_fk')); assert.ok(repr.startsWith('geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=\'panda_link0\')'));
  assert.ok(repr.includes('geometry_msgs.msg.Quaternion(x=0.9239556994689482, y=-0.3824994972792076, z=1.3254497398040232e-12, w=3.200197673974012e-12)'));
  assert.equal(buildScript([]), 'from pymoveit2 import connect\n\nwith connect() as robot:\n    pass\n');
  assert.equal(buildScript(['time.sleep(1.0)']), 'from pymoveit2 import connect\nimport time\n\nwith connect() as robot:\n    time.sleep(1.0)\n');
});
test('live always returns Promises and resolves from state events; interrupt ends waits', async () => {
  const sim = new MoveGroupSim(), s = createSession(sim, { driver: 'live' });
  assert.ok(s.invoke('robot.joint_positions') instanceof Promise);
  const goal = [...sim.q]; goal[0] += 0.3;
  const motion = s.invoke('robot.move_to_configuration', [goal]); assert.ok(motion instanceof Promise);
  for (let i = 0; i < 1000 && sim.isBusy('arm'); i++) sim.step(1 / 120);
  assert.equal(await motion, true);
  const pending = s.sleep(5); s.interrupt(); assert.equal(await pending, null);
  const next = [...sim.q]; next[0] += 0.3; await s.invoke('robot.arm.move_to_configuration', [next]);
  const wait = s.invoke('robot.wait_until_executed', [], { timeout_sec: 0.01 }); sim.step(0.02); assert.equal(await wait, false);
  await s.invoke('robot.cancel_execution');
});
test('orientation constraints accept a per-axis tolerance triplet', () => {
  const s = session(), quat = s.sim.getState().ee.quat_xyzw;
  assert.equal(s.invoke('robot.arm.set_path_orientation_constraint', [quat], { tolerance: [0.1, 0.1, 3.14] }), null);
  assert.equal(s.invoke('robot.arm.clear_goal_constraints'), null);
  assert.equal(s.invoke('robot.arm.set_orientation_goal', [[1, 0, 0, 0]], { tolerance: [0.01, 0.01, 0.01] }), null);
  assert.equal(s.invoke('robot.arm.set_pose_goal', [], { position: [0.4, 0, 0.4], quat_xyzw: [1, 0, 0, 0], tolerance_orientation: [0.01, 0.01, 0.01] }), null);
  assert.throws(() => s.invoke('robot.arm.set_position_goal', [[0.4, 0, 0.4]], { tolerance: [0.01, 0.01, 0.01] }), e => e instanceof PyError && e.type === 'ValueError');
});
