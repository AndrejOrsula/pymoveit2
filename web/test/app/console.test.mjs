// The console's Python subset and the commander shortcuts, against a fake
// session that records what the console asks of pymoveit2.

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { PythonConsole, parse, repr } from '../../src/console/python.js';
import { expand, isCommand } from '../../src/console/commander.js';
import { PANDA } from '../../src/core/panda.js';
import { MoveGroupSim } from '../../src/core/sim.js';
import { createSession } from '../../src/api/session.js';

function fakeSession() {
  const calls = [];
  const attributes = { 'robot.arm.max_velocity': 0.0 };
  return {
    calls,
    attributes,
    invoke: async (path, args = [], kwargs = {}) => {
      calls.push({ path, args, kwargs });
      if (path === 'robot.compute_fk') {
        return { __class__: 'PoseStamped', header: { frame_id: 'panda_link0' }, pose: { position: { x: 0.3, y: 0, z: 0.5 }, orientation: { x: 1, y: 0, z: 0, w: 0 } } };
      }
      return true;
    },
    getattr: (path) => {
      if (!(path in attributes)) throw Object.assign(new Error(`no ${path}`), { type: 'AttributeError' });
      return attributes[path];
    },
    setattr: (path, value) => {
      attributes[path] = value;
    },
    members: (path) =>
      ({
        robot: [{ name: 'move_to_pose', kind: 'method', python_signature: 'move_to_pose(position=None, quat_xyzw=None, *, wait=True, timeout_sec=None, **kwargs) -> bool', doc: 'Plan and move.' }],
        'robot.arm': [
          { name: 'compute_fk', kind: 'method' },
          { name: 'max_velocity', kind: 'property' },
        ],
      })[path] ?? [],
    sleep: () => {},
    interrupt: () => {},
  };
}

function run(lines) {
  const session = fakeSession();
  const out = [];
  const console = new PythonConsole(session, { write: (kind, text) => out.push([kind, text]) });
  return { session, out, console, done: (async () => { for (const line of lines) await console.run(line); })() };
}

test('parses calls, keyword arguments and nested literals', () => {
  const [statement] = parse("robot.move_to_pose([0.4, 0.0, 0.4], quat_xyzw=(1.0, 0, 0, 0), cartesian=True)");
  assert.equal(statement.type, 'expr');
  assert.equal(statement.value.type, 'call');
  assert.equal(statement.value.args.length, 1);
  assert.deepEqual(statement.value.kwargs.map(([name]) => name), ['quat_xyzw', 'cartesian']);
});

test('rejects blocks with a pointer to the Script tab', () => {
  assert.throws(() => parse('for i in range(3):'), /Script tab/);
});

test('calls go to the session with positional and keyword arguments', async () => {
  const { session, out, done } = run(['robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0], cartesian=True)']);
  await done;
  assert.deepEqual(session.calls[0], { path: 'robot.move_to_pose', args: [[0.4, 0, 0.4], [1, 0, 0, 0]], kwargs: { cartesian: true } });
  assert.deepEqual(out, [['out', 'True']]);
});

test('RobotSession forwards unknown methods to the arm', async () => {
  const { session, done } = run(['robot.compute_fk()']);
  await done;
  assert.equal(session.calls[0].path, 'robot.compute_fk');
});

test('attribute assignment and reads use setattr and getattr', async () => {
  const { session, out, done } = run(['robot.arm.max_velocity = 0.5', 'robot.arm.max_velocity']);
  await done;
  assert.equal(session.attributes['robot.arm.max_velocity'], 0.5);
  assert.deepEqual(out.at(-1), ['out', '0.5']);
});

test('variables, message fields and arithmetic', async () => {
  const { out, done } = run(['pose = robot.compute_fk()', 'p = pose.pose.position', 'print(p.x + 0.1, round(p.z * 3, 3))', 'import math', 'math.degrees(math.pi)']);
  await done;
  assert.deepEqual(out, [['out', '0.4 1.5'], ['out', '180.0']]);
});

test('message fields stay ROS messages, as in rclpy', async () => {
  const { out, done } = run(['pose = robot.compute_fk()', 'pose.pose.position', 'pose.header']);
  await done;
  assert.deepEqual(out, [['out', 'geometry_msgs.msg.Point(x=0.3, y=0.0, z=0.5)'], ['out', "std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='panda_link0')"]]);
  const subscript = run(['robot.compute_fk().pose.position["x"]']);
  await assert.rejects(subscript.done, (error) => error.type === 'TypeError' && /'Point' object is not subscriptable/.test(error.message));
});

test('Python errors keep their type', async () => {
  const { done } = run(['undefined_name']);
  await assert.rejects(done, (error) => error.type === 'NameError');
  const second = run(['[1, 2][5]']);
  await assert.rejects(second.done, (error) => error.type === 'IndexError');
});

test('comparisons chain, and bad arithmetic and ranges raise instead of hanging', async () => {
  const { out, done } = run(['3 < 2 < 1', '1 < 2 < 3', '1 < 3 > 2 == 2', '-1 % 3', 'len(range(10, 0, -3))']);
  await done;
  assert.deepEqual(out, [['out', 'False'], ['out', 'True'], ['out', 'True'], ['out', '2'], ['out', '4']]);
  for (const [line, type] of [['1 % 0', 'ZeroDivisionError'], ['range(0, 3, 0)', 'ValueError'], ['range(10 ** 9)', 'ValueError'], ['range(1.5)', 'TypeError']]) {
    await assert.rejects(run([line]).done, (error) => error.type === type, line);
  }
});

test('reprs follow Python', () => {
  assert.equal(repr([1.5, 'a', null, true]), "[1.5, 'a', None, True]");
});

test('help shows the real signature', async () => {
  const { out, done } = run(['help(robot.move_to_pose)']);
  await done;
  assert.match(out[0][1], /^move_to_pose\(position=None, quat_xyzw=None, \*, wait=True/);
});

test('completion lists members after a dot', () => {
  const { console } = run([]);
  const result = console.complete('robot.arm.max');
  assert.deepEqual(result.items.map((item) => item.name), ['max_velocity']);
  assert.equal(result.from, 'robot.arm.'.length);
});

test('every commander planner plans and moves on the simulator', async () => {
  const context = { state: () => ({ ee: { position: [0.3, 0.0, 0.5], quat_xyzw: [1, 0, 0, 0] } }), model: PANDA, random: () => 0.5 };
  for (const planner of ['ompl', 'rrtstar', 'ptp', 'lin']) {
    const sim = new MoveGroupSim(), output = [], python = new PythonConsole(createSession(sim, { driver: 'sync' }), { write: (kind, text) => output.push(text) });
    for (const line of [...expand(`planner ${planner}`, context).python, 'robot.move_to_pose([0.4, 0.1, 0.45], [1.0, 0.0, 0.0, 0.0])', 'robot.last_failure()']) await python.run(line);
    assert.deepEqual(output, ['True'], planner);
  }
});

test('commander shortcuts expand into pymoveit2 Python', () => {
  const state = () => ({ ee: { position: [0.3, 0.0, 0.5], quat_xyzw: [1, 0, 0, 0] } });
  const context = { state, model: PANDA, random: () => 0.5 };
  assert.ok(isCommand('go ready'));
  assert.ok(!isCommand('open()'));
  assert.ok(!isCommand('help(robot)'));
  assert.deepEqual(expand('go ready', context).python, ['robot.move_to_configuration()']);
  assert.deepEqual(expand('go extended', context).python, ["robot.move_to_configuration(robot.joint_positions('extended'))"]);
  assert.deepEqual(expand('go up 0.05', context).python, ['robot.move_to_pose([0.3, 0.0, 0.55], [1.0, 0.0, 0.0, 0.0], cartesian=True)']);
  assert.deepEqual(expand('speed 0.5', context).python, ['robot.arm.max_velocity = 0.5', 'robot.arm.max_acceleration = 0.5']);
  assert.deepEqual(expand('planner ptp', context).python, ["robot.arm.pipeline_id = 'pilz_industrial_motion_planner'", "robot.arm.planner_id = 'PTP'"]);
  assert.match(expand('add box crate 0.5 0 0.2', context).python[0], /^robot\.add_collision_box\('crate', \[0\.1, 0\.1, 0\.1\], position=\[0\.5, 0\.0, 0\.2\]/);
  assert.throws(() => expand('go nowhere', context), /usage/);
  assert.throws(() => expand('speed 2', context), /usage/);
});

test('a session that records statements gets each line as typed', async () => {
  const session = fakeSession();
  const lines = [];
  session.statement = async (source, run, { keep }) => {
    await run();
    lines.push([source, keep]);
  };
  const console = new PythonConsole(session, { write: () => {} });
  for (const line of ['from pymoveit2 import connect', 'import math', 'x = 0.4', '  robot.move_to_pose([x, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])', 'robot.arm.max_velocity', '', 'pose = robot.compute_fk(); pose.pose.position.x = 0.5']) {
    await console.run(line);
  }
  assert.deepEqual(lines, [
    ['from pymoveit2 import connect', false],
    ['import math', true],
    ['x = 0.4', true],
    ['robot.move_to_pose([x, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])', false],
    ['robot.arm.max_velocity', false],
    ['pose = robot.compute_fk(); pose.pose.position.x = 0.5', true],
  ]);
  assert.equal(session.calls.length, 2, 'the lines still ran');
});

test('commander shortcuts reject extra and partial arguments like a Python call would', () => {
  const state = () => ({ ee: { position: [0.3, 0.0, 0.5], quat_xyzw: [1, 0, 0, 0] } });
  const context = { state, model: PANDA, random: () => 0.5 };
  const joints = PANDA.joints.map(() => '0');
  for (const line of [
    'pose 0.3 0 0.5 1',
    'pose 0.3 0 0.5 1 0 0',
    'pose 0.3 0 0.5 1 0 0 0 9',
    'line 0.3 0 0.5 0',
    `joints ${[...joints, '0'].join(' ')}`,
    'ik 0.3 0 0.5 1',
    'add box crate 0.5 0 0.2 0.1 9',
    'go up 0.05 9',
    'go ready now',
    'open now',
    'close 1',
    'where 1',
    'clear all',
    'remove a b',
    'attach a b',
    'detach a b',
    'speed 0.5 0.6',
    'planner ptp lin',
    'stop now',
    'why not',
  ]) assert.throws(() => expand(line, context), /takes/, line);
  assert.throws(() => expand(`joints ${joints.slice(1).join(' ')}`, context), /takes/);
  assert.deepEqual(expand('pose 0.3 0 0.5', context).python, ['robot.move_to_pose([0.3, 0.0, 0.5], [1.0, 0.0, 0.0, 0.0])']);
  assert.deepEqual(expand('pose 0.3 0 0.5 0 1 0 0', context).python, ['robot.move_to_pose([0.3, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0])']);
  assert.deepEqual(expand(`joints ${joints.join(' ')}`, context).python, [`robot.move_to_configuration([${PANDA.joints.map(() => '0.0').join(', ')}])`]);
  assert.match(expand('add sphere ball 0.5 0 0.2 0.2', context).python[0], /^robot\.add_collision_sphere\('ball', 0\.1,/);
  assert.deepEqual(expand('open', context).python, ['robot.gripper.open()']);
  for (const line of ['help', 'commands', 'go up', 'go up 0.1', 'go ready', 'remove a', 'attach a', 'detach a', 'speed 0.5', 'planner ptp', 'clear', 'stop', 'why']) {
    assert.ok(expand(line, context), line);
  }
  assert.throws(() => expand('go', context), { message: 'go takes 1 argument (0 given); type help' });
  assert.throws(() => expand('remove a b', context), { message: 'remove takes 1 argument (2 given); type help' });
  assert.throws(() => expand('stop now', context), { message: 'stop takes 0 arguments (1 given); type help' });
  assert.throws(() => expand('pose 0.3 0 0.5 1', context), { message: 'pose takes 3 or 7 arguments (4 given); type help' });
  assert.throws(() => expand('go up 0.05 9', context), { message: 'go takes 1 or 2 arguments (3 given); type help' });
});
