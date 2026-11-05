// Runs the Script tab's examples in real Python (Pyodide from node_modules)
// against the simulation, as the browser's worker does.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { loadPyodide } from 'pyodide';

import { ScriptRunner } from '../src/py/runner.js';
import { EXAMPLES } from '../src/ui/examples.js';
import { MoveGroupSim } from '../src/core/sim.js';
import { robotById } from '../src/core/robots.js';

const shim = await readFile(new URL('../src/py/pymoveit2_shim.py', import.meta.url), 'utf8');
const pyodide = await loadPyodide();
const runner = new ScriptRunner(pyodide, { shim });

const text = (result) => result.output.map((entry) => entry.text).join('\n');
const withAdvancingClock = fn => {
  const descriptor = Object.getOwnPropertyDescriptor(performance, 'now');
  const now = performance.now.bind(performance);
  let seconds = 0;
  Object.defineProperty(performance, 'now', { configurable: true, value: () => now() + (++seconds * 1000) });
  try { return fn(); }
  finally { if (descriptor) Object.defineProperty(performance, 'now', descriptor); else delete performance.now; }
};
const withExpiredSimulationBudget = fn => {
  const step = MoveGroupSim.prototype.step;
  MoveGroupSim.prototype.step = function (dt) {
    const state = step.call(this, dt);
    this.time = 600;
    this.emit('state', this.getState());
    return this.getState();
  };
  try { return fn(); }
  finally { MoveGroupSim.prototype.step = step; }
};

for (const [robot, examples] of Object.entries(EXAMPLES)) for (const example of examples) {
  test(`${robot} example "${example.title}" runs without errors`, () => {
    const result = runner.run(example.source, new MoveGroupSim({ model: robotById(robot) }).snapshot());
    assert.equal(result.error, null, JSON.stringify(result.error));
    assert.ok(result.frames.length > 10, 'the robot moved');
    assert.ok(result.calls.length > 0, 'pymoveit2 was called');
    assert.ok(result.lines.length > 0, 'lines were traced');
    if (example.id === 'quick-start') assert.equal(result.exit, null, text(result));
    if (example.id === 'square') assert.doesNotMatch(text(result), /failed/, text(result));
    if (example.id === 'plan-then-execute') assert.match(text(result), /Succeeded: True/);
    if (example.id === 'obstacle-course') assert.match(text(result), /reached\n.*reached\n.*GOAL_IN_COLLISION/);
  });
}

test('the shim mirrors pymoveit2 semantics', () => {
  const result = runner.run(`
from pymoveit2 import connect, MoveIt2State

with connect() as robot:
    print(robot.arm.query_state() == MoveIt2State.IDLE)
    print(robot.move_to_pose([2.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]))
    print(robot.last_failure().split(":")[0])
    pose = robot.compute_fk()
    print(pose.header.frame_id, round(pose.pose.position.z, 2) > 0)
    robot.max_velocity = 0.3
    print(robot.arm.max_velocity)
    try:
        robot.move_to_pose([0.4, 0.0, 0.4], quaternion=[1.0, 0.0, 0.0, 0.0])
    except TypeError as error:
        print("TypeError", error)
`);
  assert.equal(result.error, null, JSON.stringify(result.error));
  const lines = text(result).trim().split('\n');
  assert.equal(lines[0], 'True');
  assert.equal(lines[1], 'False');
  assert.equal(lines[2], 'NO_IK_SOLUTION');
  assert.equal(lines[3], 'panda_link0 True');
  assert.equal(lines[4], '0.0');
  assert.match(lines[5], /^TypeError .*unexpected keyword argument 'quaternion'/);
});

test('errors report the script line, and runaway loops stop', () => {
  const failing = runner.run('from pymoveit2 import connect\n\nwith connect() as robot:\n    robot.move_to_configuration([0.0] * 9)\n');
  assert.ok(failing.error, 'an error was reported');
  assert.equal(failing.error.type, 'ValueError');
  assert.equal(failing.error.line, 4);

  // A script that does not parse names its line too.
  const unparsed = runner.run('from pymoveit2 import connect\n\nwith connect() as robot:\n    robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]\n');
  assert.equal(unparsed.error?.type, 'SyntaxError');
  assert.equal(unparsed.error.line, 4);

  const forever = runner.run('import time\nwhile True:\n    time.sleep(1)\n');
  assert.equal(forever.error?.type, 'TimeoutError');

  // One long sleep on the last line must not run past the limit either.
  const start = performance.now();
  const sleepy = runner.run('import time\ntime.sleep(1e9)\n');
  assert.equal(sleepy.error?.type, 'TimeoutError');
  assert.equal(sleepy.error.line, 2);
  assert.ok(performance.now() - start < 20_000, 'the long sleep stopped at the limit');
});

test('a synchronous wait enforces the fixed script budget and reports the original call line', () => {
  const result = withAdvancingClock(() => runner.run(`from pymoveit2 import connect

with connect() as robot:
    goal = robot.joint_positions()
    goal[0] += 0.5
    robot.move_to_configuration(goal)
`));
  assert.equal(result.error?.type, 'TimeoutError', JSON.stringify(result.error));
  assert.equal(result.error.line, 6);
  const failedWait = result.calls.find(call => call.path === 'robot.move_to_configuration' && call.status === 'raised');
  assert.ok(failedWait, 'the wait surfaced through the session error path');
  assert.ok(failedWait.t <= 0.3, `wait failed after ${failedWait.t} simulated seconds`);

  const caught = withAdvancingClock(() => runner.run(`from pymoveit2 import connect

with connect() as robot:
    goal = robot.joint_positions()
    goal[0] += 0.5
    try:
        robot.move_to_configuration(goal)
    except TimeoutError:
        robot.move_to_configuration(goal)
`));
  assert.equal(caught.error?.type, 'TimeoutError', JSON.stringify(caught.error));
  assert.equal(caught.error.line, 7, 'catching the first timeout does not reset or move its origin');
  assert.equal(caught.calls.filter(call => call.path === 'robot.move_to_configuration').length, 1, 'the retry is stopped before invocation');
});

test('the fixed 600-second simulation budget stops a wait without mutable runner limits', () => {
  const result = withExpiredSimulationBudget(() => runner.run(`from pymoveit2 import connect

with connect() as robot:
    goal = robot.joint_positions()
    goal[0] += 0.5
    robot.move_to_configuration(goal)
`));
  assert.equal(result.error?.type, 'TimeoutError', JSON.stringify(result.error));
  assert.equal(result.error.line, 6);
  const failedWait = result.calls.find(call => call.path === 'robot.move_to_configuration' && call.status === 'raised');
  assert.ok(failedWait, 'the wait surfaced through the session error path');
  assert.equal(failedWait.t, 600);
});

test('nested handlers cannot swallow a budget timeout or make later bridge calls', () => {
  const result = withExpiredSimulationBudget(() => runner.run(`import time
from pymoveit2 import connect

with connect() as robot:
    try:
        try:
            time.sleep(1)
        except TimeoutError:
            pass
    except TimeoutError:
        pass
    try:
        robot.compute_fk()
    except TimeoutError:
        pass
    print("continued")
    raise SystemExit(0)
`));
  assert.equal(result.error?.type, 'TimeoutError', JSON.stringify(result.error));
  assert.equal(result.error.message, 'The demo stopped the script: it ran longer than the browser allows.');
  assert.equal(result.error.line, 7);
  assert.equal(result.error.traceback, 'TimeoutError: The demo stopped the script: it ran longer than the browser allows.');
  assert.equal(result.exit, null);
  assert.equal(text(result), 'continued');
  assert.equal(result.calls.some(call => call.path === 'robot.compute_fk'), false, 'bridge calls stop after budget exhaustion');
});

test('non-finite numbers reach pymoveit2 validation as a ValueError', () => {
  const result = runner.run('from pymoveit2 import connect\n\nwith connect() as robot:\n    robot.move_to_pose([float("nan"), 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])\n');
  assert.equal(result.error?.type, 'ValueError', JSON.stringify(result.error));
  assert.match(result.error.message, /finite/);
});

test('each connect() gives a fresh, recorded session with default parameters', () => {
  const result = runner.run(`from pymoveit2 import connect

with connect() as robot:
    robot.arm.max_velocity = 0.5
    print(robot.move_to_configuration(robot.joint_positions("extended")))
with connect() as robot:
    print(robot.arm.max_velocity)
    print(robot.move_to_configuration())
for args, kwargs in (((), {"nodename": "x"}), (("a", "b"), {})):
    try:
        connect(*args, **kwargs)
    except TypeError as error:
        print("TypeError", error)
`, new MoveGroupSim().snapshot());
  assert.equal(result.error, null, JSON.stringify(result.error));
  assert.deepEqual(text(result).trim().split('\n'), ['True', '0.0', 'True', "TypeError connect() got an unexpected keyword argument 'nodename'", 'TypeError connect() takes from 0 to 1 positional arguments but 2 were given']);
  assert.ok(result.calls.some((call) => call.path === 'robot.move_to_configuration' && call.line === 8), 'the second session is recorded');
  assert.deepEqual(result.snapshot.q, new MoveGroupSim().q);
});

test('a plain Pose, such as compute_fk().pose, is a valid pose goal', () => {
  const result = runner.run(`from pymoveit2 import connect

with connect() as robot:
    pose = robot.compute_fk().pose
    print(robot.move_to_pose(pose=pose))
    print(robot.compute_ik(pose.position, pose.orientation) is not None)
`, new MoveGroupSim().snapshot());
  assert.equal(result.error, null, JSON.stringify(result.error));
  assert.deepEqual(text(result).trim().split('\n'), ['True', 'True']);
});
