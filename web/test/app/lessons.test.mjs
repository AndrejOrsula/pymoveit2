// Runs every lesson's setup and steps through the console interpreter and the
// pymoveit2 stand-in (sync driver), and checks each step's `expect`, so the
// lesson text cannot drift from what the simulation does.

import { test } from 'node:test';
import assert from 'node:assert/strict';

import { PythonConsole } from '../../src/console/python.js';
import { MoveGroupSim } from '../../src/core/sim.js';
import { createSession } from '../../src/api/session.js';
import { LESSONS } from '../../src/ui/lessons.js';
import { stepPassed } from '../../src/ui/learn.js';
import { PRESETS } from '../../src/ui/control.js';
import { robotById } from '../../src/core/robots.js';

// A learner may open any lesson after moving the arm: start each lesson from
// another named state, and reset the world first when the lesson asks, as the
// Learn tab does (`App.resetWorld`).
function movedSim(robot) {
  const sim = new MoveGroupSim({ model: robotById(robot) });
  const elsewhere = Object.values(sim.model.named_states[sim.model.group_name]).find((q) => q.some((x, i) => x !== sim.q[i]));
  sim.reset({ ...sim.snapshot(), q: elsewhere });
  return sim;
}

for (const [robot, lessons] of Object.entries(LESSONS)) for (const lesson of lessons) {
  test(`${robot} lesson "${lesson.title}" behaves as written, after the arm moved elsewhere`, async () => {
    const sim = movedSim(robot);
    if (lesson.reset) sim.reset();
    const session = createSession(sim, { driver: 'sync' });
    const quiet = new PythonConsole(session, { write: () => {} });
    for (const line of lesson.setup ?? []) await quiet.run(line);
    let output = [];
    const python = new PythonConsole(session, { write: (kind, text) => output.push({ kind, text }) });
    for (const [i, step] of lesson.steps.entries()) {
      if (!step.code) continue;
      output = [];
      for (const line of step.code.split('\n')) {
        try {
          await python.run(line);
        } catch (error) {
          // As in the Learn tab, an exception ends the step.
          output.push({ kind: 'err', text: `${error.type ?? 'Error'}: ${error.message}` });
          break;
        }
      }
      const lines = output.filter((entry) => entry.kind !== 'note');
      const last = lines.at(-1)?.text ?? '';
      const transcript = `step ${i + 1}: ${step.code}\n--> ${lines.map((entry) => `[${entry.kind}] ${entry.text}`).join('\n--> ')}`;
      if (step.expect) assert.match(last, step.expect, transcript);
      assert.ok(stepPassed(step, lines), transcript);
    }
  });
}

test('FK lesson point expectations compare finite coordinates within 1 mm', () => {
  const panda = LESSONS.panda.find(({ title }) => title === 'Poses and orientations').steps[0];
  const omx = LESSONS.open_manipulator_x.find(({ id }) => id === 'frames').steps[1];
  const lines = (text) => [{ kind: 'out', text }];
  assert.deepEqual(panda.expectPosition, [0.307, 0, 0.5903]);
  assert.deepEqual(omx.expectPosition, [0.0237, 0, 0.1855]);
  assert.equal(stepPassed(panda, lines('geometry_msgs.msg.Point(x=0.3070195700516105, y=-5.221329615610068e-12, z=0.5902695582766448)')), true);
  assert.equal(stepPassed(omx, lines('geometry_msgs.msg.Point(x=0.0237206939369997, y=-7.611374536556099e-17, z=0.1854985044125177)')), true);
  assert.equal(stepPassed(omx, lines('geometry_msgs.msg.Point(x=0.024699, y=0.000999, z=0.186499)')), true);
  for (const output of [
    'geometry_msgs.msg.Point(x=0.0247, y=0.0, z=0.1855)',
    'geometry_msgs.msg.Point(x=0.0237, y=0.001, z=0.1855)',
    'geometry_msgs.msg.Point(x=0.0237, y=0.0, z=0.1865)',
    'geometry_msgs.msg.Point(x=NaN, y=0.0, z=0.1855)',
    'geometry_msgs.msg.Point(x=0.0237, y=Infinity, z=0.1855)',
    'geometry_msgs.msg.Point(x=0.0237, y=0.0)',
    'geometry_msgs.msg.Point(x=0x1, y=0.0, z=0.1855)',
  ]) assert.equal(stepPassed(omx, lines(output)), false, output);
});

test('lesson texts stay short: at most 200 visible characters each', () => {
  const visible = (html) => html.replace(/<[^>]+>/g, '').replace(/&[a-z]+;/g, ' ');
  for (const [robot, lessons] of Object.entries(LESSONS)) for (const lesson of lessons) {
    const texts = [['intro', lesson.intro], ['outro', lesson.outro], ...lesson.steps.flatMap((step, i) => [[`step ${i + 1}`, step.text], [`step ${i + 1} after`, step.after]])];
    for (const [where, html] of texts) {
      if (html) assert.ok(visible(html).length <= 200, `${robot}/${lesson.id} ${where}: ${visible(html).length} characters`);
    }
  }
});

test('scene presets run and leave the robot clear', async () => {
  for (const [robot, presets] of Object.entries(PRESETS)) {
    const sim = new MoveGroupSim({ model: robotById(robot) });
    const session = createSession(sim, { driver: 'sync' });
    for (const [name, { code }] of Object.entries(presets)) {
      sim.clearScene();
      const errors = [];
      const python = new PythonConsole(session, { write: (kind, text) => kind === 'err' && errors.push(text) });
      for (const line of code.split('\n')) await python.run(line);
      assert.deepEqual(errors, [], `${robot} ${name}`);
      assert.equal(sim.getObjects().length, code.split('\n').length, `${robot} ${name}`);
      assert.deepEqual(sim.getState().contacts, [], `${robot} ${name}`);
      // Positions are in the planning frame, where the scene draws them.
      const written = [...code.matchAll(/position=\[([^\]]+)\]/g)].map((m) => m[1].split(', ').map(Number));
      assert.deepEqual(sim.getObjects().map((o) => o.pose.position.map((x) => Math.round(x * 1e4) / 1e4)), written, `${robot} ${name}`);
    }
  }
});
