import assert from 'node:assert/strict';
import { afterEach, it } from 'node:test';

import { WALL_TIME_LIMIT } from '../../src/py/runner.js';
import { PythonWorker } from '../../src/ui/script.js';

const workers = [];
const realSetTimeout = globalThis.setTimeout;
const realClearTimeout = globalThis.clearTimeout;
class FakeWorker {
  constructor() {
    this.listeners = new Map();
    workers.push(this);
  }

  addEventListener(type, listener) { this.listeners.set(type, listener); }
  postMessage(message) { this.messages ??= []; this.messages.push(message); }
  terminate() { this.terminated = true; }
  emit(data) { this.listeners.get('message')({ data }); }
}

afterEach(() => {
  delete globalThis.Worker;
  globalThis.setTimeout = realSetTimeout;
  globalThis.clearTimeout = realClearTimeout;
  workers.length = 0;
});

it('stops a native Python call after the wall limit, without timing Python startup', async () => {
  globalThis.Worker = FakeWorker;
  const timers = new Map();
  let nextTimer = 1;
  globalThis.setTimeout = (callback, delay) => {
    const id = nextTimer++;
    timers.set(id, { callback, delay });
    return id;
  };
  globalThis.clearTimeout = (id) => timers.delete(id);
  const advanceClock = () => {
    const due = [...timers.values()];
    timers.clear();
    for (const { callback } of due) callback();
  };

  const python = new PythonWorker({ onStatus: () => {} });
  const firstRun = python.run('while True: pass', {});
  const first = workers[0];
  assert.deepEqual([...timers.values()], []);
  first.emit({ type: 'ready', version: 'current' });
  await Promise.resolve();
  assert.deepEqual([...timers.values()].map(({ delay }) => delay), [WALL_TIME_LIMIT * 1000]);

  advanceClock();
  assert.equal(first.terminated, true);
  await assert.rejects(firstRun, /30-second wall limit/);

  const secondRun = python.run('pass', {});
  const second = workers[1];
  first.emit({ type: 'result', id: 1, result: { stale: true } });
  second.emit({ type: 'ready', version: 'current' });
  await Promise.resolve();
  second.emit({ type: 'result', id: 2, result: { duration: 0 } });
  assert.deepEqual(await secondRun, { duration: 0 });
  assert.equal(timers.size, 0);

  const thirdRun = python.run('pass', {});
  await Promise.resolve();
  second.emit({ type: 'failed', id: 3, message: 'worker failure' });
  await assert.rejects(thirdRun, /worker failure/);
  assert.equal(timers.size, 0);

  const fourthRun = python.run('while True: pass', {});
  await Promise.resolve();
  assert.equal(timers.size, 1);
  python.terminate();
  await assert.rejects(fourthRun, /stopped/);
  assert.equal(timers.size, 0);
});

it('rejects a run stopped during Python startup and ignores late readiness after restart', async () => {
  globalThis.Worker = FakeWorker;
  const statuses = [];
  const python = new PythonWorker({ onStatus: (status) => statuses.push(status) });
  const firstRun = python.run('pass', {});
  const first = workers[0];

  python.terminate();
  await assert.rejects(firstRun, /stopped/);
  const secondRun = python.run('pass', {});
  const second = workers[1];

  first.emit({ type: 'ready', version: 'stale' });
  second.emit({ type: 'ready', version: 'current' });
  await Promise.resolve();
  assert.equal(second.messages.some(({ type }) => type === 'run'), true);
  second.emit({ type: 'result', id: 1, result: { duration: 0 } });
  assert.deepEqual(await secondRun, { duration: 0 });
  assert.deepEqual(statuses.map(({ phase }) => phase), ['ready']);
});

it('does not send a cancelled ready run to a replacement worker', async () => {
  globalThis.Worker = FakeWorker;
  globalThis.setTimeout = () => 1;
  globalThis.clearTimeout = () => {};
  const python = new PythonWorker({ onStatus: () => {} });
  const firstReady = python.load();
  const first = workers[0];
  first.emit({ type: 'ready', version: 'current' });
  await firstReady;

  const cancelledRun = python.run('oldSource', {});
  python.terminate();
  const replacementReady = python.load();
  replacementReady.catch(() => {});
  await Promise.resolve();
  const replacement = workers[1];

  python.terminate();
  await assert.rejects(cancelledRun, /stopped/);
  await assert.rejects(replacementReady, /stopped/);
  assert.deepEqual(replacement.messages, [{ type: 'load' }]);
});
