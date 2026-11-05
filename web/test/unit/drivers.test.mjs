import test from 'node:test';
import assert from 'node:assert/strict';
import { runLive, runSync } from '../../src/api/drivers.js';

class Sim {
  time = 0;
  listeners = new Set();
  step(dt) { this.time += dt; }
  on(_event, listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }
  advance(seconds) {
    this.time += seconds;
    for (const listener of [...this.listeners]) listener();
  }
}

test('sync waits without a timeout past 120 simulated seconds', () => {
  const sim = new Sim();
  const result = runSync(sim, (function* () {
    const done = yield { until: () => sim.time >= 121, timeout: null };
    return done;
  }()));
  assert.equal(result, true);
  assert.ok(sim.time >= 121);
});

test('sync wait preserves finite and zero timeouts', () => {
  const sim = new Sim();
  const wait = timeout => runSync(sim, (function* () {
    return yield { until: () => false, timeout };
  }()));
  assert.equal(wait(0.2), false);
  assert.ok(sim.time >= 0.199);
  sim.time = 0;
  assert.equal(wait(0), false);
  assert.equal(sim.time, 0);
});

test('live wait without a timeout resolves after 120 simulated seconds', async () => {
  const sim = new Sim(), pending = new Set();
  const result = runLive({ sim, pending }, (function* () {
    const done = yield { until: () => sim.time >= 121, timeout: null };
    return done;
  }()));
  sim.advance(120);
  sim.advance(1);
  assert.equal(await result, true);
  assert.equal(pending.size, 0);
  assert.equal(sim.listeners.size, 0);
});

test('live timeout, interrupt, throwing predicate, and reentrant completion clean listeners', async () => {
  const makeSession = () => {
    const sim = new Sim(), pending = new Set();
    return { sim, pending };
  };

  const timed = makeSession();
  const timedResult = runLive(timed, (function* () {
    return yield { until: () => false, timeout: 1 };
  }()));
  timed.sim.advance(1);
  assert.equal(await timedResult, false);
  assert.equal(timed.pending.size, 0);
  assert.equal(timed.sim.listeners.size, 0);

  const zeroTimeout = makeSession();
  const zeroResult = runLive(zeroTimeout, (function* () {
    return yield { until: () => false, timeout: 0 };
  }()));
  assert.equal(await zeroResult, false);
  assert.equal(zeroTimeout.pending.size, 0);
  assert.equal(zeroTimeout.sim.listeners.size, 0);

  const interrupted = makeSession();
  const interruptedResult = runLive(interrupted, (function* () {
    return yield { until: () => false };
  }()));
  [...interrupted.pending][0]();
  assert.equal(await interruptedResult, false);
  assert.equal(interrupted.pending.size, 0);
  assert.equal(interrupted.sim.listeners.size, 0);

  const throwing = makeSession();
  const thrown = runLive(throwing, (function* () {
    yield { until: () => { throw new Error('predicate failed'); } };
  }()));
  await assert.rejects(thrown, /predicate failed/);
  assert.equal(throwing.pending.size, 0);
  assert.equal(throwing.sim.listeners.size, 0);

  const reentrant = makeSession();
  let ready = false;
  const reentrantResult = runLive(reentrant, (function* () {
    yield { until: () => ready };
    return 'finished';
  }()));
  ready = true;
  reentrant.sim.advance(0);
  assert.equal(await reentrantResult, 'finished');
  assert.equal(reentrant.pending.size, 0);
  assert.equal(reentrant.sim.listeners.size, 0);
});

test('interrupt during a state-event snapshot cannot complete a stale waiter twice', async () => {
  const sim = new Sim(), pending = new Set();
  let interrupted = false;
  sim.on('state', () => {
    if (!interrupted) {
      interrupted = true;
      [...pending][0]?.();
    }
  });
  const result = runLive({ sim, pending }, (function* () {
    yield { until: () => sim.time >= 1 };
    const completed = yield { until: () => false };
    return completed ? 'stale callback completed the next wait' : 'interrupted';
  }()));

  sim.advance(1);
  assert.equal(pending.size, 1, 'only the new wait remains pending');
  [...pending][0]();
  assert.equal(await result, 'interrupted');
  assert.equal(pending.size, 0);
  assert.equal(sim.listeners.size, 1, 'only the pre-existing event listener remains');
});
