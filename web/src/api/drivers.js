// Execute the same method generators either synchronously or on simulation ticks.
import { PyError } from './errors.js';

/** Resolve generator waits by advancing simulation time. */
export function runSync(sim, generator) {
  let next = generator.next();
  while (!next.done) {
    const wait = next.value, deadline = wait.timeout == null ? Infinity : sim.time + wait.timeout;
    while (!wait.until() && sim.time < deadline - 1e-10) sim.step(Math.min(1 / 120, deadline - sim.time));
    next = generator.next(wait.until());
  }
  return next.value;
}
/** Resolve generator waits from state events; interrupt resumes each waiter as timed out. */
export function runLive(session, generator) {
  return new Promise((resolve, reject) => {
    function advance(input) {
      let next;
      try { next = generator.next(input); } catch (error) { reject(error); return; }
      if (next.done) { resolve(next.value); return; }
      const wait = next.value, deadline = wait.timeout == null ? Infinity : session.sim.time + wait.timeout;
      let settled = false;
      let unsubscribe = () => {};
      const cleanup = () => { unsubscribe(); session.pending.delete(interrupt); };
      const finish = success => {
        if (settled) return;
        settled = true;
        cleanup();
        advance(success);
      };
      const interrupt = () => finish(false);
      const tick = () => {
        if (settled) return;
        try {
          if (wait.until()) finish(true);
          else if (session.sim.time >= deadline - 1e-10) finish(false);
        } catch (error) {
          if (settled) return;
          settled = true;
          cleanup();
          reject(error);
        }
      };
      session.pending.add(interrupt); unsubscribe = session.sim.on('state', tick); tick();
    }
    advance();
  });
}

/** Validate the driver selector before creating a session. */
export function checkDriver(driver) { if (!['sync', 'live'].includes(driver)) throw new PyError('ValueError', `Unknown session driver '${driver}'.`); }
