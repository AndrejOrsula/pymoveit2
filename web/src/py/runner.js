// Runs a whole Python script against a private copy of the simulation, with
// Pyodide, and records what happened (robot frames, scene changes, calls,
// output, the line running at each moment) so the page can replay it.
// Shared by the Web Worker and the Node tests.

import { MoveGroupSim } from '../core/sim.js';
import { robotById } from '../core/robots.js';
import { createSession } from '../api/session.js';
import { bindArguments } from '../api/binder.js';
import { SIGNATURES } from '../api/signatures.js';
import { PyError } from '../api/errors.js';

/** Simulated seconds a script may run before it is stopped. */
export const SIM_TIME_LIMIT = 600;
/** Wall-clock seconds a script may run before it is stopped. */
export const WALL_TIME_LIMIT = 30;
const FRAME_INTERVAL = 1 / 30;
const TIMEOUT_MESSAGE = 'The demo stopped the script: it ran longer than the browser allows.';

const PRELUDE = `
import sys
import time as _time

import _pymoveit2_bridge as _bridge


_TOO_LONG = ${JSON.stringify(TIMEOUT_MESSAGE)}


def _sleep(seconds):
    seconds = float(seconds)
    if seconds != seconds:
        raise ValueError("Invalid value NaN (not a number)")
    if seconds < 0:
        raise ValueError("sleep length must be non-negative")
    # The bridge sleeps at most until the simulated-time limit.
    result = json.loads(_bridge.sleep(seconds))
    if "error" in result:
        error_type = getattr(sys.modules["builtins"], result["error"]["type"], RuntimeError)
        raise error_type(result["error"]["message"])
    if not result["value"]:
        raise TimeoutError(_TOO_LONG)


_time.sleep = _sleep
_time.monotonic = _bridge.now
_time.perf_counter = _bridge.now
_time.time = lambda: 1767225600.0 + _bridge.now()


def _tracer(frame, event, arg):
    if frame.f_code.co_filename != "<script>":
        return None
    if event == "line" and not _bridge.line(frame.f_lineno):
        raise TimeoutError(_TOO_LONG)
    return _tracer


def _run_script(source):
    code = compile(source, "<script>", "exec")
    sys.settrace(_tracer)
    try:
        exec(code, {"__name__": "__main__", "__file__": "<script>"})
    finally:
        sys.settrace(None)
`;

const RUN_AND_REPORT = `
import json
import traceback

_status = {"error": None, "exit": None}
try:
    _run_script(_SOURCE)
except SystemExit as exc:
    _status["exit"] = exc.code if isinstance(exc.code, int) else (0 if exc.code is None else 1)
    if exc.code not in (None, 0) and not isinstance(exc.code, int):
        print(exc.code)
except BaseException as exc:
    frames = [f for f in traceback.extract_tb(exc.__traceback__) if f.filename == "<script>"]
    line = frames[-1].lineno if frames else None
    # A script that does not compile has no frame of its own; the error names the line.
    if isinstance(exc, SyntaxError) and exc.filename == "<script>":
        line = exc.lineno
    _status["error"] = {
        "type": type(exc).__name__,
        "message": str(exc),
        "line": line,
        "traceback": "".join(traceback.format_exception_only(type(exc), exc)).strip(),
    }
json.dumps(_status)
`;

const toJson = (value) => JSON.stringify(value === undefined ? null : value);
// Python sends NaN and infinities as {"__float__": "nan" | "inf" | "-inf"}.
const NON_FINITE = { nan: NaN, inf: Infinity, '-inf': -Infinity };
const fromJson = (text) => JSON.parse(text, (key, value) => (value && typeof value === 'object' && typeof value.__float__ === 'string' ? NON_FINITE[value.__float__] : value));

export class ScriptRunner {
  /**
   * `pyodide` is a loaded Pyodide instance and `shim` the source of
   * pymoveit2_shim.py, which Python imports as `pymoveit2`.
   */
  constructor(pyodide, { shim, onProgress = () => {} }) {
    this.pyodide = pyodide;
    this.shim = shim;
    this.onProgress = onProgress;
    this.state = null;
    this.#install();
  }

  #install() {
    const py = this.pyodide;
    const bridge = {
      invoke: (path, args, kwargs) => this.#call(() => this.state.session.invoke(path, fromJson(args), fromJson(kwargs))),
      getattr: (path) => this.#call(() => this.state.session.getattr(path)),
      setattr: (path, value) => this.#call(() => this.state.session.setattr(path, fromJson(value))),
      members: (path) => toJson(this.state.session.members(path)),
      connect: (args, kwargs) => this.#call(() => this.state.connect(fromJson(args), fromJson(kwargs))),
      sleep: (seconds) => {
        const state = this.state;
        const left = SIM_TIME_LIMIT - (state.sim.getState().time - state.t0);
        return this.#call(() => {
          state.session.sleep(Math.max(0, Math.min(seconds, left)));
          return seconds <= left;
        });
      },
      now: () => this.state.sim.getState().time,
      line: (line) => this.#line(line),
    };
    py.registerJsModule('_pymoveit2_bridge', bridge);
    py.FS.mkdirTree('/home/pyodide/demo');
    py.FS.writeFile('/home/pyodide/demo/pymoveit2.py', this.shim);
    py.runPython("import sys\nsys.path.insert(0, '/home/pyodide/demo')");
    py.runPython(PRELUDE);
    py.setStdout({ batched: (text) => this.#output('out', text) });
    py.setStderr({ batched: (text) => this.#output('err', text) });
  }

  #call(fn) {
    if (this.state?.budgetExceeded) {
      return toJson({ error: { type: 'TimeoutError', message: TIMEOUT_MESSAGE } });
    }
    try {
      return toJson({ value: fn() });
    } catch (error) {
      if (error instanceof PyError || error?.name === 'PyError') {
        return toJson({ error: { type: error.type, message: error.message } });
      }
      return toJson({ error: { type: 'RuntimeError', message: `demo error: ${error?.message ?? error}` } });
    }
  }

  #line(line) {
    const state = this.state;
    state.line = line;
    state.lines.add(line);
    const wall = (performance.now() - state.wallStart) / 1000;
    if (wall - state.lastProgress > 0.1) {
      state.lastProgress = wall;
      this.onProgress({ line, time: state.sim.getState().time - state.t0 });
    }
    const withinBudget = wall < WALL_TIME_LIMIT && state.sim.getState().time - state.t0 < SIM_TIME_LIMIT;
    if (!withinBudget) {
      state.budgetExceeded = true;
      state.timeoutLine ??= line;
    }
    return withinBudget;
  }

  #checkBudget() {
    const state = this.state;
    if (!state) return;
    const wall = (performance.now() - state.wallStart) / 1000;
    const simulated = state.sim.getState().time - state.t0;
    if (wall >= WALL_TIME_LIMIT || simulated >= SIM_TIME_LIMIT) {
      state.budgetExceeded = true;
      state.timeoutLine ??= state.line;
      throw new PyError('TimeoutError', TIMEOUT_MESSAGE);
    }
  }

  #output(stream, text) {
    const state = this.state;
    if (!state) return;
    state.output.push({ t: state.sim.getState().time - state.t0, line: state.line, stream, text });
  }

  /**
   * Runs `source` starting from the page's sim `snapshot`. Returns everything
   * the page needs to replay the run: {frames, scenes, calls, logs, plans,
   * executions, output, lines, error, exit, duration, snapshot}.
   */
  run(source, snapshot) {
    const sim = new MoveGroupSim({ model: robotById(snapshot?.robot) });
    if (snapshot) sim.reset(snapshot);
    const t0 = sim.getState().time;
    const state = {
      sim,
      session: createSession(sim, { driver: 'sync' }),
      t0,
      line: null,
      lines: new Set(),
      wallStart: performance.now(),
      lastProgress: 0,
      frames: [],
      scenes: [],
      calls: [],
      logs: [],
      plans: [],
      executions: [],
      output: [],
      connect: () => null,
    };
    // Each connect() gives a new session with default parameters, as on a real robot.
    let offCalls = () => {};
    const recordCalls = (session) => session.on('call', (call) => {
      if (call.status !== 'started') state.calls.push({ ...call, t: time(), line: state.line });
    });
    state.connect = (args, kwargs) => {
      bindArguments('connect', SIGNATURES.connect, args, kwargs);
      offCalls();
      state.session = createSession(sim, { driver: 'sync' });
      offCalls = recordCalls(state.session);
      return null;
    };
    this.state = state;
    const time = () => sim.getState().time - t0;
    let lastFrame = -Infinity;
    const record = (s) => {
      if (s.time - t0 - lastFrame < FRAME_INTERVAL - 1e-9) return;
      lastFrame = s.time - t0;
      state.frames.push({ t: lastFrame, q: s.q.slice(), fingers: s.fingers, line: state.line, mode: s.mode });
    };
    record(sim.getState());
    const off = [
      sim.on('state', record),
      sim.on('state', () => this.#checkBudget()),
      sim.on('scene', ({ objects }) => state.scenes.push({ t: time(), objects: structuredClone(objects) })),
      sim.on('log', (entry) => state.logs.push({ ...entry, t: time() })),
      sim.on('plan', ({ request, result }) => state.plans.push({ t: time(), request, result: { ...result, debug: { ee_path: result.debug?.ee_path, tree: result.debug?.tree } } })),
      sim.on('execution', (event) => state.executions.push({ t: time(), ...event })),
      () => offCalls(),
    ];
    offCalls = recordCalls(state.session);
    this.pyodide.globals.set('_SOURCE', source);
    let status;
    try {
      status = JSON.parse(this.pyodide.runPython(RUN_AND_REPORT));
      if (state.budgetExceeded) {
        status.error = {
          type: 'TimeoutError',
          message: TIMEOUT_MESSAGE,
          line: state.timeoutLine,
          traceback: `TimeoutError: ${TIMEOUT_MESSAGE}`,
        };
        status.exit = null;
      }
    } finally {
      off.forEach((unsubscribe) => unsubscribe());
      this.state = null;
    }
    // Let motions still running when the script ended finish, as the robot would.
    let settle = 0;
    while (sim.isBusy() && settle < 60 * 120) {
      sim.step(1 / 120);
      settle += 1;
      record(sim.getState());
    }
    record(sim.getState());
    return {
      frames: state.frames,
      scenes: state.scenes,
      calls: state.calls,
      logs: state.logs,
      plans: state.plans,
      executions: state.executions,
      output: state.output,
      lines: [...state.lines],
      error: status.error,
      exit: status.exit,
      duration: sim.getState().time - t0,
      snapshot: sim.snapshot(),
    };
  }
}
