// The pymoveit2 playground: wires the simulated move_group, the pymoveit2
// stand-in, the 3D view and the side panels together.

import './style.css';
import { ROBOTS, robotFromSearch } from './core/robots.js';
import { MoveGroupSim } from './core/sim.js';
import { fk } from './core/kinematics.js';
import { rpyFromQuat } from './core/math.js';
import { createSession } from './api/session.js';
import { buildScript, pyRepr } from './api/codegen.js';
import { Viewport } from './view/viewport.js';
import { TrajectoryPlot, pointTime } from './ui/plot.js';
import { Tabs } from './ui/tabs.js';
import { LearnPanel } from './ui/learn.js';
import { ControlPanel } from './ui/control.js';
import { ConsolePanel } from './ui/console.js';
import { ScriptPanel } from './ui/script.js';
import { LogPanel } from './ui/log.js';
import { PythonConsole } from './console/python.js';
import { escapeHtml, highlightPython } from './ui/highlight.js';

// eslint-disable-next-line no-undef
const BUILD = __BUILD_INFO__;
const DEMO_ID = 'pymoveit2';

function notifyHost(type, extra = {}) {
  if (window.parent === window) return;
  window.parent.postMessage({ type, protocol: 1, id: DEMO_ID, ...extra }, location.origin);
}

const $ = (selector) => document.querySelector(selector);
const fmt = (value, digits = 3) => (Object.is(Number(value.toFixed(digits)), -0) ? 0 : value).toFixed(digits);

const rpyDegrees = (quat) => rpyFromQuat(quat).map((a) => (a * 180) / Math.PI);

class App extends EventTarget {
  constructor() {
    super();
    this.model = robotFromSearch(location.search);
    this.sim = new MoveGroupSim({ model: this.model });
    this.objects = [];
    this.selectedObject = null;
    this.recorder = [];
    this.replaying = false;
    this.busy = new Set();
    this.override = null;
    this.overrides = new Map();
    // The running line of each Python console, by call id, while its calls run.
    this.caller = null;
    this.callers = new Map();
    this.execution = null;
    this.planGhost = null;
    this.executedPath = [];
    this.goalVisible = true;
    this.#wireSim();
    this.#newSession();
    this.goal = this.sim.getState().ee;
  }

  // --------------------------------------------------------------- session

  #newSession() {
    this.sessionOff?.forEach((off) => off());
    this.session = createSession(this.sim, { driver: 'live' });
    this.sessionOff = [
      this.session.on('call', (call) => this.#onCall(call)),
      this.session.on('ros', (ros) => this.#onRos(ros)),
      this.session.on('hint', (hint) => {
        this.toast(hint.text, 6000);
        this.dispatchEvent(new CustomEvent('hint', { detail: hint }));
      }),
    ];
    this.dispatchEvent(new CustomEvent('session'));
  }

  /** Calls a pymoveit2 method; errors become a toast and an undefined result. */
  #takeOver() {
    if (this.replaying) this.stopReplay();
  }

  async call(path, args = [], kwargs = {}, options = {}) {
    this.#takeOver();
    this.override = { ...options, path };
    try {
      return await this.session.invoke(path, args, kwargs);
    } catch (error) {
      this.#reportError(error);
      return undefined;
    } finally {
      this.override = null;
    }
  }

  /** Sets a pymoveit2 attribute, such as robot.arm.max_velocity. */
  async set(path, value) {
    this.#takeOver();
    try {
      await this.#setattr(path, value);
    } catch (error) {
      this.#reportError(error);
    }
  }

  async #setattr(path, value, line = null) {
    const before = this.lastCallId;
    await this.session.setattr(path, value);
    if (this.lastCallId === before) {
      const python = `${path} = ${pyRepr(value)}`;
      this.#showLastCall(python, []);
      if (line) line.push({ python });
      else this.#record(python);
    }
    this.dispatchEvent(new CustomEvent('settings'));
  }

  /**
   * A session for one of the page's Python consoles. Anything that acts on
   * the robot first ends a script replay, whose paused simulation would
   * otherwise hold the action until the replay ends. The session script gets
   * each line as typed; a line that raises leaves only the calls that worked.
   */
  pythonSession() {
    let line = null;
    // The session announces a call before its first await, so the call is the line's.
    const tagged = (run) => {
      this.caller = line;
      try {
        return run();
      } finally {
        this.caller = null;
      }
    };
    return {
      invoke: (...args) => (this.#takeOver(), tagged(() => this.session.invoke(...args))),
      getattr: (...args) => this.session.getattr(...args),
      setattr: (path, value) => (this.#takeOver(), tagged(() => this.#setattr(path, value, line))),
      members: (...args) => this.session.members(...args),
      sleep: (...args) => (this.#takeOver(), tagged(() => this.session.sleep(...args))),
      interrupt: (...args) => this.session.interrupt(...args),
      statement: async (source, run, { keep }) => {
        const calls = (line = []);
        let ok = false;
        try {
          await run();
          ok = true;
        } finally {
          line = null;
          if (ok && (keep || calls.length)) this.#record(source);
          else for (const { python, repeat } of calls) this.#record(python, repeat);
        }
      },
    };
  }

  /** Runs console-style Python lines (used by scene presets). */
  async runPython(source) {
    const python = new PythonConsole(this.pythonSession(), { write: (kind, text) => kind === 'err' && this.toast(text) });
    try {
      await python.run(source);
    } catch (error) {
      this.#reportError(error);
    }
  }

  #reportError(error) {
    if (error?.type) this.toast(`${error.type}: ${error.message}`, 6000);
    else {
      console.error(error);
      this.toast(`Internal error: ${error?.message ?? error}. Reload the page if the scene stops responding.`, 6000);
    }
  }

  interrupt() {
    this.session.interrupt();
    if (this.sim.isBusy('arm')) this.sim.cancel('arm');
  }

  #onCall(call) {
    if (call.status === 'started') {
      if (this.override && !this.overrides.has(call.id) && call.path === this.override.path) this.overrides.set(call.id, this.override);
      if (this.caller) this.callers.set(call.id, this.caller);
      return;
    }
    const override = this.overrides.get(call.id) ?? {};
    this.overrides.delete(call.id);
    const line = this.callers.get(call.id);
    this.callers.delete(call.id);
    this.lastCallId = call.id;
    let python = override.display ?? call.python;
    if (override.assign) python = `${override.assign} = ${python}`;
    const ok = call.status === 'returned' && call.result !== false;
    if (!override.quiet || !ok) {
      this.#showLastCall(python, call.ros ?? [], call.status === 'raised' ? `${call.error?.type}: ${call.error?.message}` : pyRepr(call.result ?? null), ok);
    }
    // A call that raised changed nothing, and would stop the session script.
    if (call.status !== 'raised') {
      if (line) line.push({ python, repeat: override.repeat });
      else this.#record(python, override.repeat);
    }
    this.dispatchEvent(new CustomEvent('call', { detail: { ...call, python } }));
  }

  #onRos(ros) {
    this.dispatchEvent(new CustomEvent('ros', { detail: ros }));
  }

  #showLastCall(python, ros, result = null, ok = true) {
    const code = $('#last-call-code');
    const shown = result && result !== 'None' && result.length < 90 ? `${highlightPython(python)}  <span class="tok-comment"># ${escapeHtml(result)}</span>` : highlightPython(python);
    code.innerHTML = `<code>${shown}</code>`;
    code.dataset.ok = String(ok);
    this.lastPython = python;
    $('#last-call-ros').innerHTML = ros
      .slice(0, 6)
      .map((call) => {
        const status = call.status ? `<span class="result" data-ok="${!/FAIL|ABORT|CANCEL|INVALID|NO_|COLLISION|TIMED/.test(call.status)}">${escapeHtml(call.status)}</span>` : '';
        return `<li title="${escapeHtml(`${call.type}: ${call.summary ?? ''}`)}"><span class="kind">${escapeHtml(call.kind)}</span><span class="name">${escapeHtml(call.name)}</span>${status}</li>`;
      })
      .join('');
  }

  #record(python, repeat) {
    if (this.replaying) return;
    const last = this.recorder.at(-1);
    // Setting the same attribute again, as a slider does, keeps only the last value.
    const attribute = (line) => /^(\w+(?:\.\w+)+) = /.exec(line ?? '')?.[1];
    if (repeat && last?.python === python) last.count += 1;
    else if (!repeat && last?.count === 1 && attribute(python) && attribute(python) === attribute(last.python)) last.python = python;
    else this.recorder.push({ python, count: 1, repeat });
    if (this.recorder.length > 300) this.recorder.shift();
    this.dispatchEvent(new CustomEvent('recorded'));
  }

  /** The calls made so far as a runnable script, or null when there are none. */
  sessionScript() {
    if (!this.recorder.length) return null;
    const statements = [];
    for (const { python, count, repeat } of this.recorder) {
      if (count > 1 && repeat) {
        statements.push(`for _ in range(${count}):\n    ${python}\n    time.sleep(${repeat})`);
      } else statements.push(python);
    }
    return buildScript(statements);
  }

  // ------------------------------------------------------------------- sim

  #wireSim() {
    this.sim.on('state', (state) => this.#onState(state));
    this.sim.on('scene', ({ objects }) => this.#onScene(objects));
    this.sim.on('plan', ({ request, result }) => this.#onPlan(request, result));
    this.sim.on('execution', (event) => this.#onExecution(event));
    this.sim.on('log', (entry) => this.dispatchEvent(new CustomEvent('log', { detail: entry })));
  }

  #onState(state) {
    if (this.replaying) return;
    this.#showState(state);
    this.dispatchEvent(new CustomEvent('state', { detail: state }));
  }

  #showState(state, mode = state.mode) {
    this.viewport?.setRobotState(state.q, state.fingers);
    this.viewport?.setContacts(state.contacts ?? []);
    const chip = $('#state-chip');
    const label = this.replaying ? 'REPLAY' : mode.toUpperCase();
    if (chip.textContent !== label) {
      chip.textContent = label;
      chip.dataset.state = this.replaying ? 'executing' : mode;
    }
    if (this.execution && mode === 'executing') {
      const t = state.time - this.execution.start;
      this.plot.setPlayhead(t);
      if (this.executedPath.length < 2000) {
        this.executedPath.push(state.ee.position.slice());
        if (this.executedPath.length % 3 === 0) this.viewport?.setPath(this.executedPath, 'executed');
      }
    }
    const now = performance.now();
    if (now - (this.lastReadout ?? 0) > 100) {
      this.lastReadout = now;
      const p = state.ee.position;
      const [roll, pitch, yaw] = rpyDegrees(state.ee.quat_xyzw);
      $('#ee-readout').innerHTML = `<span class="muted">${this.model.end_effector_name}</span> x ${fmt(p[0])} y ${fmt(p[1])} z ${fmt(p[2])} m<br><span class="muted">rpy</span> ${fmt(roll, 0)}° ${fmt(pitch, 0)}° ${fmt(yaw, 0)}°`;
    }
  }

  #onScene(objects) {
    this.objects = objects;
    this.viewport?.setObjects(objects);
    if (this.selectedObject && !objects.some((object) => object.id === this.selectedObject)) this.selectObject(null);
    if (!this.replaying) this.dispatchEvent(new CustomEvent('scene', { detail: { objects } }));
  }

  #onPlan(request, result) {
    const planner = request.pipeline_id === 'pilz_industrial_motion_planner' ? `Pilz · ${request.planner_id || 'PTP'}` : request.cartesian ? 'Cartesian path' : `OMPL · ${(request.planner_id || 'RRTConnect').replace('kConfigDefault', '')}`;
    $('#planner-chip').textContent = planner;
    const debug = result.debug ?? {};
    this.viewport?.setTree(debug.tree ?? []);
    this.viewport?.setPath(debug.ee_path ?? [], 'plan');
    this.viewport?.setPath([], 'executed');
    const ok = result.error_code === 'SUCCESS';
    // Show the target of every pose goal, wherever it came from, as the goal marker.
    const goal = request.pose_goal;
    const { end_effector_name: ee, planning_frame: frame } = this.model;
    if (!request.cartesian && goal?.position && goal.quat_xyzw && (goal.target_link ?? ee) === ee && (goal.frame_id ?? frame) === frame) {
      this.setGoal(goal, 'plan');
    }
    this.planGhost = debug.goal_state ? { q: debug.goal_state, tone: ok ? 'goal' : 'bad' } : null;
    if (debug.goal_state) this.viewport?.setGhost(debug.goal_state, undefined, ok ? 'goal' : 'bad');
    else if (!ok) this.viewport?.setGhost(null);
    if (result.trajectory?.points?.length) {
      this.plot.setTrajectory(result.trajectory, this.model.joints);
      const duration = pointTime(result.trajectory.points.at(-1));
      const fraction = request.cartesian ? ` · ${Math.round((result.fraction ?? 1) * 100)}% of the line` : '';
      this.#closeReplay();
      $('#timeline-title').textContent = 'Planned trajectory';
      $('#timeline-meta').textContent = `${duration.toFixed(2)} s · ${result.trajectory.points.length} points · planned in ${(result.planning_time ?? 0).toFixed(3)} s${fraction}`;
    } else {
      $('#timeline-meta').textContent = `${result.error_code}${result.detail ? `: ${result.detail}` : ''}`;
    }
  }

  #onExecution(event) {
    if (event.target !== 'arm' || this.replaying) return;
    if (event.status === 'started') {
      this.execution = { id: event.id, start: this.sim.getState().time };
      this.executedPath = [];
      if (event.trajectory?.points?.length) {
        this.#closeReplay();
        this.plot.setTrajectory(event.trajectory, this.model.joints);
        $('#timeline-title').textContent = 'Executing';
      }
    } else {
      this.execution = null;
      $('#timeline-title').textContent = event.status === 'succeeded' ? 'Executed trajectory' : `Execution ${event.status}`;
      if (event.status === 'succeeded') {
        this.planGhost = null;
        this.viewport?.setGhost(null);
      }
      this.plot.setPlayhead(null);
    }
  }

  // ------------------------------------------------------------------ goal

  /** pymoveit2 reads poses in base_link_name; the scene shows the planning frame. */
  frameKwargs() {
    const { base_link_name, planning_frame } = this.model;
    return base_link_name === planning_frame ? {} : { frame_id: planning_frame };
  }

  setGoal(pose, source) {
    this.goal = { position: pose.position.slice(), quat_xyzw: pose.quat_xyzw.slice() };
    if (source !== 'viewport') this.viewport?.setGoalMarker(this.goal);
    this.dispatchEvent(new CustomEvent('goal', { detail: { pose: this.goal, source } }));
    this.#scheduleGoalPreview();
  }

  setGoalVisible(visible) {
    this.goalVisible = visible;
    this.viewport?.setGoalMarker(visible ? this.goal : null);
    if (visible) this.#scheduleGoalPreview();
    else this.viewport?.setGoalLabel(null);
  }

  #scheduleGoalPreview() {
    if (this.previewPending || !this.goalVisible) return;
    this.previewPending = true;
    requestAnimationFrame(() => {
      this.previewPending = false;
      this.#goalPreview();
    });
  }

  #goalPreview() {
    if (!this.viewport || this.replaying) return;
    let q = null;
    try {
      q = this.sim.computeIK({ position: this.goal.position, quat_xyzw: this.goal.quat_xyzw }, { avoid_collisions: true, preview: true });
    } catch {
      q = null;
    }
    if (q && !Array.isArray(q)) q = q.position ?? q.q ?? null;
    if (q) {
      this.viewport.setGhost(q.slice(0, this.model.joints.length), undefined, 'preview');
      this.viewport.setGoalLabel(null);
    } else {
      this.viewport.setGhost(null);
      this.viewport.setGoalLabel('No IK solution here', 'bad');
    }
  }

  previewJoints(q) {
    this.viewport?.setGhost(q, undefined, 'preview');
  }

  selectObject(id) {
    this.selectedObject = id;
    this.viewport?.select(id ?? (this.goalVisible ? 'goal' : null));
    this.dispatchEvent(new CustomEvent('select', { detail: { id } }));
  }

  setLayer(name, visible) {
    const button = document.querySelector(`.hud-toggle[data-layer="${name}"]`);
    button?.setAttribute('aria-pressed', String(visible));
    this.viewport?.setLayer(name, visible);
  }

  // ------------------------------------------------------------ world reset

  /** Back to the default state with an empty scene and a new connection (default parameters). */
  async resetWorld() {
    if (this.replaying) this.stopReplay();
    this.planGhost = null;
    this.session.interrupt();
    this.sim.reset();
    this.#newSession();
    this.setGoal(this.sim.getState().ee, 'reset');
    this.viewport?.setPath([], 'plan');
    this.viewport?.setPath([], 'executed');
    this.viewport?.setTree([]);
    this.viewport?.setGhost(null);
    this.plot.setTrajectory(null);
    this.#closeReplay();
    $('#timeline-meta').textContent = '';
    $('#timeline-title').textContent = 'Trajectory';
    this.#onScene(this.sim.getObjects());
    this.dispatchEvent(new CustomEvent('state', { detail: this.sim.getState() }));
  }

  setBusy(name, busy) {
    if (busy) this.busy.add(name);
    else this.busy.delete(name);
  }

  /** Resolves once no motion is running; a caller can cancel its wait. */
  async whenIdle({ shouldStop = () => false } = {}) {
    while (this.sim.isBusy()) {
      if (shouldStop()) throw new Error('stopped');
      await new Promise((resolve) => setTimeout(resolve, 50));
    }
    if (shouldStop()) throw new Error('stopped');
  }

  // ---------------------------------------------------------------- replay

  /**
   * Replays a script run recorded by the Python worker, then adopts its final
   * state. Resolves to "finished" or "stopped". Once it ends, dragging the
   * slider replays it again from there: `from` is that time, and such a replay
   * prints and logs nothing twice and leaves the simulation as it is.
   */
  replay(result, { onLine = () => {}, onOutput = () => {}, from = null } = {}) {
    this.stopReplay();
    this.replaying = true;
    const initialObjects = this.objects;
    const duration = Math.max(result.duration, result.frames.at(-1)?.t ?? 0);
    const scrubber = $('#scrubber');
    scrubber.hidden = false;
    scrubber.max = String(Math.round(duration * 1000));
    scrubber.value = '0';
    $('#timeline-title').textContent = 'Script replay';
    $('#timeline-meta').textContent = `${duration.toFixed(1)} s simulated · ${result.calls.length} calls`;
    this.viewport?.setGoalMarker(null);
    this.viewport?.setGhost(null);
    const again = from !== null;
    let printed = again ? result.output.length : 0;
    let logged = again ? { calls: result.calls.length, logs: result.logs.length } : { calls: 0, logs: 0 };
    const state = { t: 0, playing: !again, last: performance.now() };
    const apply = (t, forward) => {
      state.t = Math.max(0, Math.min(duration, t));
      const frame = frameAt(result.frames, state.t);
      if (frame) {
        this.viewport?.setRobotState(frame.q, frame.fingers);
        onLine(frame.line);
        this.#showState({ ...this.sim.getState(), q: frame.q, fingers: frame.fingers, mode: 'executing', ee: fk(this.model, frame.q), contacts: [] }, 'executing');
      }
      const scene = latest(result.scenes, state.t);
      this.viewport?.setObjects(scene ? scene.objects : initialObjects);
      const plan = latest(result.plans, state.t);
      if (plan) {
        this.viewport?.setPath(plan.result.debug?.ee_path ?? [], 'plan');
        this.viewport?.setTree(plan.result.debug?.tree ?? []);
      }
      const execution = latest(result.executions.filter((e) => e.target === 'arm' && e.status === 'started'), state.t);
      if (execution && execution !== state.execution) {
        state.execution = execution;
        this.plot.setTrajectory(execution.trajectory, this.model.joints);
      }
      if (state.execution) this.plot.setPlayhead(state.t - state.execution.t);
      scrubber.value = String(Math.round(state.t * 1000));
      if (forward) {
        while (printed < result.output.length && result.output[printed].t <= state.t + 1e-9) onOutput(result.output[printed++]);
        while (logged.calls < result.calls.length && result.calls[logged.calls].t <= state.t + 1e-9) {
          const call = result.calls[logged.calls++];
          this.#showLastCall(call.python, call.ros ?? [], call.status === 'raised' ? `${call.error?.type}: ${call.error?.message}` : pyRepr(call.result ?? null), call.status === 'returned');
          for (const ros of call.ros ?? []) this.dispatchEvent(new CustomEvent('ros', { detail: ros }));
        }
        while (logged.logs < result.logs.length && result.logs[logged.logs].t <= state.t + 1e-9) {
          this.dispatchEvent(new CustomEvent('log', { detail: result.logs[logged.logs++] }));
        }
      }
    };
    const finish = (how) => {
      if (!this.replaying) return;
      cancelAnimationFrame(state.raf);
      while (printed < result.output.length) onOutput(result.output[printed++]);
      this.replaying = false;
      this.replayState = null;
      if (!again) this.sim.reset(result.snapshot);
      this.#onScene(this.sim.getObjects());
      this.#showState(this.sim.getState());
      this.setGoal(again ? this.goal : this.sim.getState().ee, 'reset');
      if (!this.goalVisible) this.viewport?.setGoalMarker(null);
      onLine(null);
      this.dispatchEvent(new CustomEvent('state', { detail: this.sim.getState() }));
      scrubber.oninput = () => this.replay(result, { onLine, from: Number(scrubber.value) / 1000 });
      scrubber.onchange = null;
      state.resolve(how);
    };
    // Dragging pauses on the frame under the thumb; letting go plays on from there.
    scrubber.oninput = () => {
      state.playing = false;
      apply(Number(scrubber.value) / 1000, false);
    };
    scrubber.onchange = () => {
      state.playing = true;
    };
    const tick = (now) => {
      if (!this.replaying) return;
      if (state.playing) {
        apply(state.t + Math.min(0.1, (now - state.last) / 1000), true);
        if (state.t >= duration) {
          finish('finished');
          return;
        }
      }
      state.last = now;
      state.raf = requestAnimationFrame(tick);
    };
    this.replayState = { finish };
    return new Promise((resolve) => {
      state.resolve = resolve;
      apply(from ?? 0, !again);
      state.raf = requestAnimationFrame(tick);
    });
  }

  stopReplay() {
    this.replayState?.finish('stopped');
  }

  /** A new plan, motion or reset replaces the timeline, so the last run can no longer be replayed. */
  #closeReplay() {
    const scrubber = $('#scrubber');
    scrubber.hidden = true;
    scrubber.oninput = null;
    scrubber.onchange = null;
  }

  // -------------------------------------------------------------------- ui

  toast(text, ms = 2500) {
    let toast = $('.toast');
    if (!toast) {
      toast = document.createElement('div');
      toast.className = 'toast';
      toast.setAttribute('role', 'status');
      document.body.append(toast);
    }
    toast.textContent = text;
    toast.hidden = false;
    clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => {
      toast.hidden = true;
    }, ms);
  }

  showTab(name) {
    this.tabs.select(name);
  }

  /** Keeps the address bar a shareable link to the open tab, lesson or example. */
  updateLink() {
    if (!this.tabs || !this.learnPanel || !this.scriptPanel) return;
    const tab = document.querySelector('[role="tab"][aria-selected="true"]')?.id.slice('tab-'.length) ?? 'learn';
    const url = new URL(location.href);
    for (const key of LINK_KEYS) url.searchParams.delete(key);
    if (tab === 'learn' && this.learnPanel.lesson) url.searchParams.set('lesson', this.learnPanel.lesson.id);
    else if (tab === 'script' && this.scriptPanel.examples.some((example) => example.id === this.scriptPanel.choice)) url.searchParams.set('example', this.scriptPanel.choice);
    else if (tab !== 'learn') url.searchParams.set('tab', tab);
    if (url.href !== location.href) history.replaceState(history.state, '', url);
  }

  /** Opens what a link names: ?lesson=pick, ?example=square or ?tab=console. */
  openLink() {
    const params = new URLSearchParams(location.search);
    const lesson = params.get('lesson');
    const example = params.get('example');
    const tab = params.get('tab');
    if (this.learnPanel.lessons.some((entry) => entry.id === lesson)) this.learnPanel.open(lesson);
    else if (this.scriptPanel.examples.some((entry) => entry.id === example)) this.openScript(example);
    else if (['control', 'console', 'script', 'log'].includes(tab)) this.showTab(tab);
  }

  openConsole(text) {
    this.showTab('console');
    this.consolePanel.prefill(text.split('\n'));
  }

  openScript(id) {
    this.showTab('script');
    this.scriptPanel.select(id);
  }
}

const LINK_KEYS = ['lesson', 'example', 'tab'];

function frameAt(frames, t) {
  if (!frames.length) return null;
  let lo = 0;
  let hi = frames.length - 1;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (frames[mid].t <= t) lo = mid;
    else hi = mid - 1;
  }
  const a = frames[lo];
  const b = frames[lo + 1];
  if (!b || b.t <= a.t) return a;
  const s = (t - a.t) / (b.t - a.t);
  return { ...a, q: a.q.map((v, i) => v + (b.q[i] - v) * s), fingers: a.fingers + (b.fingers - a.fingers) * s };
}

function latest(entries, t) {
  let found = null;
  for (const entry of entries) {
    if (entry.t <= t + 1e-9) found = entry;
    else break;
  }
  return found;
}

// ---------------------------------------------------------------- startup

function setStatus(state, text) {
  $('#status').dataset.state = state;
  $('#status-text').textContent = text;
}

function fail(error) {
  console.error(error);
  setStatus('error', 'The demo failed to start.');
  const fault = $('#demo-fault');
  fault.hidden = false;
  fault.textContent = 'The demo failed to start. Reload the page, or try another browser.';
  notifyHost('wasm-demo-error', { build: BUILD.commit, message: String(error?.message ?? error) });
}

async function start() {
  const app = new App();
  window.pymoveit2Demo = app;
  app.plot = new TrajectoryPlot($('#plot'));
  const picker = $('#robot');
  picker.innerHTML = Object.values(ROBOTS).map((robot) => `<option value="${robot.id}">${escapeHtml(robot.label)}</option>`).join('');
  picker.value = app.model.id;
  picker.addEventListener('change', () => {
    const url = new URL(location.href);
    // Lessons and examples differ per robot, so the new page starts at its overview.
    for (const key of LINK_KEYS) url.searchParams.delete(key);
    url.searchParams.set('robot', picker.value);
    location.assign(url);
  });
  $('#scene').setAttribute('aria-label', `The ${app.model.label} arm in a 3D scene; drag to orbit`);
  $('#last-call-code code').textContent = `with connect() as robot:  # in the SRDF's "${app.model.default_state}" state`;

  const canvas = $('#scene');
  canvas.tabIndex = 0;
  if (Viewport.supported()) {
    app.viewport = new Viewport(canvas, $('#viewport'), { model: app.model, assetBase: 'assets/' });
    try {
      await app.viewport.load();
    } catch (error) {
      fail(error);
      return;
    }
    app.viewport.addEventListener('goal-change', (event) => app.setGoal(event.detail, 'viewport'));
    app.viewport.addEventListener('goal-commit', (event) => app.setGoal(event.detail, 'viewport'));
    app.viewport.addEventListener('object-moved', (event) => {
      const { id, pose } = event.detail;
      app.call('robot.move_collision', [id, pose.position.map((v) => Number(v.toFixed(4))), pose.quat_xyzw.map((v) => Number(v.toFixed(4)))], app.frameKwargs());
    });
    app.viewport.addEventListener('select', (event) => {
      const { kind, id } = event.detail;
      if (kind === 'object') app.selectObject(id);
      else app.selectObject(null);
    });
    app.viewport.addEventListener('contextlost', () => {
      const message = $('#viewport-message');
      message.hidden = false;
      message.textContent = 'The 3D view stopped. Reload the page to restore it.';
    });
    document.addEventListener('demo-theme', () => {
      app.viewport.applyTheme();
      app.plot.draw();
    });
  } else {
    const message = $('#viewport-message');
    message.hidden = false;
    message.textContent = 'WebGL is off in this browser, so there is no 3D view. Lessons, console and scripts still work.';
    // Their buttons only act on the 3D view.
    for (const hud of document.querySelectorAll('.hud-top-right, .hud-bottom-right')) hud.hidden = true;
  }

  app.tabs = new Tabs($('.tabs'));
  app.learnPanel = new LearnPanel($('#panel-learn'), app);
  app.controlPanel = new ControlPanel($('#panel-control'), app);
  app.consolePanel = new ConsolePanel($('#panel-console'), app);
  app.scriptPanel = new ScriptPanel($('#panel-script'), app);
  app.logPanel = new LogPanel($('#panel-log'), app);
  app.tabs.addEventListener('change', (event) => {
    // On a touch screen focus would open the keyboard over the console.
    if (event.detail.name === 'console' && !event.detail.keys && !matchMedia('(pointer: coarse)').matches) app.consolePanel.focus();
    app.updateLink();
  });

  // Pointing at the trajectory plot previews the robot at that moment, like
  // the trajectory slider of RViz's MotionPlanning panel.
  const plotCanvas = $('#plot');
  plotCanvas.title = 'Point at the plot to see the robot at that moment';
  const clearPreview = () => {
    if (app.plot.hover === null) return;
    app.plot.setHover(null);
    app.viewport?.setGhost(app.planGhost?.q ?? null, undefined, app.planGhost?.tone ?? 'goal');
  };
  const preview = (event) => {
    const t = app.replaying || app.execution ? null : app.plot.timeAt(event.clientX);
    if (t === null) return clearPreview();
    app.plot.setHover(t);
    app.viewport?.setGhost(app.plot.positionsAt(t), undefined, 'preview');
  };
  plotCanvas.addEventListener('pointerdown', preview);
  plotCanvas.addEventListener('pointermove', preview);
  // A lifted finger leaves its preview in place; touching anything else clears it.
  plotCanvas.addEventListener('pointerleave', (event) => {
    if (event.pointerType !== 'touch') clearPreview();
  });
  document.addEventListener('pointerdown', (event) => {
    if (event.target !== plotCanvas) clearPreview();
  }, true);

  // HUD.
  for (const button of document.querySelectorAll('.hud-toggle')) {
    app.viewport?.setLayer(button.dataset.layer, button.getAttribute('aria-pressed') === 'true');
    button.addEventListener('click', () => app.setLayer(button.dataset.layer, button.getAttribute('aria-pressed') !== 'true'));
  }
  const gizmo = $('#gizmo-mode');
  const toggleGizmo = () => {
    const rotate = app.viewport?.gizmoMode !== 'rotate';
    app.viewport?.setGizmoMode(rotate ? 'rotate' : 'translate');
    gizmo.textContent = rotate ? 'Turn' : 'Move';
  };
  gizmo.addEventListener('click', toggleGizmo);
  canvas.addEventListener('keydown', (event) => {
    if (event.key === 'r' && !event.ctrlKey && !event.metaKey) toggleGizmo();
  });
  $('#view-reset').addEventListener('click', () => app.viewport?.resetView());
  $('#last-call-copy').addEventListener('click', () => navigator.clipboard?.writeText(app.lastPython ?? '').then(() => app.toast('Copied'), () => {}));
  for (const button of document.querySelectorAll('[data-plot]')) {
    button.addEventListener('click', () => {
      for (const other of document.querySelectorAll('[data-plot]')) other.setAttribute('aria-pressed', String(other === button));
      app.plot.setSeries(button.dataset.plot);
    });
  }
  new ResizeObserver(() => {
    app.viewport?.resize();
    app.plot.draw();
  }).observe($('#viewport'));
  new ResizeObserver(() => app.plot.draw()).observe($('#plot'));

  // First state.
  const initial = app.sim.getState();
  app.viewport?.setRobotState(initial.q, initial.fingers);
  app.setGoal(initial.ee, 'reset');
  app.selectObject(null);
  app.controlPanel.syncSettings();
  app.plot.draw();
  app.dispatchEvent(new CustomEvent('state', { detail: initial }));

  let last = performance.now();
  const frame = (now) => {
    const dt = Math.min(0.05, Math.max(0, (now - last) / 1000));
    last = now;
    if (!app.replaying) app.sim.step(dt);
    app.viewport?.render();
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
  app.openLink();

  setStatus('ready', 'Ready · simulated MoveIt 2');
  document.body.dataset.ready = 'true';
  notifyHost('wasm-demo-ready', { build: BUILD.commit });
}

window.addEventListener('unhandledrejection', (event) => {
  if (document.body.dataset.ready !== 'true') fail(event.reason);
});

start().catch(fail);
