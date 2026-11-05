// The Control tab: a MoveIt-commander-like panel. Every button calls the same
// pymoveit2 API the lessons teach, so the "Last call" strip shows its Python.

import { poseInLink } from '../core/kinematics.js';
import { quatFromRpy, rpyFromQuat } from '../core/math.js';
import { pyRepr } from '../api/codegen.js';
import { escapeHtml } from './highlight.js';

const DEG = 180 / Math.PI;
const PLANNERS = [
  ['ompl', 'RRTConnectkConfigDefault', 'OMPL · RRTConnect'],
  ['ompl', 'RRTstarkConfigDefault', 'OMPL · RRT*'],
  ['pilz_industrial_motion_planner', 'PTP', 'Pilz · PTP'],
  ['pilz_industrial_motion_planner', 'LIN', 'Pilz · LIN'],
];
// Shapes sized for the Panda; `s` scales them to the robot.
const SHAPES = {
  box: { label: 'Box', call: (id, p, s) => ['robot.add_collision_box', [id, [0.1 * s, 0.1 * s, 0.1 * s]], p] },
  sphere: { label: 'Sphere', call: (id, p, s) => ['robot.add_collision_sphere', [id, 0.06 * s], p] },
  cylinder: { label: 'Cylinder', call: (id, p, s) => ['robot.add_collision_cylinder', [id, 0.2 * s, 0.04 * s], p] },
  cone: { label: 'Cone', call: (id, p, s) => ['robot.add_collision_cone', [id, 0.15 * s, 0.06 * s], p] },
  mesh: { label: 'Suzanne', call: (id, p, s) => ['robot.add_collision_mesh', ['examples/assets/suzanne.stl', id], { ...p, scale: 0.12 * s }] },
};
// Where new objects appear (a row along `step`), their scale, and the
// orientation that keeps the gripper level, per robot.
const PLACES = {
  panda: { spot: [0.55, -0.18, 0.3], step: [0, 0.12, 0], scale: 1, level: { quat: [1.0, 0.0, 0.0, 0.0], tolerance: [0.1, 0.1, 3.1416] } },
  open_manipulator_x: { spot: [-0.09, -0.2, 0.08], step: [0.06, 0, 0], scale: 0.3, level: null },
};
// Servo pad: label, key, linear, angular (unit directions).
const PAD = [
  ['+X', 'w', [1, 0, 0], [0, 0, 0]],
  ['−X', 's', [-1, 0, 0], [0, 0, 0]],
  ['+Y', 'a', [0, 1, 0], [0, 0, 0]],
  ['−Y', 'd', [0, -1, 0], [0, 0, 0]],
  ['+Z', 'q', [0, 0, 1], [0, 0, 0]],
  ['−Z', 'e', [0, 0, -1], [0, 0, 0]],
  ['+Rx', 'i', [0, 0, 0], [1, 0, 0]],
  ['−Rx', 'k', [0, 0, 0], [-1, 0, 0]],
  ['+Ry', 'j', [0, 0, 0], [0, 1, 0]],
  ['−Ry', 'l', [0, 0, 0], [0, -1, 0]],
  ['+Rz', 'u', [0, 0, 0], [0, 0, 1]],
  ['−Rz', 'o', [0, 0, 0], [0, 0, -1]],
];
const IDENTITY = [0.0, 0.0, 0.0, 1.0];
const round = (value, digits = 4) => Number(value.toFixed(digits)) || 0;

export class ControlPanel {
  constructor(panel, app) {
    this.app = app;
    this.panel = panel;
    this.place = PLACES[app.model.id];
    this.jointsDirty = false;
    this.trajectory = null;
    this.ikSolution = null;
    this.counter = {};
    this.servoTimer = null;
    panel.innerHTML = this.#markup();
    this.$ = (selector) => panel.querySelector(selector);
    this.#wireMotion();
    this.#wireJoints();
    this.#wirePlanning();
    this.#wireGripper();
    this.#wireScene();
    this.#wireKinematics();
    this.#wireServo();
    app.addEventListener('state', (event) => this.#onState(event.detail));
    app.addEventListener('scene', (event) => this.#renderObjects(event.detail.objects));
    app.addEventListener('goal', (event) => this.#showGoal(event.detail));
    app.addEventListener('select', (event) => this.#highlightObject(event.detail.id ?? null));
    app.addEventListener('session', () => this.syncSettings());
    app.addEventListener('settings', () => this.syncSettings());
  }

  #markup() {
    const model = this.app.model;
    // A position-only arm hides the orientation fields.
    const free = model.position_only_ik ? 'hidden' : '';
    const joints = model.joints
      .map((joint, i) => `<div class="slider-row">
          <label for="joint-${i}">J${i + 1}</label>
          <input type="range" id="joint-${i}" data-joint="${i}" min="${joint.lower}" max="${joint.upper}" step="0.005" value="0">
          <output id="joint-${i}-out" for="joint-${i}">0°</output>
        </div>`)
      .join('');
    const planners = PLANNERS.map(([pipeline, planner, label], i) => `<option value="${i}">${escapeHtml(label)}</option>`).join('');
    const pad = PAD.map(([label, key], i) => `<button type="button" class="btn small" data-pad="${i}" title="Hold to move (key ${key.toUpperCase()})">${label}</button>`).join('');
    return `
      <details class="card" open>
        <summary>Move</summary>
        <div class="segmented" role="group" aria-label="Goal type">
          <button type="button" data-goal-kind="pose" aria-pressed="true">Pose goal</button>
          <button type="button" data-goal-kind="joints" aria-pressed="false">Joint goal</button>
        </div>
        <div data-goal-section="pose">
          <p class="hint">${this.app.model.position_only_ik
            ? `Drag the goal ball, or type a position in <code>${this.app.model.planning_frame}</code>. Orientation is free on this 4-joint arm.`
            : `Drag the goal ball, or type a pose in <code>${this.app.model.planning_frame}</code>.`}</p>
          <div class="grid-3">
            <label class="field"><span>x (m)</span><input type="number" id="goal-x" step="0.01"></label>
            <label class="field"><span>y (m)</span><input type="number" id="goal-y" step="0.01"></label>
            <label class="field"><span>z (m)</span><input type="number" id="goal-z" step="0.01"></label>
            <label class="field" ${free}><span>roll (°)</span><input type="number" id="goal-roll" step="5"></label>
            <label class="field" ${free}><span>pitch (°)</span><input type="number" id="goal-pitch" step="5"></label>
            <label class="field" ${free}><span>yaw (°)</span><input type="number" id="goal-yaw" step="5"></label>
          </div>
          <p class="hint mono" id="goal-quat" ${free}></p>
          <div class="btn-row">
            <label class="check"><input type="checkbox" id="goal-cartesian"> Straight line</label>
            <label class="check" ${this.place.level ? '' : 'hidden'}><input type="checkbox" id="goal-level"> Keep level</label>
          </div>
        </div>
        <div data-goal-section="joints" hidden>
          <p class="hint">Drag a slider to preview.</p>
          <div id="joint-sliders">${joints}</div>
          <div class="btn-row"><button type="button" class="btn small" id="joints-current">Copy current</button></div>
        </div>
        <div class="btn-row">
          <button type="button" class="btn primary" id="move">Move</button>
          <button type="button" class="btn" id="plan">Plan</button>
          <button type="button" class="btn" id="execute" disabled>Execute</button>
          <button type="button" class="btn danger" id="stop">Stop</button>
        </div>
        <div class="btn-row">
          ${Object.keys(model.named_states[model.group_name]).map((name) => `<button type="button" class="btn small" data-named="${name}">${name}</button>`).join('')}
          <button type="button" class="btn small" id="goal-here" title="Put the goal marker at the gripper">Goal to gripper</button>
        </div>
      </details>

      <details class="card" open>
        <summary>Gripper</summary>
        <div class="btn-row">
          <button type="button" class="btn" id="gripper-open">Open</button>
          <button type="button" class="btn" id="gripper-close">Close</button>
          <span class="hint" id="gripper-state"></span>
        </div>
        <div class="slider-row">
          <label for="gripper-position">finger</label>
          <input type="range" id="gripper-position" min="${model.fingers.lower}" max="${model.fingers.upper}" step="0.001" value="${model.gripper.open_joint_positions[0]}">
          <output id="gripper-position-out" for="gripper-position"></output>
        </div>
      </details>

      <details class="card" open>
        <summary>Planning</summary>
        <label class="field"><span>Planner</span><select id="planner">${planners}</select></label>
        <div class="slider-row" style="margin-top:0.5rem">
          <label for="max-velocity">velocity</label>
          <input type="range" id="max-velocity" min="0.05" max="1" step="0.05" value="1">
          <output id="max-velocity-out" for="max-velocity">1.00</output>
        </div>
        <div class="slider-row">
          <label for="max-acceleration">accel.</label>
          <input type="range" id="max-acceleration" min="0.05" max="1" step="0.05" value="1">
          <output id="max-acceleration-out" for="max-acceleration">1.00</output>
        </div>
        <div class="grid-2" style="margin-top:0.5rem">
          <label class="field"><span>allowed_planning_time (s)</span><input type="number" id="planning-time" min="0.1" max="10" step="0.5"></label>
          <label class="field"><span>num_planning_attempts</span><input type="number" id="planning-attempts" min="1" max="20" step="1"></label>
        </div>
      </details>

      <details class="card" open>
        <summary>Scene</summary>
        <div class="btn-row" role="group" aria-label="Add a collision object">
          ${Object.entries(SHAPES).map(([type, { label }]) => `<button type="button" class="btn small" data-add="${type}">+ ${label}</button>`).join('')}
        </div>
        <div class="btn-row" role="group" aria-label="Preset scenes">
          ${Object.entries(PRESETS[model.id]).map(([name, { label }]) => `<button type="button" class="btn small" data-preset="${name}">${label}</button>`).join('')}
          <button type="button" class="btn small danger" id="scene-clear">Clear all</button>
        </div>
        <p class="hint" id="scene-empty">No objects. Add one, then click it in the scene to move it.</p>
        <ul class="object-list" id="object-list"></ul>
      </details>

      <details class="card">
        <summary>Kinematics</summary>
        <div class="btn-row">
          <button type="button" class="btn small" id="fk">compute_fk()</button>
          <button type="button" class="btn small" id="ik">compute_ik(goal)</button>
          <button type="button" class="btn small" id="ik-move" disabled>Move to solution</button>
        </div>
        <pre class="code-line" id="kinematics-result" hidden></pre>
      </details>

      <details class="card">
        <summary>Servo</summary>
        <div class="btn-row">
          <button type="button" class="btn small" id="servo-enable">Enable</button>
          <button type="button" class="btn small" id="servo-disable">Disable</button>
          <label class="field" style="flex:1"><select id="servo-frame" aria-label="Command frame">
            <option value="${model.base_link_name}">base frame (${model.base_link_name})</option>
            <option value="${model.end_effector_name}">end-effector frame (${model.end_effector_name})</option>
            ${model.planning_frame === model.base_link_name ? '' : `<option value="${model.planning_frame}">planning frame (${model.planning_frame})</option>`}
          </select></label>
        </div>
        <div class="slider-row">
          <label for="servo-speed">speed</label>
          <input type="range" id="servo-speed" min="0.02" max="0.3" step="0.01" value="0.1">
          <output id="servo-speed-out" for="servo-speed">0.10</output>
        </div>
        <div class="grid-3" id="servo-pad" style="margin-top:0.5rem">${pad}</div>
        <p class="hint">Hold a button, or focus the scene and hold W/S A/D Q/E (move) or I/K J/L U/O (turn). Each 50 ms sends <code>robot.servo(linear, angular)</code>.</p>
      </details>`;
  }

  // ------------------------------------------------------------------ motion

  get goalKind() {
    return this.$('[data-goal-kind][aria-pressed="true"]').dataset.goalKind;
  }

  #wireMotion() {
    for (const button of this.panel.querySelectorAll('[data-goal-kind]')) {
      button.addEventListener('click', () => this.setGoalKind(button.dataset.goalKind));
    }
    const fields = ['x', 'y', 'z', 'roll', 'pitch', 'yaw'].map((name) => this.$(`#goal-${name}`));
    for (const field of fields) field.addEventListener('change', () => this.#goalFromFields());
    this.$('#move').addEventListener('click', () => this.move());
    this.$('#plan').addEventListener('click', () => this.plan());
    this.$('#execute').addEventListener('click', () => this.execute());
    this.$('#stop').addEventListener('click', () => this.app.call('robot.arm.cancel_execution'));
    this.$('#goal-here').addEventListener('click', () => this.app.setGoal(this.app.sim.getState().ee, 'reset'));
    for (const button of this.panel.querySelectorAll('[data-named]')) {
      button.addEventListener('click', () => {
        const name = button.dataset.named;
        const model = this.app.model;
        if (name === model.default_state) this.app.call('robot.move_to_configuration');
        else this.app.call('robot.move_to_configuration', [model.named_states[model.group_name][name]], {}, { display: `robot.move_to_configuration(robot.joint_positions('${name}'))` });
      });
    }
  }

  setGoalKind(kind) {
    for (const button of this.panel.querySelectorAll('[data-goal-kind]')) {
      button.setAttribute('aria-pressed', String(button.dataset.goalKind === kind));
    }
    for (const section of this.panel.querySelectorAll('[data-goal-section]')) section.hidden = section.dataset.goalSection !== kind;
    this.app.setGoalVisible(kind === 'pose');
    if (kind === 'joints') this.#previewJoints();
  }

  /** Shows the goal in the fields; `all` includes the one being edited. */
  #showGoal({ pose }, all = false) {
    const [roll, pitch, yaw] = rpyFromQuat(pose.quat_xyzw);
    const values = { x: pose.position[0], y: pose.position[1], z: pose.position[2], roll: roll * DEG, pitch: pitch * DEG, yaw: yaw * DEG };
    for (const [name, value] of Object.entries(values)) {
      const field = this.$(`#goal-${name}`);
      if (all || document.activeElement !== field) field.value = round(value, name.length === 1 ? 3 : 1);
    }
    this.$('#goal-quat').textContent = `quat_xyzw = [${pose.quat_xyzw.map((v) => round(v, 3)).join(', ')}]`;
  }

  #goalFromFields() {
    const value = (name) => Number(this.$(`#goal-${name}`).value);
    // An emptied field, or text a number field cannot read, goes back to the goal's value rather than to 0.
    if (['x', 'y', 'z', 'roll', 'pitch', 'yaw'].some((name) => this.$(`#goal-${name}`).value === '')) {
      this.#showGoal({ pose: this.app.goal }, true);
      return;
    }
    const quat = quatFromRpy(value('roll') / DEG, value('pitch') / DEG, value('yaw') / DEG);
    this.app.setGoal({ position: [value('x'), value('y'), value('z')], quat_xyzw: Array.from(quat) }, 'fields');
  }

  #poseKwargs() {
    const goal = this.app.goal;
    const model = this.app.model;
    const kwargs = { position: goal.position.map((v) => round(v)), quat_xyzw: goal.quat_xyzw.map((v) => round(v)) };
    // Goals default to base_link_name; the marker lives in the planning frame.
    Object.assign(kwargs, this.app.frameKwargs());
    if (this.$('#goal-cartesian').checked) kwargs.cartesian = true;
    else if (model.position_only_ik) kwargs.tolerance_orientation = 3.1416;
    return kwargs;
  }

  async #withLevelConstraint(run) {
    const level = this.place.level && this.$('#goal-level').checked;
    if (level) await this.app.call('robot.set_path_orientation_constraint', [this.place.level.quat], { tolerance: this.place.level.tolerance });
    try {
      return await run();
    } finally {
      if (level) await this.app.call('robot.clear_path_constraints');
    }
  }

  async move() {
    if (this.goalKind === 'joints') {
      const q = this.#sliderValues();
      this.jointsDirty = false;
      return this.app.call('robot.move_to_configuration', [q]);
    }
    return this.#withLevelConstraint(() => this.app.call('robot.move_to_pose', [], this.#poseKwargs()));
  }

  async plan() {
    const kwargs = this.goalKind === 'joints' ? { joint_positions: this.#sliderValues() } : this.#poseKwargs();
    const trajectory = await this.#withLevelConstraint(() => this.app.call('robot.arm.plan', [], kwargs, { assign: 'trajectory' }));
    this.trajectory = trajectory ?? null;
    this.$('#execute').disabled = !this.trajectory;
  }

  async execute() {
    if (!this.trajectory) return;
    const trajectory = this.trajectory;
    this.trajectory = null;
    this.$('#execute').disabled = true;
    await this.app.call('robot.arm.execute', [trajectory], {}, { display: 'robot.arm.execute(trajectory)' });
  }

  // ------------------------------------------------------------------ joints

  #wireJoints() {
    for (const slider of this.panel.querySelectorAll('[data-joint]')) {
      slider.addEventListener('input', () => {
        this.jointsDirty = true;
        this.#jointOutput(slider);
        this.#previewJoints();
      });
    }
    this.$('#joints-current').addEventListener('click', () => {
      this.jointsDirty = false;
      this.#onState(this.app.sim.getState());
      this.#previewJoints();
    });
  }

  #jointOutput(slider) {
    this.$(`#${slider.id}-out`).textContent = `${Math.round(Number(slider.value) * DEG)}°`;
  }

  #sliderValues() {
    return [...this.panel.querySelectorAll('[data-joint]')].map((slider) => round(Number(slider.value), 3));
  }

  #previewJoints() {
    if (this.goalKind !== 'joints') return;
    this.app.previewJoints(this.jointsDirty ? this.#sliderValues() : null);
  }

  #onState(state) {
    if (!this.jointsDirty) {
      for (const slider of this.panel.querySelectorAll('[data-joint]')) {
        if (document.activeElement === slider) continue;
        slider.value = state.q[Number(slider.dataset.joint)];
        this.#jointOutput(slider);
      }
    }
    const { gripper: g } = this.app.model, position = state.fingers;
    const gripper = this.$('#gripper-position');
    if (document.activeElement !== gripper) {
      gripper.value = position;
      this.$('#gripper-position-out').textContent = `${Math.round(position * 1000)} mm`;
    }
    const [open, closed] = [g.open_joint_positions[0], g.closed_joint_positions[0]], tolerance = 0.03 * Math.abs(open - closed);
    const text = Math.abs(position - open) <= tolerance ? 'open' : Math.abs(position - closed) <= tolerance ? 'closed' : `pads ${(2000 * (g.grasp.offset + position)).toFixed(0)} mm apart`;
    if (this.$('#gripper-state').textContent !== text) this.$('#gripper-state').textContent = text;
  }

  // ---------------------------------------------------------------- planning

  #wirePlanning() {
    this.$('#planner').addEventListener('change', async (event) => {
      const [pipeline, planner] = PLANNERS[Number(event.target.value)];
      await this.app.set('robot.arm.pipeline_id', pipeline);
      await this.app.set('robot.arm.planner_id', planner);
    });
    for (const name of ['max-velocity', 'max-acceleration']) {
      const slider = this.$(`#${name}`);
      slider.addEventListener('input', () => {
        this.$(`#${name}-out`).textContent = Number(slider.value).toFixed(2);
      });
      slider.addEventListener('change', () => this.app.set(`robot.arm.${name.replace('-', '_')}`, round(Number(slider.value), 2)));
    }
    // Values outside a field's bounds are clamped to them; an emptied field shows the current value again.
    for (const [id, name, read] of [['planning-time', 'allowed_planning_time', Number], ['planning-attempts', 'num_planning_attempts', (text) => Math.round(Number(text))]]) {
      const field = this.$(`#${id}`);
      field.addEventListener('change', () => {
        if (field.value === '') this.syncSettings();
        else this.app.set(`robot.arm.${name}`, Math.min(Number(field.max), Math.max(Number(field.min), read(field.value))));
      });
    }
  }

  /** Shows the session's current planning parameters. */
  async syncSettings() {
    const get = async (name) => {
      try {
        return await this.app.session.getattr(`robot.arm.${name}`);
      } catch {
        return undefined;
      }
    };
    const pipeline = (await get('pipeline_id')) || 'ompl';
    const planner = (await get('planner_id')) || 'RRTConnectkConfigDefault';
    const index = PLANNERS.findIndex(([p, id]) => (p === pipeline || (p === 'ompl' && !pipeline)) && id === planner);
    this.$('#planner').value = String(Math.max(0, index));
    for (const name of ['max_velocity', 'max_acceleration']) {
      const value = Number(await get(name)) || 0;
      const id = name.replace('_', '-');
      // 0.0 means "not set", which MoveIt treats as 1.0.
      this.$(`#${id}`).value = value > 0 ? value : 1;
      this.$(`#${id}-out`).textContent = (value > 0 ? value : 1).toFixed(2);
    }
    this.$('#planning-time').value = (await get('allowed_planning_time')) ?? 5;
    this.$('#planning-attempts').value = (await get('num_planning_attempts')) ?? 1;
  }

  // ----------------------------------------------------------------- gripper

  #wireGripper() {
    this.$('#gripper-open').addEventListener('click', () => this.app.call('robot.gripper.open'));
    this.$('#gripper-close').addEventListener('click', () => this.app.call('robot.gripper.close'));
    const slider = this.$('#gripper-position');
    slider.addEventListener('input', () => {
      this.$('#gripper-position-out').textContent = `${Math.round(Number(slider.value) * 1000)} mm`;
    });
    slider.addEventListener('change', () => this.app.call('robot.gripper.move_to_position', [round(Number(slider.value), 3)]));
  }

  // ------------------------------------------------------------------- scene

  #nextId(prefix) {
    const taken = new Set(this.app.objects.map((object) => object.id));
    let n = 1;
    while (taken.has(`${prefix}${n}`)) n += 1;
    return `${prefix}${n}`;
  }

  #wireScene() {
    for (const button of this.panel.querySelectorAll('[data-add]')) {
      button.addEventListener('click', async () => {
        const type = button.dataset.add;
        const id = this.#nextId(type === 'mesh' ? 'suzanne' : type);
        const { spot, step, scale } = this.place, k = this.app.objects.length % 4;
        const pose = { position: spot.map((x, i) => round(x + k * step[i], 3)), quat_xyzw: IDENTITY };
        const [path, args, kwargs] = SHAPES[type].call(id, pose, scale);
        args[1] = Array.isArray(args[1]) ? args[1].map((v) => round(v, 3)) : round(args[1], 3);
        if (args[2] !== undefined) args[2] = round(args[2], 3);
        if (kwargs.scale) kwargs.scale = round(kwargs.scale, 3);
        await this.app.call(path, args, { ...kwargs, ...this.app.frameKwargs() });
        this.app.selectObject(id);
      });
    }
    for (const button of this.panel.querySelectorAll('[data-preset]')) {
      button.addEventListener('click', () => this.app.runPython(PRESETS[this.app.model.id][button.dataset.preset].code));
    }
    this.$('#scene-clear').addEventListener('click', () => this.app.call('robot.clear_all_collision_objects'));
    this.$('#object-list').addEventListener('click', (event) => {
      const item = event.target.closest('li');
      if (!item) return;
      const id = item.dataset.id;
      const action = event.target.closest('[data-action]')?.dataset.action;
      if (action === 'remove') this.app.call('robot.remove_collision_object', [id]);
      else if (action === 'attach') {
        const { fingers } = this.app.model;
        this.app.call('robot.attach_collision_object', [id], { touch_links: [fingers.parent, ...fingers.links] });
      } else if (action === 'detach') this.app.call('robot.detach_collision_object', [id]);
      else if (!event.target.closest('input')) this.app.selectObject(id);
    });
    this.$('#object-list').addEventListener('change', (event) => {
      const input = event.target.closest('[data-action="allow"]');
      if (input) this.app.call('robot.allow_collisions', [input.closest('li').dataset.id, input.checked]);
    });
  }

  #renderObjects(objects) {
    this.$('#scene-empty').hidden = objects.length > 0;
    const selected = this.app.selectedObject;
    this.$('#object-list').innerHTML = objects
      .map((object) => {
        const attached = Boolean(object.attached);
        return `<li data-id="${escapeHtml(object.id)}" aria-current="${object.id === selected}">
          <span class="id">${escapeHtml(object.id)}</span>
          <span class="badge">${escapeHtml(object.type)}</span>
          ${attached ? `<span class="badge">on ${escapeHtml(object.attached.link)}</span>` : ''}
          <label class="check" title="allow_collisions: MoveIt ignores this object"><input type="checkbox" data-action="allow" ${object.allowed ? 'checked' : ''}> allow</label>
          <button type="button" class="btn small" data-action="${attached ? 'detach' : 'attach'}">${attached ? 'Detach' : 'Attach'}</button>
          <button type="button" class="btn small danger" data-action="remove" aria-label="Remove ${escapeHtml(object.id)}">✕</button>
        </li>`;
      })
      .join('');
  }

  #highlightObject(id) {
    for (const item of this.panel.querySelectorAll('#object-list li')) item.setAttribute('aria-current', String(item.dataset.id === id));
  }

  // -------------------------------------------------------------- kinematics

  #wireKinematics() {
    // Python prints message fields as floats: 0.0, not 0.
    const floats = (values) => values.map((v) => pyRepr(round(v, 3), 'float')).join(', ');
    const show = (text) => {
      const pre = this.$('#kinematics-result');
      pre.hidden = false;
      pre.textContent = text;
    };
    this.$('#fk').addEventListener('click', async () => {
      const result = await this.app.call('robot.compute_fk');
      if (result) {
        const { position: p, orientation: o } = result.pose;
        show(`position    = [${floats([p.x, p.y, p.z])}]\nquat_xyzw   = [${floats([o.x, o.y, o.z, o.w])}]\nframe_id    = '${result.header.frame_id}'`);
      }
    });
    this.$('#ik').addEventListener('click', async () => {
      // compute_ik() reads the pose in base_link_name.
      const goal = poseInLink(this.app.model, this.app.sim.getState().q, this.app.goal);
      const result = await this.app.call('robot.compute_ik', [goal.position.map((v) => round(v)), goal.quat_xyzw.map((v) => round(v))], {}, { assign: 'solution' });
      this.ikSolution = result?.position ? result.position.slice(0, this.app.model.joints.length) : null;
      this.$('#ik-move').disabled = !this.ikSolution;
      if (this.ikSolution) {
        show(`solution.position = [${floats(this.ikSolution)}]`);
        this.app.previewJoints(this.ikSolution);
      } else show('None: no collision-free solution for this pose');
    });
    this.$('#ik-move').addEventListener('click', () => {
      if (this.ikSolution) this.app.call('robot.move_to_configuration', [this.ikSolution], {}, { display: 'robot.move_to_configuration(solution.position)' });
    });
  }

  // ------------------------------------------------------------------- servo

  #wireServo() {
    this.$('#servo-enable').addEventListener('click', () => this.app.call('robot.servo.enable'));
    this.$('#servo-disable').addEventListener('click', () => this.app.call('robot.servo.disable'));
    this.$('#servo-frame').addEventListener('change', (event) => this.app.set('robot.servo.frame_id', event.target.value));
    const speed = this.$('#servo-speed');
    speed.addEventListener('input', () => {
      this.$('#servo-speed-out').textContent = Number(speed.value).toFixed(2);
    });
    const pad = this.$('#servo-pad');
    let padKey = null;
    pad.addEventListener('pointerdown', (event) => {
      const button = event.target.closest('[data-pad]');
      if (!button) return;
      button.setPointerCapture(event.pointerId);
      this.startServo(Number(button.dataset.pad));
    });
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) pad.addEventListener(type, () => this.stopServo());
    pad.addEventListener('keydown', (event) => {
      if (!['Enter', ' '].includes(event.key)) return;
      const button = event.target.closest('[data-pad]');
      if (!button) return;
      event.preventDefault();
      if (event.repeat || padKey) return;
      padKey = { key: event.key, button };
      this.startServo(Number(button.dataset.pad));
    });
    const releasePadKey = (event) => {
      if (!padKey || (event.key && event.key !== padKey.key)) return;
      padKey = null;
      this.stopServo();
    };
    pad.addEventListener('keyup', releasePadKey);
    pad.addEventListener('focusout', () => releasePadKey({}));
    // Without WebGL there is no scene to take the keys.
    const canvas = this.app.viewport?.canvas;
    const held = new Set();
    window.addEventListener('keyup', releasePadKey);
    window.addEventListener('blur', () => {
      held.clear();
      releasePadKey({});
      this.stopServo();
    });
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        held.clear();
        releasePadKey({});
        this.stopServo();
      }
    });
    if (!canvas) return;
    canvas.addEventListener('keydown', (event) => {
      const index = PAD.findIndex(([, key]) => key === event.key.toLowerCase());
      if (index < 0 || event.ctrlKey || event.metaKey || event.altKey) return;
      event.preventDefault();
      if (held.has(index)) return;
      held.add(index);
      this.startServo(index);
    });
    canvas.addEventListener('keyup', (event) => {
      const index = PAD.findIndex(([, key]) => key === event.key.toLowerCase());
      if (index < 0) return;
      held.delete(index);
      if (!held.size) this.stopServo();
    });
    canvas.addEventListener('blur', () => {
      held.clear();
      this.stopServo();
    });
  }

  startServo(index) {
    this.stopServo();
    const [, , linear, angular] = PAD[index];
    const speed = Number(this.$('#servo-speed').value);
    // Rotation pads turn at 0.5 rad/s per 0.1 m/s.
    const command = {
      linear: linear.map((v) => round(v * speed, 3)),
      angular: angular.map((v) => round(v * speed * 5, 3)),
    };
    const kwargs = {};
    if (linear.some(Boolean)) kwargs.linear = command.linear;
    if (angular.some(Boolean)) kwargs.angular = command.angular;
    const send = (quiet = false) => this.app.call('robot.servo', [], kwargs, { ...(quiet ? { quiet: true } : {}), repeat: 0.05 });
    send();
    this.servoTimer = setInterval(() => send(true), 50);
  }

  stopServo() {
    if (this.servoTimer) clearInterval(this.servoTimer);
    this.servoTimer = null;
  }
}

const box = (id, size, position, frame = '') => `robot.add_collision_box('${id}', [${size.join(', ')}], position=[${position.join(', ')}], quat_xyzw=[0.0, 0.0, 0.0, 1.0]${frame})`;
const WORLD = ", frame_id='world'";
const TABLE = box('table', ['0.8', '1.2', '0.04'], ['0.55', '0.0', '-0.02']);
/** Preset scenes per robot: a label and the Python that builds them. */
export const PRESETS = {
  panda: {
    table: { label: 'Table', code: TABLE },
    shelf: {
      label: 'Shelf',
      code: [
        TABLE,
        box('shelf_left', ['0.3', '0.02', '0.6'], ['0.6', '-0.25', '0.3']),
        box('shelf_right', ['0.3', '0.02', '0.6'], ['0.6', '0.25', '0.3']),
        box('shelf_board', ['0.3', '0.52', '0.02'], ['0.6', '0.0', '0.3']),
        box('shelf_top', ['0.3', '0.52', '0.02'], ['0.6', '0.0', '0.6']),
      ].join('\n'),
    },
    pick: { label: 'Cube on a table', code: [TABLE, box('cube', ['0.04', '0.04', '0.05'], ['0.5', '-0.25', '0.025'])].join('\n') },
  },
  // Objects rest on the floor, at z = 0.025. Positions are in world, since
  // pymoveit2's default frame, link2, turns with joint1.
  open_manipulator_x: {
    wall: { label: 'Wall', code: box('wall', ['0.12', '0.02', '0.06'], ['0.0', '-0.06', '0.055'], WORLD) },
    pick: {
      label: 'Cube on the floor',
      code: box('cube', ['0.025', '0.025', '0.025'], ['0.0', '-0.12', '0.0375'], WORLD),
    },
  },
};
