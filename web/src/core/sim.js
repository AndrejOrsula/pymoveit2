// Browser-only move_group, independent arm/hand controllers and servo node.
import { PANDA } from './panda.js';
import { Events } from './events.js';
import { plan as makePlan } from './planning.js';
import { fk, jacobian, linkTransforms } from './kinematics.js';
import { solveIK, dampedStep } from './ik.js';
import { checkState, minimumDistance, resolvedObject, objectSpheres } from './collision.js';
import { sampleTrajectory } from './timing.js';
import { createRng, mat4FromPose, mat4Multiply, mat4InvertRigid, mat4TransformPoint, poseFromMat4, quatRotate, norm, sub, lerp, clamp } from './math.js';

const copy = value => structuredClone(value);

/** Deterministic move_group simulation advanced exclusively by step(dt). */
export class MoveGroupSim extends Events {
  constructor({ model = PANDA, seed = 1, q = model.named_states[model.group_name][model.default_state], fingers = model.gripper.open_joint_positions[0] } = {}) {
    super();
    this.model = model; this.rng = createRng(seed); this.initial = { q: [...q], fingers, rng_state: this.rng.state };
    this.nextId = 1; this.log = []; this.executions = {}; this.objects = []; this.time = 0;
    this.reset();
  }
  /** Read the current robot/controller state without exposing mutable arrays. */
  getState() {
    return { time: this.time, q: [...this.q], velocities: [...this.velocities], fingers: this.fingers, mode: this.executions.arm ? 'executing' : norm(this.velocities) > 1e-9 ? 'servo' : 'idle', ee: fk(this.model, this.q), contacts: this._contacts() };
  }
  /** Contacts of the current state; every scene change goes through _scene(), which drops the cache. */
  _contacts() {
    const key = `${this.q.join()}|${this.fingers}`;
    if (this.contactCache?.key !== key) this.contactCache = { key, contacts: checkState(this.model, this.q, this.fingers, this.objects).contacts };
    return this.contactCache.contacts;
  }
  /** JSON-serializable robot, scene, RNG, servo and controller checkpoint. */
  snapshot() {
    return copy({ robot: this.model.id, time: this.time, q: this.q, fingers: this.fingers, objects: this.objects, servo: this.servo, rng_state: this.rng.state, executions: this.executions, nextId: this.nextId, velocities: this.velocities });
  }
  /** Restore a checkpoint, or return to the constructor's initial robot state. */
  reset(snapshot) {
    const s = copy(snapshot ?? this.initial);
    if (s.robot && s.robot !== this.model.id) throw new RangeError(`Snapshot of '${s.robot}' cannot restore '${this.model.id}'`);
    for (const target of Object.keys(this.executions)) this.cancel(target);
    this.time = s.time ?? 0; this.q = [...s.q]; this.fingers = s.fingers;
    this.velocities = s.velocities ?? this.q.map(() => 0); this.objects = s.objects ?? [];
    this.servo = s.servo ?? { enabled: false, command: null, active_command_type: null };
    this.rng.state = s.rng_state ?? this.initial.rng_state;
    this.executions = s.executions ?? {}; this.nextId = s.nextId ?? 1;
    this.log = []; this._scene(); this.emit('state', this.getState());
  }
  /** Append a node log and retain the last 500 entries. */
  writeLog(level, node, text) {
    const entry = { time: this.time, level, node, text };
    this.log.push(entry); if (this.log.length > 500) this.log.shift(); this.emit('log', entry);
  }
  /** Return a detached copy of the bounded log. */
  getLog() { return copy(this.log); }
  _scene() {
    this.contactCache = null;
    this._updateAttached();
    this.sceneDirty = true;
    this.emit('scene', { objects: this.getObjects() });
  }
  _updateAttached() {
    const transforms = linkTransforms(this.model, this.q, this.fingers);
    this.objects = this.objects.map(o => resolvedObject(o, transforms));
  }
  /** Apply ADD (replace), REMOVE, APPEND or MOVE using MoveIt operation numbers. */
  applyCollisionObject(object) {
    const operation = object.operation ?? 0, old = this.objects.find(o => o.id === object.id);
    if (operation === 1 || operation === 'remove') {
      this.objects = this.objects.filter(o => object.id && o.id !== object.id);
    } else if (operation === 3 || operation === 'move') {
      if (old) {
        old.pose = copy(object.pose);
        if (old.attached) old.attached.relative_pose = poseFromMat4(mat4Multiply(mat4InvertRigid(linkTransforms(this.model, this.q, this.fingers)[old.attached.link]), mat4FromPose(old.pose)));
      }
    } else {
      const next = { attached: null, allowed: old?.allowed ?? false, ...copy(object) };
      delete next.operation;
      // The contract holds one shape per object; APPEND can update that shape only.
      this.objects = this.objects.filter(o => o.id !== object.id); this.objects.push(next);
    }
    this._scene();
  }
  /** Attach an existing world object, preserving its current world transform. */
  attachObject(id, link = this.model.end_effector_name, touch_links = [], weight = 0) {
    const object = this.objects.find(o => o.id === id), transform = linkTransforms(this.model, this.q, this.fingers)[link];
    if (!object || !transform) return false;
    object.attached = { link, touch_links: [...touch_links], weight, relative_pose: poseFromMat4(mat4Multiply(mat4InvertRigid(transform), mat4FromPose(object.pose))) };
    this._scene(); return true;
  }
  /** Return an attached object to the world at its current pose. */
  detachObject(id) {
    this._updateAttached(); const object = this.objects.find(o => o.id === id);
    if (!object?.attached) return false;
    object.attached = null; this._scene(); return true;
  }
  /** Detach every object while preserving its world pose. */
  detachAll() { this._updateAttached(); for (const object of this.objects) object.attached = null; this._scene(); }
  /** Set the object's allowed-collision matrix entry. */
  allowCollisions(id, allow) { const o = this.objects.find(o => o.id === id); if (o) o.allowed = !!allow; this._scene(); }
  /** Remove all world and attached geometry. */
  clearScene() { this.objects = []; this._scene(); }
  /** Read detached scene objects with current attachment world poses. */
  getObjects() { this._updateAttached(); return copy(this.objects); }
  /** Plan and emit move_group-style request, warning and result logs. */
  plan(request) {
    this.writeLog('info', 'move_group', 'Planning request received for MoveGroup action. Forwarding to planning pipeline.');
    if (!request.cartesian && request.pipeline_id !== 'pilz_industrial_motion_planner') this.writeLog('info', 'move_group', `Planner configuration '${this.model.group_name}[${request.planner_id || 'RRTConnectkConfigDefault'}]' will use planner 'geometric::RRTConnect'...`);
    for (const [field, value] of [['velocity', request.max_velocity_scaling_factor ?? 0], ['acceleration', request.max_acceleration_scaling_factor ?? 0]]) {
      if (!(value > 0 && value <= 1)) this.writeLog('warn', 'move_group', `Invalid max_${field}_scaling_factor ${Number(value).toFixed(6)} specified, defaulting to 1.000000 instead.`);
    }
    const result = makePlan(this.model, request, { q: this.q, fingers: this.fingers, objects: this.getObjects(), rng: this.rng });
    this.writeLog(result.error_code === 'SUCCESS' ? 'info' : 'warn', 'move_group', result.error_code === 'SUCCESS' ? `Solution found in ${result.planning_time.toFixed(3)} seconds` : `${result.error_code}: ${result.detail ?? 'Unable to solve the planning problem.'}`);
    this.emit('plan', { request: copy(request), result: copy(result) }); return result;
  }
  /** Compute poses for one or more named links. */
  computeFK(q = this.q, links = [this.model.end_effector_name]) { return links.map(link => fk(this.model, q, link)); }
  /** The /compute_ik service: collision-aware IK with the simulation RNG and the robot's IK settings. */
  computeIK(target, opts = {}) { return solveIK(this.model, target, { seed: this.q, rng: this.rng, position_only: this.model.position_only_ik, isValid: q => checkState(this.model, q, this.fingers, this.getObjects()).valid, ...opts }); }
  /** Submit to one controller; stale starts abort with INVALID_MOTION_PLAN. */
  execute(trajectory, target = 'arm') {
    this.cancel(target);
    const names = target === 'arm' ? this.model.joint_names : this.model.gripper.joint_names;
    const indices = names.map(n => trajectory?.joint_names?.indexOf(n) ?? -1);
    const wellFormed = trajectory?.points?.length > 0 && trajectory.joint_names.length === names.length && indices.every(i => i >= 0) && trajectory.points.every((p, i) => p.positions?.length === names.length && p.positions.every(Number.isFinite) && Number.isFinite(p.time_from_start) && p.time_from_start >= 0 && (!i || p.time_from_start > trajectory.points[i - 1].time_from_start || (p.time_from_start === trajectory.points[i - 1].time_from_start && p.positions.every((x, j) => x === trajectory.points[i - 1].positions[j]))));
    if (wellFormed) trajectory = { joint_names: [...names], points: trajectory.points.map(p => ({ positions: indices.map(i => p.positions[i]), velocities: indices.map(i => p.velocities?.[i] ?? 0), accelerations: indices.map(i => p.accelerations?.[i] ?? 0), time_from_start: p.time_from_start })) };
    const id = this.nextId++, event = { id, target, trajectory: copy(trajectory), error_code: 'SUCCESS' };
    const current = target === 'arm' ? this.q : names.map(() => this.fingers);
    if (!wellFormed || trajectory.points.some(p => p.velocities.some(v => !Number.isFinite(v)) || p.accelerations.some(v => !Number.isFinite(v))) || trajectory.points[0].positions.some((x, i) => Math.abs(x - current[i]) > 0.01)) {
      this.writeLog('warn', 'move_group', 'Trajectory start state differs from the current state, or the trajectory is invalid.');
      this.emit('execution', { ...event, status: 'aborted', error_code: 'INVALID_MOTION_PLAN' });
      return id;
    }
    this.executions[target] = { ...event, start: this.time };
    if (target === 'arm') this.servo.command = null;
    this.writeLog('info', this.model.controllers[target], 'Received new action goal, accepted.');
    this.emit('execution', { ...event, status: 'started' }); return id;
  }
  _finish(target, status, error_code) {
    const execution = this.executions[target]; if (!execution) return;
    delete this.executions[target];
    if (target === 'arm') this.velocities.fill(0);
    const { id, trajectory } = execution;
    this.emit('execution', { id, trajectory: copy(trajectory), target, status, error_code });
    this.writeLog(status === 'succeeded' ? 'info' : 'warn', this.model.controllers[target], `${status}: ${error_code}`);
  }
  /** Cancel the tracked arm or hand execution. */
  cancel(target = 'arm') { if (!this.executions[target]) return false; this._finish(target, 'canceled', 'PREEMPTED'); return true; }
  /** Enable servo command admission. */
  servoEnable() { this.servo.enabled = true; return true; }
  /** Disable servo and discard pending commands. */
  servoDisable() { this.servo.enabled = false; this.servo.command = null; if (!this.executions.arm) this.velocities.fill(0); return true; }
  /** Pause servo, matching the modern pause_servo service. */
  servoPause() { return this.servoDisable(); }
  _command(command, type) {
    if (this.executions.arm) { this.writeLog('warn', 'servo_node', 'Trajectory execution active, ignoring servo command.'); return false; }
    if (!this.servo.enabled) return false;
    this.servo.command = { ...command, expires: this.time + 0.1 }; this.servo.active_command_type = type; return true;
  }
  /** Send speed-unit twist in the named robot frame. */
  servoTwist(linear, angular, frame_id = this.model.base_link_name) {
    const transform = linkTransforms(this.model, this.q, this.fingers)[frame_id];
    if (!transform) { this.writeLog('warn', 'servo_node', `Unknown command frame '${frame_id}'.`); return false; }
    const rotation = poseFromMat4(transform).quat_xyzw;
    return this._command({ linear: quatRotate(rotation, linear), angular: quatRotate(rotation, angular) }, 1);
  }
  /** Send a finite velocity command for configured arm joints. */
  servoJog(joint_names, velocities) {
    const dq = this.q.map(() => 0);
    for (let i = 0; i < joint_names.length; i++) { const j = this.model.joint_names.indexOf(joint_names[i]); if (j < 0) return false; dq[j] = velocities[i]; }
    return this._command({ dq }, 0);
  }
  /** The finger position at which closing fingers stop on an object between the pads, or -Infinity. */
  _graspPosition() {
    const g = this.model.gripper.grasp, [c, a] = [g.closing, g.approach], l = 3 - c - a;
    const frameInverse = mat4InvertRigid(linkTransforms(this.model, this.q, this.fingers)[g.frame]);
    const pad = g.offset + this.fingers + 1e-7;
    // Center p and half extents e of a shape, in the pad frame, that fits between the pads.
    const fits = (p, e) => Math.abs(p[l]) < e[l] + g.lateral && p[a] + e[a] >= g.depth[0] && p[a] - e[a] <= g.depth[1] && Math.abs(p[c]) + e[c] <= pad;
    let width = -Infinity;
    for (const object of this.objects) {
      const local = mat4Multiply(frameInverse, mat4FromPose(object.pose)), p = Array.from(local.slice(12, 15));
      const d = object.type === 'box' ? object.dimensions : object.type === 'sphere' ? Array(3).fill(2 * object.dimensions[0]) : null;
      if (!d) {
        for (const sphere of objectSpheres(object)) {
          const point = mat4TransformPoint(frameInverse, sphere), r = sphere[3];
          if (fits(point, [r, r, r])) width = Math.max(width, Math.abs(point[c]) + r);
        }
        continue;
      }
      const ext = [0, 1, 2].map(r => d.reduce((sum, x, k) => sum + Math.abs(local[k * 4 + r]) * x / 2, 0));
      if (fits(p, ext)) width = Math.max(width, Math.abs(p[c]) + ext[c]);
    }
    return width - g.offset;
  }
  _servoStep(dt, previousTime) {
    const command = this.servo.command;
    if (!this.servo.enabled || !command || command.expires <= previousTime) { this.velocities.fill(0); return; }
    const duration = Math.min(dt, command.expires - previousTime);
    if (duration <= 0) { this.velocities.fill(0); return; }
    let dq = command.dq ? [...command.dq] : dampedStep(jacobian(this.model, this.q), [...command.linear, ...command.angular], 0.008);
    let factor = Math.min(1, ...dq.map((v, i) => this.model.joints[i].max_velocity / Math.max(Math.abs(v), 1e-12)));
    if (!command.dq && norm(dq) > 8 * Math.max(norm([...command.linear, ...command.angular]), 0.001)) {
      factor *= 0.25;
      this.writeLog('warn', 'servo_node', 'Close to a singularity, decelerating');
    }
    const distance = this.model.servo.check_collisions ? minimumDistance(this.model, this.q, this.fingers, this.objects) : Infinity;
    if (distance < 0.02) { factor = 0; this.writeLog('warn', 'servo_node', 'Collision detected, halting'); }
    else if (distance < 0.06) { factor *= (distance - 0.02) / 0.04; this.writeLog('warn', 'servo_node', 'Collision proximity, decelerating'); }
    dq = dq.map(v => v * factor);
    if (this.q.some((x, i) => x + dq[i] * duration < this.model.joints[i].lower || x + dq[i] * duration > this.model.joints[i].upper)) {
      this.velocities.fill(0); this.writeLog('warn', 'servo_node', 'Joint limit reached, halting'); return;
    }
    const next = this.q.map((v, i) => clamp(v + dq[i] * duration, this.model.joints[i].lower, this.model.joints[i].upper));
    if (this.model.servo.check_collisions && !checkState(this.model, next, this.fingers, this.objects).valid) { this.velocities.fill(0); this.writeLog('warn', 'servo_node', 'Collision detected, halting'); return; }
    this.velocities = next.map((v, i) => (v - this.q[i]) / dt); this.q = next;
    if (command.expires <= this.time + 1e-10) this.velocities.fill(0);
  }
  /** Advance at most 50 ms, integrate both controllers, then emit one state event. */
  step(dt) {
    dt = clamp(Number.isFinite(dt) ? dt : 0, 0, 0.05);
    const previousTime = this.time; this.time += dt;
    if (this.sceneDirty && this.executions.arm) {
      const e = this.executions.arm, duration = e.trajectory.points.at(-1).time_from_start;
      let previous = [...this.q];
      for (let t = Math.max(0, previousTime - e.start); t <= duration + 0.019; t += 0.02) {
        const sample = sampleTrajectory(e.trajectory, Math.min(t, duration));
        const steps = Math.max(1, Math.ceil(norm(sub(sample.positions, previous)) / 0.015));
        let valid = true;
        for (let k = 1; k <= steps; k++) if (!checkState(this.model, lerp(previous, sample.positions, k / steps), this.fingers, this.objects).valid) { valid = false; break; }
        if (!valid) { this._finish('arm', 'aborted', 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'); break; }
        previous = sample.positions;
      }
    }
    this.sceneDirty = false;
    const arm = this.executions.arm;
    if (arm) {
      const t = this.time - arm.start, sample = sampleTrajectory(arm.trajectory, t);
      this.q = sample.positions; this.velocities = sample.velocities;
      if (t >= arm.trajectory.points.at(-1).time_from_start) this._finish('arm', 'succeeded', 'SUCCESS');
    } else this._servoStep(dt, previousTime);
    const hand = this.executions.gripper;
    if (hand) {
      const goal = hand.trajectory.points.at(-1).positions[0], contact = goal < this.fingers ? this._graspPosition() : -Infinity;
      const target = Math.max(goal, contact), delta = clamp(target - this.fingers, -this.model.fingers.max_velocity * dt, this.model.fingers.max_velocity * dt);
      this.fingers = clamp(this.fingers + delta, this.model.fingers.lower, this.model.fingers.upper);
      if (Math.abs(this.fingers - target) < 1e-8) { this.fingers = target; this._finish('gripper', 'succeeded', 'SUCCESS'); }
    }
    this._updateAttached(); const state = this.getState(); this.emit('state', state); return state;
  }
  /** Whether a controller or a nonzero unexpired servo command is active. */
  isBusy(target) {
    if (target) return !!this.executions[target];
    const c = this.servo.command;
    return !!(this.executions.arm || this.executions.gripper || (this.servo.enabled && c && c.expires > this.time && norm(c.dq ?? [...c.linear, ...c.angular]) > 0));
  }
}
