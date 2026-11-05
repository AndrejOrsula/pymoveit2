// Python-shaped facade, call tracing and shared sync/live generator dispatch.
import { Events } from '../core/events.js';
import { PyError } from './errors.js';
import { SIGNATURES, resolveSpec } from './signatures.js';
import { bindArguments } from './binder.js';
import { formatCall, pyRepr } from './codegen.js';
import { describeErrorCode } from './diagnostics.js';
import { checkDriver, runSync, runLive } from './drivers.js';
import { armOperation } from './arm.js';
import { sceneOperation, planningScene } from './scene.js';
import { gripperOperation, servoOperation } from './gripper-servo.js';
import { jointState, finiteFloat, valueError } from './values.js';

const classes = { robot: 'RobotSession', 'robot.arm': 'MoveIt2', 'robot.gripper': 'GripperInterface', 'robot.servo': 'MoveIt2Servo', 'robot.description': 'RobotDescription' };
const sceneMethods = new Set(['remove_collision_object', 'attach_collision_object', 'detach_collision_object', 'detach_all_collision_objects', 'move_collision', 'allow_collisions', 'process_allow_collision_future', 'clear_all_collision_objects', 'process_clear_all_collision_objects_future', 'cancel_clear_all_collision_objects_future', 'update_planning_scene']);
const opaque = (cls, repr) => ({ __class__: 'opaque', repr: repr ?? `<pymoveit2.${cls} object>` });
const pythonType = v => v === null ? 'NoneType' : Array.isArray(v) ? 'list' : ({ boolean: 'bool', number: 'float', string: 'str', object: 'dict' }[typeof v] ?? typeof v);
const truth = value => value == null ? false : Array.isArray(value) || typeof value === 'string' ? value.length > 0 : typeof value === 'object' ? !!value.__class__ || Object.keys(value).length > 0 : !!value;

function numericCast(value, integer = false) {
  const type = integer ? 'int' : 'float';
  if (!['string', 'number', 'boolean'].includes(typeof value)) throw new PyError('TypeError', `${type}() argument must be a string${integer ? ', a bytes-like object or a real number' : ' or a real number'}, not '${pythonType(value)}'`);
  const result = typeof value === 'string' && (!value.trim() || (integer && !/^[+-]?\d+$/.test(value.trim()))) ? NaN : Number(value);
  if (Number.isNaN(result) && !(typeof value === 'number' || (!integer && typeof value === 'string' && /^[+-]?nan$/i.test(value)))) valueError(integer ? `invalid literal for int() with base 10: ${pyRepr(value)}` : `could not convert string to float: ${pyRepr(value)}`);
  return integer ? Math.trunc(result) : result;
}

// RobotDescription's default arm state names, in order of preference.
const ARM_STATE_NAMES = ['ready', 'home', 'default', 'start', 'rest', 'up', 'zero'];

function checkGroup(m, name) {
  if (!m.groups[name]) valueError(`Unknown group '${name}' (known: ${pyRepr(Object.keys(m.groups).sort())}).`);
}

/** The joints a group's states set: the gripper's states may skip its mimic joint. */
const stateJoints = (m, group) => group === m.gripper.group_name ? m.gripper.joint_names : m.groups[group].joint_names;

function namedPositions(s, state_name, group_name) {
  const m = s.sim.model, name = group_name ?? m.group_name;
  checkGroup(m, name);
  const states = m.named_states[name] ?? {}, known = Object.keys(states);
  let state = state_name;
  if (state == null && known.length === 1) state = known[0];
  if (state == null) state = ARM_STATE_NAMES.find(n => Object.hasOwn(states, n));
  if (state == null) valueError(known.length ? `Group '${name}' defines no default state (known: ${pyRepr(known.sort())}), pass the state name explicitly.` : `The SRDF defines no state for group '${name}'.`);
  if (!Object.hasOwn(states, state)) valueError(`Group '${name}' has no state '${state}' (known: ${pyRepr(known.sort())}).`);
  return [...states[state]];
}

function* descriptionOperation(s, method, a) {
  const m = s.sim.model;
  if (method === 'joint_positions') return namedPositions(s, a.state_name, a.group_name);
  if (method === 'moveit2_kwargs') {
    const group = a.group_name ?? m.group_name;
    checkGroup(m, group);
    const g = m.groups[group];
    return { joint_names: [...g.joint_names], base_link_name: g.base_link_name, end_effector_name: g.end_effector_name, group_name: group };
  }
  if (method === 'moveit2_gripper_kwargs') {
    const group = a.group_name ?? m.gripper.group_name;
    return { gripper_joint_names: [...m.gripper.joint_names], open_gripper_joint_positions: namedPositions(s, a.open_state ?? 'open', group), closed_gripper_joint_positions: namedPositions(s, a.closed_state ?? 'close', group), gripper_group_name: group };
  }
  if (method === 'group') {
    const group = s.getattr('robot.description.groups')[a.group_name];
    if (!group) checkGroup(m, a.group_name);
    return group;
  }
  valueError(`The browser demo does not implement ${method}().`);
}

function* operation(s, path, args, kwargs, call) {
  if (path === 'time.sleep') {
    const seconds = finiteFloat(args[0] ?? kwargs.seconds, 'seconds', 0), until = s.sim.time + seconds;
    yield { until: () => s.sim.time >= until - 1e-10, timeout: seconds }; return null;
  }
  const spec = resolveSpec(path);
  if (!spec?.params) throw s.attributeError(path);
  const method = classes[path] ? '__call__' : path.split('.').at(-1);
  const a = bindArguments(method === '__call__' ? '__call__' : path, spec, args, kwargs);
  if (path === 'connect') return opaque('session.RobotSession');
  if (path.startsWith('robot.servo')) { s.ensureServo(call); return yield* servoOperation(s, method, a, call); }
  if (path.startsWith('robot.gripper')) return yield* gripperOperation(s, method, a, call);
  if (path.startsWith('robot.description.')) return yield* descriptionOperation(s, method, a);
  if (path === 'robot.joint_positions') return namedPositions(s, a.state_name, null);
  if (path === 'robot.close') {
    s.sim.cancel('arm'); s.sim.cancel('gripper');
    if (s.servoInitialized) yield* servoOperation(s, 'shutdown', { timeout_sec: 1 }, call);
    s.closed = true; return true;
  }
  if (['robot.move_to_pose', 'robot.move_to_configuration'].includes(path)) {
    const forwarded = { ...a.kwargs, timeout_sec: a.timeout_sec };
    if (method === 'move_to_pose') { forwarded.position = a.position; forwarded.quat_xyzw = a.quat_xyzw; }
    else forwarded.joint_positions = a.joint_positions ?? namedPositions(s, null, null);
    const armSpec = SIGNATURES.MoveIt2.methods[method];
    const armArgs = bindArguments(`MoveIt2.${method}`, armSpec, [], forwarded);
    const submitted = yield* armOperation(s, method, armArgs, call);
    if (!submitted || !a.wait) return submitted;
    return yield* armOperation(s, 'wait_until_executed', { timeout_sec: a.timeout_sec }, call);
  }
  if (sceneMethods.has(method) || method.startsWith('add_collision_')) return yield* sceneOperation(s, method, a, call);
  if (method === 'destroy') { s.sim.cancel('arm'); s.closed = true; return null; }
  return yield* armOperation(s, method, a, call);
}

/** Session wrapper matching the public Python object paths and controller behavior. */
export class Session extends Events {
  constructor(sim, { driver = 'live' } = {}) {
    super(); checkDriver(driver); this.sim = sim; this.driver = driver; this.pending = new Set(); this.closed = false;
    this.nextCallId = 1; this.nextFutureId = 1; this.futures = new Map(); this.shadows = {}; this.executionResults = new Map();
    this.config = { max_velocity: 0, max_acceleration: 0, num_planning_attempts: 5, allowed_planning_time: 0.5, pipeline_id: '', planner_id: '', cartesian_avoid_collisions: true, cartesian_jump_threshold: 0, cartesian_prismatic_jump_threshold: 0, cartesian_revolute_jump_threshold: 0 };
    this.servoConfig = { frame_id: sim.model.base_link_name, linear_speed: 1, angular_speed: 1 }; this.servoInitialized = false; this.servoClosed = false;
    this.goals = [{}]; this.pathConstraints = { position: [], orientation: [], joint: [] }; this.planFailure = null;
    this.workspace = { min_corner: [-1, -1, -1], max_corner: [1, 1, 1], frame_id: sim.model.base_link_name };
    this.controllers = Object.fromEntries(['arm', 'gripper'].map(target => [target, { active: null, success: false, failure: null, error: null, reported: true }]));
    this.unsubscribeExecution = sim.on('execution', event => {
      const c = this.controllers[event.target];
      if (event.status === 'started') { c.active = event.id; c.success = false; c.failure = null; c.error = null; c.reported = false; }
      else {
        c.active = null; c.success = event.status === 'succeeded'; c.failure = describeErrorCode(event.error_code); c.error = event.error_code; c.reported = false;
        this.executionResults.set(event.id, c.success);
        if (this.executionResults.size > 500) this.executionResults.delete(this.executionResults.keys().next().value);
      }
    });
  }
  /** Raise AttributeError with the Python object's class name. */
  attributeError(path) {
    const parts = path.split('.'), owner = parts.slice(0, -1).join('.'), name = parts.at(-1);
    return new PyError('AttributeError', `'${classes[owner] ?? 'MoveIt2'}' object has no attribute '${name}'`);
  }
  /** Lazy RobotSession.servo construction uses MoveIt2Servo's enable-at-init default. */
  ensureServo(call) {
    if (this.servoInitialized) return;
    this.servoInitialized = true; this.sim.servoEnable();
    this.ros(call, 'service', '/servo_node/pause_servo', 'std_srvs/srv/SetBool', 'pause=False', 'SUCCESS');
  }
  /** Create an immediately completed opaque Python Future handle. */
  future(result) { const id = this.nextFutureId++; this.futures.set(id, structuredClone(result)); return { __class__: 'Future', id, done: true }; }
  /** Emit one ROS observation per transport request and associate it with its facade call. */
  ros(call, kind, name, type, summary, status, duration) {
    const ros = { kind, name, type, summary, time: this.sim.time, ...(status ? { status } : {}), ...(duration == null ? {} : { duration }) };
    if (call) call.ros.push(ros); this.emit('ros', ros); return ros;
  }
  /** Invoke a Python method; sync returns values, live always returns a Promise. */
  invoke(path, args = [], kwargs = {}) {
    const call = { id: this.nextCallId++, path, args: structuredClone(args), kwargs: structuredClone(kwargs), python: formatCall(path, args, kwargs), status: 'started', ros: [] };
    this.emit('call', structuredClone(call));
    const returned = result => { this.emit('call', { ...structuredClone(call), status: 'returned', result: structuredClone(result) }); return result; };
    const raised = error => {
      const e = error instanceof PyError ? error : new PyError('RuntimeError', error.message);
      if (e.type === 'ValueError' && ['move_to_pose', 'move_to_configuration', 'execute'].includes(path.split('.').at(-1))) {
        const c = this.controllers[path.startsWith('robot.gripper.') ? 'gripper' : 'arm'];
        if (!c.active) { c.success = false; c.failure = 'The goal did not succeed and the server gave no reason. Check the `move_group` log.'; c.error = null; c.reported = false; }
      }
      this.emit('call', { ...structuredClone(call), status: 'raised', error: e.toJSON() }); throw e;
    };
    const generator = operation(this, path, args, kwargs, call);
    if (this.driver === 'live') return runLive(this, generator).then(returned, raised);
    try { return returned(runSync(this.sim, generator)); } catch (error) { return raised(error); }
  }
  /** Read a Python property, forwarded arm property or session-local shadow attribute. */
  getattr(path) {
    if (Object.hasOwn(this.shadows, path)) return structuredClone(this.shadows[path]);
    if (classes[path]) { if (path === 'robot.servo') this.ensureServo(); return opaque(classes[path]); }
    const name = path.split('.').at(-1), model = this.sim.model;
    let owner = path.split('.').slice(0, -1).join('.');
    if (owner === 'robot') {
      if (name === 'node') return opaque('node', '<rclpy.node.Node object>');
      if (name === 'executor') return opaque('executor', '<rclpy.executors.MultiThreadedExecutor object>');
      owner = 'robot.arm';
    }
    if (resolveSpec(path)?.params) return { __class__: 'method', path };
    if (owner === 'robot.arm') {
      if (Object.hasOwn(this.config, name)) return this.config[name];
      const state = this.controllers.arm;
      const properties = { joint_names: [...model.joint_names], joint_state: jointState(model.joint_names, this.sim.q, this.sim.velocities), motion_succeeded: state.success, planning_scene: planningScene(this.sim), base_link_name: model.base_link_name, end_effector_name: model.end_effector_name, group_name: model.group_name, new_joint_state_available: true, planning_scene_cache_dirty: false, planning_scene_mutation_quarantined: false, workspace_frame_id: this.workspace.frame_id, current_operation: state.active ? opaque('ActionOperation') : null };
      if (Object.hasOwn(properties, name)) return properties[name];
    }
    if (owner === 'robot.gripper') {
      const tolerance = 0.1 * Math.abs(model.gripper.open_joint_positions[0] - model.gripper.closed_joint_positions[0]);
      const properties = { is_open: Math.abs(this.sim.fingers - model.gripper.open_joint_positions[0]) <= tolerance, is_closed: Math.abs(this.sim.fingers - model.gripper.closed_joint_positions[0]) <= tolerance, motion_succeeded: this.controllers.gripper.success, joint_names: [...model.gripper.joint_names], joint_state: jointState(model.gripper.joint_names, model.gripper.joint_names.map(() => this.sim.fingers)), backend: opaque('moveit2_gripper.MoveIt2Gripper'), interface: opaque('MoveIt2Gripper', "<class 'pymoveit2.moveit2_gripper.MoveIt2Gripper'>"), gripper_command_action_client: null, new_joint_state_available: true };
      if (Object.hasOwn(properties, name)) return properties[name];
    }
    if (owner === 'robot.servo') {
      this.ensureServo();
      if (Object.hasOwn(this.servoConfig, name)) return this.servoConfig[name];
      const properties = { is_enabled: this.sim.servo.enabled, active_command_type: this.sim.servo.active_command_type, uses_legacy_interface: false, remote_state_unknown: false, is_closed: this.servoClosed };
      if (Object.hasOwn(properties, name)) return properties[name];
    }
    if (owner === 'robot.description') {
      const groups = Object.fromEntries(Object.entries(model.groups).map(([group, g]) => [group, { __class__: 'MoveGroupDescription', name: group, joint_names: [...g.joint_names], passive_joint_names: [...g.passive_joint_names], base_link_name: g.base_link_name, end_effector_name: g.end_effector_name, named_states: Object.fromEntries(Object.entries(model.named_states[group] ?? {}).map(([state, q]) => [state, Object.fromEntries(stateJoints(model, group).map((joint, i) => [joint, q[i]]))])) }]));
      const properties = { name: model.name, arm_group_name: model.group_name, gripper_group_name: model.gripper.group_name, group_names: Object.keys(model.groups), groups };
      if (Object.hasOwn(properties, name)) return structuredClone(properties[name]);
    }
    throw this.attributeError(path);
  }
  /** Apply real Python setter conversions; RobotSession assignment creates a local attribute. */
  setattr(path, value) {
    const name = path.split('.').at(-1), owner = path.split('.').slice(0, -1).join('.');
    if (owner === 'robot' && !Object.hasOwn(SIGNATURES.RobotSession.properties, name)) {
      this.shadows[path] = structuredClone(value);
      if (resolveSpec(`robot.arm.${name}`)) this.emit('hint', { text: `Assigning robot.${name} stores an attribute on RobotSession. Set robot.arm.${name} to configure MoveIt2.` });
      return null;
    }
    const spec = resolveSpec(path);
    if (!spec) { this.shadows[path] = structuredClone(value); return null; }
    if (spec.readonly || spec.params) throw new PyError('AttributeError', `property '${name}' of '${classes[owner]}' object has no setter`);
    if (owner === 'robot.arm') {
      if (name === 'motion_succeeded') this.controllers.arm.success = truth(value);
      else if (['max_velocity', 'max_acceleration', 'allowed_planning_time', 'cartesian_jump_threshold', 'cartesian_prismatic_jump_threshold', 'cartesian_revolute_jump_threshold'].includes(name)) this.config[name] = numericCast(value);
      else if (name === 'num_planning_attempts') this.config[name] = numericCast(value, true);
      else if (name === 'cartesian_avoid_collisions') this.config[name] = truth(value);
      else this.config[name] = value;
      return null;
    }
    if (owner === 'robot.gripper' && name === 'motion_succeeded') { this.controllers.gripper.success = truth(value); return null; }
    if (owner === 'robot.servo') { this.ensureServo(); this.servoConfig[name] = name === 'frame_id' ? value : finiteFloat(value, name); return null; }
    throw this.attributeError(path);
  }
  /** dir/help/autocomplete records with Python signatures and docstrings. */
  members(path = 'robot') {
    const cls = classes[path]; if (!cls) return [];
    const entries = {};
    const add = table => {
      for (const [name, spec] of Object.entries(table.methods)) if (name !== '__init__') entries[name] = { name, kind: 'method', ...spec };
      for (const [name, spec] of Object.entries(table.properties)) entries[name] = { name, kind: 'property', ...spec };
    };
    if (path === 'robot') add(SIGNATURES.MoveIt2);
    add(SIGNATURES[cls]);
    for (const key of Object.keys(this.shadows)) if (key.split('.').slice(0, -1).join('.') === path) entries[key.split('.').at(-1)] = { name: key.split('.').at(-1), kind: 'property', type: 'any', readonly: false, doc: 'Session attribute.' };
    return Object.values(entries).sort((a, b) => a.name.localeCompare(b.name));
  }
  /** time.sleep semantics under the selected driver. */
  sleep(seconds) { return this.invoke('time.sleep', [seconds]); }
  /** Resolve all currently pending live waits as timed out. */
  interrupt() { for (const interrupt of [...this.pending]) interrupt(); }
}

/** Create the browser demo's Python-compatible session, using live or sync waits. */
export function createSession(sim, options = {}) { return new Session(sim, options); }
