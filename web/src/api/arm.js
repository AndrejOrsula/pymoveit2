// Generator implementations of planning, kinematics and arm lifecycle methods.
import { describeErrorCode } from './diagnostics.js';
import { poseGoal, jointGoal, finiteFloat, finiteVector, orientationTolerance, statePositions, poseStamped, jointState, pyTrajectory, coreTrajectory, valueError } from './values.js';
import { linkTransforms } from '../core/kinematics.js';
import { checkState } from '../core/collision.js';
import { satisfiesConstraints } from '../core/planning.js';
import { mergeGoalConstraints, canonicalGoalConstraints } from '../core/goal-alternatives.js';
import { mat4FromPose, mat4InvertRigid, mat4Multiply, mat4TransformPoint, poseFromMat4, quatMultiply } from '../core/math.js';

function inPlanningFrame(s, goal) {
  if (goal.frame_id === s.sim.model.planning_frame) return goal;
  const transform = linkTransforms(s.sim.model, s.sim.q, s.sim.fingers)[goal.frame_id];
  if (!transform) return goal;
  return { ...goal, ...(goal.position ? { position: mat4TransformPoint(transform, goal.position) } : {}), ...(goal.quat_xyzw ? { quat_xyzw: quatMultiply(poseFromMat4(transform).quat_xyzw, goal.quat_xyzw) } : {}), frame_id: s.sim.model.planning_frame };
}

// Constraint messages become the planner's constraints; exported for their unit tests.
export function jointConstraint(joint) {
  const above = joint.tolerance_above ?? 0, below = joint.tolerance_below ?? 0;
  return { joint_name: joint.joint_name, position: joint.position, tolerance: Math.min(above, below), tolerance_above: above, tolerance_below: below };
}

export function orientationConstraint(orientation, frame) {
  const rotation_frame = poseFromMat4(frame).quat_xyzw;
  const tolerance = ['x', 'y', 'z'].map(axis => orientation[`absolute_${axis}_axis_tolerance`] ?? 0);
  return { quat_xyzw: quatMultiply(rotation_frame, finiteVector(orientation.orientation, 'quat_xyzw', 4)), rotation_frame, link: orientation.link_name, tolerance, parameterization: orientation.parameterization ?? 0 };
}

const PRIMITIVE_TYPES = { 1: 'box', 2: 'sphere', 3: 'cylinder', 4: 'cone' };

export function positionConstraint(position, frame) {
  const primitives = position.constraint_region?.primitives ?? [], poses = position.constraint_region?.primitive_poses ?? [];
  const regions = primitives.flatMap((primitive, i) => {
    if (!poses[i]) return [];
    const local = { position: finiteVector(poses[i].position, 'position', 3), quat_xyzw: finiteVector(poses[i].orientation ?? [0, 0, 0, 1], 'quat_xyzw', 4) };
    return [{ type: PRIMITIVE_TYPES[primitive.type], dimensions: primitive.dimensions, pose: poseFromMat4(mat4Multiply(frame, mat4FromPose(local))) }];
  });
  return { link: position.link_name, regions, target_point_offset: finiteVector(position.target_point_offset ?? [0, 0, 0], 'target_point_offset', 3) };
}

const LINK_CONSTRAINTS = [['orientation', orientationConstraint], ['position', positionConstraint]];

/** Converts one kind of link constraint; null when a frame is unknown. */
function linkConstraints(messages, convert, transforms, base) {
  const converted = [];
  for (const constraint of messages) {
    const frame = transforms[constraint.header?.frame_id || base];
    if (!frame) return null;
    // MoveIt ignores a constraint on an unknown link: configure() fails and decide() passes.
    if (transforms[constraint.link_name]) converted.push(convert(constraint, frame));
  }
  return converted;
}

function ikConstraints(s, message) {
  if (!message) return { position: [], orientation: [], joint: [] };
  if (message.position || message.orientation || message.joint) return structuredClone(message);
  const m = s.sim.model, transforms = linkTransforms(m, s.sim.q, s.sim.fingers);
  const c = { joint: (message.joint_constraints ?? []).map(jointConstraint) };
  for (const [kind, convert] of LINK_CONSTRAINTS) {
    c[kind] = linkConstraints(message[`${kind}_constraints`] ?? [], convert, transforms, m.base_link_name);
    if (!c[kind]) return { invalid: true };
  }
  return c;
}

function requestFor(s, a) {
  const model = s.sim.model;
  finiteFloat(a.cartesian_max_step ?? 0.0025, 'cartesian_max_step', 0);
  if (a.cartesian && a.cartesian_max_step === 0) valueError('`cartesian_max_step` must be positive.');
  // pymoveit2 snapshots the move goal, then clears goal and path constraints for the next plan.
  const goals = s.goals.filter(g => g.joint_goal || g.pose_goal), pathConstraints = s.pathConstraints;
  s.goals = [{}]; s.pathConstraints = { position: [], orientation: [], joint: [] };
  finiteFloat(a.weight_position ?? 1, 'weight', 0);
  finiteFloat(a.weight_orientation ?? 1, 'weight', 0);
  let goal = structuredClone(goals.at(-1) ?? {});
  const explicit = a.joint_positions != null || a.pose != null || a.position != null || a.quat_xyzw != null;
  if (a.joint_positions != null) goal = mergeGoalConstraints(goal, { joint_goal: jointGoal(a, model.joint_names) });
  if (a.pose != null || a.position != null || a.quat_xyzw != null) goal = mergeGoalConstraints(goal, { pose_goal: inPlanningFrame(s, poseGoal(a, model)) });
  if (!goal.joint_goal && !goal.pose_goal) valueError('Planning requires a goal constraint.');
  if (a.cartesian && (!goal.pose_goal?.position || !goal.pose_goal?.quat_xyzw)) valueError('Cartesian planning requires a pose goal (position and orientation); joint-space goals cannot be planned with `cartesian=True`.');
  return {
    ...canonicalGoalConstraints(goal), ...(!explicit && goals.length > 1 ? { goal_alternatives: goals.map(canonicalGoalConstraints) } : {}), start: statePositions(a.start_joint_state, model.joint_names, s.sim.q), path_constraints: pathConstraints, group_name: model.group_name,
    pipeline_id: s.config.pipeline_id, planner_id: s.config.planner_id, allowed_planning_time: s.config.allowed_planning_time, num_planning_attempts: s.config.num_planning_attempts,
    max_velocity_scaling_factor: s.config.max_velocity, max_acceleration_scaling_factor: s.config.max_acceleration,
    cartesian: a.cartesian ?? false, cartesian_max_step: a.cartesian_max_step ?? 0.0025, cartesian_jump_threshold: s.config.cartesian_jump_threshold, cartesian_avoid_collisions: s.config.cartesian_avoid_collisions,
  };
}

function plannedTrajectory(s, result, cartesian, threshold) {
  if (!result) { s.planFailure = 'the planner returned no response'; return null; }
  if (result.error_code !== 'SUCCESS') { s.planFailure = describeErrorCode(result.error_code); return null; }
  if (cartesian && result.fraction < threshold) {
    const f = n => Number.isInteger(n) ? n.toFixed(1) : String(n);
    s.planFailure = `the Cartesian planner reached only ${f(result.fraction)} of the path, short of the threshold ${f(threshold)}; lower \`cartesian_fraction_threshold\`, or move the goal closer so a straight path exists`;
    return null;
  }
  s.planFailure = null; return pyTrajectory(result.trajectory);
}

/** Arm method bodies, shared by both drivers and RobotSession's wrappers. */
export function* armOperation(s, method, a, call) {
  const sim = s.sim, model = sim.model, controller = s.controllers.arm;
  if (s.closed && !['last_failure', 'query_state'].includes(method)) return ['move_to_pose', 'move_to_configuration', 'execute', 'wait_until_executed', 'cancel_execution'].includes(method) ? false : null;
  switch (method) {
    case 'move_to_pose':
    case 'move_to_configuration': {
      if (method === 'move_to_pose') poseGoal(a, model, true);
      const planArgs = { ...a, ...(method === 'move_to_configuration' ? { tolerance_joint_position: a.tolerance, weight_joint_position: a.weight } : {}) };
      const trajectory = yield* armOperation(s, 'plan', planArgs, call);
      return yield* armOperation(s, 'execute', { joint_trajectory: trajectory }, call);
    }
    case 'plan':
    case 'plan_async': {
      const timeout = a.timeout_sec == null ? null : finiteFloat(a.timeout_sec, 'timeout_sec');
      const request = requestFor(s, a), goal = request.pose_goal;
      if (timeout !== null && timeout <= 0) { s.planFailure = 'planning timed out; raise `timeout_sec`, or `allowed_planning_time` when the planner itself needs longer'; return null; }
      let result;
      if (goal && !linkTransforms(model, sim.q)[goal.target_link]) result = { error_code: 'INVALID_LINK_NAME' };
      else if (goal && goal.frame_id !== model.planning_frame) result = { error_code: 'FRAME_TRANSFORM_FAILURE' };
      else result = sim.plan(request);
      s.ros(call, 'service', request.cartesian ? '/compute_cartesian_path' : '/plan_kinematic_path', request.cartesian ? 'moveit_msgs/srv/GetCartesianPath' : 'moveit_msgs/srv/GetMotionPlan', `${request.joint_goal ? 'joint' : 'pose'} goal, ${result.planner ?? 'planner'}: ${result.error_code}`, result.error_code, result.planning_time);
      if (method === 'plan_async') return s.future(result);
      return plannedTrajectory(s, result, request.cartesian, a.cartesian_fraction_threshold ?? 0);
    }
    case 'get_trajectory': return plannedTrajectory(s, s.futures.get(a.future?.id), a.cartesian, a.cartesian_fraction_threshold);
    case 'execute': {
      const trajectory = coreTrajectory(a.joint_trajectory);
      if (!trajectory) {
        if (controller.active) return false;
        controller.failure = s.planFailure ? s.planFailure[0].toUpperCase() + s.planFailure.slice(1).replace(/\.+$/, '') + '.' : 'There is no trajectory to execute.';
        controller.success = false; controller.error = null; controller.reported = false; return false;
      }
      s.ros(call, 'action', '/execute_trajectory', 'moveit_msgs/action/ExecuteTrajectory', 'Execute planned arm trajectory', 'ACCEPTED');
      sim.execute(trajectory, 'arm'); return true;
    }
    case 'wait_until_executed': return yield* waitForController(s, 'arm', a);
    case 'cancel_execution': {
      if (!sim.isBusy('arm')) return false;
      s.ros(call, 'service', '/execute_trajectory/_action/cancel_goal', 'action_msgs/srv/CancelGoal', 'Cancel tracked arm goal');
      return sim.cancel('arm');
    }
    case 'stop_all_trajectory_execution': s.ros(call, 'topic', '/trajectory_execution_event', 'std_msgs/msg/String', 'stop'); sim.cancel('arm'); sim.cancel('gripper'); return null;
    case 'last_failure': return controller.failure;
    case 'get_last_execution_error_code': return controller.error ? { __class__: 'MoveItErrorCodes', val: errorValue(controller.error), message: '', source: '' } : null;
    case 'query_state': return stateEnum(sim.isBusy('arm') ? 'EXECUTING' : 'IDLE');
    case 'get_execution_future': return controller.active ? s.future({ execution_id: controller.active }) : null;
    case 'compute_fk':
    case 'compute_fk_async': {
      const q = statePositions(a.joint_state, model.joint_names, sim.q), links = a.fk_link_names ?? [model.end_effector_name];
      const transforms = linkTransforms(model, q, sim.fingers);
      let result = null;
      if (links.every(link => transforms[link])) {
        // pymoveit2 asks for poses in base_link_name, at the requested state.
        const base = mat4InvertRigid(transforms[model.base_link_name]);
        const poses = links.map(link => poseStamped(poseFromMat4(mat4Multiply(base, transforms[link])), model.base_link_name, sim.time));
        result = a.fk_link_names == null ? poses[0] ?? null : poses;
      }
      s.ros(call, 'service', '/compute_fk', 'moveit_msgs/srv/GetPositionFK', `Forward kinematics for ${links.join(', ')}`);
      return method.endsWith('_async') ? s.future(result) : result;
    }
    case 'get_compute_fk_result':
    case 'get_compute_ik_result': return s.futures.get(a.future?.id) ?? null;
    case 'compute_ik':
    case 'compute_ik_async': {
      const seed = statePositions(a.start_joint_state, model.joint_names, sim.q);
      const constraints = ikConstraints(s, a.constraints);
      // pymoveit2 stamps the pose with base_link_name; MoveIt places that frame at the seed state.
      const base = linkTransforms(model, seed, sim.fingers)[model.base_link_name];
      const target = { position: mat4TransformPoint(base, finiteVector(a.position, 'position', 3)), quat_xyzw: quatMultiply(poseFromMat4(base).quat_xyzw, finiteVector(a.quat_xyzw, 'quat_xyzw', 4)) };
      const q = sim.computeIK(target, { seed, target_link: a.ik_link_name ?? model.end_effector_name, joint_constraints: constraints.joint, isValid: candidate => checkState(model, candidate, sim.fingers, sim.getObjects()).valid && satisfiesConstraints(model, candidate, constraints) });
      const result = q ? jointState(model.joint_names, q) : null;
      s.ros(call, 'service', '/compute_ik', 'moveit_msgs/srv/GetPositionIK', `Collision-aware inverse kinematics: ${q ? 'SUCCESS' : 'NO_IK_SOLUTION'}`);
      return method.endsWith('_async') ? s.future(result) : result;
    }
    case 'clear_goal_constraints': s.goals = [{}]; return null;
    case 'create_new_goal_constraint': s.goals.push({}); return null;
    case 'clear_path_constraints': s.pathConstraints = { position: [], orientation: [], joint: [] }; return null;
    case 'set_pose_goal': s.goals[s.goals.length - 1] = mergeGoalConstraints(s.goals.at(-1), { pose_goal: inPlanningFrame(s, poseGoal(a, model, true)) }); return null;
    case 'set_position_goal':
    case 'set_orientation_goal': {
      // `tolerance` is a float for position goals and a float or triplet for orientation goals.
      const tolerance = method === 'set_position_goal' ? { tolerance_position: a.tolerance } : { tolerance_orientation: a.tolerance };
      const goal = inPlanningFrame(s, poseGoal({ ...a, tolerance: undefined, ...tolerance }, model));
      s.goals[s.goals.length - 1] = mergeGoalConstraints(s.goals.at(-1), { pose_goal: goal }); return null;
    }
    case 'set_joint_goal': s.goals[s.goals.length - 1] = mergeGoalConstraints(s.goals.at(-1), { joint_goal: jointGoal(a, model.joint_names) }); return null;
    case 'set_path_position_constraint': {
      const goal = inPlanningFrame(s, poseGoal(a, model));
      s.pathConstraints.position.push({ position: goal.position, link: goal.target_link, tolerance: finiteFloat(a.tolerance, 'tolerance', 0) }); return null;
    }
    case 'set_path_orientation_constraint': {
      const goal = inPlanningFrame(s, poseGoal(a, model));
      s.pathConstraints.orientation.push({ quat_xyzw: goal.quat_xyzw, link: goal.target_link, tolerance: orientationTolerance(a.tolerance), parameterization: a.parameterization }); return null;
    }
    case 'set_path_joint_constraint': {
      const goal = jointGoal(a, model.joint_names);
      goal.positions.forEach((position, i) => s.pathConstraints.joint.push({ joint_name: goal.joint_names[i], position, tolerance: goal.tolerance })); return null;
    }
    case 'set_workspace_parameters': {
      const min = finiteVector(a.min_corner, 'min_corner'), max = finiteVector(a.max_corner, 'max_corner');
      if (min.length !== 3 || max.length !== 3) valueError('Workspace corners must contain exactly three values!');
      s.workspace = { min_corner: min, max_corner: max, frame_id: a.frame_id ?? model.base_link_name }; return null;
    }
    case 'wait_for_joint_state': return true;
    case 'reset_new_joint_state_checker': return null;
    case 'force_reset_executing_state': sim.cancel('arm'); controller.failure = null; return null;
    case 'reset_controller': {
      const q = statePositions(a.joint_state, model.joint_names, sim.q);
      sim.cancel('arm'); sim.q = q; controller.success = true; controller.reported = false;
      s.ros(call, 'action', '/execute_trajectory', 'moveit_msgs/action/ExecuteTrajectory', 'Reset controller with a dummy joint trajectory'); return true;
    }
  }
  // Builder helpers return ROS-shaped constraints, reusing the same validation.
  if (method === 'create_joint_constraints') {
    const g = jointGoal(a, model.joint_names);
    return g.positions.map((position, i) => ({ __class__: 'JointConstraint', joint_name: g.joint_names[i], position, tolerance_above: g.tolerance, tolerance_below: g.tolerance, weight: a.weight }));
  }
  if (method === 'create_position_constraint' || method === 'create_orientation_constraint') {
    const goal = poseGoal(a, model);
    return { __class__: method === 'create_position_constraint' ? 'PositionConstraint' : 'OrientationConstraint', header: { frame_id: goal.frame_id }, link_name: goal.target_link, ...(goal.position ? { constraint_region: { primitives: [{ type: 2, dimensions: [a.tolerance] }], primitive_poses: [{ position: Object.fromEntries(['x', 'y', 'z'].map((k, i) => [k, goal.position[i]])), orientation: { x: 0, y: 0, z: 0, w: 1 } }] }, target_point_offset: { x: 0, y: 0, z: 0 } } : { orientation: Object.fromEntries(['x', 'y', 'z', 'w'].map((k, i) => [k, goal.quat_xyzw[i]])), ...Object.fromEntries(['x', 'y', 'z'].map((axis, i) => [`absolute_${axis}_axis_tolerance`, Array.isArray(a.tolerance) ? a.tolerance[i] : a.tolerance])), parameterization: a.parameterization }), weight: a.weight };
  }
  valueError(`The browser demo does not implement ${method}().`);
}

/** Block once for a tracked operation, preserving timeout and cancel-on-timeout behavior. */
export function* waitForController(s, target, a) {
  const controller = s.controllers[target], active = controller.active;
  if (!active && controller.reported) return false;
  if (active) {
    const finished = yield { until: () => !s.sim.isBusy(target) || controller.active !== active, timeout: a.timeout_sec ?? null };
    if (!finished) { if (a.cancel_on_timeout) s.sim.cancel(target); return false; }
    if (controller.active && controller.active !== active) return s.executionResults.get(active) ?? false;
    if (s.executionResults.has(active)) { controller.reported = true; return s.executionResults.get(active); }
  }
  controller.reported = true; return controller.success;
}

/** Python MoveIt2State enum observation. */
export const stateEnum = name => ({ __class__: 'MoveIt2State', name, value: { IDLE: 0, REQUESTING: 1, EXECUTING: 2 }[name] });

/** Numeric MoveIt message values for simulated controller results. */
export const errorValue = name => ({ SUCCESS: 1, PLANNING_FAILED: -1, INVALID_MOTION_PLAN: -2, MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: -3, CONTROL_FAILED: -4, TIMED_OUT: -6, PREEMPTED: -7, NO_IK_SOLUTION: -31 }[name] ?? 99999);
