// Seeded joint-space RRTConnect, constrained projection and Cartesian planning.
import { fk, linkTransforms } from './kinematics.js';
import { expandGoalAlternatives } from './goal-alternatives.js';
import { solveIK } from './ik.js';
import { checkState, primitiveDistance } from './collision.js';
import { timeParameterize, synchronizedTrapezoid, cartesianTrapezoid, sampleTrajectory } from './timing.js';
import { norm, sub, lerp, quatSlerp, quatMultiply, quatInverse, quatRotate, rotationVector, eulerXYZFromQuat, createRng, mat4TransformPoint, poseFromMat4 } from './math.js';

/** MoveIt orientation, spherical position and symmetric joint path constraints. */
export function satisfiesConstraints(model, q, constraints = {}) {
  if (constraints.invalid) return false;
  for (const c of constraints.orientation ?? []) {
    const actual = fk(model, q, c.link ?? model.end_effector_name).quat_xyzw;
    const difference = quatMultiply(quatInverse(c.quat_xyzw), actual);
    // MoveIt uses intrinsic XYZ, or a rotation vector expressed in frame_id.
    // Reference: moveit_core/kinematic_constraints/src/kinematic_constraint.cpp.
    const error = c.parameterization === 1 ? quatRotate(quatInverse(c.rotation_frame ?? [0, 0, 0, 1]), quatRotate(c.quat_xyzw, rotationVector(difference))) : eulerXYZFromQuat(difference);
    const tolerance = Array.isArray(c.tolerance) ? c.tolerance : Array(3).fill(c.tolerance ?? 0.001);
    if (c.parameterization !== 1 && Math.abs(Math.cos(error[1])) < 4 * Number.EPSILON && Math.abs(error[0]) > tolerance[2]) { error[2] = error[0]; error[0] = 0; }
    if (error.some((x, i) => Math.abs(x) > tolerance[i] + 1e-8)) return false;
  }
  for (const c of constraints.position ?? []) {
    const pose = fk(model, q, c.link ?? model.end_effector_name);
    const point = c.target_point_offset ? pose.position.map((x, i) => x + quatRotate(pose.quat_xyzw, c.target_point_offset)[i]) : pose.position;
    if (c.regions ? !c.regions.some(region => primitiveDistance(point, region) <= 1e-8) : norm(sub(point, c.position)) > c.tolerance + 1e-8) return false;
  }
  for (const c of constraints.joint ?? []) {
    const delta = q[model.joint_names.indexOf(c.joint_name)] - c.position;
    if (delta > (c.tolerance_above ?? c.tolerance) + 1e-8 || -delta > (c.tolerance_below ?? c.tolerance) + 1e-8) return false;
  }
  return true;
}

/** Plan a deterministic collision-checked trajectory and bounded visualization data. */
export function plan(model, request, ctx) {
  const alternatives = expandGoalAlternatives(request);
  if (alternatives) {
    let last, total = 0;
    for (const alternative of alternatives) {
      last = plan(model, alternative, ctx); total += last.planning_time;
      if (last.error_code === 'SUCCESS') break;
    }
    return { ...last, planning_time: Number(total.toFixed(6)) };
  }
  const start = [...(request.start ?? ctx.q)], fingers = ctx.fingers ?? model.gripper.open_joint_positions[0], objects = ctx.objects ?? [];
  const rng = ctx.rng ?? createRng(1), constraints = request.path_constraints ?? {};
  const goalRequirements = request.goal_requirements ?? {};
  const planner = request.cartesian ? 'Cartesian' : request.pipeline_id === 'pilz_industrial_motion_planner' ? request.planner_id || 'PTP' : 'RRTConnect';
  const tree = [], debug = { tree };
  let work = 0;
  const result = (error_code, extra = {}) => ({ error_code, planner, planning_time: Number((0.001 + work * 0.00001).toFixed(6)), debug, ...extra });
  const collision = q => { work++; return checkState(model, q, fingers, objects); };
  const limits = q => q.length === model.joints.length && q.every((x, i) => Number.isFinite(x) && x >= model.joints[i].lower - 1e-9 && x <= model.joints[i].upper + 1e-9);
  const valid = q => limits(q) && satisfiesConstraints(model, q, constraints) && collision(q).valid;
  if (request.group_name && request.group_name !== model.group_name) return result('INVALID_GROUP_NAME');
  if (!request.cartesian && request.pipeline_id && !['ompl', 'pilz_industrial_motion_planner'].includes(request.pipeline_id)) return result('PLANNING_FAILED', { detail: `Unknown planning pipeline '${request.pipeline_id}'.` });
  if (!request.cartesian && request.pipeline_id === 'pilz_industrial_motion_planner' && request.planner_id && !['PTP', 'LIN'].includes(request.planner_id)) return result('PLANNING_FAILED', { detail: `Unknown Pilz planner '${request.planner_id}'.` });
  if (!request.cartesian && request.pipeline_id !== 'pilz_industrial_motion_planner' && request.planner_id && !['RRTConnectkConfigDefault', 'RRTstarkConfigDefault'].includes(request.planner_id)) return result('PLANNING_FAILED', { detail: `Unknown OMPL planner '${request.planner_id}'.` });
  if (!limits(start)) return result('START_STATE_INVALID');
  if (request.pose_goal) {
    const goal = { ...request.pose_goal }, transforms = linkTransforms(model, start, fingers);
    if (goal.target_link && !transforms[goal.target_link]) return result('INVALID_LINK_NAME');
    if (goal.frame_id && goal.frame_id !== model.planning_frame) {
      const frame = transforms[goal.frame_id];
      if (!frame) return result('FRAME_TRANSFORM_FAILURE');
      if (goal.position) goal.position = mat4TransformPoint(frame, goal.position);
      if (goal.quat_xyzw) goal.quat_xyzw = quatMultiply(poseFromMat4(frame).quat_xyzw, goal.quat_xyzw);
      goal.frame_id = model.planning_frame;
    }
    request = { ...request, pose_goal: goal };
  }
  const initial = collision(start);
  if (!initial.valid && !(request.cartesian && request.cartesian_avoid_collisions === false)) { debug.contacts = initial.contacts; return result('START_STATE_IN_COLLISION'); }
  if (!satisfiesConstraints(model, start, constraints)) return result('START_STATE_VIOLATES_PATH_CONSTRAINTS');
  const project = q => {
    // Projection is for states outside the allowed path region. Re-solving a
    // state already inside that region can move it away from the requested goal.
    if (satisfiesConstraints(model, q, constraints)) return q;
    for (const c of constraints.orientation ?? []) {
      q = solveIK(model, { quat_xyzw: c.quat_xyzw }, { seed: q, target_link: c.link, restarts: 0, max_iterations: 12, tolerance_orientation: 0.00002 });
      if (!q) return null;
    }
    return limits(q) && satisfiesConstraints(model, q, constraints) ? q : null;
  };
  const constrained = (constraints.orientation ?? []).length > 0;
  const edge = (a, b, step = constrained ? 0.008 : 0.02) => {
    const n = Math.max(1, Math.ceil(norm(sub(b, a)) / step));
    for (let k = 1; k <= n; k++) if (!valid(lerp(a, b, k / n))) return false;
    return true;
  };
  const finish = (path, extra = {}, endpointGoals = null) => {
    const endpoint = path.at(-1);
    if (!satisfiesConstraints(model, endpoint, constraints) || !satisfiesConstraints(model, endpoint, goalRequirements)) return result('PLANNING_FAILED', { detail: 'Planned endpoint violates goal or path constraints.' });
    if (endpointGoals && !endpointGoals.some(goal => norm(sub(endpoint, goal)) <= 1e-8)) return result('PLANNING_FAILED', { detail: 'Planned endpoint does not match a requested goal state.' });
    debug.goal_state = [...path.at(-1)];
    const count = Math.min(400, path.length);
    debug.ee_path = Array.from({ length: count }, (_, i) => fk(model, path[Math.round(i * (path.length - 1) / Math.max(1, count - 1))]).position);
    const trajectory = planner === 'PTP' ? synchronizedTrapezoid(model, start, path.at(-1), request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor) : planner === 'LIN' ? cartesianTrapezoid(model, path, fk(model, start, request.pose_goal.target_link), fk(model, path.at(-1), request.pose_goal.target_link), request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor) : timeParameterize(model, path, request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor);
    // The executed Hermite curve, including corner rounding, must pass the same gates.
    for (let i = 1; i < trajectory.points.length; i++) {
      if (!(request.cartesian && request.cartesian_avoid_collisions === false) && !edge(trajectory.points[i - 1].positions, trajectory.points[i].positions, 0.008)) return result('PLANNING_FAILED', { detail: 'Time-parameterized path failed dense validity checks.' });
      if (!(request.cartesian && request.cartesian_avoid_collisions === false)) for (let k = 1; k < 5; k++) {
        const t = trajectory.points[i - 1].time_from_start + (trajectory.points[i].time_from_start - trajectory.points[i - 1].time_from_start) * k / 5;
        if (!valid(sampleTrajectory(trajectory, t).positions)) return result('PLANNING_FAILED', { detail: 'Interpolated trajectory failed dense validity checks.' });
      }
    }
    return result('SUCCESS', { trajectory, ...extra });
  };
  if (request.cartesian || planner === 'LIN') {
    if (!request.pose_goal) return result('INVALID_GOAL_CONSTRAINTS');
    const goal = request.pose_goal, link = goal.target_link ?? model.end_effector_name, first = fk(model, start, link);
    const end = { position: goal.position ?? first.position, quat_xyzw: goal.quat_xyzw ?? first.quat_xyzw };
    const maxStep = request.cartesian ? request.cartesian_max_step ?? 0.0025 : 0.0025;
    if (!(maxStep > 0)) return result('INVALID_GOAL_CONSTRAINTS');
    const angle = norm(rotationVector(quatMultiply(end.quat_xyzw, quatInverse(first.quat_xyzw))));
    const steps = Math.max(1, Math.ceil(norm(sub(end.position, first.position)) / maxStep), Math.ceil(angle / 0.025));
    const path = [start], distances = [];
    for (let i = 1; i <= steps; i++) {
      const pose = { position: lerp(first.position, end.position, i / steps), quat_xyzw: quatSlerp(first.quat_xyzw, end.quat_xyzw, i / steps) };
      const q = solveIK(model, pose, { seed: path.at(-1), target_link: link, rng, restarts: 0, tolerance_position: 0.00005, tolerance_orientation: 0.0001, position_only: model.position_only_ik });
      if (!q || !satisfiesConstraints(model, q, constraints)) break;
      if ((!request.cartesian || request.cartesian_avoid_collisions !== false) && !edge(path.at(-1), q)) break;
      distances.push(norm(sub(q, path.at(-1)))); path.push(q);
    }
    if (request.cartesian_jump_threshold > 0 && distances.length) {
      const mean = distances.reduce((a, b) => a + b, 0) / distances.length;
      const jump = distances.findIndex(d => d > request.cartesian_jump_threshold * mean);
      if (jump >= 0) path.splice(jump + 1);
    }
    const fraction = (path.length - 1) / steps;
    if (planner === 'LIN' && fraction < 1) return result('PLANNING_FAILED', { detail: 'LIN failed: inverse kinematics, path constraints or collision blocked the straight Cartesian line.' });
    if (fraction === 0) return result('SUCCESS', { fraction: 0, trajectory: { joint_names: [...model.joint_names], points: [] } });
    return finish(path, request.cartesian ? { fraction } : {});
  }
  let goals = [];
  if (request.joint_goal) {
    const { positions, joint_names: names } = request.joint_goal;
    let goal = [...positions];
    if (names) {
      if (names.length !== positions.length || new Set(names).size !== names.length || names.some(n => !model.joint_names.includes(n))) return result('INVALID_GOAL_CONSTRAINTS');
      goal = [...start]; names.forEach((n, i) => { goal[model.joint_names.indexOf(n)] = positions[i]; });
    }
    // MoveIt's GOAL_STATE_INVALID is used for a well-formed but out-of-bounds state.
    if (!limits(goal)) return result('GOAL_STATE_INVALID');
    const c = collision(goal);
    if (!c.valid) { debug.contacts = c.contacts; return result('GOAL_IN_COLLISION'); }
    goals.push(goal);
  } else if (request.pose_goal) {
    let anyIK = false;
    // With position_only_ik the IK sampler ignores the goal orientation, so the
    // sampled state must still meet it within tolerance_orientation.
    const goal = request.pose_goal, free = model.position_only_ik && goal.position && goal.quat_xyzw;
    const orientation = free ? { orientation: [{ quat_xyzw: goal.quat_xyzw, link: goal.target_link, tolerance: goal.tolerance_orientation ?? 0.001, parameterization: 0 }] } : {};
    let missed = false;
    const sampled = candidate => {
      if (!satisfiesConstraints(model, candidate, goalRequirements)) return false;
      if (satisfiesConstraints(model, candidate, orientation)) return true;
      missed = true; return false;
    };
    for (let i = 0; i < 5; i++) {
      const seed = i === 0 ? start : model.joints.map(j => rng.uniform(j.lower, j.upper));
      const q = solveIK(model, goal, { seed, rng, target_link: goal.target_link, restarts: i === 0 ? 6 : 1, position_only: model.position_only_ik, joint_constraints: goalRequirements.joint, isValid: sampled });
      if (!q) continue;
      anyIK = true;
      const c = collision(q);
      if (c.valid) goals.push(q); else debug.contacts = c.contacts;
    }
    if (!goals.length) return result(anyIK ? 'GOAL_IN_COLLISION' : 'NO_IK_SOLUTION', missed && !anyIK ? { detail: 'The kinematics solver reaches the position but not the orientation: it solves position only (position_only_ik), so free the orientation with tolerance_orientation=3.1416' } : {});
    goals.sort((a, b) => norm(sub(a, start)) - norm(sub(b, start)));
  } else return result('INVALID_GOAL_CONSTRAINTS');
  goals = goals.filter(q => satisfiesConstraints(model, q, goalRequirements));
  if (!goals.length) return result('PLANNING_FAILED', { detail: 'The conjunctive goal constraints have no sampled solution.' });
  goals = goals.filter(q => satisfiesConstraints(model, q, constraints));
  if (!goals.length) return result('GOAL_VIOLATES_PATH_CONSTRAINTS');
  const interpolatePath = (a, b) => {
    const path = [a], n = Math.max(1, Math.ceil(norm(sub(b, a)) / (constrained ? 0.025 : 0.1)));
    for (let k = 1; k <= n; k++) {
      const q = constrained ? project(lerp(a, b, k / n)) : lerp(a, b, k / n);
      if (!q || !edge(path.at(-1), q)) return null;
      path.push(q);
    }
    return path;
  };
  for (const goal of goals) {
    const direct = interpolatePath(start, goal);
    if (direct) return finish(direct, {}, goals);
  }
  if (planner === 'PTP') return result('PLANNING_FAILED', { detail: 'PTP joint path collides or violates path constraints.' });
  let a = [{ q: start, parent: -1 }], b = goals.map(q => ({ q, parent: -1 })), swapped = false;
  const trace = (nodes, index) => { const p = []; while (index >= 0) { p.push(nodes[index].q); index = nodes[index].parent; } return p.reverse(); };
  const extend = (nodes, target) => {
    let index = 0, distance = Infinity;
    nodes.forEach((node, i) => { const d = norm(sub(node.q, target)); if (d < distance) { index = i; distance = d; } });
    if (distance < 1e-8) return { index, reached: true };
    let q = lerp(nodes[index].q, target, Math.min(1, 0.12 / distance));
    if (constrained) q = project(q);
    if (!q || norm(sub(q, nodes[index].q)) < 1e-7 || !edge(nodes[index].q, q)) return null;
    if (tree.length < 2000) tree.push([fk(model, nodes[index].q).position, fk(model, q).position]);
    nodes.push({ q, parent: index });
    return { index: nodes.length - 1, reached: norm(sub(q, target)) < 0.005 };
  };
  const budget = Math.min(12000, Math.max(300, Math.floor((request.allowed_planning_time ?? 2.5) * (request.num_planning_attempts || 1) * 1000)));
  for (let iter = 0; iter < budget; iter++) {
    const target = iter % 8 === 0 ? b[0].q : model.joints.map(j => rng.uniform(j.lower, j.upper));
    const grown = extend(a, target);
    if (grown) {
      let connected = null;
      for (let k = 0; k < 100; k++) { connected = extend(b, a[grown.index].q); if (!connected || connected.reached) break; }
      if (connected?.reached && edge(a[grown.index].q, b[connected.index].q)) {
        let path = [...trace(a, grown.index), ...trace(b, connected.index).reverse()];
        if (swapped) path.reverse();
        for (let k = 0; k < (request.planner_id === 'RRTstarkConfigDefault' ? 160 : 60); k++) {
          let i = Math.floor(rng.next() * path.length), j = Math.floor(rng.next() * path.length);
          if (i > j) [i, j] = [j, i];
          if (j <= i + 1) continue;
          const shortcut = interpolatePath(path[i], path[j]);
          if (shortcut) path.splice(i, j - i + 1, ...shortcut);
        }
        return finish(path, {}, goals);
      }
    }
    [a, b] = [b, a]; swapped = !swapped;
  }
  return result('PLANNING_FAILED', { detail: 'RRTConnect exhausted the deterministic planning budget.' });
}
