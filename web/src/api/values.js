// Validation and ROS-shaped Python values at the facade boundary.
import { PyError } from './errors.js';

/** Raise a Python ValueError. */
export function valueError(message) { throw new PyError('ValueError', message); }
/** Match _validation.finite_float, including rejection of Python booleans. */
export function finiteFloat(value, name, minimum = null) {
  if (typeof value !== 'number') valueError(`\`${name}\` must be a finite number.`);
  if (!Number.isFinite(value) || (minimum !== null && value < minimum)) valueError(`\`${name}\` must be finite${minimum === null ? '' : ` greater than or equal to ${Number.isInteger(minimum) ? minimum.toFixed(1) : minimum}`}.`);
  return value;
}
/** Match finite_vector; also accept Point and Quaternion message fields. */
export function finiteVector(values, name, length = null) {
  if (!Array.isArray(values) && values && typeof values === 'object' && 'x' in values) values = ['x', 'y', 'z', ...(length === 4 ? ['w'] : [])].map(k => values[k]);
  if (!Array.isArray(values)) valueError(`\`${name}\` must be a sequence of finite numbers.`);
  const result = values.map(value => finiteFloat(value, name));
  if (length !== null && result.length !== length) valueError(`\`${name}\` must contain exactly ${length} values.`);
  return result;
}
/** Validate configured or explicit joint names. */
export function jointNames(values, name = 'joint_names') {
  if (!Array.isArray(values)) valueError(`\`${name}\` must be a nonempty sequence of joint names.`);
  if (!values.length || values.some(v => typeof v !== 'string' || !v)) valueError(`\`${name}\` must contain nonempty strings.`);
  if (new Set(values).size !== values.length) valueError(`\`${name}\` must not contain duplicates.`);
  return [...values];
}
/** Construct a JointState observation. */
export const jointState = (name, position, velocity = []) => ({ __class__: 'JointState', name: [...name], position: [...position], velocity: [...velocity], effort: [] });
/** Extract an ordered start state, preserving JointState name semantics. */
export function statePositions(value, names, fallback) {
  if (value == null) return [...fallback];
  if (Array.isArray(value)) return finiteVector(value, 'joint_positions', names.length);
  if (!value.name || !value.position) valueError('Expected a JointState with names and position arrays.');
  const given = jointNames(value.name, 'JointState.name');
  finiteVector(value.position, 'JointState.position', given.length);
  for (const field of ['velocity', 'effort']) {
    const values = finiteVector(value[field] ?? [], `JointState.${field}`);
    if (values.length && values.length !== given.length) valueError(`JointState.${field} must be empty or match name length.`);
  }
  if (names.some(n => !given.includes(n))) valueError('JointState must contain every configured joint.');
  return names.map(n => value.position[given.indexOf(n)]);
}
/** Make a Python PoseStamped with ROS message field spelling. */
export function poseStamped(pose, frame_id, time = 0) {
  const [x, y, z] = pose.position, [qx, qy, qz, w] = pose.quat_xyzw;
  return { __class__: 'PoseStamped', header: { stamp: duration(time), frame_id }, pose: { position: { x, y, z }, orientation: { x: qx, y: qy, z: qz, w } } };
}
/** ROS duration fields with normalized nanoseconds. */
export function duration(time) {
  let sec = Math.floor(time), nanosec = Math.round((time - sec) * 1e9);
  if (nanosec === 1e9) { sec++; nanosec = 0; }
  return { sec, nanosec };
}
/** Convert core trajectory seconds to a Python JointTrajectory. */
export function pyTrajectory(trajectory) {
  if (!trajectory) return null;
  return { __class__: 'JointTrajectory', joint_names: [...trajectory.joint_names], points: trajectory.points.map(p => ({ ...structuredClone(p), time_from_start: duration(p.time_from_start) })) };
}
/** Convert a Python JointTrajectory to the core's numeric seconds. */
export function coreTrajectory(trajectory) {
  if (trajectory == null) return null;
  if (trajectory.__class__ !== 'JointTrajectory') valueError('`joint_trajectory` must be a JointTrajectory or None.');
  return { joint_names: [...trajectory.joint_names], points: trajectory.points.map(p => ({ positions: [...p.positions], velocities: p.velocities?.length ? [...p.velocities] : p.positions.map(() => 0), accelerations: p.accelerations?.length ? [...p.accelerations] : p.positions.map(() => 0), time_from_start: typeof p.time_from_start === 'number' ? p.time_from_start : p.time_from_start.sec + p.time_from_start.nanosec / 1e9 })) };
}
/** Normalize a Pose/PoseStamped or explicit component goal, preserving omitted components. */
export function poseGoal(a, model, requireBoth = false) {
  let { position, quat_xyzw, frame_id } = a;
  if (a.pose != null) {
    if (!['Pose', 'PoseStamped'].includes(a.pose.__class__)) valueError('`pose` must be a `Pose` or `PoseStamped`!');
    if (a.pose.header?.frame_id && frame_id && a.pose.header.frame_id !== frame_id) valueError('Explicit frame_id conflicts with PoseStamped frame.');
    frame_id = a.pose.header?.frame_id || frame_id;
    const pose = a.pose.pose ?? a.pose;
    position = pose.position; quat_xyzw = pose.orientation;
  }
  if (requireBoth && (position == null || quat_xyzw == null)) valueError('Either `pose` or `position` and `quat_xyzw` must be specified!');
  return {
    ...(position == null ? {} : { position: finiteVector(position, 'position', 3) }),
    ...(quat_xyzw == null ? {} : { quat_xyzw: finiteVector(quat_xyzw, 'quat_xyzw', 4) }),
    target_link: a.target_link ?? model.end_effector_name, frame_id: frame_id ?? model.base_link_name,
    // `tolerance` may be an orientation triplet, which only orientation goals read.
    tolerance_position: finiteFloat(a.tolerance_position ?? (Array.isArray(a.tolerance) ? 0.001 : a.tolerance ?? 0.001), 'tolerance', 0),
    tolerance_orientation: orientationTolerance(a.tolerance_orientation ?? a.tolerance ?? 0.001),
  };
}
/** Scalar or per-axis nonnegative orientation tolerance. */
export function orientationTolerance(value) {
  if (typeof value === 'number' || typeof value === 'boolean') return finiteFloat(value, 'tolerance', 0);
  const tolerance = finiteVector(value, 'tolerance', 3);
  if (tolerance.some(v => v < 0)) valueError('Orientation tolerances must be nonnegative.');
  return tolerance;
}
/** Validate a joint goal, including pymoveit2's prefix-goal behavior. */
export function jointGoal(a, configured) {
  const positions = finiteVector(a.joint_positions, 'joint_positions');
  if (!positions.length) valueError('`joint_positions` must not be empty.');
  let names;
  if (a.joint_names == null) {
    if (positions.length > configured.length) valueError('Too many joint positions for configured joints.');
    names = configured.slice(0, positions.length);
  } else {
    names = jointNames(a.joint_names);
    if (names.length !== positions.length) valueError('Explicit joint names and positions must have equal length.');
    if (names.some(n => !configured.includes(n))) valueError('Joint goals contain unknown joint names.');
  }
  finiteFloat(a.weight ?? a.weight_joint_position ?? 1, 'weight', 0);
  return { positions, joint_names: names, tolerance: finiteFloat(a.tolerance ?? a.tolerance_joint_position ?? 0.001, 'tolerance', 0) };
}
