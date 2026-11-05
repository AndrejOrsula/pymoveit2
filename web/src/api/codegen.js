// Readable Python representations and runnable pymoveit2 lesson scripts.
import { resolveSpec, SIGNATURES } from './signatures.js';

const stringRepr = s => "'" + s.replace(/\\/g, '\\\\').replace(/'/g, "\\'").replace(/\n/g, '\\n').replace(/\r/g, '\\r').replace(/\t/g, '\\t').replace(/[\x00-\x08\x0b\x0c\x0e-\x1f\x7f]/g, c => `\\x${c.charCodeAt(0).toString(16).padStart(2, '0')}`) + "'";
const floatRepr = value => {
  if (Object.is(value, -0)) return '-0.0';
  const repr = String(value);
  return Number.isInteger(value) && !/[eE]/.test(repr) ? `${repr}.0` : repr;
};
const messageNames = {
  PoseStamped: 'geometry_msgs.msg.PoseStamped', Pose: 'geometry_msgs.msg.Pose', Point: 'geometry_msgs.msg.Point', Quaternion: 'geometry_msgs.msg.Quaternion',
  Header: 'std_msgs.msg.Header', Time: 'builtin_interfaces.msg.Time', Duration: 'builtin_interfaces.msg.Duration',
  JointState: 'sensor_msgs.msg.JointState', JointTrajectory: 'trajectory_msgs.msg.JointTrajectory', JointTrajectoryPoint: 'trajectory_msgs.msg.JointTrajectoryPoint',
  PlanningScene: 'moveit_msgs.msg.PlanningScene', MoveItErrorCodes: 'moveit_msgs.msg.MoveItErrorCodes',
};
function message(value, cls) {
  let fields = { ...value }; delete fields.__class__;
  if (cls === 'PoseStamped') fields = { header: { __class__: 'Header', ...fields.header }, pose: { __class__: 'Pose', ...fields.pose } };
  if (cls === 'Header') fields = { stamp: { __class__: 'Time', sec: 0, nanosec: 0, ...fields.stamp }, frame_id: fields.frame_id ?? '' };
  if (cls === 'Pose') fields = { position: { __class__: 'Point', ...fields.position }, orientation: { __class__: 'Quaternion', ...fields.orientation } };
  if (cls === 'JointState') fields = { header: { __class__: 'Header', ...fields.header }, name: fields.name ?? [], position: fields.position ?? [], velocity: fields.velocity ?? [], effort: fields.effort ?? [] };
  if (cls === 'JointTrajectory') fields = { header: { __class__: 'Header', ...fields.header }, joint_names: fields.joint_names, points: fields.points.map(p => ({ __class__: 'JointTrajectoryPoint', ...p })) };
  if (cls === 'JointTrajectoryPoint') fields = { ...fields, effort: fields.effort ?? [], time_from_start: { __class__: 'Duration', ...fields.time_from_start } };
  return `${messageNames[cls] ?? cls}(${Object.entries(fields).map(([key, val]) => `${key}=${pyRepr(val, ['position', 'velocity', 'effort', 'positions', 'velocities', 'accelerations'].includes(key) ? 'list[float]' : ['x', 'y', 'z', 'w'].includes(key) ? 'float' : undefined)}`).join(', ')})`;
}

/** Python repr, with ROS message constructors and round-trip float literals. */
export function pyRepr(value, typeHint = '') {
  if (value == null) return 'None';
  if (typeof value === 'boolean') return value ? 'True' : 'False';
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) return Number.isNaN(value) ? "float('nan')" : value > 0 ? "float('inf')" : "float('-inf')";
    return typeHint === 'float' || !Number.isInteger(value) || Object.is(value, -0) ? floatRepr(value) : String(value);
  }
  if (typeof value === 'string') return stringRepr(value);
  if (Array.isArray(value)) {
    const content = value.map(x => pyRepr(x, typeHint.includes('float') ? 'float' : '')).join(', ');
    return typeHint.startsWith('tuple') ? `(${content}${value.length === 1 ? ',' : ''})` : `[${content}]`;
  }
  if (value.__class__ === 'MoveIt2State') return `<MoveIt2State.${value.name}: ${value.value}>`;
  if (value.__class__ === 'Future') return `<Future at 0x${value.id.toString(16)} state=finished returned ${value.result?.__class__ ?? 'object'}>`;
  if (value.__class__ === 'opaque') return value.repr;
  if (value.__class__) return message(value, value.__class__);
  return `{${Object.entries(value).map(([k, v]) => `${stringRepr(k)}: ${pyRepr(v)}`).join(', ')}}`;
}

function hint(path, name, spec) {
  let type = spec?.params?.find(p => p.name === name)?.type;
  if ((!type || type === 'any') && path.startsWith('robot.')) type = SIGNATURES.MoveIt2.methods[path.split('.').at(-1)]?.params.find(p => p.name === name)?.type;
  if (['position', 'quat_xyzw', 'joint_positions', 'size', 'dimensions', 'linear', 'angular', 'velocities', 'min_corner', 'max_corner'].includes(name)) return 'list[float]';
  return type ?? '';
}

/** Format exactly the explicitly supplied arguments; never invent omitted defaults. */
export function formatCall(path, args = [], kwargs = {}) {
  const spec = resolveSpec(path), positional = spec?.params?.filter(p => p.kind === 'positional') ?? [];
  const values = args.map((value, i) => pyRepr(value, hint(path, positional[i]?.name, spec)));
  for (const [key, value] of Object.entries(kwargs)) values.push(`${key}=${pyRepr(value, hint(path, key, spec))}`);
  return `${path}(${values.join(', ')})`;
}

/** Wrap recorded statements in a runnable connect() context-manager script. */
export function buildScript(statements) {
  const imports = `from pymoveit2 import connect\n${statements.some(s => /\btime\.sleep\s*\(/.test(s)) ? 'import time\n' : ''}`;
  return `${imports}\nwith connect() as robot:\n${(statements.length ? statements : ['pass']).map(s => s.split('\n').map(line => `    ${line}`).join('\n')).join('\n')}\n`;
}
