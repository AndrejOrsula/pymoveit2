// Frame, quaternion and deterministic random helpers shared by the simulator.

/** Add two vectors. */
export const add = (a, b) => a.map((x, i) => x + b[i]);
/** Subtract two vectors. */
export const sub = (a, b) => a.map((x, i) => x - b[i]);
/** Scale a vector. */
export const scale = (a, s) => a.map(x => x * s);
/** Dot product. */
export const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
/** Euclidean norm. */
export const norm = a => Math.hypot(...a);
/** Cross product. */
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
/** Linear interpolation. */
export const lerp = (a, b, t) => a.map((x, i) => x + (b[i] - x) * t);
/** Clamp a scalar. */
export const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
/** Normalize an xyzw quaternion. */
export function quatNormalize(q) {
  const n = norm(q);
  return n > 1e-15 ? scale(q, 1 / n) : [0, 0, 0, 1];
}
/** Hamilton product, with xyzw storage. */
export function quatMultiply(a, b) {
  return [...add(add(scale(b.slice(0, 3), a[3]), scale(a.slice(0, 3), b[3])), cross(a, b)), a[3] * b[3] - dot(a.slice(0, 3), b.slice(0, 3))];
}
/** Inverse quaternion, including non-unit inputs. */
export function quatInverse(q) {
  const n = dot(q, q);
  return n > 1e-30 ? [-q[0] / n, -q[1] / n, -q[2] / n, q[3] / n] : [0, 0, 0, 1];
}
/** URDF fixed-axis roll, pitch, yaw rotation: Rz Ry Rx. */
export function quatFromRpy(roll, pitch, yaw) {
  const [sr, sp, sy] = [roll, pitch, yaw].map(x => Math.sin(x / 2));
  const [cr, cp, cy] = [roll, pitch, yaw].map(x => Math.cos(x / 2));
  return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy];
}
/** Extract fixed-axis XYZ Euler angles; roll is zero at gimbal lock. */
export function rpyFromQuat(q) {
  const [x, y, z, w] = quatNormalize(q);
  const s = clamp(2 * (w * y - z * x), -1, 1);
  if (Math.abs(s) > 1 - 1e-12) return [0, Math.asin(s), 2 * Math.atan2(z, w)];
  return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), Math.asin(s), Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
}
/** Intrinsic XYZ Euler angles (Rx Ry Rz), used by MoveIt orientation constraints. */
export function eulerXYZFromQuat(q) {
  const m = mat4FromPose({ quat_xyzw: q });
  const cosine = Math.sqrt((m[0] ** 2 + m[4] ** 2 + m[9] ** 2 + m[10] ** 2) / 2);
  if (cosine > 4 * Number.EPSILON) return [Math.atan2(-m[9], m[10]), Math.atan2(m[8], cosine), Math.atan2(-m[4], m[0])];
  return [m[8] > 0 ? Math.atan2(m[1] + m[6], m[5] - m[2]) : Math.atan2(m[6] - m[1], m[5] + m[2]), Math.atan2(m[8], cosine), 0];
}
/** Rotate a point by a quaternion. */
export function quatRotate(q, p) {
  q = quatNormalize(q);
  const t = scale(cross(q, p), 2);
  return add(p, add(scale(t, q[3]), cross(q, t)));
}
/** Shortest quaternion geodesic angle. */
export const quatAngle = (a, b) => 2 * Math.acos(clamp(Math.abs(dot(quatNormalize(a), quatNormalize(b))), 0, 1));
/** Shortest spherical interpolation. */
export function quatSlerp(a, b, t) {
  a = quatNormalize(a); b = quatNormalize(b);
  let d = dot(a, b);
  if (d < 0) { b = scale(b, -1); d = -d; }
  if (d > 0.9995) return quatNormalize(lerp(a, b, t));
  const angle = Math.acos(clamp(d, -1, 1));
  return add(scale(a, Math.sin((1 - t) * angle) / Math.sin(angle)), scale(b, Math.sin(t * angle) / Math.sin(angle)));
}
/** Shortest rotation vector for a quaternion. */
export function rotationVector(q) {
  q = quatNormalize(q);
  if (q[3] < 0) q = scale(q, -1);
  const s = norm(q.slice(0, 3));
  return s < 1e-12 ? scale(q.slice(0, 3), 2) : scale(q.slice(0, 3), 2 * Math.atan2(s, q[3]) / s);
}
/** Pose to column-major rigid transform. */
export function mat4FromPose(pose = {}) {
  const p = pose.position ?? [0, 0, 0];
  const [x, y, z, w] = quatNormalize(pose.quat_xyzw ?? [0, 0, 0, 1]);
  return new Float64Array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 0, 2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 0, 2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y), 0, ...p, 1]);
}
/** Rigid transform to pose. */
export function poseFromMat4(m) {
  const trace = m[0] + m[5] + m[10];
  let q;
  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    q = [(m[6] - m[9]) / s, (m[8] - m[2]) / s, (m[1] - m[4]) / s, s / 4];
  } else {
    const i = m[0] > m[5] && m[0] > m[10] ? 0 : m[5] > m[10] ? 1 : 2;
    const j = (i + 1) % 3, k = (i + 2) % 3;
    const s = 2 * Math.sqrt(1 + m[i * 5] - m[j * 5] - m[k * 5]);
    q = [0, 0, 0, (m[j * 4 + k] - m[k * 4 + j]) / s];
    q[i] = s / 4; q[j] = (m[j * 4 + i] + m[i * 4 + j]) / s; q[k] = (m[k * 4 + i] + m[i * 4 + k]) / s;
  }
  return { position: Array.from(m.slice(12, 15)), quat_xyzw: quatNormalize(q) };
}
/** Multiply column-major matrices. */
export function mat4Multiply(a, b) {
  const out = new Float64Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) for (let k = 0; k < 4; k++) out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
  return out;
}
/** Invert a rigid matrix. */
export function mat4InvertRigid(m) {
  const out = mat4FromPose();
  for (let c = 0; c < 3; c++) for (let r = 0; r < 3; r++) out[c * 4 + r] = m[r * 4 + c];
  for (let r = 0; r < 3; r++) out[12 + r] = -dot([out[r], out[4 + r], out[8 + r]], Array.from(m.slice(12, 15)));
  return out;
}
/** Transform a point. */
export const mat4TransformPoint = (m, p) => [0, 1, 2].map(r => m[r] * p[0] + m[r + 4] * p[1] + m[r + 8] * p[2] + m[r + 12]);
/** Mulberry32 RNG; state getter/setter supports JSON snapshots. */
export function createRng(seed = 1) {
  let state = seed >>> 0;
  return {
    next() { let t = state = (state + 0x6D2B79F5) >>> 0; t = Math.imul(t ^ t >>> 15, t | 1); t ^= t + Math.imul(t ^ t >>> 7, t | 61); return ((t ^ t >>> 14) >>> 0) / 4294967296; },
    uniform(a, b) { return a + (b - a) * this.next(); },
    normal() { return Math.sqrt(-2 * Math.log(Math.max(this.next(), Number.MIN_VALUE))) * Math.cos(2 * Math.PI * this.next()); },
    get state() { return state; },
    set state(value) { state = value >>> 0; },
  };
}
