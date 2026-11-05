// Bounded damped-least-squares inverse kinematics with deterministic restarts.
import { fk, jacobian } from './kinematics.js';
import { sub, norm, dot, clamp, quatMultiply, quatInverse, rotationVector, createRng } from './math.js';

/** Solve a small dense linear system by pivoted Gaussian elimination. */
export function solveLinear(matrix, vector) {
  const a = matrix.map((row, i) => [...row, vector[i]]), n = vector.length;
  for (let i = 0; i < n; i++) {
    let p = i;
    for (let j = i + 1; j < n; j++) if (Math.abs(a[j][i]) > Math.abs(a[p][i])) p = j;
    if (Math.abs(a[p][i]) < 1e-15) return null;
    [a[i], a[p]] = [a[p], a[i]];
    const d = a[i][i];
    for (let k = i; k <= n; k++) a[i][k] /= d;
    for (let j = 0; j < n; j++) if (j !== i) {
      const f = a[j][i];
      for (let k = i; k <= n; k++) a[j][k] -= f * a[i][k];
    }
  }
  return a.map(row => row[n]);
}
/** Damped right pseudoinverse product J^T (J J^T + lambda² I)^-1 e. */
export function dampedStep(J, error, damping = 0.025) {
  const a = J.map((r, i) => J.map((s, j) => dot(r, s) + (i === j ? damping * damping : 0)));
  const y = solveLinear(a, error);
  return y ? J[0].map((_, c) => J.reduce((s, r, i) => s + r[c] * y[i], 0)) : Array(J[0].length).fill(0);
}
/** Solve full, position-only or orientation-only goals, preferring the supplied seed.
 * `position_only` drops the orientation, as KDL does with `position_only_ik`. */
export function solveIK(model, target, opts = {}) {
  if (opts.position_only && target.position) target = { ...target, quat_xyzw: undefined };
  if (!target.position && !target.quat_xyzw) return null;
  if (target.position && (target.position.length !== 3 || target.position.some(x => !Number.isFinite(x)))) return null;
  if (target.quat_xyzw && (target.quat_xyzw.length !== 4 || target.quat_xyzw.some(x => !Number.isFinite(x)) || norm(target.quat_xyzw) < 1e-12)) return null;
  if (target.position && norm(sub(target.position, model.reach.center)) > model.reach.radius) return null;
  const inputSeed = opts.seed ?? opts.q ?? model.named_states[model.group_name][model.default_state];
  if (inputSeed.length !== model.joints.length || inputSeed.some(x => !Number.isFinite(x))) return null;
  const seed = inputSeed.map((x, i) => clamp(x, model.joints[i].lower, model.joints[i].upper));
  const rng = opts.rng ?? createRng(1), link = opts.target_link ?? target.target_link ?? model.end_effector_name;
  if (link !== model.end_effector_name && !model.ik_links.includes(link)) return null;
  const pt = opts.tolerance_position ?? target.tolerance_position ?? 0.001;
  const angular = opts.tolerance_orientation ?? target.tolerance_orientation ?? 0.001;
  const tolerances = Array.isArray(angular) ? angular : [angular, angular, angular];
  const jointConstraints = (opts.joint_constraints ?? []).filter(c => model.joint_names.includes(c.joint_name));
  let best = null, bestDistance = Infinity;
  const attempts = opts.restarts ?? 8;
  for (let attempt = 0; attempt <= attempts; attempt++) {
    let q = attempt === 0 ? [...seed] : model.joints.map((j, i) => attempt < 3 ? clamp(seed[i] + rng.normal() * 0.65, j.lower, j.upper) : rng.uniform(j.lower, j.upper));
    for (let iter = 0; iter < (opts.max_iterations ?? 90); iter++) {
      const pose = fk(model, q, link);
      const pe = target.position ? sub(target.position, pose.position) : [];
      const oe = target.quat_xyzw ? rotationVector(quatMultiply(target.quat_xyzw, quatInverse(pose.quat_xyzw))) : [];
      const je = jointConstraints.map(c => c.position - q[model.joint_names.indexOf(c.joint_name)]);
      if ((!pe.length || norm(pe) <= pt) && (!oe.length || oe.every((x, i) => Math.abs(x) <= tolerances[i])) && je.every((x, i) => Math.abs(x) <= jointConstraints[i].tolerance)) {
        if (!opts.isValid || opts.isValid(q)) {
          const d = norm(sub(q, seed));
          if (d < bestDistance) { best = [...q]; bestDistance = d; }
        }
        break;
      }
      const full = jacobian(model, q, link);
      const J = [...(pe.length ? full.slice(0, 3) : []), ...(oe.length ? full.slice(3) : []), ...jointConstraints.map(c => model.joint_names.map(name => name === c.joint_name ? 1 : 0))];
      const dq = dampedStep(J, [...pe, ...oe, ...je], 0.012);
      const factor = Math.min(1, 0.25 / Math.max(...dq.map(Math.abs), 1e-10));
      q = q.map((x, i) => clamp(x + dq[i] * factor, model.joints[i].lower, model.joints[i].upper));
    }
    if (best && (attempt >= (opts.prefer_solutions ?? 1) || bestDistance < 0.05)) break;
  }
  return best;
}
