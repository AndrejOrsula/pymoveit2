// Deterministic path timing, synchronized Pilz profiles and Hermite sampling.
import { norm, sub, lerp, clamp, quatAngle } from './math.js';

/** MoveIt scaling semantics: invalid factors use the full configured limit. */
export const scalingFactor = value => value > 0 && value <= 1 ? value : 1;

function hermite(a, b, t) {
  const h = b.time_from_start - a.time_from_start;
  if (h <= 0) return { positions: [...b.positions], velocities: [...b.velocities], accelerations: [...b.accelerations] };
  const u = clamp((t - a.time_from_start) / h, 0, 1);
  return {
    positions: a.positions.map((x, i) => u === 1 ? b.positions[i] : x + (3 * u ** 2 - 2 * u ** 3) * (b.positions[i] - x) + (u ** 3 - 2 * u ** 2 + u) * h * a.velocities[i] + (u ** 3 - u ** 2) * h * b.velocities[i]),
    velocities: a.positions.map((x, i) => ((6 * u ** 2 - 6 * u) * (x - b.positions[i]) / h + (3 * u ** 2 - 4 * u + 1) * a.velocities[i] + (3 * u ** 2 - 2 * u) * b.velocities[i]) || 0),
    accelerations: a.positions.map((x, i) => ((12 * u - 6) * (x - b.positions[i]) / h ** 2 + (6 * u - 4) * a.velocities[i] / h + (6 * u - 2) * b.velocities[i] / h) || 0),
  };
}

/** Cubic Hermite interpolation; clamps to stationary endpoints outside the time range. */
export function sampleTrajectory(trajectory, t) {
  const points = trajectory.points;
  if (!points.length) return { positions: [], velocities: [] };
  if (t <= points[0].time_from_start) return { positions: [...points[0].positions], velocities: [...points[0].velocities] };
  const last = points.at(-1);
  if (t >= last.time_from_start) return { positions: [...last.positions], velocities: [...last.velocities] };
  let low = 0, high = points.length - 1;
  while (high - low > 1) { const mid = (low + high) >> 1; if (points[mid].time_from_start <= t) low = mid; else high = mid; }
  const { positions, velocities } = hermite(points[low], points[high], t);
  return { positions, velocities };
}

function resample(names, duration, at, step = 0.1) {
  const points = [];
  for (let i = 0; i * step < duration - 1e-10; i++) points.push({ ...at(i * step), time_from_start: i * step });
  points.push({ ...at(duration), time_from_start: duration });
  points[0].velocities.fill(0); points.at(-1).velocities.fill(0);
  return { joint_names: [...names], points };
}

function limitedResample(model, duration, at, velocityScaling, accelerationScaling) {
  const vl = model.joints.map(j => j.max_velocity * scalingFactor(velocityScaling));
  const al = model.joints.map(j => j.max_acceleration * scalingFactor(accelerationScaling));
  let stretch = 1, trajectory, finalFactor = 1;
  // The 0.1 s output grid can introduce derivative extrema that weren't on the
  // original curve. Bound the actual execution cubics, then resample again.
  for (let iteration = 0; iteration < 12; iteration++) {
    trajectory = resample(model.joint_names, duration * stretch, t => {
      const p = at(t / stretch);
      return { positions: p.positions, velocities: p.velocities.map(v => v / stretch || 0), accelerations: p.accelerations.map(a => a / stretch ** 2 || 0) };
    });
    let factor = 1;
    for (let i = 1; i < trajectory.points.length; i++) {
      const a = trajectory.points[i - 1], b = trajectory.points[i], h = b.time_from_start - a.time_from_start;
      const left = hermite(a, b, a.time_from_start), right = hermite(a, b, b.time_from_start);
      for (let j = 0; j < vl.length; j++) {
        factor = Math.max(factor, Math.abs(a.velocities[j]) / vl[j], Math.abs(b.velocities[j]) / vl[j], Math.sqrt(Math.max(Math.abs(left.accelerations[j]), Math.abs(right.accelerations[j])) / al[j]));
        const denominator = right.accelerations[j] - left.accelerations[j];
        const u = -left.accelerations[j] / denominator;
        if (u > 0 && u < 1) factor = Math.max(factor, Math.abs(hermite(a, b, a.time_from_start + u * h).velocities[j]) / vl[j]);
      }
    }
    if (factor <= 1.000001) return trajectory;
    finalFactor = factor;
    stretch *= factor * 1.05;
  }
  // A degenerate output grid can resist regridding. Preserve the checked cubics
  // with uniform stretching rather than returning a curve beyond its limits.
  const ratio = finalFactor * 1.05;
  return { ...trajectory, points: trajectory.points.map(p => ({ ...p, time_from_start: p.time_from_start * ratio, velocities: p.velocities.map(v => v / ratio), accelerations: p.accelerations.map(a => a / ratio ** 2) })) };
}

/** Arc-length forward/backward timing with continuous monotone cubic joint velocities. */
export function timeParameterize(model, path, velocityScaling = 1, accelerationScaling = 1) {
  const names = model.joint_names;
  const clean = path.filter((p, i) => i === 0 || norm(sub(p, path[i - 1])) > 1e-10);
  if (!clean.length) return { joint_names: [...names], points: [] };
  if (clean.length === 1) return { joint_names: [...names], points: [{ positions: [...clean[0]], velocities: names.map(() => 0), accelerations: names.map(() => 0), time_from_start: 0 }] };
  const q = [clean[0]];
  for (let i = 1; i < clean.length; i++) {
    const n = Math.max(2, Math.ceil(norm(sub(clean[i], clean[i - 1])) / 0.05));
    for (let k = 1; k <= n; k++) q.push(lerp(clean[i - 1], clean[i], k / n));
  }
  const vl = model.joints.map(j => j.max_velocity * scalingFactor(velocityScaling));
  const al = model.joints.map(j => j.max_acceleration * scalingFactor(accelerationScaling));
  const ds = q.slice(1).map((p, i) => norm(sub(p, q[i])));
  const tangents = q.slice(1).map((p, i) => sub(p, q[i]).map(x => x / ds[i]));
  const vmax = tangents.map(r => Math.min(...r.map((x, j) => vl[j] / Math.max(Math.abs(x), 1e-10))));
  const amax = tangents.map(r => Math.min(...r.map((x, j) => al[j] / Math.max(Math.abs(x), 1e-10))));
  const speed = q.map((_, i) => i === 0 || i === q.length - 1 ? 0 : Math.min(vmax[i - 1], vmax[i]));
  for (let i = 1; i < speed.length; i++) speed[i] = Math.min(speed[i], Math.sqrt(speed[i - 1] ** 2 + 2 * amax[i - 1] * ds[i - 1]));
  for (let i = speed.length - 2; i >= 0; i--) speed[i] = Math.min(speed[i], Math.sqrt(speed[i + 1] ** 2 + 2 * amax[i] * ds[i]));
  const knots = [];
  let time = 0;
  q.forEach((p, i) => {
    if (i) time += 2 * ds[i - 1] / Math.max(speed[i - 1] + speed[i], 1e-8);
    const velocities = names.map((_, j) => {
      if (!i || i === q.length - 1 || tangents[i - 1][j] * tangents[i][j] <= 0) return 0;
      return speed[i] * 2 * tangents[i - 1][j] * tangents[i][j] / (tangents[i - 1][j] + tangents[i][j]);
    });
    knots.push({ positions: [...p], velocities, accelerations: names.map(() => 0), time_from_start: time });
  });
  // Stretch against analytic cubic derivatives; curvature therefore consumes acceleration.
  let stretch = 1;
  for (let i = 1; i < knots.length; i++) for (let k = 0; k <= 12; k++) {
    const s = hermite(knots[i - 1], knots[i], knots[i - 1].time_from_start + (knots[i].time_from_start - knots[i - 1].time_from_start) * k / 12);
    for (let j = 0; j < names.length; j++) stretch = Math.max(stretch, Math.abs(s.velocities[j]) / vl[j], Math.sqrt(Math.abs(s.accelerations[j]) / al[j]));
  }
  stretch *= 1.08;
  for (const knot of knots) { knot.time_from_start *= stretch; knot.velocities = knot.velocities.map(v => v / stretch); }
  let index = 1;
  const duration = knots.at(-1).time_from_start;
  return limitedResample(model, duration, t => {
    if (t < knots[index - 1].time_from_start) index = 1;
    while (index < knots.length - 1 && knots[index].time_from_start < t) index++;
    return hermite(knots[index - 1], knots[index], t);
  }, velocityScaling, accelerationScaling);
}

function trapezoid(distance, velocity, acceleration) {
  if (distance < 1e-12) return { duration: 0, at: () => [1, 0, 0] };
  const ramp = Math.min(velocity / acceleration, Math.sqrt(distance / acceleration));
  const cruise = Math.max(0, distance / (acceleration * ramp) - ramp), duration = 2 * ramp + cruise;
  return { duration, at(t) {
    if (t >= duration) return [1, 0, 0];
    if (t < ramp) return [0.5 * acceleration * t * t / distance, acceleration * t / distance, acceleration / distance];
    if (t < ramp + cruise) return [acceleration * ramp * (t - ramp / 2) / distance, acceleration * ramp / distance, 0];
    const left = duration - t;
    return [1 - 0.5 * acceleration * left * left / distance, acceleration * left / distance, -acceleration / distance];
  } };
}

/** Synchronized joint PTP: all moving joints share one trapezoidal progress profile. */
export function synchronizedTrapezoid(model, start, goal, velocityScaling = 1, accelerationScaling = 1) {
  const delta = sub(goal, start);
  const v = Math.min(...delta.map((x, i) => model.joints[i].max_velocity * scalingFactor(velocityScaling) * 0.97 / Math.max(Math.abs(x), 1e-12)));
  const a = Math.min(...delta.map((x, i) => model.joints[i].max_acceleration * scalingFactor(accelerationScaling) * 0.97 / Math.max(Math.abs(x), 1e-12)));
  const profile = trapezoid(norm(delta) < 1e-12 ? 0 : 1, v, a);
  return limitedResample(model, profile.duration, t => {
    const [s, ds, dds] = profile.at(t);
    return { positions: lerp(start, goal, s), velocities: delta.map(x => x * ds), accelerations: delta.map(x => x * dds) };
  }, velocityScaling, accelerationScaling);
}

/** Pilz Cartesian progress profile; translation and rotation finish together. */
export function cartesianProfile(start, goal, velocityScaling = 1, accelerationScaling = 1) {
  const distance = norm(sub(goal.position, start.position)), angle = quatAngle(start.quat_xyzw, goal.quat_xyzw);
  const v = Math.min(1 / Math.max(distance, 1e-12), 1.57 / Math.max(angle, 1e-12)) * scalingFactor(velocityScaling);
  const a = Math.min(2.25 / Math.max(distance, 1e-12), 3.5 / Math.max(angle, 1e-12)) * scalingFactor(accelerationScaling);
  return trapezoid(distance + angle < 1e-12 ? 0 : 1, v, a);
}

/** Pilz LIN timing over equally spaced Cartesian IK waypoints, slowed for joint limits. */
export function cartesianTrapezoid(model, path, start, goal, velocityScaling = 1, accelerationScaling = 1) {
  if (path.length < 2) return timeParameterize(model, path, velocityScaling, accelerationScaling);
  const n = path.length - 1, profile = cartesianProfile(start, goal, velocityScaling, accelerationScaling);
  const knots = path.map((positions, i) => ({ positions, velocities: positions.map((_, j) => (path[Math.min(n, i + 1)][j] - path[Math.max(0, i - 1)][j]) * n / (i === 0 || i === n ? 1 : 2)), accelerations: positions.map(() => 0), time_from_start: i / n }));
  function at(t) {
    const [s, velocity, acceleration] = profile.at(t), i = Math.max(1, Math.min(n, Math.ceil(s * n)));
    const sample = hermite(knots[i - 1], knots[i], s);
    return { positions: sample.positions, velocities: sample.velocities.map(v => v * velocity), accelerations: sample.accelerations.map((a, j) => a * velocity ** 2 + sample.velocities[j] * acceleration) };
  }
  let stretch = 1;
  const vl = model.joints.map(j => j.max_velocity * scalingFactor(velocityScaling));
  const al = model.joints.map(j => j.max_acceleration * scalingFactor(accelerationScaling));
  for (let i = 0; i <= 400; i++) {
    const sample = at(profile.duration * i / 400);
    for (let j = 0; j < vl.length; j++) stretch = Math.max(stretch, Math.abs(sample.velocities[j]) / vl[j], Math.sqrt(Math.abs(sample.accelerations[j]) / al[j]));
  }
  stretch *= 1.08;
  return limitedResample(model, profile.duration * stretch, t => {
    const sample = at(t / stretch);
    return { positions: sample.positions, velocities: sample.velocities.map(v => v / stretch), accelerations: sample.accelerations.map(a => a / stretch ** 2) };
  }, velocityScaling, accelerationScaling);
}
