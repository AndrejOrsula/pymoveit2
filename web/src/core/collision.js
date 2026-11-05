// Sphere robot collision checks against oriented scene geometry and attachments.
import { SUZANNE_SPHERES } from './geometry.js';
import { linkTransforms } from './kinematics.js';
import { mat4FromPose, mat4InvertRigid, mat4Multiply, mat4TransformPoint, poseFromMat4, norm, sub, dot, cross, clamp } from './math.js';

const pairKey = (a, b) => [a, b].sort().join('|');
const segmentDistance = (p, a, b) => {
  const d = sub(b, a), s = d.reduce((v, x) => v + x * x, 0);
  const t = s ? clamp(sub(p, a).reduce((v, x, i) => v + x * d[i], 0) / s, 0, 1) : 0;
  return norm(sub(p, a.map((x, i) => x + t * d[i])));
};

function localDistance(p, object) {
  const d = object.dimensions;
  if (object.type === 'sphere') return norm(p) - d[0];
  if (object.type === 'box') {
    const q = p.map((x, i) => Math.abs(x) - d[i] / 2);
    return norm(q.map(x => Math.max(x, 0))) + Math.min(Math.max(...q), 0);
  }
  if (object.type === 'cylinder') {
    const q = [Math.hypot(p[0], p[1]) - d[1], Math.abs(p[2]) - d[0] / 2];
    return norm(q.map(x => Math.max(x, 0))) + Math.min(Math.max(...q), 0);
  }
  if (object.type === 'cone') {
    const v = [Math.hypot(p[0], p[1]), p[2]], h = d[0] / 2, r = d[1];
    const distance = Math.min(segmentDistance(v, [0, -h], [r, -h]), segmentDistance(v, [r, -h], [0, h]));
    const inside = v[1] >= -h && v[1] <= h && v[0] <= r * (h - v[1]) / (2 * h);
    return inside ? -distance : distance;
  }
  return Infinity;
}

function distanceFunction(object) {
  if (object.type === 'mesh') {
    const spheres = objectSpheres(object);
    return point => Math.min(...spheres.map(s => norm(sub(point, s.slice(0, 3))) - s[3]));
  }
  const inverse = mat4InvertRigid(mat4FromPose(object.pose));
  return point => localDistance(mat4TransformPoint(inverse, point), object);
}

/** Signed distance from a world-space point to a solid primitive surface. */
export function primitiveDistance(point, object) { return distanceFunction(object)(point); }

/** Conservative sphere cover for scene geometry, used for attached-object checks. */
export function objectSpheres(object) {
  let spheres;
  const d = object.dimensions;
  if (object.type === 'mesh') {
    const s = object.scale ?? [1, 1, 1];
    spheres = SUZANNE_SPHERES.map(p => [p[0] * s[0], p[1] * s[1], p[2] * s[2], p[3] * Math.max(...s.map(Math.abs))]);
  } else if (object.type === 'sphere') spheres = [[0, 0, 0, d[0]]];
  else {
    // Small cells keep attached boxes much tighter than a single bounding sphere.
    const ext = object.type === 'box' ? d : [2 * d[1], 2 * d[1], d[0]];
    const counts = ext.map(x => Math.max(1, Math.ceil(x / 0.025)));
    spheres = [];
    for (let i = 0; i < counts[0]; i++) for (let j = 0; j < counts[1]; j++) for (let k = 0; k < counts[2]; k++) {
      const p = [i, j, k].map((v, axis) => -ext[axis] / 2 + (v + 0.5) * ext[axis] / counts[axis]);
      const radius = Math.hypot(...ext.map((x, axis) => x / counts[axis])) / 2;
      if (object.type !== 'box' && primitiveDistance(p, { ...object, pose: {} }) > radius) continue;
      spheres.push([...p, radius]);
    }
  }
  const transform = mat4FromPose(object.pose);
  return spheres.map(s => [...mat4TransformPoint(transform, s), s[3]]);
}

/** Whether two oriented boxes {pose, dimensions} overlap by more than 1e-6 (separating axis test). */
function boxesOverlap(a, b) {
  const ma = mat4FromPose(a.pose), mb = mat4FromPose(b.pose);
  const axesA = [0, 1, 2].map(c => Array.from(ma.slice(4 * c, 4 * c + 3))), axesB = [0, 1, 2].map(c => Array.from(mb.slice(4 * c, 4 * c + 3)));
  const d = sub(Array.from(mb.slice(12, 15)), Array.from(ma.slice(12, 15)));
  const candidates = [...axesA, ...axesB, ...axesA.flatMap(u => axesB.map(v => cross(u, v)))];
  for (const axis of candidates) {
    const length = norm(axis);
    if (length < 1e-9) continue;
    const n = axis.map(x => x / length);
    const radius = (axes, size) => axes.reduce((sum, u, i) => sum + Math.abs(dot(u, n)) * size[i] / 2, 0);
    if (Math.abs(dot(d, n)) > radius(axesA, a.dimensions) + radius(axesB, b.dimensions) - 1e-6) return false;
  }
  return true;
}

/** Whether a scene object penetrates a robot box. Spheres and boxes are exact; other shapes use their sphere cover. */
function objectHitsBox(object, box) {
  if (object.type === 'box') return boxesOverlap(object, box);
  const inside = distanceFunction(box);
  const cover = object.type === 'sphere' ? [[...object.pose.position, object.dimensions[0]]] : objectSpheres(object);
  return cover.some(s => inside(s.slice(0, 3)) < s[3] - 1e-6);
}

/** Update an attached pose using optional serialized attachment.relative_pose metadata. */
export function resolvedObject(object, transforms) {
  if (!object.attached?.relative_pose) return object;
  return { ...object, pose: poseFromMat4(mat4Multiply(transforms[object.attached.link], mat4FromPose(object.attached.relative_pose))) };
}

function sceneGeometry(model, q, fingers, objects) {
  const transforms = linkTransforms(model, q, fingers), boxes = model.collision_boxes ?? {};
  const links = [...new Set([...Object.keys(model.collision_spheres), ...Object.keys(boxes)])];
  const robot = links.map(link => ({
    id: link,
    spheres: (model.collision_spheres[link] ?? []).map(s => [...mat4TransformPoint(transforms[link], s), s[3]]),
    boxes: (boxes[link] ?? []).map(b => ({ type: 'box', dimensions: b.size, pose: poseFromMat4(mat4Multiply(transforms[link], mat4FromPose({ position: b.xyz }))) })),
  }));
  return { robot, objects: objects.map(o => resolvedObject(o, transforms)) };
}

/** Self contact between two links: inner sphere covers against each other and against exact boxes. */
function linksTouch(a, b) {
  // The generated outer covers overlap in SRDF transport/ready states.
  // Inner covers for self checks avoid those mesh-cover false positives;
  // robot/world checks retain every sphere's full radius.
  if (a.spheres.some(s => b.spheres.some(t => norm(sub(s.slice(0, 3), t.slice(0, 3))) < 0.65 * (s[3] + t[3]) - 1e-6))) return true;
  const sphereInBox = (spheres, boxes) => boxes.some(box => { const inside = distanceFunction(box); return spheres.some(s => inside(s.slice(0, 3)) < 0.65 * s[3] - 1e-6); });
  return sphereInBox(a.spheres, b.boxes) || sphereInBox(b.spheres, a.boxes) || a.boxes.some(x => b.boxes.some(y => boxesOverlap(x, y)));
}

/** Check robot/world, enabled self pairs and attached/world/robot collisions. */
export function checkState(model, q, fingers, objects = [], opts = {}) {
  const scene = sceneGeometry(model, q, fingers, objects), contacts = [];
  const active = scene.objects.filter(o => !o.allowed);
  const distances = new Map(active.map(o => [o, distanceFunction(o)]));
  const disabled = new Set(model.disabled_collisions.map(([a, b]) => pairKey(a, b)));
  for (const j of [...model.joints, ...model.fixed]) disabled.add(pairKey(j.parent, j.child));
  for (let i = 0; i < scene.robot.length; i++) {
    const a = scene.robot[i];
    if (opts.self !== false) for (let j = i + 1; j < scene.robot.length; j++) {
      const b = scene.robot[j];
      if (!disabled.has(pairKey(a.id, b.id)) && linksTouch(a, b)) contacts.push([a.id, b.id]);
    }
    for (const o of active) {
      if (o.attached && (o.attached.link === a.id || o.attached.touch_links.includes(a.id))) continue;
      if (a.spheres.some(s => distances.get(o)(s.slice(0, 3)) < s[3] - 1e-6) || a.boxes.some(box => objectHitsBox(o, box))) contacts.push([a.id, o.id]);
    }
  }
  for (let i = 0; i < active.length; i++) {
    const a = active[i];
    if (!a.attached) continue;
    const cover = objectSpheres(a);
    for (let j = 0; j < active.length; j++) {
      const b = active[j];
      if (i === j || (b.attached && j < i)) continue;
      if (cover.some(s => distances.get(b)(s.slice(0, 3)) < s[3] - 1e-6)) contacts.push([a.id, b.id]);
    }
  }
  return { valid: contacts.length === 0, contacts };
}

/** Minimum robot/world clearance for servo slowdown; self clearance is omitted. */
export function minimumDistance(model, q, fingers, objects = []) {
  const scene = sceneGeometry(model, q, fingers, objects);
  const distances = new Map(scene.objects.map(o => [o, distanceFunction(o)]));
  let distance = Infinity;
  for (const a of scene.robot) for (const o of scene.objects) {
    if (o.allowed || (o.attached && (o.attached.link === a.id || o.attached.touch_links.includes(a.id)))) continue;
    for (const s of a.spheres) distance = Math.min(distance, distances.get(o)(s.slice(0, 3)) - s[3]);
    // Boxes report contact only; a clearance of zero is enough to halt Servo.
    if (a.boxes.some(box => objectHitsBox(o, box))) distance = Math.min(distance, 0);
  }
  for (const object of scene.objects) {
    if (!object.attached || object.allowed) continue;
    for (const other of scene.objects) {
      if (other.id === object.id || other.allowed) continue;
      for (const sphere of objectSpheres(object)) distance = Math.min(distance, distances.get(other)(sphere.slice(0, 3)) - sphere[3]);
    }
  }
  return distance;
}
