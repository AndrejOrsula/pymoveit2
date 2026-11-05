// URDF-tree forward kinematics and geometric Jacobian for any RobotModel.
import { mat4FromPose, mat4InvertRigid, mat4Multiply, poseFromMat4, quatFromRpy, cross, sub, scale } from './math.js';

const origin = joint => mat4FromPose({ position: joint.xyz, quat_xyzw: quatFromRpy(...joint.rpy) });
const rotation = (axis, angle) => {
  const s = Math.sin(angle / 2);
  return mat4FromPose({ quat_xyzw: [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)] });
};
const translation = (axis, distance) => mat4FromPose({ position: scale(axis, distance) });

const orders = new WeakMap();
/** Arm (index i) and fixed joints sorted parents first, computed once per model. */
function jointOrder(model) {
  if (orders.has(model)) return orders.get(model);
  const pending = [...model.joints.map((joint, i) => ({ joint, i, origin: origin(joint) })), ...model.fixed.map(joint => ({ joint, origin: origin(joint) }))];
  const known = new Set([model.planning_frame]), order = [];
  while (pending.length) {
    const next = pending.findIndex(({ joint }) => known.has(joint.parent));
    if (next < 0) throw new RangeError(`Joint '${pending[0].joint.name}' is not connected to '${model.planning_frame}'`);
    const [entry] = pending.splice(next, 1);
    known.add(entry.joint.child); order.push(entry);
  }
  orders.set(model, order);
  return order;
}

/** Every link's transform in the planning frame: fixed joints, the arm and the fingers. */
export function linkTransforms(model, q, fingers = model.gripper.open_joint_positions[0]) {
  const transforms = { [model.planning_frame]: mat4FromPose() };
  for (const { joint, i, origin: placed } of jointOrder(model)) {
    const frame = mat4Multiply(transforms[joint.parent], placed);
    transforms[joint.child] = i === undefined ? frame : mat4Multiply(frame, rotation(joint.axis, q[i]));
  }
  const f = model.fingers;
  f.links.forEach((link, i) => {
    transforms[link] = mat4Multiply(mat4Multiply(transforms[f.parent], origin(f.origins[i])), translation(f.axes[i], fingers));
  });
  return transforms;
}
/** Forward pose of any link, by default the end effector. */
export function fk(model, q, link = model.end_effector_name) {
  const matrix = linkTransforms(model, q)[link];
  if (!matrix) throw new RangeError(`Unknown link '${link}'`);
  return poseFromMat4(matrix);
}
/** Planning-frame geometric Jacobian, linear rows then angular rows, one column per arm joint. */
export function jacobian(model, q, link = model.end_effector_name) {
  const t = linkTransforms(model, q), p = Array.from(t[link].slice(12, 15));
  const J = Array.from({ length: 6 }, () => Array(model.joints.length).fill(0));
  // Only joints on the path from the link to the root move it.
  const path = new Set();
  let child = model.fingers.links.includes(link) ? model.fingers.parent : link;
  for (let entry; (entry = jointOrder(model).find(e => e.joint.child === child)); child = entry.joint.parent) path.add(entry.joint.name);
  model.joints.forEach((j, i) => {
    if (!path.has(j.name)) return;
    const m = t[j.child], axis = [0, 1, 2].map(r => m[r] * j.axis[0] + m[r + 4] * j.axis[1] + m[r + 8] * j.axis[2]);
    const v = cross(axis, sub(p, Array.from(m.slice(12, 15))));
    [...v, ...axis].forEach((x, r) => { J[r][i] = x; });
  });
  return J;
}
/** A planning-frame pose expressed in the frame of `link` at state q. */
export function poseInLink(model, q, pose, link = model.base_link_name) {
  return poseFromMat4(mat4Multiply(mat4InvertRigid(linkTransforms(model, q)[link]), mat4FromPose(pose)));
}
