// Planning-scene Python methods and their ROS topic/service observations.
import { poseGoal, finiteVector, valueError } from './values.js';
import { linkTransforms } from '../core/kinematics.js';
import { mat4FromPose, mat4Multiply, poseFromMat4 } from '../core/math.js';

/** Build a detached Python PlanningScene from the simulation's object registry. */
export function planningScene(sim) {
  const objects = sim.getObjects();
  return { __class__: 'PlanningScene', world: { collision_objects: objects.filter(o => !o.attached) }, robot_state: { attached_collision_objects: objects.filter(o => o.attached).map(o => ({ link_name: o.attached.link, touch_links: o.attached.touch_links, weight: o.attached.weight, object: o })) } };
}

/** Scene method bodies; service futures resolve immediately in both drivers. */
export function* sceneOperation(s, method, a, call) {
  const sim = s.sim, model = sim.model;
  const topic = summary => s.ros(call, 'topic', '/collision_object', 'moveit_msgs/msg/CollisionObject', summary);
  const attached = summary => s.ros(call, 'topic', '/attached_collision_object', 'moveit_msgs/msg/AttachedCollisionObject', summary);
  const fetch = () => s.ros(call, 'service', '/get_planning_scene', 'moveit_msgs/srv/GetPlanningScene', 'Read current planning scene');
  const apply = summary => s.ros(call, 'service', '/apply_planning_scene', 'moveit_msgs/srv/ApplyPlanningScene', summary, 'SUCCESS');
  if (method.startsWith('add_collision_')) {
    const goal = poseGoal(a, model, true);
    let pose = { position: goal.position, quat_xyzw: goal.quat_xyzw };
    if (goal.frame_id !== model.planning_frame) {
      const transform = linkTransforms(model, sim.q, sim.fingers)[goal.frame_id];
      if (!transform) valueError(`Unknown frame '${goal.frame_id}' in browser demo.`);
      pose = poseFromMat4(mat4Multiply(transform, mat4FromPose(pose)));
    }
    let type = method.slice('add_collision_'.length), dimensions;
    if (type === 'primitive') {
      type = { 1: 'box', 2: 'sphere', 3: 'cylinder', 4: 'cone' }[a.primitive_type];
      if (!Number.isInteger(a.primitive_type) || !type) valueError('Unsupported primitive_type; expected box, sphere, cylinder or cone.');
      dimensions = a.dimensions;
    } else dimensions = type === 'box' ? a.size : type === 'sphere' ? [a.radius] : type === 'mesh' ? [] : [a.height, a.radius];
    if (type !== 'mesh') {
      dimensions = finiteVector(dimensions, 'dimensions', { box: 3, sphere: 1, cylinder: 2, cone: 2 }[type]);
      if (dimensions.some(v => v <= 0)) valueError('Primitive dimensions must be positive.');
    } else if (typeof a.filepath !== 'string' || !a.filepath.endsWith('suzanne.stl')) valueError('The browser demo ships only examples/assets/suzanne.stl; other mesh paths are unavailable.');
    const scale = typeof a.scale === 'number' ? Array(3).fill(a.scale) : a.scale ?? [1, 1, 1];
    sim.applyCollisionObject({ id: a.id, type, dimensions, pose, operation: a.operation, ...(type === 'mesh' ? { mesh: 'suzanne', scale: finiteVector(scale, 'scale', 3) } : {}) });
    topic(`Add ${type} '${a.id}'`); return null;
  }
  switch (method) {
    case 'remove_collision_object': sim.applyCollisionObject({ id: a.id, operation: 1 }); topic(`Remove '${a.id}'`); return null;
    case 'move_collision': {
      const goal = poseGoal(a, model, true);
      let pose = { position: goal.position, quat_xyzw: goal.quat_xyzw };
      if (goal.frame_id !== model.planning_frame) {
        const t = linkTransforms(model, sim.q, sim.fingers)[goal.frame_id];
        if (!t) valueError(`Unknown frame '${goal.frame_id}' in browser demo.`);
        pose = poseFromMat4(mat4Multiply(t, mat4FromPose(pose)));
      }
      sim.applyCollisionObject({ id: a.id, pose, operation: 3 }); topic(`Move '${a.id}'`); return null;
    }
    case 'attach_collision_object': sim.attachObject(a.id, a.link_name ?? model.end_effector_name, a.touch_links ?? [], a.weight); attached(`Attach '${a.id}' to ${a.link_name ?? model.end_effector_name}`); return null;
    case 'detach_collision_object': sim.detachObject(a.id); attached(`Detach '${a.id}'`); return null;
    case 'detach_all_collision_objects': sim.detachAll(); attached('Detach all objects'); return null;
    case 'update_planning_scene': fetch(); return true;
    case 'allow_collisions':
      if (typeof a.id !== 'string' || !a.id) valueError('Collision object id must be a nonempty string.');
      if (typeof a.allow !== 'boolean') valueError('`allow` must be a bool.');
      fetch(); sim.allowCollisions(a.id, a.allow); apply(`Allow collisions with '${a.id}': ${a.allow}`); return s.future(true);
    case 'clear_all_collision_objects': fetch(); sim.clearScene(); apply('Remove all world and attached collision objects'); return s.future(true);
    case 'process_allow_collision_future':
    case 'process_clear_all_collision_objects_future': return s.futures.get(a.future?.id) === true;
    case 'cancel_clear_all_collision_objects_future': return null;
  }
  valueError(`Unknown scene method ${method}().`);
}
