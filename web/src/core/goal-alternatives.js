// Compatible request metadata for Python's OR-separated goal constraint sets.

/**
 * Expand optional goal_alternatives without changing the pinned PlanRequest.
 * Each alternative holds one ordinary joint_goal or pose_goal. The public
 * request retains a normal goal so existing consumers can display it.
 */
export function expandGoalAlternatives(request) {
  if (!request.goal_alternatives?.length) return null;
  const { goal_alternatives, joint_goal, pose_goal, ...settings } = request;
  return goal_alternatives.map(goal => ({ ...settings, ...goal }));
}

const empty = () => ({ position: [], orientation: [], joint: [] });
function poseConstraints(goal) {
  const c = empty();
  if (goal?.position) c.position.push({ position: goal.position, link: goal.target_link, tolerance: goal.tolerance_position ?? 0.001 });
  if (goal?.quat_xyzw) c.orientation.push({ quat_xyzw: goal.quat_xyzw, link: goal.target_link, tolerance: goal.tolerance_orientation ?? 0.001, parameterization: 0 });
  return c;
}

/** Append conjunctive Python goal builders without discarding earlier constraints. */
export function mergeGoalConstraints(previous, addition) {
  const next = structuredClone(previous), requirements = next.goal_requirements ?? empty();
  next.goal_requirements = requirements;
  if (addition.joint_goal) {
    if (next.joint_goal) next.joint_goal.positions.forEach((position, i) => requirements.joint.push({ joint_name: next.joint_goal.joint_names[i], position, tolerance: next.joint_goal.tolerance }));
    next.joint_goal = structuredClone(addition.joint_goal);
  }
  if (addition.pose_goal) {
    const old = next.pose_goal, goal = addition.pose_goal;
    const oldConstraints = poseConstraints(old);
    if (goal.position) requirements.position.push(...oldConstraints.position);
    if (goal.quat_xyzw) requirements.orientation.push(...oldConstraints.orientation);
    next.pose_goal = { ...old, ...structuredClone(goal) };
  }
  return next;
}

/** Emit one pinned goal kind plus optional final-state goal_requirements metadata. */
export function canonicalGoalConstraints(goal) {
  const requirements = structuredClone(goal.goal_requirements ?? empty());
  if (goal.pose_goal && goal.joint_goal) goal.joint_goal.positions.forEach((position, i) => requirements.joint.push({ joint_name: goal.joint_goal.joint_names[i], position, tolerance: goal.joint_goal.tolerance }));
  return { ...(goal.pose_goal ? { pose_goal: goal.pose_goal } : { joint_goal: goal.joint_goal }), goal_requirements: requirements };
}
