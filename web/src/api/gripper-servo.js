// Shared generator bodies for the MoveIt2Gripper backend and modern Servo API.
import { finiteFloat, finiteVector, valueError } from './values.js';
import { waitForController, stateEnum, errorValue } from './arm.js';

/** Nonblocking gripper submissions and one-shot controller waits. */
export function* gripperOperation(s, method, a, call) {
  const sim = s.sim, c = s.controllers.gripper, m = sim.model;
  const tolerance = 0.1 * Math.abs(m.gripper.open_joint_positions[0] - m.gripper.closed_joint_positions[0]);
  if (s.closed) return method === 'last_failure' ? c.failure : false;
  if (['__call__', 'toggle'].includes(method)) {
    if (Math.abs(sim.fingers - m.gripper.open_joint_positions[0]) <= tolerance) method = 'close';
    else if (Math.abs(sim.fingers - m.gripper.closed_joint_positions[0]) <= tolerance) method = 'open';
    else { sim.writeLog('warn', 'pymoveit2', 'Cannot toggle the gripper until a complete joint-state observation establishes whether it is open or closed.'); return false; }
  }
  if (['open', 'close', 'move_to_position', 'reset_open', 'reset_closed'].includes(method)) {
    const position = method === 'move_to_position' ? finiteFloat(a.position, 'position') : method.includes('open') ? m.gripper.open_joint_positions[0] : m.gripper.closed_joint_positions[0];
    if (a.skip_if_noop && Math.abs(position - sim.fingers) <= tolerance) return false;
    if (position < m.fingers.lower || position > m.fingers.upper) { c.failure = 'GOAL_STATE_INVALID: The goal state is not valid for the group. Check the joint count and the limits against `joint_names`.'; c.success = false; return false; }
    if (method.startsWith('reset_')) {
      sim.cancel('gripper'); sim.fingers = position; c.success = true; c.reported = false;
      s.ros(call, 'action', '/execute_trajectory', 'moveit_msgs/action/ExecuteTrajectory', 'Reset gripper with a dummy joint trajectory'); return true;
    }
    s.ros(call, 'service', '/plan_kinematic_path', 'moveit_msgs/srv/GetMotionPlan', 'Plan hand joint configuration', 'SUCCESS');
    const duration = Math.abs(position - sim.fingers) / m.fingers.max_velocity;
    const names = m.gripper.joint_names, all = x => names.map(() => x);
    const trajectory = { joint_names: [...names], points: [
      { positions: all(sim.fingers), velocities: all(0), accelerations: all(0), time_from_start: 0 },
      { positions: all(position), velocities: all(0), accelerations: all(0), time_from_start: duration },
    ] };
    s.ros(call, 'action', '/execute_trajectory', 'moveit_msgs/action/ExecuteTrajectory', 'Execute hand joint trajectory', 'ACCEPTED');
    sim.execute(trajectory, 'gripper'); return true;
  }
  switch (method) {
    case 'wait_until_executed': return yield* waitForController(s, 'gripper', a);
    case 'query_state': return stateEnum(sim.isBusy('gripper') ? 'EXECUTING' : 'IDLE');
    case 'last_failure': return c.failure;
    case 'cancel_execution':
      if (!sim.isBusy('gripper')) return false;
      s.ros(call, 'service', '/execute_trajectory/_action/cancel_goal', 'action_msgs/srv/CancelGoal', 'Cancel tracked hand goal'); return sim.cancel('gripper');
    case 'get_last_execution_error_code': return c.error ? { __class__: 'MoveItErrorCodes', val: errorValue(c.error) } : null;
    case 'get_execution_future': return c.active ? s.future({ execution_id: c.active }) : null;
    case 'force_reset_executing_state': sim.cancel('gripper'); return null;
    case 'reset_new_joint_state_checker': return null;
    case 'destroy': sim.cancel('gripper'); return null;
  }
  valueError(`Unknown gripper method ${method}().`);
}

/** Acknowledged modern Servo lifecycle and scaled twist/jog publications. */
export function* servoOperation(s, method, a, call) {
  const sim = s.sim;
  const lifecycle = enable => {
    s.ros(call, 'service', '/servo_node/pause_servo', 'std_srvs/srv/SetBool', `pause=${!enable}`, 'SUCCESS');
    return enable ? sim.servoEnable() : sim.servoDisable();
  };
  if (method === 'enable') return s.closed || s.servoClosed ? false : lifecycle(true);
  if (method === 'disable') return lifecycle(false);
  if (method === 'shutdown' || method === 'destroy') { lifecycle(false); s.servoClosed = true; return method === 'destroy' ? null : true; }
  if (method === 'wait_until_ready' || method === 'wait_until_command_ready') {
    if (s.servoClosed) return false;
    if (!sim.servo.enabled) return false;
    if (method === 'wait_until_command_ready' && a.command_type != null) {
      if (![0, 1].includes(a.command_type)) valueError(`Unsupported Servo command type: ${a.command_type}`);
      if (sim.servo.active_command_type !== a.command_type) { s.ros(call, 'service', '/servo_node/switch_command_type', 'moveit_msgs/srv/ServoCommandType', a.command_type === 0 ? 'JOINT_JOG' : 'TWIST', 'SUCCESS'); sim.servo.active_command_type = a.command_type; }
    }
    return true;
  }
  if (s.closed || s.servoClosed) return false;
  if (method === '__call__') a = { ...a, enable_if_disabled: true };
  let linear, angular, names, velocities;
  if (method === 'servo_jog') {
    if (!Array.isArray(a.joint_names)) valueError('`joint_names` must be a sequence of strings');
    names = a.joint_names; velocities = finiteVector(a.velocities, 'velocities');
    if (names.length !== velocities.length) valueError('`joint_names` and `velocities` must have the same length!');
    if (names.some(n => typeof n !== 'string' || !n)) valueError('`joint_names` must contain non-empty strings');
  } else { linear = finiteVector(a.linear, 'linear', 3); angular = finiteVector(a.angular, 'angular', 3); }
  if (!sim.servo.enabled && !a.enable_if_disabled) return false;
  if (!sim.servo.enabled) lifecycle(true);
  const type = method === 'servo_jog' ? 0 : 1;
  if (sim.servo.active_command_type !== type) s.ros(call, 'service', '/servo_node/switch_command_type', 'moveit_msgs/srv/ServoCommandType', type === 0 ? 'JOINT_JOG' : 'TWIST', 'SUCCESS');
  const sent = type === 0 ? sim.servoJog(names, velocities) : sim.servoTwist(linear.map(x => x * s.servoConfig.linear_speed), angular.map(x => x * s.servoConfig.angular_speed), s.servoConfig.frame_id);
  if (sent) s.ros(call, 'topic', type === 0 ? '/servo_node/delta_joint_cmds' : '/servo_node/delta_twist_cmds', type === 0 ? 'control_msgs/msg/JointJog' : 'geometry_msgs/msg/TwistStamped', type === 0 ? 'Joint velocity command' : `Twist in ${s.servoConfig.frame_id}`);
  return sent;
}
