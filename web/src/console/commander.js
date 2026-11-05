// Short commands in the spirit of moveit_commander's command line ("go ready",
// "go up 0.1", "open"). Each one expands into the pymoveit2 Python that does
// the same thing, so the console can echo that code before running it.
// Positions are in the planning frame; directions are the robot's own.

import { linkTransforms, poseInLink } from '../core/kinematics.js';

const UP = [0, 0, 1];
const PLANNERS = {
  ompl: ['ompl', 'RRTConnectkConfigDefault'],
  rrtconnect: ['ompl', 'RRTConnectkConfigDefault'],
  rrtstar: ['ompl', 'RRTstarkConfigDefault'],
  ptp: ['pilz_industrial_motion_planner', 'PTP'],
  lin: ['pilz_industrial_motion_planner', 'LIN'],
};

const states = (model) => Object.keys(model.named_states[model.group_name]);

/** The argument counts each command accepts; anything else is rejected, as a Python call would be. */
function arity(word, words, model) {
  // Only a direction takes a distance: "go up 0.1", but "go ready".
  const go = directions(model)[words[0]?.toLowerCase()] ? [1, 2] : [1];
  const counts = {
    help: [0], commands: [0], go, pose: [3, 7], line: [3, 7], joints: [model.joints.length],
    open: [0], close: [0], where: [0], ik: [3], add: [5, 6], remove: [1], clear: [0],
    attach: [1], detach: [1], speed: [1], planner: [1], stop: [0], why: [0],
  };
  return counts[word];
}

/** Throws when a known command gets an argument count it does not accept. */
function checkArity(word, words, model) {
  const counts = arity(word, words, model);
  if (!counts || counts.includes(words.length)) return;
  const plural = counts.at(-1) === 1 ? '' : 's';
  throw new Error(`${word} takes ${counts.join(' or ')} argument${plural} (${words.length} given); type help`);
}

/** One line of help per command, shown by `help`. */
export function commands(model) {
  const step = model.reach.radius > 1 ? 0.1 : 0.05;
  return [
    [`go ${states(model).join('|')}`, 'move to a named state from the SRDF'],
    ['go up|down|left|right|forward|backward [m]', `straight-line move of the gripper (default ${step} m)`],
    ['go random', 'move to random joint positions'],
    [model.position_only_ik ? 'pose x y z' : 'pose x y z [qx qy qz qw]', `plan and move the gripper to a ${model.position_only_ik ? 'position' : 'pose'} in ${model.planning_frame}`],
    ['line x y z', 'like pose, but along a straight Cartesian line'],
    [`joints j1 … j${model.joints.length}`, 'move to joint positions in radians'],
    ['open | close', 'open or close the gripper'],
    ['where', 'print the joint positions and the gripper pose'],
    ['ik x y z', `solve inverse kinematics without moving (x y z in ${model.base_link_name}, like compute_fk())`],
  ['add box|sphere|cylinder name x y z [size]', 'add a collision object'],
  ['remove name | clear', 'remove one object or all of them'],
  ['attach name | detach name', 'attach an object to the hand or release it'],
  ['speed 0.1…1', 'scale the arm\'s velocity and acceleration'],
  ['planner ompl|rrtstar|ptp|lin', 'pick the planning pipeline and planner'],
  ['stop', 'cancel the running motion'],
  ['why', 'print why the last motion failed'],
  ];
}

const round = (value, digits = 3) => {
  const r = Number(value.toFixed(digits));
  return Object.is(r, -0) ? 0 : r;
};
const float = (value) => {
  const r = round(value, 4);
  return Number.isInteger(r) ? `${r}.0` : String(r);
};
const vec = (values) => `[${values.map(float).join(', ')}]`;
const pyList = (names) => `[${names.map((n) => `'${n}'`).join(', ')}]`;

function numbers(words, count, what) {
  const values = words.slice(0, count).map(Number);
  if (values.some((v) => !Number.isFinite(v))) {
    throw new Error(`${what} needs ${count} numbers`);
  }
  return values;
}

/** True when a console line reads as a commander command rather than Python. */
export function isCommand(line) {
  const trimmed = line.trim();
  const word = trimmed.split(/\s+/)[0];
  const known = new Set(['go', 'pose', 'line', 'joints', 'open', 'close', 'where', 'ik', 'add', 'remove', 'clear', 'attach', 'detach', 'speed', 'planner', 'stop', 'why', 'help', 'commands']);
  if (!known.has(word)) return false;
  // "help(robot)", "open = 3" and "close()" are Python.
  const rest = trimmed.slice(word.length);
  return !/^\s*[(.=[]/.test(rest);
}

/** Unit directions in the planning frame: forward is the x axis of the arm's mount. */
function directions(model) {
  const m = linkTransforms(model, model.joints.map(() => 0))[model.joints[0].parent];
  const forward = [m[0], m[1], 0].map((x) => round(x, 6));
  const left = [-forward[1], forward[0], 0];
  const negate = (v) => v.map((x) => (x ? -x : 0));
  return { up: UP, down: negate(UP), forward, backward: negate(forward), back: negate(forward), left, right: negate(left) };
}

/**
 * Expands one command into Python statements.
 * `state()` returns the sim's SimState and `model` the robot model.
 * Returns {python: string[]} or {help: true}; throws Error with a usage hint.
 */
export function expand(line, { state, model, random = Math.random }) {
  const [word, ...words] = line.trim().split(/\s+/);
  checkArity(word, words, model);
  const ee = () => state().ee;
  // pymoveit2 reads goals and objects in base_link_name; name the planning frame when they differ.
  const frame = model.base_link_name === model.planning_frame ? '' : `, frame_id='${model.planning_frame}'`;
  // A 4-joint arm cannot hold an arbitrary orientation, so its goals free it.
  const free = model.position_only_ik ? ', tolerance_orientation=3.1416' : '';
  switch (word) {
    case 'help':
    case 'commands':
      return { help: true, python: [] };
    case 'go': {
      const target = (words[0] ?? '').toLowerCase();
      if (target === model.default_state || target === 'home') return { python: ['robot.move_to_configuration()'] };
      if (states(model).includes(target)) return { python: [`robot.move_to_configuration(robot.joint_positions('${target}'))`] };
      const direction = directions(model)[target];
      if (direction) {
        const distance = words[1] === undefined ? (model.reach.radius > 1 ? 0.1 : 0.05) : Number(words[1]);
        if (!Number.isFinite(distance)) throw new Error(`usage: go ${target} [meters]`);
        const { position, quat_xyzw } = ee();
        const goal = position.map((p, i) => p + direction[i] * distance);
        return { python: [`robot.move_to_pose(${vec(goal)}, ${vec(quat_xyzw)}${frame}, cartesian=True)`] };
      }
      if (target === 'random' || target === 'rand') {
        const q = model.joints.map(({ lower, upper }) => {
          const margin = 0.15 * (upper - lower);
          return lower + margin + random() * (upper - lower - 2 * margin);
        });
        return { python: [`robot.move_to_configuration(${vec(q)})`] };
      }
      throw new Error(`usage: go ${states(model).join('|')}|up|down|left|right|forward|backward [m]|random`);
    }
    case 'pose':
    case 'line': {
      const position = numbers(words, 3, word);
      const quat = words.length >= 7 ? numbers(words.slice(3), 4, 'the orientation') : ee().quat_xyzw;
      const cartesian = word === 'line' ? ', cartesian=True' : free;
      return { python: [`robot.move_to_pose(${vec(position)}, ${vec(quat)}${frame}${cartesian})`] };
    }
    case 'joints': {
      const q = numbers(words, model.joints.length, 'joints');
      return { python: [`robot.move_to_configuration(${vec(q)})`] };
    }
    case 'open':
      return { python: ['robot.gripper.open()'] };
    case 'close':
      return { python: ['robot.gripper.close()'] };
    case 'where':
      return { python: ['robot.joint_state.position', 'robot.compute_fk()'] };
    case 'ik': {
      const position = numbers(words, 3, 'ik');
      // compute_ik() reads the pose in base_link_name, as compute_fk() reports it.
      const { quat_xyzw } = poseInLink(model, state().q, ee());
      return { python: [`robot.compute_ik(${vec(position)}, ${vec(quat_xyzw)})`] };
    }
    case 'add': {
      const [type, id, ...rest] = words;
      if (!['box', 'sphere', 'cylinder'].includes(type) || !id) throw new Error('usage: add box|sphere|cylinder name x y z [size]');
      const position = numbers(rest, 3, 'add');
      const size = rest[3] === undefined ? 0.1 : Number(rest[3]);
      if (!Number.isFinite(size) || size <= 0) throw new Error('the size must be a positive number');
      const name = `'${id.replace(/'/g, '')}'`;
      if (type === 'box') return { python: [`robot.add_collision_box(${name}, ${vec([size, size, size])}, position=${vec(position)}, quat_xyzw=[0.0, 0.0, 0.0, 1.0]${frame})`] };
      if (type === 'sphere') return { python: [`robot.add_collision_sphere(${name}, ${float(size / 2)}, position=${vec(position)}, quat_xyzw=[0.0, 0.0, 0.0, 1.0]${frame})`] };
      return { python: [`robot.add_collision_cylinder(${name}, ${float(size * 2)}, ${float(size / 2)}, position=${vec(position)}, quat_xyzw=[0.0, 0.0, 0.0, 1.0]${frame})`] };
    }
    case 'remove':
      if (!words[0]) throw new Error('usage: remove name');
      return { python: [`robot.remove_collision_object('${words[0].replace(/'/g, '')}')`] };
    case 'clear':
      return { python: ['robot.clear_all_collision_objects()'] };
    case 'attach':
      if (!words[0]) throw new Error('usage: attach name');
      return { python: [`robot.attach_collision_object('${words[0].replace(/'/g, '')}', touch_links=${pyList([model.fingers.parent, ...model.fingers.links])})`] };
    case 'detach':
      if (!words[0]) throw new Error('usage: detach name');
      return { python: [`robot.detach_collision_object('${words[0].replace(/'/g, '')}')`] };
    case 'speed': {
      const factor = Number(words[0]);
      if (!(factor > 0 && factor <= 1)) throw new Error('usage: speed 0.1…1 (a fraction of the joint limits)');
      return { python: [`robot.arm.max_velocity = ${float(factor)}`, `robot.arm.max_acceleration = ${float(factor)}`] };
    }
    case 'planner': {
      const choice = PLANNERS[(words[0] ?? '').toLowerCase()];
      if (!choice) throw new Error('usage: planner ompl|rrtstar|ptp|lin');
      return { python: [`robot.arm.pipeline_id = '${choice[0]}'`, `robot.arm.planner_id = '${choice[1]}'`] };
    }
    case 'stop':
      return { python: ['robot.arm.cancel_execution()'] };
    case 'why':
      return { python: ['robot.last_failure()'] };
    default:
      throw new Error(`unknown command '${word}'; type help`);
  }
}
