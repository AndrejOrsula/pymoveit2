// The ROBOTIS OpenMANIPULATOR-X as the RMinS course configures it:
// open_manipulator_description (URDF) and open_manipulator_moveit_config
// (SRDF, joint_limits.yaml, kinematics.yaml, servo.yaml) of
// snt-spacer/open_manipulator-RMinS at 5a2e3de9a7eb (Apache-2.0), without the
// course's mounting platform. Velocity and acceleration limits are
// joint_limits.yaml's. Lengths are meters, angles radians.

import { OMX_LINK_SPHERES } from './geometry.js';

// The URDF writes quarter turns as 1.5708.
const QUARTER = 1.5708;
const finger = (y) => [
  [-0.006, 0.008 * y, 0.012, 0.018],
  [-0.006, 0.008 * y, -0.012, 0.018],
  [0.03, 0.011 * y, 0, 0.012],
  [0.052, 0.009 * y, 0, 0.011],
];

/** @type {import('./types.js').RobotModel} */
export const OPEN_MANIPULATOR_X = Object.freeze({
  id: 'open_manipulator_x',
  name: 'open_manipulator_x',
  label: 'OpenMANIPULATOR-X',
  source: 'open_manipulator-RMinS (Apache-2.0)',
  // What `RobotDescription` discovers from this SRDF, so what `connect()` uses.
  // The SRDF lists the arm group by joints, and pymoveit2 then takes the first
  // link those joints move as the base: link2, which joint1 turns.
  group_name: 'arm',
  base_link_name: 'link2',
  end_effector_name: 'link5',
  planning_frame: 'world',
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4'],
  default_state: 'home',
  joints: [
    { name: 'joint1', parent: 'link1', child: 'link2', xyz: [0.012, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 1], lower: -Math.PI, upper: Math.PI, max_velocity: 5.0, max_acceleration: 5.0 },
    { name: 'joint2', parent: 'link2', child: 'link3', xyz: [0, 0, 0.0595], rpy: [0, 0, 0], axis: [0, 1, 0], lower: -1.5, upper: 1.5, max_velocity: 5.0, max_acceleration: 5.0 },
    { name: 'joint3', parent: 'link3', child: 'link4', xyz: [0.024, 0, 0.128], rpy: [0, 0, 0], axis: [0, 1, 0], lower: -1.5, upper: 1.4, max_velocity: 5.0, max_acceleration: 5.0 },
    { name: 'joint4', parent: 'link4', child: 'link5', xyz: [0.124, 0, 0], rpy: [0, 0, 0], axis: [0, 1, 0], lower: -1.7, upper: 1.97, max_velocity: 5.0, max_acceleration: 5.0 },
  ],
  // base_link is the course's platform, on its side under the arm. The demo
  // keeps its frame, so world coordinates match the course; the arm faces -y.
  fixed: [
    { name: 'world_to_base', parent: 'world', child: 'base_link', xyz: [0, 0, 0], rpy: [QUARTER, 0, 0] },
    { name: 'base_to_arm', parent: 'base_link', child: 'link1', xyz: [0, 0.025, -0.1263], rpy: [-QUARTER, -QUARTER, 0] },
    { name: 'end_effector_joint', parent: 'link5', child: 'end_effector_link', xyz: [0.0817, 0, 0], rpy: [0, 0, 0] },
  ],
  // gripper_right_joint mimics gripper_left_joint and slides the other way.
  fingers: {
    joint_names: ['gripper_left_joint', 'gripper_right_joint'],
    links: ['gripper_left_link', 'gripper_right_link'],
    parent: 'link5',
    origins: [{ xyz: [0.0817, 0.021, 0], rpy: [0, 0, 0] }, { xyz: [0.0817, -0.021, 0], rpy: [0, 0, 0] }],
    axes: [[0, 1, 0], [0, -1, 0]],
    lower: -0.011,
    upper: 0.02,
    // joint_limits.yaml allows 5 m/s, which would close the fingers in a blink;
    // the demo slows them so the motion stays visible.
    max_velocity: 0.05,
    max_acceleration: 5.0,
  },
  gripper: {
    group_name: 'gripper',
    // What `moveit2_gripper_kwargs()` derives: the joints both states set.
    joint_names: ['gripper_left_joint'],
    // SRDF group states of `gripper`.
    open_joint_positions: [0.019],
    closed_joint_positions: [-0.01],
    // Palm pads, in link5: they close along y, an object fits between them at
    // x 0.104 to 0.147 and within 1.3 cm of the z center line, and a pad is at
    // y = ±(0.021 + finger position).
    grasp: { frame: 'link5', closing: 1, approach: 0, depth: [0.104, 0.147], lateral: 0.013, offset: 0.021 },
  },
  // Links rigidly attached to the arm's tip, valid IK targets.
  ik_links: ['link5', 'end_effector_link'],
  // IK rejects positions beyond this sphere around joint2 at once (the
  // end_effector_link reaches 0.336 m from it).
  reach: { center: [0, 0.1143, 0.0845], radius: 0.4 },
  // kinematics.yaml: KDL with position_only_ik, as a 4-joint arm cannot reach
  // most orientations.
  position_only_ik: true,
  controllers: { arm: 'arm_controller', gripper: 'gripper_controller' },
  // servo.yaml turns collision checking off.
  servo: { check_collisions: false },
  // The viewer, as for the Panda; the floor is where link1 stands.
  view: { assets: 'open_manipulator_x', target: [0, 0.02, 0.13], camera: [0.44, -0.42, 0.36], ground: 0.025, scale: 0.3 },
  // Every SRDF group as `RobotDescription.group()` derives it; mimic joints are passive.
  groups: {
    arm: { joint_names: ['joint1', 'joint2', 'joint3', 'joint4'], passive_joint_names: [], base_link_name: 'link2', end_effector_name: 'link5' },
    gripper: { joint_names: ['gripper_left_joint'], passive_joint_names: ['gripper_right_joint'], base_link_name: 'gripper_left_link', end_effector_name: 'gripper_right_link' },
  },
  // SRDF group states, in the order of the joints each state sets.
  named_states: {
    arm: {
      init: [0, 0, 0, 0],
      home: [0, -1, 0.7, 0.3],
    },
    gripper: {
      open: [0.019],
      close: [-0.01],
    },
  },
  // SRDF <disable_collisions>: link pairs the self-collision check skips.
  disabled_collisions: [
    ['gripper_left_link', 'gripper_right_link'],
    ['gripper_left_link', 'link4'],
    ['gripper_left_link', 'link5'],
    ['gripper_right_link', 'link4'],
    ['gripper_right_link', 'link5'],
    ['link1', 'link2'],
    ['link1', 'link3'],
    ['link2', 'link3'],
    ['link3', 'link4'],
    ['link4', 'link5'],
  ],
  // Collision spheres per link, in the link's frame (web/scripts/build_assets.py);
  // four spheres per palm, set by hand like the Panda's fingers, cover the
  // palm and its pad (y 0 to 0.016 from x 0.02, z within ±0.0165).
  collision_spheres: {
    link1: OMX_LINK_SPHERES.link1,
    link2: OMX_LINK_SPHERES.link2,
    link3: OMX_LINK_SPHERES.link3,
    link4: OMX_LINK_SPHERES.link4,
    link5: OMX_LINK_SPHERES.link5,
    gripper_left_link: finger(1),
    gripper_right_link: finger(-1),
  },
  // In place of the platform, base_link collides as a floor slab: the floor
  // grid's 0.72 m square around the world origin, whose top face is the floor
  // link1 stands on (world z = 0.025). In base_link's frame, y points up.
  collision_boxes: { base_link: [{ xyz: [0, 0.015, 0], size: [0.72, 0.02, 0.72] }] },
  // Visual meshes per link (web/static/assets/open_manipulator_x/*.glb, link frames).
  meshes: {
    link1: 'link1',
    link2: 'link2',
    link3: 'link3',
    link4: 'link4',
    link5: 'link5',
    gripper_left_link: 'gripper_left_palm',
    gripper_right_link: 'gripper_right_palm',
  },
});
