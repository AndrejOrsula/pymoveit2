// The Franka Emika Panda as the MoveIt 2 quick start of the pymoveit2 README
// configures it: moveit_resources_panda_description (URDF) and
// moveit_resources_panda_moveit_config (SRDF, joint_limits.yaml), branch ros2.
// Position limits are the URDF's safety_controller soft limits, which MoveIt
// uses where they are tighter than the hard limits. Velocity and acceleration
// limits are joint_limits.yaml's. Lengths are meters, angles radians.

import { LINK_SPHERES } from './geometry.js';

const HALF_PI = 1.57079632679;

/** @type {import('./types.js').RobotModel} */
export const PANDA = Object.freeze({
  id: 'panda',
  name: 'panda',
  label: 'Franka Emika Panda',
  source: 'moveit_resources (BSD-3-Clause) and franka_description meshes (Apache-2.0)',
  // What `RobotDescription` discovers from this SRDF, so what `connect()` uses.
  group_name: 'panda_arm',
  base_link_name: 'panda_link0',
  end_effector_name: 'panda_link8',
  planning_frame: 'panda_link0',
  joint_names: [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
  ],
  default_state: 'ready',
  // Each revolute joint rotates its child link about its own z axis.
  joints: [
    { name: 'panda_joint1', parent: 'panda_link0', child: 'panda_link1', xyz: [0, 0, 0.333], rpy: [0, 0, 0], axis: [0, 0, 1], lower: -2.8973, upper: 2.8973, max_velocity: 2.175, max_acceleration: 3.75 },
    { name: 'panda_joint2', parent: 'panda_link1', child: 'panda_link2', xyz: [0, 0, 0], rpy: [-HALF_PI, 0, 0], axis: [0, 0, 1], lower: -1.7628, upper: 1.7628, max_velocity: 2.175, max_acceleration: 1.875 },
    { name: 'panda_joint3', parent: 'panda_link2', child: 'panda_link3', xyz: [0, -0.316, 0], rpy: [HALF_PI, 0, 0], axis: [0, 0, 1], lower: -2.8973, upper: 2.8973, max_velocity: 2.175, max_acceleration: 2.5 },
    { name: 'panda_joint4', parent: 'panda_link3', child: 'panda_link4', xyz: [0.0825, 0, 0], rpy: [HALF_PI, 0, 0], axis: [0, 0, 1], lower: -3.0718, upper: 0.0175, max_velocity: 2.175, max_acceleration: 3.125 },
    { name: 'panda_joint5', parent: 'panda_link4', child: 'panda_link5', xyz: [-0.0825, 0.384, 0], rpy: [-HALF_PI, 0, 0], axis: [0, 0, 1], lower: -2.8973, upper: 2.8973, max_velocity: 2.61, max_acceleration: 3.75 },
    { name: 'panda_joint6', parent: 'panda_link5', child: 'panda_link6', xyz: [0, 0, 0], rpy: [HALF_PI, 0, 0], axis: [0, 0, 1], lower: -0.0175, upper: 3.7525, max_velocity: 2.61, max_acceleration: 5.0 },
    { name: 'panda_joint7', parent: 'panda_link6', child: 'panda_link7', xyz: [0.088, 0, 0], rpy: [HALF_PI, 0, 0], axis: [0, 0, 1], lower: -2.8973, upper: 2.8973, max_velocity: 2.61, max_acceleration: 5.0 },
  ],
  // Fixed joints after joint 7.
  fixed: [
    { name: 'panda_joint8', parent: 'panda_link7', child: 'panda_link8', xyz: [0, 0, 0.107], rpy: [0, 0, 0] },
    { name: 'panda_hand_joint', parent: 'panda_link8', child: 'panda_hand', xyz: [0, 0, 0], rpy: [0, 0, -0.785398163397] },
    // Not in moveit_resources' URDF: the point between the fingertips, which
    // franka_description names panda_hand_tcp. Lessons use it for grasping.
    { name: 'panda_hand_tcp_joint', parent: 'panda_hand', child: 'panda_hand_tcp', xyz: [0, 0, 0.1034], rpy: [0, 0, 0] },
  ],
  // Both fingers slide along the hand's y axis, the right one mirrored: its
  // frame (and mesh) is turned by pi about z, so it slides along its own +y.
  fingers: {
    joint_names: ['panda_finger_joint1', 'panda_finger_joint2'],
    links: ['panda_leftfinger', 'panda_rightfinger'],
    parent: 'panda_hand',
    origins: [{ xyz: [0, 0, 0.0584], rpy: [0, 0, 0] }, { xyz: [0, 0, 0.0584], rpy: [0, 0, Math.PI] }],
    axes: [[0, 1, 0], [0, 1, 0]],
    lower: 0.0,
    upper: 0.04,
    max_velocity: 0.1,
    max_acceleration: 1.0,
  },
  gripper: {
    group_name: 'hand',
    // What `moveit2_gripper_kwargs()` derives: the joints both states set.
    joint_names: ['panda_finger_joint1', 'panda_finger_joint2'],
    // SRDF group states of `hand`.
    open_joint_positions: [0.035, 0.035],
    closed_joint_positions: [0.0, 0.0],
    // Fingertip pads, in the hand frame: they close along y, an object fits
    // between them at z 0.065 to 0.115 and within 1 cm of the x center line,
    // and a pad is at y = ±(offset + finger position).
    grasp: { frame: 'panda_hand', closing: 1, approach: 2, depth: [0.065, 0.115], lateral: 0.01, offset: 0 },
  },
  // Links rigidly attached to the arm's tip, valid IK targets.
  ik_links: ['panda_link8', 'panda_hand', 'panda_hand_tcp'],
  // IK rejects positions beyond this sphere at once (the Panda reaches 0.855 m).
  reach: { center: [0, 0, 0], radius: 1.4 },
  // kinematics.yaml: KDL solves position and orientation.
  position_only_ik: false,
  controllers: { arm: 'panda_arm_controller', gripper: 'panda_hand_controller' },
  servo: { check_collisions: true },
  // The viewer: mesh directory, camera target and position, floor height and
  // the scale of the floor and markers.
  view: { assets: 'panda', target: [0.2, 0, 0.55], camera: [1.8, -1.35, 1.15], ground: 0, scale: 1 },
  // Every SRDF group as `RobotDescription.group()` derives it; mimic joints are passive.
  groups: {
    panda_arm: { joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'], passive_joint_names: [], base_link_name: 'panda_link0', end_effector_name: 'panda_link8' },
    hand: { joint_names: ['panda_finger_joint1'], passive_joint_names: ['panda_finger_joint2'], base_link_name: 'panda_hand', end_effector_name: 'panda_rightfinger' },
    panda_arm_hand: { joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1'], passive_joint_names: ['panda_finger_joint2'], base_link_name: 'panda_link0', end_effector_name: 'panda_rightfinger' },
  },
  // SRDF group states, in each group's joint order (both fingers for `hand`).
  named_states: {
    panda_arm: {
      ready: [0, -0.785, 0, -2.356, 0, 1.571, 0.785],
      extended: [0, 0, 0, 0, 0, 1.571, 0.785],
      transport: [0, -0.5599, 0, -2.97, 0, 0, 0.785],
    },
    hand: {
      open: [0.035, 0.035],
      close: [0.0, 0.0],
    },
  },
  // SRDF <disable_collisions>: link pairs the self-collision check skips.
  disabled_collisions: [
    ['panda_hand', 'panda_leftfinger'],
    ['panda_hand', 'panda_link3'],
    ['panda_hand', 'panda_link4'],
    ['panda_hand', 'panda_link6'],
    ['panda_hand', 'panda_link7'],
    ['panda_hand', 'panda_rightfinger'],
    ['panda_leftfinger', 'panda_link3'],
    ['panda_leftfinger', 'panda_link4'],
    ['panda_leftfinger', 'panda_link6'],
    ['panda_leftfinger', 'panda_link7'],
    ['panda_leftfinger', 'panda_rightfinger'],
    ['panda_link0', 'panda_link1'],
    ['panda_link0', 'panda_link2'],
    ['panda_link0', 'panda_link3'],
    ['panda_link0', 'panda_link4'],
    ['panda_link1', 'panda_link2'],
    ['panda_link1', 'panda_link3'],
    ['panda_link1', 'panda_link4'],
    ['panda_link2', 'panda_link3'],
    ['panda_link2', 'panda_link4'],
    ['panda_link2', 'panda_link6'],
    ['panda_link3', 'panda_link4'],
    ['panda_link3', 'panda_link5'],
    ['panda_link3', 'panda_link6'],
    ['panda_link3', 'panda_link7'],
    ['panda_link3', 'panda_rightfinger'],
    ['panda_link4', 'panda_link5'],
    ['panda_link4', 'panda_link6'],
    ['panda_link4', 'panda_link7'],
    ['panda_link4', 'panda_rightfinger'],
    ['panda_link5', 'panda_link6'],
    ['panda_link6', 'panda_link7'],
    ['panda_link6', 'panda_rightfinger'],
    ['panda_link7', 'panda_rightfinger'],
  ],
  // Collision spheres per link, in the link's frame (web/scripts/build_assets.py).
  collision_spheres: {
    panda_link0: LINK_SPHERES.link0,
    panda_link1: LINK_SPHERES.link1,
    panda_link2: LINK_SPHERES.link2,
    panda_link3: LINK_SPHERES.link3,
    panda_link4: LINK_SPHERES.link4,
    panda_link5: LINK_SPHERES.link5,
    panda_link6: LINK_SPHERES.link6,
    panda_link7: LINK_SPHERES.link7,
    panda_hand: LINK_SPHERES.hand,
    // franka_description has no finger collision mesh; two spheres per finger
    // cover the visual one (x within ±0.011, y 0..0.026, z 0..0.054).
    panda_leftfinger: [[0, 0.012, 0.016, 0.014], [0, 0.009, 0.042, 0.013]],
    panda_rightfinger: [[0, 0.012, 0.016, 0.014], [0, 0.009, 0.042, 0.013]],
  },
  // Visual meshes per link (web/static/assets/panda/*.glb, link frames, Z up).
  meshes: {
    panda_link0: 'link0',
    panda_link1: 'link1',
    panda_link2: 'link2',
    panda_link3: 'link3',
    panda_link4: 'link4',
    panda_link5: 'link5',
    panda_link6: 'link6',
    panda_link7: 'link7',
    panda_hand: 'hand',
    panda_leftfinger: 'finger',
    panda_rightfinger: 'finger',
  },
});
