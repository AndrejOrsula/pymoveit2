// The guided lessons. Each step's `code` is real pymoveit2 Python that the
// console interpreter runs line by line against the simulated robot. `expect`
// matches printed text; `expectPosition` checks Point coordinates within 1 mm.
// The UI and tests check that every lesson behaves as its text says.
//
// Text is trusted HTML written here, not user input. `part` groups lessons in
// the overview.

import { OMX_LESSONS } from './lessons-omx.js';

/** Touch links for a cube held between the fingers. */
const HAND = "['panda_hand', 'panda_leftfinger', 'panda_rightfinger']";

const PANDA_LESSONS = [
  {
    id: 'hello',
    part: 'Move',
    title: 'Hello, robot',
    api: ['connect()', 'move_to_pose', 'gripper.close', 'move_to_configuration'],
    intro: `<p><code>connect()</code> joins the robot's ROS&nbsp;2 system and gives you <code>robot</code>, which drives the arm, the gripper and servo. Here it is ready; your own script starts with:</p><div class="code"><pre><code>with connect() as robot:</code></pre></div>`,
    steps: [
      {
        text: `Move the gripper to a <strong>pose</strong>: a position in meters from the base <code>panda_link0</code>, and an orientation as a quaternion <code>[x, y, z, w]</code>. <code>[1, 0, 0, 0]</code> points the hand down.`,
        code: 'robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])',
        expect: /^True$/,
        after: `<code>True</code>: MoveIt found a collision-free path; pymoveit2 executed it and waited. The <em>Last call</em> strip under the scene shows the ROS&nbsp;2 calls: <code>/plan_kinematic_path</code>, then <code>/execute_trajectory</code>.`,
      },
      {
        text: 'Close the gripper. Gripper commands return as soon as the goal is sent; the fingers keep moving.',
        code: 'robot.gripper.close()',
        expect: /^True$/,
      },
      {
        text: `Go back to the <code>ready</code> state of the robot's SRDF. Without arguments, <code>move_to_configuration()</code> picks the arm's default named state.`,
        code: 'robot.move_to_configuration()',
        expect: /^True$/,
        after: `That is the whole README example. Every call returns <code>True</code> or <code>False</code>; when it is <code>False</code>, <code>robot.last_failure()</code> says why.`,
      },
    ],
    outro: `<p>Try it yourself: drag the goal ball in the scene and press <strong>Move</strong> in the Control tab, or type <code>go extended</code> in the Console.</p>`,
  },
  {
    id: 'joints',
    part: 'Move',
    title: 'Joint space and named states',
    api: ['joint_state', 'joint_names', 'joint_positions', 'move_to_configuration'],
    intro: `<p>The Panda has seven revolute joints. A <em>configuration</em> is one angle per joint, in radians. Joint goals are the most predictable goals: the arm ends exactly there.</p>`,
    steps: [
      {
        text: 'Which joints does the arm have, and where are they now?',
        code: 'robot.joint_names\nrobot.joint_state.position',
        expect: /^\[/,
      },
      {
        text: `The SRDF names some configurations. <code>joint_positions(name)</code> reads one; the Panda's are <code>ready</code>, <code>extended</code> and <code>transport</code>.`,
        code: "robot.joint_positions('extended')",
        expect: /^\[0\.0, 0\.0, /,
      },
      {
        text: 'Move there.',
        code: "robot.move_to_configuration(robot.joint_positions('extended'))",
        expect: /^True$/,
      },
      {
        text: 'Or give the seven angles yourself. Look at the plot under the scene: every joint starts and stops together.',
        code: 'robot.move_to_configuration([0.5, -0.3, 0.0, -2.2, 0.0, 1.9, 1.3])',
        expect: /^True$/,
        after: `Switch the plot to <strong>Velocities</strong>: each curve stays within that joint's velocity limit. That is MoveIt's time parameterization.`,
      },
      {
        text: 'Back to ready.',
        code: 'robot.move_to_configuration()',
        expect: /^True$/,
      },
    ],
    outro: `<p>In the Control tab, the <strong>Joints</strong> sliders preview a configuration as a ghost before you move.</p>`,
  },
  {
    id: 'poses',
    part: 'Move',
    title: 'Poses and orientations',
    api: ['compute_fk', 'move_to_pose', 'quat_xyzw'],
    reset: true,
    intro: `<p>Pose goals say where the <em>end effector</em> should be, not how the joints get there; MoveIt solves inverse kinematics for you. The end effector here is <code>panda_link8</code>, the flange the hand is mounted on.</p>`,
    steps: [
      {
        text: `Where is the end effector now? <code>compute_fk()</code> asks MoveIt's <code>/compute_fk</code> service for a <code>PoseStamped</code>; <code>.pose.position</code> is its x, y and z in meters.`,
        code: 'robot.compute_fk().pose.position',
        expectPosition: [0.307, 0, 0.5903],
      },
      {
        text: 'Pointing down, half a turn about x:',
        code: 'robot.move_to_pose([0.5, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0])',
        expect: /^True$/,
      },
      {
        text: `Same position, turned 45° about the vertical axis. Quaternions are hard to write by hand: type roll, pitch and yaw into the Control tab's Goal card, and it shows <code>quat_xyzw</code>.`,
        code: 'robot.move_to_pose([0.5, -0.2, 0.3], [0.924, 0.383, 0.0, 0.0])',
        expect: /^True$/,
        after: `Only the hand turned, by a small motion of the last joint. Quaternions are <code>[x, y, z, w]</code> in pymoveit2, the order ROS messages use.`,
      },
      {
        text: `Both parts are required. Leave out the orientation and pymoveit2 raises a <code>ValueError</code> before anything moves.`,
        code: 'robot.move_to_pose([0.3, 0.3, 0.5])',
        expect: /^ValueError: Either `pose` or `position` and `quat_xyzw` must be specified!$/,
        after: `To loosen the orientation instead, pass a larger <code>tolerance_orientation</code> in radians.`,
      },
    ],
    outro: `<p>In the Control tab, drag the goal marker; press <kbd>R</kbd> or <strong>Turn</strong> in the scene to rotate it instead.</p>`,
  },
  {
    id: 'cartesian',
    part: 'Move',
    title: 'Straight lines',
    api: ['cartesian=True', 'cartesian_fraction_threshold'],
    reset: true,
    setup: ['robot.move_to_pose([0.5, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0])'],
    intro: `<p>A free-space plan only avoids collisions; the hand may swing in an arc. With <code>cartesian=True</code>, MoveIt's <code>/compute_cartesian_path</code> moves it along a straight line, as when approaching an object.</p>`,
    steps: [
      {
        text: 'A straight line 40 cm to the left. Turn on <strong>Path</strong> in the scene to see the line.',
        code: 'robot.move_to_pose([0.5, 0.2, 0.3], [1.0, 0.0, 0.0, 0.0], cartesian=True)',
        expect: /^True$/,
      },
      {
        text: 'Put a box in the way, then try to go straight back.',
        code: "robot.add_collision_box('box', [0.1, 0.1, 0.1], position=[0.5, 0.0, 0.25], quat_xyzw=[0.0, 0.0, 0.0, 1.0])\nrobot.move_to_pose([0.5, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0], cartesian=True)",
        expect: /^True$/,
        after: `<code>True</code>, but the arm stopped before the box. By default (<code>cartesian_fraction_threshold=0.0</code>) pymoveit2 executes whatever <em>fraction</em> of the line MoveIt found; the ROS&nbsp;2 tab shows it.`,
      },
      {
        text: 'Ask for the whole line or nothing:',
        code: "robot.move_to_pose([0.5, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0], cartesian=True, cartesian_fraction_threshold=1.0)\nrobot.last_failure()",
        expect: /Cartesian planner reached only/,
      },
      {
        text: 'Without <code>cartesian=True</code>, the planner goes around it.',
        code: "robot.move_to_pose([0.5, -0.2, 0.3], [1.0, 0.0, 0.0, 0.0])\nrobot.remove_collision_object('box')",
        expect: /^True$/,
      },
    ],
    outro: `<p>The Control tab's <strong>Straight line</strong> checkbox does the same for the goal marker.</p>`,
  },
  {
    id: 'speed',
    part: 'Move',
    title: 'Speed and acceleration',
    api: ['max_velocity', 'max_acceleration'],
    reset: true,
    intro: `<p>MoveIt times each trajectory to the joint limits, scaled by two factors in (0,&nbsp;1]. pymoveit2 defaults them to <code>0.0</code>, which MoveIt replaces with 1.0, full speed, and a warning in the ROS&nbsp;2 tab.</p>`,
    steps: [
      {
        text: 'A move at full speed. Note the duration next to the plot.',
        code: "robot.move_to_configuration(robot.joint_positions('extended'))",
        expect: /^True$/,
      },
      {
        text: 'Slow down to 20% of the limits and move back.',
        code: 'robot.arm.max_velocity = 0.2\nrobot.arm.max_acceleration = 0.2\nrobot.move_to_configuration()',
        expect: /^True$/,
        after: 'The same path takes about five times as long; the plot\'s time axis stretched.',
      },
      {
        text: `A classic mistake: <code>robot.max_velocity = 1.0</code>. <code>RobotSession</code> forwards attribute <em>reads</em> it lacks to <code>robot.arm</code>, but Python stores an assignment on the session object itself.`,
        code: 'robot.max_velocity = 1.0\nrobot.arm.max_velocity',
        expect: /^0\.2$/,
        after: 'The arm still uses 0.2. Always set motion parameters on <code>robot.arm</code>.',
      },
      {
        text: 'Restore full speed.',
        code: 'robot.arm.max_velocity = 1.0\nrobot.arm.max_acceleration = 1.0',
      },
    ],
    outro: `<p>The Control tab's <strong>Planning</strong> card has sliders for both factors; the console command <code>speed 0.5</code> sets both.</p>`,
  },
  {
    id: 'planners',
    part: 'Plan',
    title: 'Planners: OMPL and Pilz',
    api: ['pipeline_id', 'planner_id'],
    reset: true,
    setup: ["robot.add_collision_box('pillar', [0.12, 0.12, 0.5], position=[0.45, 0.0, 0.25], quat_xyzw=[0.0, 0.0, 0.0, 1.0])"],
    intro: `<p>MoveIt has several planning pipelines. pymoveit2 picks one with <code>pipeline_id</code> and a planner inside it with <code>planner_id</code>. The scene has a pillar between the arm's two sides.</p>`,
    steps: [
      {
        text: `The default is OMPL's <strong>RRTConnect</strong>: it grows two random trees, one from the start and one from the goal, until they meet. Turn on <strong>Tree</strong> in the scene to see them.`,
        code: "robot.arm.pipeline_id = 'ompl'\nrobot.arm.planner_id = 'RRTConnectkConfigDefault'\nrobot.move_to_pose([0.4, -0.35, 0.3], [1.0, 0.0, 0.0, 0.0])\nrobot.move_to_pose([0.4, 0.35, 0.3], [1.0, 0.0, 0.0, 0.0])",
        layer: 'tree',
        expect: /^True$/,
        after: 'It found a way around the pillar. Run the step again: sampling planners can find a different path each time.',
      },
      {
        text: `Pilz's <strong>PTP</strong> moves every joint in a straight line from start to goal, all together: fast and repeatable, but it never avoids obstacles; it fails if the path collides.`,
        code: "robot.arm.pipeline_id = 'pilz_industrial_motion_planner'\nrobot.arm.planner_id = 'PTP'\nrobot.move_to_pose([0.4, -0.35, 0.3], [1.0, 0.0, 0.0, 0.0])\nrobot.last_failure()",
        expect: /PLANNING_FAILED|^None$/,
        after: 'Through the pillar there is no way, so PTP refuses. Industrial cells often use Pilz because the same request always gives the same motion.',
      },
      {
        text: 'Remove the pillar. <strong>LIN</strong> moves the end effector along a straight line, timed with Cartesian velocity limits.',
        code: "robot.remove_collision_object('pillar')\nrobot.arm.planner_id = 'LIN'\nrobot.move_to_pose([0.4, -0.35, 0.3], [1.0, 0.0, 0.0, 0.0])",
        expect: /^True$/,
      },
      {
        text: 'Back to the default pipeline.',
        code: "robot.arm.pipeline_id = 'ompl'\nrobot.arm.planner_id = 'RRTConnectkConfigDefault'",
      },
    ],
    outro: `<p>The planner chip in the scene's corner shows which one is active; switch it in the Control tab.</p>`,
  },
  {
    id: 'obstacles',
    part: 'Plan',
    title: 'Obstacles and failures',
    api: ['add_collision_box', 'move_collision', 'allow_collisions', 'last_failure'],
    reset: true,
    intro: `<p>MoveIt plans around everything in its <em>planning scene</em>. pymoveit2 adds boxes, spheres, cylinders, cones and meshes by publishing to <code>/collision_object</code>.</p>`,
    steps: [
      {
        text: 'A table under the workspace and a wall in front of the robot.',
        code: "robot.add_collision_box('table', [0.8, 1.2, 0.04], position=[0.55, 0.0, -0.02], quat_xyzw=[0.0, 0.0, 0.0, 1.0])\nrobot.add_collision_box('wall', [0.04, 0.5, 0.3], position=[0.45, 0.0, 0.15], quat_xyzw=[0.0, 0.0, 0.0, 1.0])",
      },
      {
        text: 'Reach over the wall.',
        code: 'robot.move_to_pose([0.6, 0.0, 0.3], [1.0, 0.0, 0.0, 0.0])',
        expect: /^True$/,
      },
      {
        text: 'Now a goal inside the wall. The call returns <code>False</code>; <code>last_failure()</code> explains it and says what to try.',
        code: 'robot.move_to_pose([0.45, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])\nrobot.last_failure()',
        expect: /GOAL_IN_COLLISION/,
      },
      {
        text: `<code>allow_collisions</code> makes MoveIt ignore an object, here the wall. Use it for objects you mean to touch, such as a part to grasp.`,
        code: "robot.allow_collisions('wall', True)\nrobot.move_to_pose([0.45, 0.0, 0.3], [1.0, 0.0, 0.0, 0.0])",
        expect: /^True$/,
        after: `The <code>&lt;Future …&gt;</code> line is the pending reply of MoveIt's <code>/apply_planning_scene</code>: pymoveit2 sent the change and does not wait for it.`,
      },
      {
        text: 'Objects can move. Slide the wall aside, and clear the scene.',
        code: "robot.move_collision('wall', [0.45, 0.45, 0.15], [0.0, 0.0, 0.0, 1.0])\nrobot.move_to_configuration()\nrobot.clear_all_collision_objects()",
        expect: /^(True|None|<)/,
      },
    ],
    outro: `<p>In the Control tab's <strong>Scene</strong> card, add objects and drag them with the gizmo. Drop one into the arm's path while it moves: MoveIt stops the motion.</p>`,
  },
  {
    id: 'constraints',
    part: 'Plan',
    title: 'Keep it level',
    api: ['set_path_orientation_constraint', 'clear_path_constraints'],
    reset: true,
    setup: ['robot.move_to_pose([0.45, -0.3, 0.35], [1.0, 0.0, 0.0, 0.0])'],
    intro: `<p>A goal says where to end; a <em>path constraint</em> must hold all the way there. A common one: keep the hand upright, as if it held a full glass.</p>`,
    steps: [
      {
        text: 'Without a constraint, the hand may tilt on the way. Watch it.',
        code: "robot.move_to_pose([0.3, 0.35, 0.55], [1.0, 0.0, 0.0, 0.0])\nrobot.move_to_pose([0.45, -0.3, 0.35], [1.0, 0.0, 0.0, 0.0])",
        expect: /^True$/,
      },
      {
        text: `Constrain the orientation: within 0.1 rad about x and y, anything about z (π). The planner now samples only states that keep the hand level, which takes longer.`,
        code: "robot.set_path_orientation_constraint([1.0, 0.0, 0.0, 0.0], tolerance=(0.1, 0.1, 3.1416))\nrobot.move_to_pose([0.3, 0.35, 0.55], [1.0, 0.0, 0.0, 0.0])",
        expect: /^True$/,
        after: 'Every state along the path kept the hand within the tolerance.',
      },
      {
        text: `pymoveit2 clears goal and path constraints after every plan, so this tilted move is unconstrained. <code>clear_path_constraints()</code> drops ones not yet planned with.`,
        code: "robot.move_to_pose([0.45, -0.3, 0.35], [0.966, 0.0, 0.259, 0.0])\nrobot.move_to_configuration()",
        expect: /^True$/,
      },
    ],
    outro: `<p>The Control tab's <strong>Keep level</strong> checkbox adds the same constraint to the goal marker's moves.</p>`,
  },
  {
    id: 'pick',
    part: 'Apply',
    title: 'Pick and place',
    api: ['gripper', 'attach_collision_object', 'detach_collision_object'],
    reset: true,
    setup: [
      "robot.add_collision_box('table', [0.8, 1.2, 0.04], position=[0.55, 0.0, -0.02], quat_xyzw=[0.0, 0.0, 0.0, 1.0])",
      "robot.add_collision_box('cube', [0.04, 0.04, 0.05], position=[0.5, -0.25, 0.025], quat_xyzw=[0.0, 0.0, 0.0, 1.0])",
    ],
    intro: `<p>Goal: pick up the cube and set it down elsewhere. The fingertips sit 0.1034&nbsp;m below the flange <code>panda_link8</code>, so the flange goes to z&nbsp;=&nbsp;0.14 to grip the cube near its top.</p>`,
    steps: [
      {
        text: 'Open the gripper and move above the cube. <code>[0.924, -0.383, 0, 0]</code> turns the fingers to its sides.',
        code: 'robot.gripper.open()\nrobot.move_to_pose([0.5, -0.25, 0.26], [0.924, -0.383, 0.0, 0.0])',
        expect: /^True$/,
      },
      {
        text: 'Go straight down. MoveIt treats touching the cube as a collision, so allow it first.',
        code: "robot.allow_collisions('cube', True)\nrobot.move_to_pose([0.5, -0.25, 0.14], [0.924, -0.383, 0.0, 0.0], cartesian=True)",
        expect: /^True$/,
      },
      {
        text: 'Close the fingers and wait for them; they stop on the cube.',
        code: 'robot.gripper.close()\nrobot.gripper.wait_until_executed()',
        expect: /^(True|False)$/,
        after: 'The fingers stopped at the cube\'s half width, 2&nbsp;cm each.',
      },
      {
        text: `Attach the cube to the hand: MoveIt now moves it with the arm and checks it for collisions. <code>touch_links</code> are the links it may touch.`,
        code: `robot.attach_collision_object('cube', touch_links=${HAND})\nrobot.move_to_pose([0.5, -0.25, 0.26], [0.924, -0.383, 0.0, 0.0], cartesian=True)\nrobot.move_to_pose([0.45, 0.25, 0.14], [0.924, -0.383, 0.0, 0.0])`,
        expect: /^True$/,
      },
      {
        text: 'Let go, and move away.',
        code: "robot.detach_collision_object('cube')\nrobot.gripper.open()\nrobot.move_to_configuration()",
        expect: /^True$/,
      },
    ],
    outro: `<p>The Script tab has this as a complete program you can edit and run in real Python.</p>`,
  },
  {
    id: 'kinematics',
    part: 'Apply',
    title: 'Forward and inverse kinematics',
    api: ['compute_fk', 'compute_ik'],
    reset: true,
    intro: `<p>Forward kinematics turns joint angles into a pose; inverse kinematics finds joint angles for a pose. pymoveit2 calls MoveIt's <code>/compute_fk</code> and <code>/compute_ik</code> services without moving the robot.</p>`,
    steps: [
      {
        text: 'Where would the flange be in the <code>extended</code> state?',
        code: "robot.compute_fk(robot.joint_positions('extended'))",
        expect: /PoseStamped/,
      },
      {
        text: 'Which joint angles reach this pose? The answer is a <code>JointState</code>; the scene previews it as a ghost.',
        code: 'solution = robot.compute_ik([0.4, 0.2, 0.4], [1.0, 0.0, 0.0, 0.0])\nsolution.position',
        expect: /^\[/,
      },
      {
        text: 'Move to the solution as a joint goal.',
        code: 'robot.move_to_configuration(solution.position)',
        expect: /^True$/,
      },
      {
        text: 'Two meters away is out of reach: there is no solution, so the call returns <code>None</code>.',
        code: 'robot.compute_ik([2.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]) is None',
        expect: /^True$/,
      },
    ],
    outro: `<p>The Control tab's <strong>Kinematics</strong> card solves IK for the goal marker as you drag it.</p>`,
  },
  {
    id: 'servo',
    part: 'Apply',
    title: 'Real-time servoing',
    api: ['servo.enable', 'servo', 'servo_jog'],
    reset: true,
    intro: `<p>A plan is a whole path. MoveIt&nbsp;Servo instead follows velocity commands, as from a joystick, and stops 0.1&nbsp;s after the last. <code>robot.servo(linear, angular)</code> publishes one twist in the base frame.</p>`,
    steps: [
      {
        text: 'Start Servo. pymoveit2 would also start it on the first command.',
        code: 'robot.servo.enable()',
        expect: /^True$/,
      },
      {
        text: "One command moves the arm until Servo's 0.1&nbsp;s timeout: about 1&nbsp;cm at 0.1&nbsp;m/s.",
        code: 'robot.servo(linear=(0.0, 0.1, 0.0))',
        expect: /^True$/,
      },
      {
        text: `Real code sends commands in a loop, as in this script. Open it in the Script tab and press <strong>Run</strong>.`,
        script: 'servo-square',
      },
      {
        text: `Or steer by hand: the Control tab's <strong>Servo</strong> pad sends a command every 50&nbsp;ms while you hold a button, and so do the keys <kbd>W</kbd> <kbd>A</kbd> <kbd>S</kbd> <kbd>D</kbd> <kbd>Q</kbd> <kbd>E</kbd> while the scene has focus.`,
        tab: 'control',
      },
    ],
    outro: `<p><code>robot.servo.servo_jog(joint_names, velocities)</code> streams joint velocities instead of a twist.</p>`,
  },
  {
    id: 'failures',
    part: 'Apply',
    title: 'When things go wrong',
    api: ['last_failure', 'wait=False', 'cancel_execution'],
    reset: true,
    intro: `<p>Robot code fails in ordinary ways. pymoveit2 returns <code>False</code> instead of raising for motion failures, raises Python errors for wrong arguments, and keeps a readable explanation of the last failure.</p>`,
    steps: [
      {
        text: 'A goal out of reach.',
        code: 'robot.move_to_pose([2.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])\nrobot.last_failure()',
        expect: /^'NO_IK_SOLUTION: /,
      },
      {
        text: 'A wrong keyword argument is a Python <code>TypeError</code>, raised before anything moves. The orientation is called <code>quat_xyzw</code>.',
        code: 'robot.move_to_pose([0.4, 0.0, 0.4], quaternion=[1.0, 0.0, 0.0, 0.0])',
        expect: /^TypeError: .*unexpected keyword argument 'quaternion'/,
      },
      {
        text: `Start a long motion without waiting, then wait at most half a second. <code>wait_until_executed</code> returns <code>False</code> on a timeout; the arm keeps moving.`,
        code: "robot.arm.max_velocity = 0.1\nrobot.move_to_configuration(robot.joint_positions('extended'), wait=False)\nrobot.arm.wait_until_executed(timeout_sec=0.5)",
        expect: /^False$/,
      },
      {
        text: 'Stop it. The motion ends early and <code>last_failure()</code> records that it was preempted.',
        code: 'robot.arm.cancel_execution()\nrobot.arm.wait_until_executed()\nrobot.last_failure()',
        expect: /PREEMPTED|CANCELED/i,
      },
      {
        text: 'Clean up.',
        code: 'robot.arm.max_velocity = 1.0\nrobot.move_to_configuration()',
        expect: /^True$/,
      },
    ],
    outro: `<p>That is the tour. Open the Console for free experiments (try <code>help(robot)</code>), or the Script tab for whole programs.</p>`,
  },
];

/** The lessons per robot id. */
export const LESSONS = { panda: PANDA_LESSONS, open_manipulator_x: OMX_LESSONS };
