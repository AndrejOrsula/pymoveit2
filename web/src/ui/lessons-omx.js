// The guided lessons for the OpenMANIPULATOR-X, in the format of lessons.js.
// Two things set this arm apart, and the lessons teach both early: pymoveit2's
// default frame, link2, turns with joint1, so goals and objects name `world`;
// and four joints cannot reach every orientation, so pose goals free it.
//
// Text is trusted HTML written here, not user input.

/** Touch links for a cube held between the fingers. */
const HAND = "['link5', 'gripper_left_link', 'gripper_right_link']";
/** Keyword arguments of a pose goal in world with a free orientation. */
const FREE = "frame_id='world', tolerance_orientation=3.1416";
/** Joint goals above the cube, and around it, with joint1 still to choose. */
const ABOVE = (joint1) => `[${joint1}, 0.75, -1.22, 1.89]`;
const GRASP = (joint1) => `[${joint1}, 0.87, -1.05, 1.75]`;

export const OMX_LESSONS = [
  {
    id: 'hello',
    part: 'Move',
    title: 'Hello, robot',
    api: ['connect()', 'move_to_pose', 'gripper.close', 'move_to_configuration'],
    intro: `<p><code>connect()</code> joins the robot's ROS&nbsp;2 system and gives you <code>robot</code>, which drives the arm, the gripper and servo. Here it is ready; your own script starts with:</p><div class="code"><pre><code>with connect() as robot:</code></pre></div>`,
    steps: [
      {
        text: `Move to a <strong>position</strong> in meters. <code>[0, 0, 0, 1]</code> means no turn; <code>tolerance_orientation=3.1416</code> lets the arm choose its tilt; <code>frame_id='world'</code> measures from a fixed frame.`,
        code: `robot.move_to_pose([0.0, -0.05, 0.12], [0.0, 0.0, 0.0, 1.0], ${FREE})`,
        expect: /^True$/,
        after: `<code>True</code>: MoveIt found a collision-free path; pymoveit2 ran it and waited. The <em>Last call</em> strip shows the ROS&nbsp;2 traffic.`,
      },
      {
        text: 'Close the gripper. Gripper commands return at once; the fingers keep moving.',
        code: 'robot.gripper.close()',
        expect: /^True$/,
      },
      {
        text: `Go home: without arguments, <code>move_to_configuration()</code> uses the SRDF's default named state.`,
        code: 'robot.move_to_configuration()',
        expect: /^True$/,
        after: `Every call returns <code>True</code> or <code>False</code>; on <code>False</code>, <code>robot.last_failure()</code> says why.`,
      },
    ],
    outro: `<p>Try it: drag the goal ball in the scene and press <strong>Move</strong> in the Control tab, or type <code>go init</code> in the Console.</p>`,
  },
  {
    id: 'joints',
    part: 'Move',
    title: 'Joints and named states',
    api: ['joint_names', 'joint_state', 'joint_positions', 'move_to_configuration'],
    intro: `<p>This arm has four revolute joints: <code>joint1</code> turns it around a vertical axis, and <code>joint2</code> to <code>joint4</code> tilt it like a shoulder, an elbow and a wrist. A <em>configuration</em> is one angle per joint, in radians.</p>`,
    steps: [
      {
        text: 'Which joints are there, and where are they now?',
        code: 'robot.joint_names\nrobot.joint_state.position',
        expect: /^\[/,
      },
      {
        text: `The SRDF names two configurations, <code>init</code> and <code>home</code>. Read <code>init</code>:`,
        code: "robot.joint_positions('init')",
        expect: /^\[0\.0, 0\.0, 0\.0, 0\.0\]$/,
      },
      {
        text: 'All zeros: the arm stands straight up. Move there.',
        code: "robot.move_to_configuration(robot.joint_positions('init'))",
        expect: /^True$/,
      },
      {
        text: 'Or give four angles yourself. Watch the plot under the scene: all joints start and stop together.',
        code: 'robot.move_to_configuration([0.8, 0.3, 0.2, -0.5])',
        expect: /^True$/,
        after: `Switch the plot to <strong>Velocities</strong>: each curve stays within that joint's velocity limit. That is MoveIt's time parameterization.`,
      },
      {
        text: 'Back home.',
        code: 'robot.move_to_configuration()',
        expect: /^True$/,
      },
    ],
    outro: `<p>The Control tab's <strong>Joints</strong> sliders preview a configuration as a ghost before you move.</p>`,
  },
  {
    id: 'frames',
    part: 'Move',
    title: 'Frames',
    api: ['base_link_name', 'compute_fk', 'frame_id'],
    reset: true,
    intro: `<p>A position means nothing without its <em>frame</em>. pymoveit2 uses the arm's <code>base_link_name</code> unless you pass <code>frame_id</code>, and <code>connect()</code> reads that name from the SRDF.</p>`,
    steps: [
      {
        text: 'Which frame is the default here?',
        code: 'robot.arm.base_link_name',
        expect: /^'link2'$/,
        after: `<code>link2</code> is the link <code>joint1</code> turns, not the fixed <code>link1</code> below it.`,
      },
      {
        text: 'Where is the gripper (<code>link5</code>)? <code>compute_fk()</code> answers in <code>link2</code>.',
        code: 'robot.compute_fk().pose.position',
        expectPosition: [0.0237, 0, 0.1855],
        after: `<code>y</code> is 0: in <code>link2</code>, the arm always moves in its own vertical plane.`,
      },
      {
        text: 'Turn <code>joint1</code> and ask again.',
        code: 'robot.move_to_configuration([1.0, -1.0, 0.7, 0.3])\nrobot.compute_fk().pose.position',
        expectPosition: [0.0237, 0, 0.1855],
        after: 'The gripper moved, but its position in <code>link2</code> did not: the frame turned with it.',
      },
      {
        text: `<code>world</code> does not move. Its axes are drawn at its origin, in front of the arm: x red, y green, z blue. The arm faces -y.`,
        code: `robot.move_to_pose([0.1, -0.05, 0.12], [0.0, 0.0, 0.0, 1.0], ${FREE})`,
        expect: /^True$/,
      },
    ],
    outro: `<p>Pass <code>frame_id='world'</code> to goals and collision objects: <code>link2</code> turns with <code>joint1</code>. <code>compute_fk()</code> reports poses in <code>link2</code>; <code>compute_ik()</code> takes its target there.</p>`,
  },
  {
    id: 'orientation',
    part: 'Move',
    title: 'Four joints, free orientation',
    api: ['tolerance_orientation', 'compute_ik'],
    reset: true,
    intro: `<p>Four joints cannot set all six numbers of a pose, three of position and three of orientation. This arm's <code>kinematics.yaml</code> enables <code>position_only_ik</code>, so IK solves for the position alone.</p>`,
    steps: [
      {
        text: 'Ask for a position and an exact orientation. MoveIt checks the orientation against <code>tolerance_orientation</code>, 0.001&nbsp;rad by default.',
        code: "robot.move_to_pose([0.1, -0.05, 0.1], [0.0, 0.0, 0.0, 1.0], frame_id='world')\nrobot.last_failure()",
        expect: /^'NO_IK_SOLUTION: /,
        after: 'The arm reaches the position, but it would have to point to its left to match that orientation. The timeline under the plot says so.',
      },
      {
        text: 'Allow any orientation: π&nbsp;rad, a half turn, about each axis.',
        code: `robot.move_to_pose([0.1, -0.05, 0.1], [0.0, 0.0, 0.0, 1.0], ${FREE})`,
        expect: /^True$/,
      },
      {
        text: 'Inverse kinematics on its own: which angles put the gripper 15&nbsp;cm ahead and 10&nbsp;cm up, in <code>link2</code>? The scene previews the answer as a ghost.',
        code: 'solution = robot.compute_ik([0.15, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0])\nsolution.position',
        expect: /^\[/,
      },
      {
        text: 'Move there as a joint goal.',
        code: 'robot.move_to_configuration(solution.position)',
        expect: /^True$/,
      },
    ],
    outro: `<p>In the Control tab, the goal marker of this arm moves with the orientation freed; the <strong>Kinematics</strong> card solves IK as you drag it.</p>`,
  },
  {
    id: 'lines',
    part: 'Move',
    title: 'Straight lines and speed',
    api: ['cartesian=True', 'cartesian_fraction_threshold', 'max_velocity'],
    reset: true,
    setup: [`robot.move_to_pose([0.1, -0.1, 0.13], [0.0, 0.0, 0.0, 1.0], ${FREE})`],
    intro: `<p>A free-space plan may swing the gripper in an arc. <code>cartesian=True</code> moves it along a straight line. Each point uses the same position-only IK, so no orientation tolerance is needed.</p>`,
    steps: [
      {
        text: 'A straight line 20&nbsp;cm to the right. Turn on <strong>Path</strong> in the scene to see it.',
        code: "robot.move_to_pose([-0.1, -0.1, 0.13], [0.0, 0.0, 0.0, 1.0], frame_id='world', cartesian=True)",
        expect: /^True$/,
      },
      {
        text: 'Now back and down, ending low enough that the fingers would hit the floor. Ask for the whole line or nothing:',
        code: "robot.move_to_pose([0.1, -0.1, 0.06], [0.0, 0.0, 0.0, 1.0], frame_id='world', cartesian=True, cartesian_fraction_threshold=1.0)\nrobot.last_failure()",
        expect: /Cartesian planner reached only/,
        after: 'By default (<code>cartesian_fraction_threshold=0.0</code>) pymoveit2 would have executed the part of the line MoveIt found.',
      },
      {
        text: `Slow down. The scaling factors multiply the joint limits; the open_manipulator-RMinS joint and pose goal examples use 0.25.`,
        code: 'robot.arm.max_velocity = 0.25\nrobot.arm.max_acceleration = 0.25\nrobot.move_to_configuration()',
        expect: /^True$/,
        after: 'The time axis of the plot stretched.',
      },
      {
        text: `A classic mistake: <code>robot.max_velocity = 1.0</code> stores a new attribute on the session, not on the arm.`,
        code: 'robot.max_velocity = 1.0\nrobot.arm.max_velocity',
        expect: /^0\.25$/,
        after: 'Always set motion parameters on <code>robot.arm</code>.',
      },
    ],
    outro: `<p>The Control tab's <strong>Straight line</strong> checkbox and <strong>Planning</strong> sliders do the same; the console command <code>speed 0.5</code> sets both factors.</p>`,
  },
  {
    id: 'obstacles',
    part: 'Apply',
    title: 'Obstacles',
    api: ['add_collision_box', 'move_collision', 'last_failure'],
    reset: true,
    intro: `<p>MoveIt plans around everything in its <em>planning scene</em>. Here the floor is part of the robot's description; add other objects with <code>add_collision_box</code> and friends, in <code>world</code> like goals.</p>`,
    steps: [
      {
        text: 'A low wall on the floor in front of the arm.',
        code: "robot.add_collision_box('wall', [0.12, 0.02, 0.06], position=[0.0, -0.06, 0.055], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id='world')",
      },
      {
        text: 'Reach past it: MoveIt finds a way over.',
        code: `robot.move_to_pose([0.0, -0.12, 0.12], [0.0, 0.0, 0.0, 1.0], ${FREE})`,
        expect: /^True$/,
      },
      {
        text: 'A goal inside the wall fails, and <code>last_failure()</code> says why.',
        code: `robot.move_to_pose([0.0, -0.06, 0.055], [0.0, 0.0, 0.0, 1.0], ${FREE})\nrobot.last_failure()`,
        expect: /GOAL_IN_COLLISION/,
      },
      {
        text: 'Objects can move. Slide the wall aside, go home, and clear the scene.',
        code: "robot.move_collision('wall', [0.12, -0.06, 0.055], [0.0, 0.0, 0.0, 1.0], frame_id='world')\nrobot.move_to_configuration()\nrobot.clear_all_collision_objects()",
        expect: /^(True|None|<)/,
        after: `The <code>&lt;Future …&gt;</code> line is the pending reply of MoveIt's <code>/apply_planning_scene</code>: pymoveit2 sent the change and does not wait for it.`,
      },
    ],
    outro: `<p>In the Control tab's <strong>Scene</strong> card, add objects and drag them with the gizmo.</p>`,
  },
  {
    id: 'pick',
    part: 'Apply',
    title: 'Pick and place',
    api: ['gripper', 'attach_collision_object', 'detach_collision_object'],
    reset: true,
    setup: ["robot.add_collision_box('cube', [0.025, 0.025, 0.025], position=[0.0, -0.12, 0.0375], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id='world')"],
    intro: `<p>A 25&nbsp;mm cube sits in front of the arm. With four joints, joint goals are often the simplest way to grasp: these put the fingers around the cube from above.</p>`,
    steps: [
      {
        text: 'Open the gripper and move above the cube.',
        code: `robot.gripper.open()\nrobot.move_to_configuration(${ABOVE('0.0')})`,
        expect: /^True$/,
      },
      {
        text: 'MoveIt treats touching the cube as a collision, so allow it, then go down.',
        code: `robot.allow_collisions('cube', True)\nrobot.move_to_configuration(${GRASP('0.0')})`,
        expect: /^True$/,
      },
      {
        text: 'Close the fingers and wait; they stop on the cube.',
        code: 'robot.gripper.close()\nrobot.gripper.wait_until_executed()',
        expect: /^(True|False)$/,
      },
      {
        text: `Attach the cube: MoveIt now carries it with the arm. <code>touch_links</code> may touch it. Only <code>joint1</code> changes between pick and place: it aims, the other three reach.`,
        code: `robot.attach_collision_object('cube', touch_links=${HAND})\nrobot.move_to_configuration(${ABOVE('0.0')})\nrobot.move_to_configuration(${ABOVE('0.5')})\nrobot.move_to_configuration(${GRASP('0.5')})`,
        expect: /^True$/,
      },
      {
        text: 'Let go, and go home.',
        code: "robot.detach_collision_object('cube')\nrobot.gripper.open()\nrobot.move_to_configuration()",
        expect: /^True$/,
      },
    ],
    outro: `<p>The Script tab has this as a complete program you can edit and run in real Python.</p>`,
  },
  {
    id: 'servo',
    part: 'Apply',
    title: 'Real-time servoing',
    api: ['servo.enable', 'servo', 'servo_jog'],
    reset: true,
    intro: `<p>A plan is a whole path. MoveIt&nbsp;Servo instead follows velocity commands, as from a joystick, and stops 0.1&nbsp;s after the last. This arm's <code>servo.yaml</code> skips collision checks: keep speeds low.</p>`,
    steps: [
      {
        text: 'Start Servo. pymoveit2 would also start it on the first command.',
        code: 'robot.servo.enable()',
        expect: /^True$/,
      },
      {
        text: `One twist, in <code>robot.servo.frame_id</code>: <code>link2</code> again, so x points along the arm, wherever <code>joint1</code> turned it.`,
        code: 'robot.servo(linear=(0.05, 0.0, 0.0))',
        expect: /^True$/,
      },
      {
        text: 'Or move one joint: <code>servo_jog</code> takes joint velocities in rad/s.',
        code: "robot.servo.servo_jog(['joint1'], [0.5])",
        expect: /^True$/,
      },
      {
        text: `Steer by hand: the Control tab's <strong>Servo</strong> pad sends a command every 50&nbsp;ms while you hold a button, and so do <kbd>W</kbd> <kbd>A</kbd> <kbd>S</kbd> <kbd>D</kbd> <kbd>Q</kbd> <kbd>E</kbd> while the scene has focus.`,
        tab: 'control',
      },
    ],
    outro: `<p>Real code sends commands in a loop; the Script tab's <strong>Servo a square</strong> shows how.</p>`,
  },
  {
    id: 'failures',
    part: 'Apply',
    title: 'When things go wrong',
    api: ['last_failure', 'wait=False', 'cancel_execution'],
    reset: true,
    intro: `<p>pymoveit2 returns <code>False</code> instead of raising for motion failures, raises Python errors for wrong arguments, and keeps a readable explanation of the last failure.</p>`,
    steps: [
      {
        text: 'A goal half a meter away, out of reach.',
        code: `robot.move_to_pose([0.5, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0], ${FREE})\nrobot.last_failure()`,
        expect: /^'NO_IK_SOLUTION: /,
      },
      {
        text: 'A wrong keyword is a Python <code>TypeError</code>, raised before anything moves. The orientation is called <code>quat_xyzw</code>.',
        code: "robot.move_to_pose([0.1, -0.05, 0.1], quaternion=[0.0, 0.0, 0.0, 1.0], frame_id='world')",
        expect: /^TypeError: .*unexpected keyword argument 'quaternion'/,
      },
      {
        text: `Start a slow motion without waiting, then wait at most half a second. <code>wait_until_executed</code> returns <code>False</code> on a timeout; the arm keeps moving.`,
        code: "robot.arm.max_velocity = 0.1\nrobot.move_to_configuration(robot.joint_positions('init'), wait=False)\nrobot.arm.wait_until_executed(timeout_sec=0.5)",
        expect: /^False$/,
      },
      {
        text: 'Stop it. <code>last_failure()</code> records that it was preempted.',
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
