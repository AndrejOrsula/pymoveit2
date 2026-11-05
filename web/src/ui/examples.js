// Complete pymoveit2 programs for the Script tab, per robot. They run unchanged
// on a real robot with `moveit_resources_panda_moveit_config` (the mesh example
// from a pymoveit2 checkout, with trimesh installed) or open_manipulator-RMinS's
// MoveIt configuration; in the browser, Pyodide runs them against the simulation.

const PANDA_EXAMPLES = [
  {
    id: 'quick-start',
    title: 'Quick start',
    source: `from pymoveit2 import connect

with connect() as robot:
    # Move to a pose while checking for failures
    if not robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]):
        print(robot.last_failure())
        exit(1)

    # Actuate the gripper
    robot.gripper.close()

    # Move to SRDF group state
    robot.move_to_configuration()
`,
  },
  {
    id: 'square',
    title: 'Draw a square',
    source: `from pymoveit2 import connect

# Corners of a 30 cm square, 35 cm above the base, gripper pointing down.
DOWN = [1.0, 0.0, 0.0, 0.0]
corners = [[0.45, -0.15, 0.35], [0.45, 0.15, 0.35], [0.6, 0.15, 0.35], [0.6, -0.15, 0.35]]

with connect() as robot:
    robot.arm.max_velocity = 0.5
    robot.move_to_pose(corners[0], DOWN)
    for corner in corners[1:] + corners[:1]:
        # Straight edges: a Cartesian path between the corners.
        if not robot.move_to_pose(corner, DOWN, cartesian=True):
            print("Edge failed:", robot.last_failure())
            break
    robot.move_to_configuration()
`,
  },
  {
    id: 'pick-and-place',
    title: 'Pick and place',
    source: `from pymoveit2 import connect

GRASP = [0.924, -0.383, 0.0, 0.0]  # fingers aligned with the cube
HAND = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]


def move(robot, position, cartesian=False):
    if not robot.move_to_pose(position, GRASP, cartesian=cartesian):
        raise RuntimeError(robot.last_failure())


with connect() as robot:
    robot.add_collision_box("table", [0.8, 1.2, 0.04], position=[0.55, 0.0, -0.02], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
    robot.add_collision_box("cube", [0.04, 0.04, 0.05], position=[0.5, -0.25, 0.025], quat_xyzw=[0.0, 0.0, 0.0, 1.0])

    robot.gripper.open()
    move(robot, [0.5, -0.25, 0.26])
    robot.allow_collisions("cube", True)
    move(robot, [0.5, -0.25, 0.14], cartesian=True)

    robot.gripper.close()
    robot.gripper.wait_until_executed()
    robot.attach_collision_object("cube", touch_links=HAND)

    move(robot, [0.5, -0.25, 0.26], cartesian=True)
    move(robot, [0.45, 0.25, 0.26])
    move(robot, [0.45, 0.25, 0.14], cartesian=True)

    robot.detach_collision_object("cube")
    robot.gripper.open()
    robot.gripper.wait_until_executed()
    move(robot, [0.45, 0.25, 0.26], cartesian=True)
    robot.move_to_configuration()
`,
  },
  {
    id: 'servo-square',
    title: 'Servo a square',
    source: `import time

from pymoveit2 import connect

SPEED = 0.1  # m/s
SIDE = 1.5  # seconds per side

with connect() as robot:
    robot.servo.enable()
    for direction in [(0.0, SPEED, 0.0), (0.0, 0.0, SPEED), (0.0, -SPEED, 0.0), (0.0, 0.0, -SPEED)]:
        start = time.monotonic()
        while time.monotonic() - start < SIDE:
            # Servo stops 0.1 s after the last command, so keep sending them.
            robot.servo(linear=direction)
            time.sleep(0.02)
    robot.servo.disable()
`,
  },
  {
    id: 'obstacle-course',
    title: 'Obstacle course',
    source: `from pymoveit2 import connect

DOWN = [1.0, 0.0, 0.0, 0.0]

with connect() as robot:
    robot.add_collision_box("wall", [0.04, 0.5, 0.3], position=[0.55, 0.0, 0.15], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
    robot.add_collision_sphere("ball", 0.07, position=[0.4, -0.3, 0.35], quat_xyzw=[0.0, 0.0, 0.0, 1.0])
    # A mesh from pymoveit2's examples; meshes need trimesh on a real robot.
    robot.add_collision_mesh("examples/assets/suzanne.stl", "monkey", position=[0.35, 0.35, 0.15], quat_xyzw=[0.0, 0.0, 0.0, 1.0], scale=0.1)

    # Over the wall, beside the monkey, then behind the wall, where the arm cannot fit.
    for goal in [[0.65, -0.15, 0.45], [0.35, 0.2, 0.35], [0.7, 0.0, 0.15]]:
        ok = robot.move_to_pose(goal, DOWN)
        print(goal, "->", "reached" if ok else robot.last_failure())

    robot.move_to_configuration()
    robot.clear_all_collision_objects()
`,
  },
  {
    id: 'plan-then-execute',
    title: 'Plan, inspect, execute',
    source: `from pymoveit2 import connect

with connect() as robot:
    arm = robot.arm
    arm.pipeline_id = "pilz_industrial_motion_planner"
    arm.planner_id = "LIN"

    # Plan without moving, look at the result, then decide.
    trajectory = arm.plan(position=[0.5, 0.2, 0.3], quat_xyzw=[1.0, 0.0, 0.0, 0.0])
    if trajectory is None:
        print("No plan:", arm.last_failure())
    else:
        duration = trajectory.points[-1].time_from_start
        print(len(trajectory.points), "points,", duration.sec + duration.nanosec * 1e-9, "s")
        arm.execute(trajectory)
        print("Succeeded:", arm.wait_until_executed())
`,
  },
];

const OMX_EXAMPLES = [
  {
    id: 'quick-start',
    title: 'Quick start',
    source: `from pymoveit2 import connect

# This arm has 4 joints: goals name the fixed world frame and free the orientation.
GOAL = dict(frame_id="world", tolerance_orientation=3.1416)

with connect() as robot:
    # Move to a position while checking for failures
    if not robot.move_to_pose([0.0, -0.05, 0.12], [0.0, 0.0, 0.0, 1.0], **GOAL):
        print(robot.last_failure())
        exit(1)

    # Actuate the gripper
    robot.gripper.close()

    # Move to SRDF group state
    robot.move_to_configuration()
`,
  },
  {
    id: 'square',
    title: 'Draw a square',
    source: `from pymoveit2 import connect

# Corners of a 10 cm square, 13 cm above the world origin, in front of the arm.
ANY = [0.0, 0.0, 0.0, 1.0]  # position-only goals ignore the orientation
corners = [[0.05, -0.02, 0.13], [-0.05, -0.02, 0.13], [-0.05, -0.12, 0.13], [0.05, -0.12, 0.13]]

with connect() as robot:
    robot.arm.max_velocity = 0.25
    robot.move_to_pose(corners[0], ANY, frame_id="world", tolerance_orientation=3.1416)
    for corner in corners[1:] + corners[:1]:
        # Straight edges: a Cartesian path between the corners.
        if not robot.move_to_pose(corner, ANY, frame_id="world", cartesian=True):
            print("Edge failed:", robot.last_failure())
            break
    robot.move_to_configuration()
`,
  },
  {
    id: 'pick-and-place',
    title: 'Pick and place',
    source: `from pymoveit2 import connect

HAND = ["link5", "gripper_left_link", "gripper_right_link"]


def above(joint1):
    return [joint1, 0.75, -1.22, 1.89]


def grasp(joint1):
    return [joint1, 0.87, -1.05, 1.75]


def move(robot, joints):
    if not robot.move_to_configuration(joints):
        raise RuntimeError(robot.last_failure())


with connect() as robot:
    # Objects default to base_link_name, link2, which joint1 turns: use world.
    robot.add_collision_box("cube", [0.025, 0.025, 0.025], position=[0.0, -0.12, 0.0375], quat_xyzw=[0.0, 0.0, 0.0, 1.0], frame_id="world")

    robot.gripper.open()
    move(robot, above(0.0))
    robot.allow_collisions("cube", True)
    move(robot, grasp(0.0))

    robot.gripper.close()
    robot.gripper.wait_until_executed()
    robot.attach_collision_object("cube", touch_links=HAND)

    # Only joint1 changes between pick and place: it aims, the others reach.
    move(robot, above(0.0))
    move(robot, above(0.5))
    move(robot, grasp(0.5))

    robot.detach_collision_object("cube")
    robot.gripper.open()
    robot.gripper.wait_until_executed()
    move(robot, above(0.5))
    robot.move_to_configuration()
`,
  },
  {
    id: 'servo-square',
    title: 'Servo a square',
    source: `import time

from pymoveit2 import connect

SPEED = 0.05  # m/s
SIDE = 1.0  # seconds per side

with connect() as robot:
    # Twists are in robot.servo.frame_id, link2 here: x points along the arm.
    robot.servo.enable()
    for direction in [(SPEED, 0.0, 0.0), (0.0, 0.0, SPEED), (-SPEED, 0.0, 0.0), (0.0, 0.0, -SPEED)]:
        start = time.monotonic()
        while time.monotonic() - start < SIDE:
            # Servo stops 0.1 s after the last command, so keep sending them.
            robot.servo(linear=direction)
            time.sleep(0.02)
    robot.servo.disable()
`,
  },
];

/** The examples per robot id. */
export const EXAMPLES = { panda: PANDA_EXAMPLES, open_manipulator_x: OMX_EXAMPLES };
