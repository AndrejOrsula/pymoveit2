// Shared shapes of the demo's simulated MoveIt 2 (src/core), its pymoveit2
// stand-in (src/api) and the interface (src/view, src/ui). JSDoc only; this
// module exports nothing at run time.
//
// Frames and units: the planning frame is the model's root link (panda_link0
// for the Panda, world for the OpenMANIPULATOR-X), z up, meters and radians.
// Quaternions are [x, y, z, w], as `quat_xyzw` is in pymoveit2.
// A Mat4 is a Float64Array(16) in column-major order, the layout of
// three.js's Matrix4.fromArray().

/** @typedef {[number, number, number]} Vec3 */
/** @typedef {[number, number, number, number]} Quat */
/** @typedef {Float64Array} Mat4 */
/** @typedef {{ position: Vec3, quat_xyzw: Quat }} Pose */

/**
 * A robot as the demo's move_group knows it: URDF, SRDF and MoveIt config
 * reduced to what the simulation needs. See src/core/panda.js and
 * src/core/open_manipulator_x.js; src/core/robots.js lists them.
 * @typedef {object} RobotModel
 * @property {string} id  Registry key and `?robot=` value.
 * @property {string} name  The URDF robot name.
 * @property {string} label  Display name.
 * @property {string} source  Where the description comes from, with licenses.
 * @property {string} group_name  What `RobotDescription` derives from the SRDF,
 *   so what `connect()` uses; the same holds for the next three names.
 * @property {string} base_link_name  The default frame of pose goals, FK and IK.
 * @property {string} end_effector_name
 * @property {string} planning_frame  The URDF root link.
 * @property {string[]} joint_names
 * @property {string} default_state  The arm's named state that `move_to_configuration()` picks.
 * @property {Array<{name: string, parent: string, child: string, xyz: Vec3, rpy: Vec3, axis: Vec3, lower: number, upper: number, max_velocity: number, max_acceleration: number}>} joints
 *   The arm's revolute joints, in `joint_names` order.
 * @property {Array<{name: string, parent: string, child: string, xyz: Vec3, rpy: Vec3}>} fixed
 *   Fixed joints, in any order, including those between the root and the arm.
 * @property {{joint_names: string[], links: string[], parent: string, origins: Array<{xyz: Vec3, rpy: Vec3}>, axes: Vec3[], lower: number, upper: number, max_velocity: number, max_acceleration: number}} fingers
 *   Prismatic fingers that all move by the same finger position along their axes.
 * @property {{group_name: string, joint_names: string[], open_joint_positions: number[], closed_joint_positions: number[],
 *   grasp: {frame: string, closing: number, approach: number, depth: [number, number], lateral: number, offset: number}}} gripper
 *   The gripper as `moveit2_gripper_kwargs()` derives it. `grasp` places the finger pads
 *   in `frame`: they close along axis `closing`, an object fits between them
 *   within `depth` along axis `approach` and within `lateral` of the third
 *   axis's center line, and a pad is at ±(offset + finger position).
 * @property {string[]} ik_links  Links rigidly attached to the arm's tip, valid IK targets.
 * @property {{center: Vec3, radius: number}} reach  IK rejects positions outside this sphere.
 * @property {boolean} position_only_ik  kinematics.yaml's `position_only_ik`: the
 *   IK service and Cartesian paths ignore orientation.
 * @property {{arm: string, gripper: string}} controllers  Controller names for logs.
 * @property {{check_collisions: boolean}} servo  servo.yaml settings that matter here.
 * @property {{assets: string, target: Vec3, camera: Vec3, ground: number, scale: number}} view
 *   Mesh directory under static/assets/, camera target and position, floor
 *   height, and the size of the floor and markers relative to the Panda's.
 * @property {Record<string, {joint_names: string[], passive_joint_names: string[], base_link_name: string, end_effector_name: string}>} groups
 *   Every SRDF group as `RobotDescription.group()` derives it.
 * @property {Record<string, Record<string, number[]>>} named_states  SRDF group states, in the order of the joints they set.
 * @property {Array<[string, string]>} disabled_collisions
 * @property {Record<string, Array<[number, number, number, number]>>} collision_spheres  Per link, in its frame.
 * @property {Record<string, Array<{xyz: Vec3, size: Vec3}>>} [collision_boxes]  Axis-aligned boxes per link, in its frame.
 * @property {Record<string, string>} meshes  GLB file per link.
 */

/**
 * A planning-scene object. Primitive dimensions follow shape_msgs/SolidPrimitive:
 * box [x, y, z]; sphere [radius]; cylinder [height, radius] and cone
 * [height, radius], both along their z axis and centered on their pose (the
 * cone's base at -height/2, its apex at +height/2). A mesh is one of the
 * demo's built-in meshes ("suzanne", examples/assets/suzanne.stl) scaled by
 * `scale`.
 * @typedef {object} CollisionObject
 * @property {string} id
 * @property {'box'|'sphere'|'cylinder'|'cone'|'mesh'} type
 * @property {number[]} dimensions
 * @property {Pose} pose  In the planning frame; for an attached object, its
 *   current world pose (kept up to date while the arm moves).
 * @property {string} [mesh]
 * @property {Vec3} [scale]
 * @property {{link: string, touch_links: string[], weight: number}|null} attached
 * @property {boolean} allowed  Collisions with it are allowed (the ACM entry
 *   that `allow_collisions(id, True)` sets).
 */

/**
 * A trajectory_msgs/JointTrajectory.
 * @typedef {object} JointTrajectory
 * @property {string[]} joint_names
 * @property {Array<{positions: number[], velocities: number[], accelerations: number[], time_from_start: number}>} points
 */

/**
 * A motion-plan request, the subset of moveit_msgs/MotionPlanRequest and
 * GetCartesianPath that pymoveit2 fills in. Exactly one goal kind is set.
 * @typedef {object} PlanRequest
 * @property {number[]} [start]  Arm joint positions; defaults to the current state.
 * @property {{positions: number[], joint_names?: string[], tolerance: number}} [joint_goal]
 * @property {{position?: Vec3, quat_xyzw?: Quat, target_link: string, frame_id: string,
 *   tolerance_position: number, tolerance_orientation: number|Vec3}} [pose_goal]
 *   A position-only goal leaves `quat_xyzw` out, an orientation-only goal `position`.
 * @property {PathConstraints} [path_constraints]
 * @property {string} pipeline_id  "" (the default, OMPL), "ompl", or "pilz_industrial_motion_planner".
 * @property {string} planner_id  "" (RRTConnect), "RRTConnectkConfigDefault", "RRTstarkConfigDefault", "PTP", "LIN".
 * @property {boolean} cartesian  Plan through compute_cartesian_path instead.
 * @property {number} cartesian_max_step
 * @property {number} cartesian_jump_threshold
 * @property {boolean} cartesian_avoid_collisions
 * @property {number} max_velocity_scaling_factor  Outside (0, 1], MoveIt's time
 *   parameterization (TOTG) warns and uses 1.0; pymoveit2 sends 0.0 by default.
 * @property {number} max_acceleration_scaling_factor  As above.
 * @property {number} allowed_planning_time
 * @property {number} num_planning_attempts
 */

/**
 * @typedef {object} PathConstraints
 * @property {Array<{quat_xyzw: Quat, link: string, tolerance: Vec3, parameterization: 0|1}>} orientation
 * @property {Array<{position: Vec3, link: string, tolerance: number}>} position  A sphere of radius `tolerance`.
 * @property {Array<{joint_name: string, position: number, tolerance: number}>} joint
 */

/**
 * @typedef {object} PlanResult
 * @property {string} error_code  "SUCCESS" or a moveit_msgs/MoveItErrorCodes name.
 * @property {string} [detail]  One sentence for the log, such as the cause of a failure.
 * @property {JointTrajectory} [trajectory]
 * @property {number} [fraction]  Cartesian plans: the share of the path achieved.
 * @property {number} planning_time  Seconds the planner reports.
 * @property {string} planner  "RRTConnect", "PTP", "LIN" or "Cartesian".
 * @property {{ee_path?: Vec3[], tree?: Array<[Vec3, Vec3]>, goal_state?: number[], contacts?: Array<[string, string]>}} debug
 *   For the viewer: the end-effector path, the planner's search tree in
 *   end-effector positions (at most 2000 edges), the goal state and, on a
 *   collision failure, the colliding pairs.
 */

/**
 * The state the viewer draws, sent on every sim step.
 * @typedef {object} SimState
 * @property {number} time  Simulated seconds since the sim started.
 * @property {number[]} q  Arm joint positions.
 * @property {number[]} velocities  Arm joint velocities.
 * @property {number} fingers  Position of each finger joint.
 * @property {'idle'|'executing'|'servo'} mode
 * @property {Pose} ee  Pose of the end effector (`end_effector_name`).
 * @property {Array<[string, string]>} contacts  Pairs in collision now.
 */

/**
 * Sim events. `sim.on(type, listener)` returns a function that unsubscribes.
 * - "state": SimState, after every step.
 * - "scene": {objects: CollisionObject[]}, after any scene change.
 * - "plan": {request: PlanRequest, result: PlanResult}, after any planning.
 * - "execution": {id: number, status: 'started'|'succeeded'|'aborted'|'canceled',
 *   error_code: string, trajectory: JointTrajectory, target: 'arm'|'gripper'}.
 * - "log": LogEntry, what move_group, servo_node and the controllers print.
 * @typedef {object} LogEntry
 * @property {number} time
 * @property {'debug'|'info'|'warn'|'error'} level
 * @property {string} node  "move_group", "servo_node", a controller such as "panda_arm_controller", "pymoveit2".
 * @property {string} text
 */

/**
 * One ROS 2 interaction that a pymoveit2 call made, for the "Under the hood"
 * view: the facade emits one per request it sends.
 * @typedef {object} RosCall
 * @property {'service'|'action'|'topic'} kind
 * @property {string} name  e.g. "/plan_kinematic_path".
 * @property {string} type  e.g. "moveit_msgs/srv/GetMotionPlan".
 * @property {string} summary  e.g. "joint goal, RRTConnect → SUCCESS in 0.04 s".
 * @property {number} time  Sim time when it was sent.
 * @property {number} [duration]  Sim seconds until its response or result.
 * @property {string} [status]  "SUCCESS", "SUCCEEDED", "ABORTED", ...
 */

/**
 * A Python value as the facade returns it. Plain JSON values stand for
 * themselves (null is None, arrays are lists). Message and enum objects carry
 * a `__class__` with the ROS or pymoveit2 type name and their fields in
 * Python's shape, so the console prints them as Python would and the Python
 * stand-in rebuilds them:
 * - {__class__: 'PoseStamped', header: {frame_id}, pose: {position: {x, y, z}, orientation: {x, y, z, w}}}
 * - {__class__: 'JointState', name: string[], position: number[], velocity: number[], effort: number[]}
 * - {__class__: 'JointTrajectory', joint_names, points: [{positions, velocities, accelerations, time_from_start: {sec, nanosec}}]}
 * - {__class__: 'MoveIt2State', name: 'IDLE'|'REQUESTING'|'EXECUTING', value: 0|1|2}
 * - {__class__: 'PlanningScene', world: {collision_objects: [{id, ...}]}, robot_state: {attached_collision_objects: [...]}}
 * @typedef {null|boolean|number|string|Array<any>|{__class__: string, [field: string]: any}} PyValue
 */

/**
 * A Python exception that the facade raises, which the console prints and the
 * Python stand-in re-raises with the same type and message.
 * @typedef {object} PyErrorShape
 * @property {'TypeError'|'ValueError'|'AttributeError'|'RuntimeError'|'NameError'|'SyntaxError'|'KeyError'|'IndexError'|'ZeroDivisionError'} type
 * @property {string} message
 */

export {};
