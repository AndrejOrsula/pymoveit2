import threading
from typing import List, Optional, Tuple, Union

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveIt2:
    """
    Python interface for MoveIt 2 that enables planning and execution of trajectories.
    For execution, this interface requires that robot utilises JointTrajectoryController.
    """

    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link_name: str,
        end_effector_name: str,
        group_name: str = "arm",
        execute_via_moveit: bool = False,
        ignore_new_calls_while_executing: bool = False,
        callback_group: Optional[CallbackGroup] = None,
        follow_joint_trajectory_action_name: str = "joint_trajectory_controller/follow_joint_trajectory",
    ):
        """
        Construct an instance of `MoveIt2` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `joint_names` - List of joint names of the robot (can be extracted from URDF)
          - `base_link_name` - Name of the robot base link
          - `end_effector_name` - Name of the robot end effector
          - `group_name` - Name of the planning group for robot arm
          - `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed otherwise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `follow_joint_trajectory_action_name` - Name of the action server for the controller
        """

        self._node = node
        self._callback_group = callback_group

        # Create subscriber for current joint states
        self._node.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.__joint_state_callback,
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        if execute_via_moveit:
            # Create action client for move action
            self.__move_action_client = ActionClient(
                node=self._node,
                action_type=MoveGroup,
                action_name="move_action",
                goal_service_qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                result_service_qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=5,
                ),
                cancel_service_qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=5,
                ),
                feedback_sub_qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                status_sub_qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                callback_group=self._callback_group,
            )
        else:
            # Otherwise create a separate service client for planning
            self._plan_kinematic_path_service = self._node.create_client(
                srv_type=GetMotionPlan,
                srv_name="plan_kinematic_path",
                qos_profile=QoSProfile(
                    durability=QoSDurabilityPolicy.VOLATILE,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
                callback_group=callback_group,
            )
            self.__kinematic_path_request = GetMotionPlan.Request()

        # Create a separate service client for Cartesian planning
        self._plan_cartesian_path_service = self._node.create_client(
            srv_type=GetCartesianPath,
            srv_name="compute_cartesian_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__cartesian_path_request = GetCartesianPath.Request()

        # Create action client for trajectory execution
        self.__follow_joint_trajectory_action_client = ActionClient(
            node=self._node,
            action_type=FollowJointTrajectory,
            action_name=follow_joint_trajectory_action_name,
            goal_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            result_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            cancel_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            feedback_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            status_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self.__collision_object_publisher = self._node.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        self.__joint_state_mutex = threading.Lock()
        self.__joint_state = None
        self.__new_joint_state_available = False
        self.__move_action_goal = self.__init_move_action_goal(
            frame_id=base_link_name,
            group_name=group_name,
            end_effector=end_effector_name,
        )

        # Flag to determine whether to execute trajectories via MoveIt2, or rather by calling a separate action with the controller itself
        # Applies to `move_to_pose()` and `move_to_configuraion()`
        self.__execute_via_moveit = execute_via_moveit

        # Flag that determines whether a new goal can be send while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing

        # Store additional variables for later use
        self.__joint_names = joint_names
        self.__base_link_name = base_link_name
        self.__end_effector_name = end_effector_name
        self.__group_name = group_name

        # Internal states that monitor the current motion requests and execution
        self.__is_motion_requested = False
        self.__is_executing = False
        self.__wait_until_executed_rate = self._node.create_rate(1000.0)

        # Event that enables waiting until async future is done
        self.__future_done_event = threading.Event()

    def move_to_pose(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        target_link: Optional[str] = None,
        frame_id: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        cartesian: bool = False,
        weight_orientation: float = 1.0,
    ):
        """
        Plan and execute motion based on previously set goals. Optional arguments can be
        passed in to internally use `set_pose_goal()` to define a goal during the call.
        """

        if self.__execute_via_moveit:
            if self.__ignore_new_calls_while_executing and self.__is_executing:
                self._node.get_logger().warn(
                    "Controller is already following a trajectory. Skipping motion."
                )
                return
            self.__is_motion_requested = True

            # Set goal
            self.set_pose_goal(
                position=position,
                quat_xyzw=quat_xyzw,
                frame_id=frame_id,
                target_link=target_link,
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation,
                weight_position=weight_position,
                weight_orientation=weight_orientation,
            )
            # Define starting state as the current state
            if self.joint_state is not None:
                self.__move_action_goal.request.start_state.joint_state = (
                    self.joint_state
                )
            # Send to goal to the server (async) - both planning and execution
            self._send_goal_async_move_action()
            # Clear all previous goal constrains
            self.clear_goal_constraints()

        else:
            # Plan via MoveIt 2 and then execute directly with the controller
            self.execute(
                self.plan(
                    position=position,
                    quat_xyzw=quat_xyzw,
                    frame_id=frame_id,
                    tolerance_position=tolerance_position,
                    tolerance_orientation=tolerance_orientation,
                    weight_position=weight_position,
                    weight_orientation=weight_orientation,
                    cartesian=cartesian,
                )
            )

    def move_to_configuration(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        cartesian: bool = False,
        weight: float = 1.0,
    ):
        """
        Plan and execute motion based on previously set goals. Optional arguments can be
        passed in to internally use `set_joint_goal()` to define a goal during the call.
        """

        if self.__execute_via_moveit:
            if self.__ignore_new_calls_while_executing and self.__is_executing:
                self._node.get_logger().warn(
                    "Controller is already following a trajectory. Skipping motion."
                )
                return
            self.__is_motion_requested = True

            # Set goal
            self.set_joint_goal(
                joint_positions=joint_positions,
                joint_names=joint_names,
                tolerance=tolerance,
                weight=weight,
            )
            # Define starting state as the current state
            if self.joint_state is not None:
                self.__move_action_goal.request.start_state.joint_state = (
                    self.joint_state
                )
            # Send to goal to the server (async) - both planning and execution
            self._send_goal_async_move_action()
            # Clear all previous goal constrains
            self.clear_goal_constraints()

        else:
            # Plan via MoveIt 2 and then execute directly with the controller
            self.execute(
                self.plan(
                    joint_positions=joint_positions,
                    joint_names=joint_names,
                    tolerance_joint_position=tolerance,
                    weight_joint_position=weight,
                    cartesian=cartesian,
                )
            )

    def plan(
        self,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        joint_positions: Optional[List[float]] = None,
        joint_names: Optional[List[str]] = None,
        frame_id: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        tolerance_joint_position: float = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
        weight_joint_position: float = 1.0,
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        cartesian: bool = False,
    ) -> Optional[JointTrajectory]:
        """
        Plan motion based on previously set goals. Optional arguments can be passed in to
        internally use `set_position_goal()`, `set_orientation_goal()` or `set_joint_goal()`
        to define a goal during the call. If no trajectory is found within the timeout
        duration, `None` is returned. To plan from the different position than the current
        one, optional argument `start_` can be defined.
        """

        if position is not None:
            self.set_position_goal(
                position=position,
                frame_id=frame_id,
                tolerance=tolerance_position,
                weight=weight_position,
            )

        if quat_xyzw is not None:
            self.set_orientation_goal(
                quat_xyzw=quat_xyzw,
                frame_id=frame_id,
                tolerance=tolerance_orientation,
                weight=weight_orientation,
            )

        if joint_positions is not None:
            self.set_joint_goal(
                joint_positions=joint_positions,
                joint_names=joint_names,
                tolerance=tolerance_joint_position,
                weight=weight_joint_position,
            )

        # Define starting state for the plan (default to the current state)
        if start_joint_state is not None:
            if isinstance(start_joint_state, JointState):
                self.__move_action_goal.request.start_state.joint_state = (
                    start_joint_state
                )
            else:
                self.__move_action_goal.request.start_state.joint_state = (
                    init_joint_state(
                        joint_names=self.__joint_names,
                        joint_positions=start_joint_state,
                    )
                )
        elif self.joint_state is not None:
            self.__move_action_goal.request.start_state.joint_state = self.joint_state

        # Plan trajectory by sending a goal (blocking)
        if cartesian:
            joint_trajectory = self._plan_cartesian_path()
        else:
            if self.__execute_via_moveit:
                # Use action client
                joint_trajectory = self._send_goal_move_action_plan_only()
            else:
                # Use service
                joint_trajectory = self._plan_kinematic_path()

        # Clear all previous goal constrains
        self.clear_goal_constraints()

        return joint_trajectory

    def execute(self, joint_trajectory: JointTrajectory):
        """
        Execute joint_trajectory by communicating directly with the controller.
        """

        if self.__ignore_new_calls_while_executing and self.__is_executing:
            self._node.get_logger().warn(
                "Controller is already following a trajectory. Skipping motion."
            )
            return
        self.__is_motion_requested = True

        follow_joint_trajectory_goal = init_follow_joint_trajectory_goal(
            joint_trajectory=joint_trajectory
        )

        if follow_joint_trajectory_goal is None:
            self._node.get_logger().warn(
                "Cannot execute motion because the provided/planned trajectory is invalid."
            )
            self.__is_motion_requested = False
            return

        self._send_goal_async_follow_joint_trajectory(goal=follow_joint_trajectory_goal)

    def wait_until_executed(self):
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        """

        if not self.__is_motion_requested:
            self._node.get_logger().warn(
                "Cannot wait until motion is executed (no motion is in progress)."
            )
            return

        while self.__is_motion_requested or self.__is_executing:
            self.__wait_until_executed_rate.sleep()

    def reset_controller(
        self, joint_state: Union[JointState, List[float]], sync: bool = True
    ):
        """
        Reset controller to a given `joint_state` by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        if not isinstance(joint_state, JointState):
            joint_state = init_joint_state(
                joint_names=self.__joint_names,
                joint_positions=joint_state,
            )
        joint_trajectory = init_dummy_joint_trajectory_from_state(joint_state)
        follow_joint_trajectory_goal = init_follow_joint_trajectory_goal(
            joint_trajectory=joint_trajectory
        )

        self._send_goal_async_follow_joint_trajectory(
            goal=follow_joint_trajectory_goal,
            wait_until_response=sync,
        )

    def set_pose_goal(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
    ):
        """
        This is direct combination of `set_position_goal()` and `set_orientation_goal()`.
        """

        self.set_position_goal(
            position=position,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance_position,
            weight=weight_position,
        )
        self.set_orientation_goal(
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance_orientation,
            weight=weight_orientation,
        )

    def set_position_goal(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set Cartesian position goal of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        # Create new position constraint
        constraint = PositionConstraint()

        # Define reference frame and target link
        constraint.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        constraint.link_name = (
            target_link if target_link is not None else self.__end_effector_name
        )

        # Define target position
        constraint.constraint_region.primitive_poses.append(Pose())
        if isinstance(position, Point):
            constraint.constraint_region.primitive_poses[0].position = position
        else:
            constraint.constraint_region.primitive_poses[0].position.x = float(
                position[0]
            )
            constraint.constraint_region.primitive_poses[0].position.y = float(
                position[1]
            )
            constraint.constraint_region.primitive_poses[0].position.z = float(
                position[2]
            )

        # Define goal region as a sphere with radius equal to the tolerance
        constraint.constraint_region.primitives.append(SolidPrimitive())
        constraint.constraint_region.primitives[0].type = 2  # Sphere
        constraint.constraint_region.primitives[0].dimensions = [tolerance]

        # Set weight of the constraint
        constraint.weight = weight

        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[
            -1
        ].position_constraints.append(constraint)

    def set_orientation_goal(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set Cartesian orientation goal of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        # Create new position constraint
        constraint = OrientationConstraint()

        # Define reference frame and target link
        constraint.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        constraint.link_name = (
            target_link if target_link is not None else self.__end_effector_name
        )

        # Define target orientation
        if isinstance(quat_xyzw, Quaternion):
            constraint.orientation = quat_xyzw
        else:
            constraint.orientation.x = float(quat_xyzw[0])
            constraint.orientation.y = float(quat_xyzw[1])
            constraint.orientation.z = float(quat_xyzw[2])
            constraint.orientation.w = float(quat_xyzw[3])

        # Define tolerances
        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance

        # Set weight of the constraint
        constraint.weight = weight

        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[
            -1
        ].orientation_constraints.append(constraint)

    def set_joint_goal(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set joint space goal. With `joint_names` specified, `joint_positions` can be
        defined for specific joints in an arbitrary order. Otherwise, first **n** joints
        passed into the constructor is used, where **n** is the length of `joint_positions`.
        """

        # Use default joint names if not specified
        if joint_names == None:
            joint_names = self.__joint_names

        for i in range(len(joint_positions)):
            # Create a new constraint for each joint
            constraint = JointConstraint()

            # Define joint name
            constraint.joint_name = joint_names[i]

            # Define the target joint position
            constraint.position = joint_positions[i]

            # Define telerances
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance

            # Set weight of the constraint
            constraint.weight = weight

            # Append to other constraints
            self.__move_action_goal.request.goal_constraints[
                -1
            ].joint_constraints.append(constraint)

    def clear_goal_constraints(self):
        """
        Clear all goal constraints that were previously set.
        Note that this function is called automatically after each `plan_kinematic_path()`.
        """

        self.__move_action_goal.request.goal_constraints = [Constraints()]

    def create_new_goal_constraint(self):
        """
        Create a new set of goal constraints that will be set together with the request. Each
        subsequent setting of goals with `set_joint_goal()`, `set_pose_goal()` and others will be
        added under this newly created set of constraints.
        """

        self.__move_action_goal.request.goal_constraints.append(Constraints())

    def compute_fk(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        fk_link_names: Optional[List[str]] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
    ) -> Optional[PoseStamped]:
        """
        Compute forward kinematics for all `fk_link_names` in a given `joint_state`.
          - `fk_link_names` defaults to end-effector
          - `joint_state` defaults to the current joint state
        """

        if not hasattr(self, "__compute_fk_client"):
            self.__init_compute_fk()

        if fk_link_names is None:
            self.__compute_fk_req.fk_link_names = [self.__end_effector_name]
        else:
            self.__compute_fk_req.fk_link_names = fk_link_names

        if joint_state is not None:
            if isinstance(joint_state, JointState):
                self.__compute_fk_req.robot_state.joint_state = joint_state
            else:
                self.__compute_fk_req.robot_state.joint_state = init_joint_state(
                    joint_names=self.__joint_names,
                    joint_positions=joint_state,
                )
        elif self.joint_state is not None:
            self.__compute_fk_req.robot_state.joint_state = self.joint_state

        stamp = self._node.get_clock().now().to_msg()
        self.__compute_fk_req.header.stamp = stamp

        if not self.__compute_fk_client.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__compute_fk_client.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        res = self.__compute_fk_client.call(self.__compute_fk_req)

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.pose_stamped
        else:
            self._node.get_logger().warn(
                f"FK computation failed! Error code: {res.error_code.val}."
            )
            return None

    def compute_ik(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        constraints: Optional[Constraints] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
    ) -> Optional[JointState]:
        """
        Compute inverse kinematics for the given pose. To indicate beginning of the search space,
        `start_joint_state` can be specified. Furthermore, `constraints` can be imposed on the
        computed IK.
          - `start_joint_state` defaults to current joint state.
          - `constraints` defaults to None.
        """

        if not hasattr(self, "__compute_ik_client"):
            self.__init_compute_ik()

        if isinstance(position, Point):
            self.__compute_ik_req.ik_request.pose_stamped.pose.position = position
        else:
            self.__compute_ik_req.ik_request.pose_stamped.pose.position.x = float(
                position[0]
            )
            self.__compute_ik_req.ik_request.pose_stamped.pose.position.y = float(
                position[1]
            )
            self.__compute_ik_req.ik_request.pose_stamped.pose.position.z = float(
                position[2]
            )
        if isinstance(quat_xyzw, Quaternion):
            self.__compute_ik_req.ik_request.pose_stamped.pose.orientation = quat_xyzw
        else:
            self.__compute_ik_req.ik_request.pose_stamped.pose.orientation.x = float(
                quat_xyzw[0]
            )
            self.__compute_ik_req.ik_request.pose_stamped.pose.orientation.y = float(
                quat_xyzw[1]
            )
            self.__compute_ik_req.ik_request.pose_stamped.pose.orientation.z = float(
                quat_xyzw[2]
            )
            self.__compute_ik_req.ik_request.pose_stamped.pose.orientation.w = float(
                quat_xyzw[3]
            )

        if start_joint_state is not None:
            if isinstance(start_joint_state, JointState):
                self.__compute_ik_req.ik_request.robot_state.joint_state = (
                    start_joint_state
                )
            else:
                self.__compute_ik_req.ik_request.robot_state.joint_state = (
                    init_joint_state(
                        joint_names=self.__joint_names,
                        joint_positions=start_joint_state,
                    )
                )
        elif self.joint_state is not None:
            self.__compute_ik_req.ik_request.robot_state.joint_state = self.joint_state

        if constraints is not None:
            self.__compute_ik_req.ik_request.constraints = constraints

        stamp = self._node.get_clock().now().to_msg()
        self.__compute_ik_req.ik_request.pose_stamped.header.stamp = stamp

        if not self.__compute_ik_client.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__compute_ik_client.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        res = self.__compute_ik_client.call(self.__compute_ik_req)

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.solution.joint_state
        else:
            self._node.get_logger().warn(
                f"IK computation failed! Error code: {res.error_code.val}."
            )
            return None

    def reset_new_joint_state_checker(self):
        """
        Reset checker of the new joint state.
        """

        self.__joint_state_mutex.acquire()
        self.__new_joint_state_available = False
        self.__joint_state_mutex.release()

    def force_reset_executing_state(self):
        """
        Force reset of internal states that block execution while `ignore_new_calls_while_executing` is being
        used. This function is applicable only in a very few edge-cases, so it should almost never be used.
        """

        self.__is_motion_requested = False
        self.__is_executing = False

    def add_collision_mesh(
        self,
        filepath: str,
        id: str,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        operation: int = CollisionObject.ADD,
        frame_id: Optional[str] = None,
    ):
        """
        Add collision object with a mesh geometry specified by `filepath`.
        Note: This function required 'trimesh' Python module to be installed.
        """

        try:
            import trimesh
        except ImportError as err:
            raise ImportError(
                "Python module 'trimesh' not found! Please install it manually in order "
                "to add collision objects into the MoveIt 2 planning scene."
            ) from err

        mesh = trimesh.load(filepath)
        msg = CollisionObject()

        if not isinstance(position, Point):
            position = Point(
                x=float(position[0]), y=float(position[1]), z=float(position[2])
            )
        if not isinstance(quat_xyzw, Quaternion):
            quat_xyzw = Quaternion(
                x=float(quat_xyzw[0]),
                y=float(quat_xyzw[1]),
                z=float(quat_xyzw[2]),
                w=float(quat_xyzw[3]),
            )

        pose = Pose()
        pose.position = position
        pose.orientation = quat_xyzw
        msg.pose = pose

        msg.meshes.append(
            Mesh(
                triangles=[MeshTriangle(vertex_indices=face) for face in mesh.faces],
                vertices=[
                    Point(x=vert[0], y=vert[1], z=vert[2]) for vert in mesh.vertices
                ],
            )
        )

        msg.id = id
        msg.operation = operation
        msg.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        msg.header.stamp = self._node.get_clock().now().to_msg()

        self.__collision_object_publisher.publish(msg)

    def remove_collision_mesh(self, id: str):
        """
        Remove collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.id = id
        msg.operation = CollisionObject.REMOVE
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self.__collision_object_publisher.publish(msg)

    def __joint_state_callback(self, msg: JointState):

        # Update only if all relevant joints are included in the message
        for joint_name in self.joint_names:
            if not joint_name in msg.name:
                return

        self.__joint_state_mutex.acquire()
        self.__joint_state = msg
        self.__new_joint_state_available = True
        self.__joint_state_mutex.release()

    def _send_goal_move_action_plan_only(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0
    ) -> Optional[JointTrajectory]:

        # Set action goal to only do planning without execution
        original_plan_only = self.__move_action_goal.planning_options.plan_only
        self.__move_action_goal.planning_options.plan_only = True

        stamp = self._node.get_clock().now().to_msg()
        self.__move_action_goal.request.workspace_parameters.header.stamp = stamp

        if not self.__move_action_client.wait_for_server(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Action server '{self.__move_action_client._action_name}' is not yet available. Better luck next time!"
            )
            return None

        move_action_result = self.__move_action_client.send_goal(
            goal=self.__move_action_goal,
            feedback_callback=None,
        )

        # Revert back to original planning/execution mode
        self.__move_action_goal.planning_options.plan_only = original_plan_only

        if move_action_result.status == GoalStatus.STATUS_SUCCEEDED:
            return move_action_result.result.planned_trajectory.joint_trajectory
        else:
            return None

    def _plan_kinematic_path(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0
    ) -> Optional[JointTrajectory]:

        # Re-use request from move action goal
        self.__kinematic_path_request.motion_plan_request = (
            self.__move_action_goal.request
        )

        stamp = self._node.get_clock().now().to_msg()
        self.__kinematic_path_request.motion_plan_request.workspace_parameters.header.stamp = (
            stamp
        )
        for (
            constraints
        ) in self.__kinematic_path_request.motion_plan_request.goal_constraints:
            for position_constraint in constraints.position_constraints:
                position_constraint.header.stamp = stamp
            for orientation_constraint in constraints.orientation_constraints:
                orientation_constraint.header.stamp = stamp

        if not self._plan_kinematic_path_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self._plan_kinematic_path_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        res = self._plan_kinematic_path_service.call(
            self.__kinematic_path_request
        ).motion_plan_response

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.trajectory.joint_trajectory
        else:
            self._node.get_logger().warn(
                f"Planning failed! Error code: {res.error_code.val}."
            )
            return None

    def _plan_cartesian_path(
        self,
        max_step: float = 0.0025,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
    ) -> Optional[JointTrajectory]:

        # Re-use request from move action goal
        self.__cartesian_path_request.start_state = (
            self.__move_action_goal.request.start_state
        )
        self.__cartesian_path_request.group_name = (
            self.__move_action_goal.request.group_name
        )
        self.__cartesian_path_request.link_name = self.__end_effector_name
        self.__cartesian_path_request.max_step = max_step

        stamp = self._node.get_clock().now().to_msg()
        self.__cartesian_path_request.header.stamp = stamp

        self.__cartesian_path_request.path_constraints = (
            self.__move_action_goal.request.path_constraints
        )
        for (
            position_constraint
        ) in self.__cartesian_path_request.path_constraints.position_constraints:
            position_constraint.header.stamp = stamp
        for (
            orientation_constraint
        ) in self.__cartesian_path_request.path_constraints.orientation_constraints:
            orientation_constraint.header.stamp = stamp
        # no header in joint_constraint message type

        target_pose = Pose()
        target_pose.position = (
            self.__move_action_goal.request.goal_constraints[-1]
            .position_constraints[-1]
            .constraint_region.primitive_poses[0]
            .position
        )
        target_pose.orientation = (
            self.__move_action_goal.request.goal_constraints[-1]
            .orientation_constraints[-1]
            .orientation
        )

        self.__cartesian_path_request.waypoints = [target_pose]

        if not self._plan_cartesian_path_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self._plan_cartesian_path_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        res = self._plan_cartesian_path_service.call(self.__cartesian_path_request)

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.solution.joint_trajectory
        else:
            self._node.get_logger().warn(
                f"Planning failed! Error code: {res.error_code.val}."
            )
            return None

    def _send_goal_async_move_action(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0
    ):

        stamp = self._node.get_clock().now().to_msg()
        self.__move_action_goal.request.workspace_parameters.header.stamp = stamp

        if not self.__move_action_client.wait_for_server(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Action server '{self.__move_action_client._action_name}' is not yet available. Better luck next time!"
            )
            self.__is_motion_requested = False
            return

        self.__send_goal_future_move_action = self.__move_action_client.send_goal_async(
            goal=self.__move_action_goal,
            feedback_callback=None,
        )

        self.__send_goal_future_move_action.add_done_callback(
            self.__response_callback_move_action
        )

    def __response_callback_move_action(self, response):

        goal_handle = response.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"Action '{self.__move_action_client._action_name}' was rejected."
            )
            self.__is_motion_requested = False
            return

        self.__is_executing = True
        self.__is_motion_requested = False

        self.__get_result_future_move_action = goal_handle.get_result_async()
        self.__get_result_future_move_action.add_done_callback(
            self.__result_callback_move_action
        )

    def __result_callback_move_action(self, res):

        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().error(
                f"Action '{self.__move_action_client._action_name}' was unsuccessful: {res.result().status}."
            )

        self.__is_executing = False

    def _send_goal_async_follow_joint_trajectory(
        self,
        goal: FollowJointTrajectory,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
        wait_until_response: bool = False,
    ):

        if not self.__follow_joint_trajectory_action_client.wait_for_server(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Action server '{self.__follow_joint_trajectory_action_client._action_name}' is not yet available. Better luck next time!"
            )
            self.__is_motion_requested = False
            return None

        action_result = self.__follow_joint_trajectory_action_client.send_goal_async(
            goal=goal,
            feedback_callback=None,
        )

        action_result.add_done_callback(
            self.__response_callback_follow_joint_trajectory
        )

        if wait_until_response:
            self.__future_done_event.clear()
            action_result.add_done_callback(
                self.__response_callback_with_event_set_follow_joint_trajectory
            )
            self.__future_done_event.wait(timeout=wait_for_server_timeout_sec)
        else:
            action_result.add_done_callback(
                self.__response_callback_follow_joint_trajectory
            )

    def __response_callback_follow_joint_trajectory(self, response):

        goal_handle = response.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"Action '{self.__follow_joint_trajectory_action_client._action_name}' was rejected."
            )
            self.__is_motion_requested = False
            return

        self.__is_executing = True
        self.__is_motion_requested = False

        self.__get_result_future_follow_joint_trajectory = (
            goal_handle.get_result_async()
        )
        self.__get_result_future_follow_joint_trajectory.add_done_callback(
            self.__result_callback_follow_joint_trajectory
        )

    def __response_callback_with_event_set_follow_joint_trajectory(self, response):

        self.__response_callback_follow_joint_trajectory(response)
        self.__future_done_event.set()

    def __result_callback_follow_joint_trajectory(self, res):

        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().error(
                f"Action '{self.__follow_joint_trajectory_action_client._action_name}' was unsuccessful: {res.result().status}."
            )

        self.__is_executing = False

    @classmethod
    def __init_move_action_goal(
        cls, frame_id: str, group_name: str, end_effector: str
    ) -> MoveGroup.Goal:

        move_action_goal = MoveGroup.Goal()
        move_action_goal.request.workspace_parameters.header.frame_id = frame_id
        # move_action_goal.request.workspace_parameters.header.stamp = "Set during request"
        move_action_goal.request.workspace_parameters.min_corner.x = -1.0
        move_action_goal.request.workspace_parameters.min_corner.y = -1.0
        move_action_goal.request.workspace_parameters.min_corner.z = -1.0
        move_action_goal.request.workspace_parameters.max_corner.x = 1.0
        move_action_goal.request.workspace_parameters.max_corner.y = 1.0
        move_action_goal.request.workspace_parameters.max_corner.z = 1.0
        # move_action_goal.request.start_state = "Set during request"
        move_action_goal.request.goal_constraints = [Constraints()]
        # move_action_goal.request.path_constraints = "Ignored"
        # move_action_goal.request.trajectory_constraints = "Ignored"
        # move_action_goal.request.reference_trajectories = "Ignored"
        # move_action_goal.request.pipeline_id = "Ignored"
        # move_action_goal.request.planner_id = "Ignored"
        move_action_goal.request.group_name = group_name
        move_action_goal.request.num_planning_attempts = 5
        move_action_goal.request.allowed_planning_time = 0.5
        move_action_goal.request.max_velocity_scaling_factor = 0.0
        move_action_goal.request.max_acceleration_scaling_factor = 0.0
        move_action_goal.request.cartesian_speed_end_effector_link = end_effector
        move_action_goal.request.max_cartesian_speed = 0.0

        # move_action_goal.planning_options.planning_scene_diff = "Ignored"
        move_action_goal.planning_options.plan_only = False
        # move_action_goal.planning_options.look_around = "Ignored"
        # move_action_goal.planning_options.look_around_attempts = "Ignored"
        # move_action_goal.planning_options.max_safe_execution_cost = "Ignored"
        # move_action_goal.planning_options.replan = "Ignored"
        # move_action_goal.planning_options.replan_attempts = "Ignored"
        # move_action_goal.planning_options.replan_delay = "Ignored"

        return move_action_goal

    def __init_compute_fk(self):

        self.__compute_fk_client = self._node.create_client(
            srv_type=GetPositionFK,
            srv_name="compute_fk",
            callback_group=self._callback_group,
        )

        self.__compute_fk_req = GetPositionFK.Request()
        self.__compute_fk_req.header.frame_id = self.__base_link_name
        # self.__compute_fk_req.header.stamp = "Set during request"
        # self.__compute_fk_req.fk_link_names = "Set during request"
        # self.__compute_fk_req.robot_state.joint_state = "Set during request"
        # self.__compute_fk_req.robot_state.multi_dof_ = "Ignored"
        # self.__compute_fk_req.robot_state.attached_collision_objects = "Ignored"
        self.__compute_fk_req.robot_state.is_diff = False

    def __init_compute_ik(self):

        # Service client for IK
        self.__compute_ik_client = self._node.create_client(
            srv_type=GetPositionIK,
            srv_name="compute_ik",
            callback_group=self._callback_group,
        )

        self.__compute_ik_req = GetPositionIK.Request()
        self.__compute_ik_req.ik_request.group_name = self.__group_name
        # self.__compute_ik_req.ik_request.robot_state.joint_state = "Set during request"
        # self.__compute_ik_req.ik_request.robot_state.multi_dof_ = "Ignored"
        # self.__compute_ik_req.ik_request.robot_state.attached_collision_objects = "Ignored"
        self.__compute_ik_req.ik_request.robot_state.is_diff = False
        # self.__compute_ik_req.ik_request.constraints = "Set during request OR Ignored"
        self.__compute_ik_req.ik_request.avoid_collisions = True
        # self.__compute_ik_req.ik_request.ik_link_name = "Ignored"
        self.__compute_ik_req.ik_request.pose_stamped.header.frame_id = (
            self.__base_link_name
        )
        # self.__compute_ik_req.ik_request.pose_stamped.header.stamp = "Set during request"
        # self.__compute_ik_req.ik_request.pose_stamped.pose = "Set during request"
        # self.__compute_ik_req.ik_request.ik_link_names = "Ignored"
        # self.__compute_ik_req.ik_request.pose_stamped_vector = "Ignored"
        # self.__compute_ik_req.ik_request.timeout.sec = "Ignored"
        # self.__compute_ik_req.ik_request.timeout.nanosec = "Ignored"

    @property
    def joint_names(self) -> List[str]:

        return self.__joint_names

    @property
    def joint_state(self) -> Optional[JointState]:

        self.__joint_state_mutex.acquire()
        joint_state = self.__joint_state
        self.__joint_state_mutex.release()
        return joint_state

    @property
    def new_joint_state_available(self):

        return self.__new_joint_state_available

    @property
    def max_velocity(self) -> float:

        return self.__move_action_goal.request.max_velocity_scaling_factor

    @max_velocity.setter
    def max_velocity(self, value: float):

        self.__move_action_goal.request.max_velocity_scaling_factor = value

    @property
    def max_acceleration(self) -> float:

        return self.__move_action_goal.request.max_acceleration_scaling_factor

    @max_acceleration.setter
    def max_acceleration(self, value: float):

        self.__move_action_goal.request.max_acceleration_scaling_factor = value

    @property
    def max_cartesian_speed(self) -> float:

        return self.__move_action_goal.request.max_cartesian_speed

    @max_cartesian_speed.setter
    def max_cartesian_speed(self, value: float):

        self.__move_action_goal.request.max_cartesian_speed = value

    @property
    def num_planning_attempts(self) -> int:

        return self.__move_action_goal.request.num_planning_attempts

    @num_planning_attempts.setter
    def num_planning_attempts(self, value: int):

        self.__move_action_goal.request.num_planning_attempts = value

    @property
    def allowed_planning_time(self) -> float:

        return self.__move_action_goal.request.allowed_planning_time

    @allowed_planning_time.setter
    def allowed_planning_time(self, value: float):

        self.__move_action_goal.request.allowed_planning_time = value


def init_joint_state(
    joint_names: List[str],
    joint_positions: Optional[List[str]] = None,
    joint_velocities: Optional[List[str]] = None,
    joint_effort: Optional[List[str]] = None,
) -> JointState:

    joint_state = JointState()

    joint_state.name = joint_names
    joint_state.position = (
        joint_positions if joint_positions is not None else [0.0] * len(joint_names)
    )
    joint_state.velocity = (
        joint_velocities if joint_velocities is not None else [0.0] * len(joint_names)
    )
    joint_state.effort = (
        joint_effort if joint_effort is not None else [0.0] * len(joint_names)
    )

    return joint_state


def init_follow_joint_trajectory_goal(
    joint_trajectory: JointTrajectory,
) -> Optional[FollowJointTrajectory.Goal]:

    if joint_trajectory is None:
        return None

    follow_joint_trajectory_goal = FollowJointTrajectory.Goal()

    follow_joint_trajectory_goal.trajectory = joint_trajectory
    # follow_joint_trajectory_goal.multi_dof_trajectory = "Ignored"
    # follow_joint_trajectory_goal.path_tolerance = "Ignored"
    # follow_joint_trajectory_goal.component_path_tolerance = "Ignored"
    # follow_joint_trajectory_goal.goal_tolerance = "Ignored"
    # follow_joint_trajectory_goal.component_goal_tolerance = "Ignored"
    # follow_joint_trajectory_goal.goal_time_tolerance = "Ignored"

    return follow_joint_trajectory_goal


def init_dummy_joint_trajectory_from_state(
    joint_state: JointState, duration_sec: int = 0, duration_nanosec: int = 0
) -> JointTrajectory:

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = joint_state.name

    point = JointTrajectoryPoint()
    point.positions = joint_state.position
    point.velocities = joint_state.velocity
    point.accelerations = [0.0] * len(joint_trajectory.joint_names)
    point.effort = joint_state.effort
    point.time_from_start.sec = duration_sec
    point.time_from_start.nanosec = duration_nanosec
    joint_trajectory.points.append(point)

    return joint_trajectory
