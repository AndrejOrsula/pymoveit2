import copy
import threading
from enum import Enum
from typing import Any, List, Optional, Tuple, Union

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
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
from rclpy.task import Future
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveIt2State(Enum):
    """
    An enum the represents the current execution state of the MoveIt2 interface.
    - IDLE: No motion is being requested or executed
    - REQUESTING: Execution has been requested, but the request has not yet been
      accepted.
    - EXECUTING: Execution has been requested and accepted, and has not yet been
      completed.
    """

    IDLE = 0
    REQUESTING = 1
    EXECUTING = 2


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
        follow_joint_trajectory_action_name: str = "DEPRECATED",
        use_move_group_action: bool = False,
    ):
        """
        Construct an instance of `MoveIt2` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `joint_names` - List of joint names of the robot (can be extracted from URDF)
          - `base_link_name` - Name of the robot base link
          - `end_effector_name` - Name of the robot end effector
          - `group_name` - Name of the planning group for robot arm
          - [DEPRECATED] `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed otherwise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - [DEPRECATED] `follow_joint_trajectory_action_name` - Name of the action server for the controller
          - `use_move_group_action` - Flag that enables execution via MoveGroup action (MoveIt 2)
                               ExecuteTrajectory action is employed otherwise
                               together with a separate planning service client
        """

        self._node = node
        self._callback_group = callback_group

        # Check for deprecated parameters
        if execute_via_moveit:
            self._node.get_logger().warn(
                "Parameter `execute_via_moveit` is deprecated. Please use `use_move_group_action` instead."
            )
            use_move_group_action = True
        if follow_joint_trajectory_action_name != "DEPRECATED":
            self._node.get_logger().warn(
                "Parameter `follow_joint_trajectory_action_name` is deprecated. `MoveIt2` uses the `execute_trajectory` action instead."
            )

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

        # Also create a separate service client for planning
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
        self._execute_trajectory_action_client = ActionClient(
            node=self._node,
            action_type=ExecuteTrajectory,
            action_name="execute_trajectory",
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

        # Create a service for getting the planning scene
        self._get_planning_scene_service = self._node.create_client(
            srv_type=GetPlanningScene,
            srv_name="get_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__planning_scene = None
        self.__old_planning_scene = None
        self.__old_allowed_collision_matrix = None

        # Create a service for applying the planning scene
        self._apply_planning_scene_service = self._node.create_client(
            srv_type=ApplyPlanningScene,
            srv_name="apply_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )

        self.__collision_object_publisher = self._node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.__attached_collision_object_publisher = self._node.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        self.__cancellation_pub = self._node.create_publisher(
            String, "/trajectory_execution_event", 1
        )

        self.__joint_state_mutex = threading.Lock()
        self.__joint_state = None
        self.__new_joint_state_available = False
        self.__move_action_goal = self.__init_move_action_goal(
            frame_id=base_link_name,
            group_name=group_name,
            end_effector=end_effector_name,
        )

        # Flag to determine whether to execute trajectories via Move Group Action, or rather by calling
        # the separate ExecuteTrajectory action
        # Applies to `move_to_pose()` and `move_to_configuration()`
        self.__use_move_group_action = use_move_group_action

        # Flag that determines whether a new goal can be sent while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing

        # Store additional variables for later use
        self.__joint_names = joint_names
        self.__base_link_name = base_link_name
        self.__end_effector_name = end_effector_name
        self.__group_name = group_name

        # Internal states that monitor the current motion requests and execution
        self.__is_motion_requested = False
        self.__is_executing = False
        self.motion_suceeded = False
        self.__execution_goal_handle = None
        self.__last_error_code = None
        self.__wait_until_executed_rate = self._node.create_rate(1000.0)
        self.__execution_mutex = threading.Lock()

        # Event that enables waiting until async future is done
        self.__future_done_event = threading.Event()

    #### Execution Polling Functions
    def query_state(self) -> MoveIt2State:
        with self.__execution_mutex:
            if self.__is_motion_requested:
                return MoveIt2State.REQUESTING
            elif self.__is_executing:
                return MoveIt2State.EXECUTING
            else:
                return MoveIt2State.IDLE

    def cancel_execution(self):
        if self.query_state() != MoveIt2State.EXECUTING:
            self._node.get_logger().warn("Attempted to cancel without active goal.")
            return None

        cancel_string = String()
        cancel_string.data = "stop"
        self.__cancellation_pub.publish(cancel_string)

    def get_execution_future(self) -> Optional[Future]:
        if self.query_state() != MoveIt2State.EXECUTING:
            self._node.get_logger().warn("Need active goal for future.")
            return None

        return self.__execution_goal_handle.get_result_async()

    def get_last_execution_error_code(self) -> Optional[MoveItErrorCodes]:
        return self.__last_error_code

    ####

    def move_to_pose(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        target_link: Optional[str] = None,
        frame_id: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        cartesian: bool = False,
        weight_orientation: float = 1.0,
        cartesian_max_step: float = 0.0025,
        cartesian_fraction_threshold: float = 0.0,
    ):
        """
        Plan and execute motion based on previously set goals. Optional arguments can be
        passed in to internally use `set_pose_goal()` to define a goal during the call.
        """

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
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
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        if self.__use_move_group_action and not cartesian:
            if self.__ignore_new_calls_while_executing and (
                self.__is_motion_requested or self.__is_executing
            ):
                self._node.get_logger().warn(
                    "Controller is already following a trajectory. Skipping motion."
                )
                return

            # Set goal
            self.set_pose_goal(
                position=pose_stamped.pose.position,
                quat_xyzw=pose_stamped.pose.orientation,
                frame_id=pose_stamped.header.frame_id,
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
            self.clear_path_constraints()

        else:
            # Plan via MoveIt 2 and then execute directly with the controller
            self.execute(
                self.plan(
                    position=pose_stamped.pose.position,
                    quat_xyzw=pose_stamped.pose.orientation,
                    frame_id=pose_stamped.header.frame_id,
                    target_link=target_link,
                    tolerance_position=tolerance_position,
                    tolerance_orientation=tolerance_orientation,
                    weight_position=weight_position,
                    weight_orientation=weight_orientation,
                    cartesian=cartesian,
                    max_step=cartesian_max_step,
                    cartesian_fraction_threshold=cartesian_fraction_threshold,
                )
            )

    def move_to_configuration(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Plan and execute motion based on previously set goals. Optional arguments can be
        passed in to internally use `set_joint_goal()` to define a goal during the call.
        """

        if self.__use_move_group_action:
            if self.__ignore_new_calls_while_executing and (
                self.__is_motion_requested or self.__is_executing
            ):
                self._node.get_logger().warn(
                    "Controller is already following a trajectory. Skipping motion."
                )
                return

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
            self.clear_path_constraints()

        else:
            # Plan via MoveIt 2 and then execute directly with the controller
            self.execute(
                self.plan(
                    joint_positions=joint_positions,
                    joint_names=joint_names,
                    tolerance_joint_position=tolerance,
                    weight_joint_position=weight,
                )
            )

    def plan(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        joint_positions: Optional[List[float]] = None,
        joint_names: Optional[List[str]] = None,
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: Union[float, Tuple[float, float, float]] = 0.001,
        tolerance_joint_position: float = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
        weight_joint_position: float = 1.0,
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        cartesian: bool = False,
        max_step: float = 0.0025,
        cartesian_fraction_threshold: float = 0.0,
    ) -> Optional[JointTrajectory]:
        """
        Call plan_async and wait on future
        """
        future = self.plan_async(
            **{
                key: value
                for key, value in locals().items()
                if key not in ["self", "cartesian_fraction_threshold"]
            }
        )

        if future is None:
            return None

        # 100ms sleep
        rate = self._node.create_rate(10)
        while not future.done():
            rate.sleep()

        return self.get_trajectory(
            future,
            cartesian=cartesian,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )

    def plan_async(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        joint_positions: Optional[List[float]] = None,
        joint_names: Optional[List[str]] = None,
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: Union[float, Tuple[float, float, float]] = 0.001,
        tolerance_joint_position: float = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
        weight_joint_position: float = 1.0,
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        cartesian: bool = False,
        max_step: float = 0.0025,
    ) -> Optional[Future]:
        """
        Plan motion based on previously set goals. Optional arguments can be passed in to
        internally use `set_position_goal()`, `set_orientation_goal()` or `set_joint_goal()`
        to define a goal during the call. If no trajectory is found within the timeout
        duration, `None` is returned. To plan from the different position than the current
        one, optional argument `start_` can be defined.
        """

        pose_stamped = None
        if pose is not None:
            if isinstance(pose, PoseStamped):
                pose_stamped = pose
            elif isinstance(pose, Pose):
                pose_stamped = PoseStamped(
                    header=Header(
                        stamp=self._node.get_clock().now().to_msg(),
                        frame_id=(
                            frame_id if frame_id is not None else self.__base_link_name
                        ),
                    ),
                    pose=pose,
                )

            self.set_position_goal(
                position=pose_stamped.pose.position,
                frame_id=pose_stamped.header.frame_id,
                target_link=target_link,
                tolerance=tolerance_position,
                weight=weight_position,
            )
            self.set_orientation_goal(
                quat_xyzw=pose_stamped.pose.orientation,
                frame_id=pose_stamped.header.frame_id,
                target_link=target_link,
                tolerance=tolerance_orientation,
                weight=weight_orientation,
            )
        else:
            if position is not None:
                if not isinstance(position, Point):
                    position = Point(
                        x=float(position[0]), y=float(position[1]), z=float(position[2])
                    )

                self.set_position_goal(
                    position=position,
                    frame_id=frame_id,
                    target_link=target_link,
                    tolerance=tolerance_position,
                    weight=weight_position,
                )

            if quat_xyzw is not None:
                if not isinstance(quat_xyzw, Quaternion):
                    quat_xyzw = Quaternion(
                        x=float(quat_xyzw[0]),
                        y=float(quat_xyzw[1]),
                        z=float(quat_xyzw[2]),
                        w=float(quat_xyzw[3]),
                    )

                self.set_orientation_goal(
                    quat_xyzw=quat_xyzw,
                    frame_id=frame_id,
                    target_link=target_link,
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

        # Plan trajectory asynchronously by service call
        if cartesian:
            future = self._plan_cartesian_path(
                max_step=max_step,
                frame_id=(
                    pose_stamped.header.frame_id
                    if pose_stamped is not None
                    else frame_id
                ),
            )
        else:
            # Use service
            future = self._plan_kinematic_path()

        # Clear all previous goal constrains
        self.clear_goal_constraints()
        self.clear_path_constraints()

        return future

    def get_trajectory(
        self,
        future: Future,
        cartesian: bool = False,
        cartesian_fraction_threshold: float = 0.0,
    ) -> Optional[JointTrajectory]:
        """
        Takes in a future returned by plan_async and returns the trajectory if the future is done
        and planning was successful, else None.

        For cartesian plans, the plan is rejected if the fraction of the path that was completed is
        less than `cartesian_fraction_threshold`.
        """
        if not future.done():
            self._node.get_logger().warn(
                "Cannot get trajectory because future is not done."
            )
            return None

        res = future.result()

        # Cartesian
        if cartesian:
            if MoveItErrorCodes.SUCCESS == res.error_code.val:
                if res.fraction >= cartesian_fraction_threshold:
                    return res.solution.joint_trajectory
                else:
                    self._node.get_logger().warn(
                        f"Planning failed! Cartesian planner completed {res.fraction} "
                        f"of the trajectory, less than the threshold {cartesian_fraction_threshold}."
                    )
                    return None
            else:
                self._node.get_logger().warn(
                    f"Planning failed! Error code: {res.error_code.val}."
                )
                return None

        # Else Kinematic
        res = res.motion_plan_response
        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.trajectory.joint_trajectory
        else:
            self._node.get_logger().warn(
                f"Planning failed! Error code: {res.error_code.val}."
            )
            return None

    def execute(self, joint_trajectory: JointTrajectory):
        """
        Execute joint_trajectory by communicating directly with the controller.
        """

        if self.__ignore_new_calls_while_executing and (
            self.__is_motion_requested or self.__is_executing
        ):
            self._node.get_logger().warn(
                "Controller is already following a trajectory. Skipping motion."
            )
            return

        execute_trajectory_goal = init_execute_trajectory_goal(
            joint_trajectory=joint_trajectory
        )

        if execute_trajectory_goal is None:
            self._node.get_logger().warn(
                "Cannot execute motion because the provided/planned trajectory is invalid."
            )
            return

        self._send_goal_async_execute_trajectory(goal=execute_trajectory_goal)

    def wait_until_executed(self) -> bool:
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        """

        if not self.__is_motion_requested:
            self._node.get_logger().warn(
                "Cannot wait until motion is executed (no motion is in progress)."
            )
            return False

        while self.__is_motion_requested or self.__is_executing:
            self.__wait_until_executed_rate.sleep()

        return self.motion_suceeded

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
        execute_trajectory_goal = init_execute_trajectory_goal(
            joint_trajectory=joint_trajectory
        )

        self._send_goal_async_execute_trajectory(
            goal=execute_trajectory_goal,
            wait_until_response=sync,
        )

    def set_pose_goal(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: Union[float, Tuple[float, float, float]] = 0.001,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
    ):
        """
        This is direct combination of `set_position_goal()` and `set_orientation_goal()`.
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
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
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        self.set_position_goal(
            position=pose_stamped.pose.position,
            frame_id=pose_stamped.header.frame_id,
            target_link=target_link,
            tolerance=tolerance_position,
            weight=weight_position,
        )
        self.set_orientation_goal(
            quat_xyzw=pose_stamped.pose.orientation,
            frame_id=pose_stamped.header.frame_id,
            target_link=target_link,
            tolerance=tolerance_orientation,
            weight=weight_orientation,
        )

    def create_position_constraint(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> PositionConstraint:
        """
        Create Cartesian position constraint of `target_link` with respect to `frame_id`.
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

        return constraint

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

        constraint = self.create_position_constraint(
            position=position,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance,
            weight=weight,
        )

        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[
            -1
        ].position_constraints.append(constraint)

    def create_orientation_constraint(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,  # 0: Euler, 1: Rotation Vector
    ) -> OrientationConstraint:
        """
        Create a Cartesian orientation constraint of `target_link` with respect to `frame_id`.
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
        if type(tolerance) == float:
            tolerance_xyz = (tolerance, tolerance, tolerance)
        else:
            tolerance_xyz = tolerance
        constraint.absolute_x_axis_tolerance = tolerance_xyz[0]
        constraint.absolute_y_axis_tolerance = tolerance_xyz[1]
        constraint.absolute_z_axis_tolerance = tolerance_xyz[2]

        # Define parameterization (how to interpret the tolerance)
        constraint.parameterization = parameterization

        # Set weight of the constraint
        constraint.weight = weight

        return constraint

    def set_orientation_goal(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,  # 0: Euler, 1: Rotation Vector
    ):
        """
        Set Cartesian orientation goal of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        constraint = self.create_orientation_constraint(
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance,
            weight=weight,
            parameterization=parameterization,
        )

        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[
            -1
        ].orientation_constraints.append(constraint)

    def create_joint_constraints(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> List[JointConstraint]:
        """
        Creates joint space constraints. With `joint_names` specified, `joint_positions` can be
        defined for specific joints in an arbitrary order. Otherwise, first **n** joints
        passed into the constructor is used, where **n** is the length of `joint_positions`.
        """

        constraints = []

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

            constraints.append(constraint)

        return constraints

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

        constraints = self.create_joint_constraints(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance=tolerance,
            weight=weight,
        )

        # Append to other constraints
        self.__move_action_goal.request.goal_constraints[-1].joint_constraints.extend(
            constraints
        )

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

    def set_path_joint_constraint(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set joint space path constraints. With `joint_names` specified, `joint_positions` can be
        defined for specific joints in an arbitrary order. Otherwise, first **n** joints
        passed into the constructor is used, where **n** is the length of `joint_positions`.
        """

        constraints = self.create_joint_constraints(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance=tolerance,
            weight=weight,
        )

        # Append to other constraints
        self.__move_action_goal.request.path_constraints.joint_constraints.extend(
            constraints
        )

    def set_path_position_constraint(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ):
        """
        Set Cartesian position path constraint of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        constraint = self.create_position_constraint(
            position=position,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance,
            weight=weight,
        )

        # Append to other constraints
        self.__move_action_goal.request.path_constraints.position_constraints.append(
            constraint
        )

    def set_path_orientation_constraint(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,  # 0: Euler Angles, 1: Rotation Vector
    ):
        """
        Set Cartesian orientation path constraint of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        constraint = self.create_orientation_constraint(
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            target_link=target_link,
            tolerance=tolerance,
            weight=weight,
            parameterization=parameterization,
        )

        # Append to other constraints
        self.__move_action_goal.request.path_constraints.orientation_constraints.append(
            constraint
        )

    def clear_path_constraints(self):
        """
        Clear all path constraints that were previously set.
        Note that this function is called automatically after each `plan_kinematic_path()`.
        """

        self.__move_action_goal.request.path_constraints = Constraints()

    def compute_fk(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        fk_link_names: Optional[List[str]] = None,
    ) -> Optional[Union[PoseStamped, List[PoseStamped]]]:
        """
        Call compute_fk_async and wait on future
        """
        future = self.compute_fk_async(
            **{key: value for key, value in locals().items() if key != "self"}
        )

        if future is None:
            return None

        # 100ms sleep
        rate = self._node.create_rate(10)
        while not future.done():
            rate.sleep()

        return self.get_compute_fk_result(future, fk_link_names=fk_link_names)

    def get_compute_fk_result(
        self,
        future: Future,
        fk_link_names: Optional[List[str]] = None,
    ) -> Optional[Union[PoseStamped, List[PoseStamped]]]:
        """
        Takes in a future returned by compute_fk_async and returns the poses
        if the future is done and successful, else None.
        """
        if not future.done():
            self._node.get_logger().warn(
                "Cannot get FK result because future is not done."
            )
            return None

        res = future.result()

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            if fk_link_names is None:
                return res.pose_stamped[0]
            else:
                return res.pose_stamped
        else:
            self._node.get_logger().warn(
                f"FK computation failed! Error code: {res.error_code.val}."
            )
            return None

    def compute_fk_async(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        fk_link_names: Optional[List[str]] = None,
    ) -> Optional[Future]:
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

        if not self.__compute_fk_client.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self.__compute_fk_client.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        return self.__compute_fk_client.call_async(self.__compute_fk_req)

    def compute_ik(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        constraints: Optional[Constraints] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
    ) -> Optional[JointState]:
        """
        Call compute_ik_async and wait on future
        """
        future = self.compute_ik_async(
            **{key: value for key, value in locals().items() if key != "self"}
        )

        if future is None:
            return None

        # 10ms sleep
        rate = self._node.create_rate(10)
        while not future.done():
            rate.sleep()

        return self.get_compute_ik_result(future)

    def get_compute_ik_result(
        self,
        future: Future,
    ) -> Optional[JointState]:
        """
        Takes in a future returned by compute_ik_async and returns the joint states
        if the future is done and successful, else None.
        """
        if not future.done():
            self._node.get_logger().warn(
                "Cannot get IK result because future is not done."
            )
            return None

        res = future.result()

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.solution.joint_state
        else:
            self._node.get_logger().warn(
                f"IK computation failed! Error code: {res.error_code.val}."
            )
            return None

    def compute_ik_async(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        constraints: Optional[Constraints] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
    ) -> Optional[Future]:
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

        return self.__compute_ik_client.call_async(self.__compute_ik_req)

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

        self.__execution_mutex.acquire()
        self.__is_motion_requested = False
        self.__is_executing = False
        self.__execution_mutex.release()

    def add_collision_primitive(
        self,
        id: str,
        primitive_type: int,
        dimensions: Tuple[float, float, float],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a primitive geometry specified by its dimensions.

        `primitive_type` can be one of the following:
            - `SolidPrimitive.BOX`
            - `SolidPrimitive.SPHERE`
            - `SolidPrimitive.CYLINDER`
            - `SolidPrimitive.CONE`
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
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
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.primitives.append(
            SolidPrimitive(type=primitive_type, dimensions=dimensions)
        )

        self.__collision_object_publisher.publish(msg)

    def add_collision_box(
        self,
        id: str,
        size: Tuple[float, float, float],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a box geometry specified by its size.
        """

        assert len(size) == 3, "Invalid size of the box!"

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.BOX,
            dimensions=size,
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_sphere(
        self,
        id: str,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a sphere geometry specified by its radius.
        """

        if quat_xyzw is None:
            quat_xyzw = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.SPHERE,
            dimensions=[
                radius,
            ],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cylinder(
        self,
        id: str,
        height: float,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cylinder geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CYLINDER,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_cone(
        self,
        id: str,
        height: float,
        radius: float,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a cone geometry specified by its height and radius.
        """

        self.add_collision_primitive(
            id=id,
            primitive_type=SolidPrimitive.CONE,
            dimensions=[height, radius],
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            operation=operation,
        )

    def add_collision_mesh(
        self,
        filepath: Optional[str],
        id: str,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
        scale: Union[float, Tuple[float, float, float]] = 1.0,
        mesh: Optional[Any] = None,
    ):
        """
        Add collision object with a mesh geometry. Either `filepath` must be
        specified or `mesh` must be provided.
        Note: This function required 'trimesh' Python module to be installed.
        """

        # Load the mesh
        try:
            import trimesh
        except ImportError as err:
            raise ImportError(
                "Python module 'trimesh' not found! Please install it manually in order "
                "to add collision objects into the MoveIt 2 planning scene."
            ) from err

        # Check the parameters
        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )
        if (filepath is None and mesh is None) or (
            filepath is not None and mesh is not None
        ):
            raise ValueError("Exactly one of `filepath` or `mesh` must be specified!")
        if mesh is not None and not isinstance(mesh, trimesh.Trimesh):
            raise ValueError("`mesh` must be an instance of `trimesh.Trimesh`!")

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=pose,
            )
        else:
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
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=(
                        frame_id if frame_id is not None else self.__base_link_name
                    ),
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        if filepath is not None:
            mesh = trimesh.load(filepath)

        # Scale the mesh
        if isinstance(scale, float):
            scale = (scale, scale, scale)
        if not (scale[0] == scale[1] == scale[2] == 1.0):
            # If the mesh was passed in as a parameter, make a copy of it to
            # avoid transforming the original.
            if filepath is not None:
                mesh = mesh.copy()
            # Transform the mesh
            transform = np.eye(4)
            np.fill_diagonal(transform, scale)
            mesh.apply_transform(transform)

        msg.meshes.append(
            Mesh(
                triangles=[MeshTriangle(vertex_indices=face) for face in mesh.faces],
                vertices=[
                    Point(x=vert[0], y=vert[1], z=vert[2]) for vert in mesh.vertices
                ],
            )
        )

        self.__collision_object_publisher.publish(msg)

    def remove_collision_object(self, id: str):
        """
        Remove collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.id = id
        msg.operation = CollisionObject.REMOVE
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self.__collision_object_publisher.publish(msg)

    def remove_collision_mesh(self, id: str):
        """
        Remove collision mesh specified by its `id`.
        Identical to `remove_collision_object()`.
        """

        self.remove_collision_object(id)

    def attach_collision_object(
        self,
        id: str,
        link_name: Optional[str] = None,
        touch_links: List[str] = [],
        weight: float = 0.0,
    ):
        """
        Attach collision object to the robot.
        """

        if link_name is None:
            link_name = self.__end_effector_name

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.ADD)
        )
        msg.link_name = link_name
        msg.touch_links = touch_links
        msg.weight = weight

        self.__attached_collision_object_publisher.publish(msg)

    def detach_collision_object(self, id: int):
        """
        Detach collision object from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.REMOVE)
        )
        self.__attached_collision_object_publisher.publish(msg)

    def detach_all_collision_objects(self):
        """
        Detach collision object from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE)
        )
        self.__attached_collision_object_publisher.publish(msg)

    def move_collision(
        self,
        id: str,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
    ):
        """
        Move collision object specified by its `id`.
        """

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
        msg.id = id
        msg.operation = CollisionObject.MOVE
        msg.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        msg.header.stamp = self._node.get_clock().now().to_msg()

        self.__collision_object_publisher.publish(msg)

    def update_planning_scene(self) -> bool:
        """
        Gets the current planning scene. Returns whether the service call was
        successful.
        """

        if not self._get_planning_scene_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._get_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return False
        self.__planning_scene = self._get_planning_scene_service.call(
            GetPlanningScene.Request()
        ).scene
        return True

    def allow_collisions(self, id: str, allow: bool) -> Optional[Future]:
        """
        Takes in the ID of an element in the planning scene. Modifies the allowed
        collision matrix to (dis)allow collisions between that object and all other
        object.

        If `allow` is True, a plan will succeed even if the robot collides with that object.
        If `allow` is False, a plan will fail if the robot collides with that object.
        Returns whether it successfully updated the allowed collision matrix.

        Returns the future of the service call.
        """
        # Update the planning scene
        if not self.update_planning_scene():
            return None
        allowed_collision_matrix = self.__planning_scene.allowed_collision_matrix
        self.__old_allowed_collision_matrix = copy.deepcopy(allowed_collision_matrix)

        # Get the location in the allowed collision matrix of the object
        j = None
        if id not in allowed_collision_matrix.entry_names:
            allowed_collision_matrix.entry_names.append(id)
        else:
            j = allowed_collision_matrix.entry_names.index(id)
        # For all other objects, (dis)allow collisions with the object with `id`
        for i in range(len(allowed_collision_matrix.entry_values)):
            if j is None:
                allowed_collision_matrix.entry_values[i].enabled.append(allow)
            elif i != j:
                allowed_collision_matrix.entry_values[i].enabled[j] = allow
        # For the object with `id`, (dis)allow collisions with all other objects
        allowed_collision_entry = AllowedCollisionEntry(
            enabled=[allow for _ in range(len(allowed_collision_matrix.entry_names))]
        )
        if j is None:
            allowed_collision_matrix.entry_values.append(allowed_collision_entry)
        else:
            allowed_collision_matrix.entry_values[j] = allowed_collision_entry

        # Apply the new planning scene
        if not self._apply_planning_scene_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._apply_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        return self._apply_planning_scene_service.call_async(
            ApplyPlanningScene.Request(scene=self.__planning_scene)
        )

    def process_allow_collision_future(self, future: Future) -> bool:
        """
        Return whether the allow collision service call is done and has succeeded
        or not. If it failed, reset the allowed collision matrix to the old one.
        """
        if not future.done():
            return False

        # Get response
        resp = future.result()

        # If it failed, restore the old planning scene
        if not resp.success:
            self.__planning_scene.allowed_collision_matrix = (
                self.__old_allowed_collision_matrix
            )

        return resp.success

    def clear_all_collision_objects(self) -> Optional[Future]:
        """
        Removes all attached and un-attached collision objects from the planning scene.

        Returns a future for the ApplyPlanningScene service call.
        """
        # Update the planning scene
        if not self.update_planning_scene():
            return None
        self.__old_planning_scene = copy.deepcopy(self.__planning_scene)

        # Remove all collision objects from the planning scene
        self.__planning_scene.world.collision_objects = []
        self.__planning_scene.robot_state.attached_collision_objects = []

        # Apply the new planning scene
        if not self._apply_planning_scene_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._apply_planning_scene_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None
        return self._apply_planning_scene_service.call_async(
            ApplyPlanningScene.Request(scene=self.__planning_scene)
        )

    def cancel_clear_all_collision_objects_future(self, future: Future):
        """
        Cancel the clear all collision objects service call.
        """
        self._apply_planning_scene_service.remove_pending_request(future)

    def process_clear_all_collision_objects_future(self, future: Future) -> bool:
        """
        Return whether the clear all collision objects service call is done and has succeeded
        or not. If it failed, restore the old planning scene.
        """
        if not future.done():
            return False

        # Get response
        resp = future.result()

        # If it failed, restore the old planning scene
        if not resp.success:
            self.__planning_scene = self.__old_planning_scene

        return resp.success

    def __joint_state_callback(self, msg: JointState):
        # Update only if all relevant joints are included in the message
        for joint_name in self.joint_names:
            if not joint_name in msg.name:
                return

        self.__joint_state_mutex.acquire()
        self.__joint_state = msg
        self.__new_joint_state_available = True
        self.__joint_state_mutex.release()

    def _plan_kinematic_path(self) -> Optional[Future]:
        # Reuse request from move action goal
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

        if not self._plan_kinematic_path_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._plan_kinematic_path_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        return self._plan_kinematic_path_service.call_async(
            self.__kinematic_path_request
        )

    def _plan_cartesian_path(
        self,
        max_step: float = 0.0025,
        frame_id: Optional[str] = None,
    ) -> Optional[Future]:
        # Reuse request from move action goal
        self.__cartesian_path_request.start_state = (
            self.__move_action_goal.request.start_state
        )

        # The below attributes were introduced in Iron and do not exist in Humble.
        if hasattr(self.__cartesian_path_request, "max_velocity_scaling_factor"):
            self.__cartesian_path_request.max_velocity_scaling_factor = (
                self.__move_action_goal.request.max_velocity_scaling_factor
            )
        if hasattr(self.__cartesian_path_request, "max_acceleration_scaling_factor"):
            self.__cartesian_path_request.max_acceleration_scaling_factor = (
                self.__move_action_goal.request.max_acceleration_scaling_factor
            )

        self.__cartesian_path_request.group_name = (
            self.__move_action_goal.request.group_name
        )
        self.__cartesian_path_request.link_name = self.__end_effector_name
        self.__cartesian_path_request.max_step = max_step

        self.__cartesian_path_request.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )

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

        if not self._plan_cartesian_path_service.service_is_ready():
            self._node.get_logger().warn(
                f"Service '{self._plan_cartesian_path_service.srv_name}' is not yet available. Better luck next time!"
            )
            return None

        return self._plan_cartesian_path_service.call_async(
            self.__cartesian_path_request
        )

    def _send_goal_async_move_action(self):
        self.__execution_mutex.acquire()
        stamp = self._node.get_clock().now().to_msg()
        self.__move_action_goal.request.workspace_parameters.header.stamp = stamp
        if not self.__move_action_client.server_is_ready():
            self._node.get_logger().warn(
                f"Action server '{self.__move_action_client._action_name}' is not yet available. Better luck next time!"
            )
            return

        self.__last_error_code = None
        self.__is_motion_requested = True
        self.__send_goal_future_move_action = self.__move_action_client.send_goal_async(
            goal=self.__move_action_goal,
            feedback_callback=None,
        )

        self.__send_goal_future_move_action.add_done_callback(
            self.__response_callback_move_action
        )

        self.__execution_mutex.release()

    def __response_callback_move_action(self, response):
        self.__execution_mutex.acquire()
        goal_handle = response.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"Action '{self.__move_action_client._action_name}' was rejected."
            )
            self.__is_motion_requested = False
            return

        self.__execution_goal_handle = goal_handle
        self.__is_executing = True
        self.__is_motion_requested = False

        self.__get_result_future_move_action = goal_handle.get_result_async()
        self.__get_result_future_move_action.add_done_callback(
            self.__result_callback_move_action
        )
        self.__execution_mutex.release()

    def __result_callback_move_action(self, res):
        self.__execution_mutex.acquire()
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().warn(
                f"Action '{self.__move_action_client._action_name}' was unsuccessful: {res.result().status}."
            )
            self.motion_suceeded = False
        else:
            self.motion_suceeded = True

        self.__last_error_code = res.result().result.error_code

        self.__execution_goal_handle = None
        self.__is_executing = False
        self.__execution_mutex.release()

    def _send_goal_async_execute_trajectory(
        self,
        goal: ExecuteTrajectory,
        wait_until_response: bool = False,
    ):
        self.__execution_mutex.acquire()

        if not self._execute_trajectory_action_client.server_is_ready():
            self._node.get_logger().warn(
                f"Action server '{self._execute_trajectory_action_client._action_name}' is not yet available. Better luck next time!"
            )
            return

        self.__last_error_code = None
        self.__is_motion_requested = True
        self.__send_goal_future_execute_trajectory = (
            self._execute_trajectory_action_client.send_goal_async(
                goal=goal,
                feedback_callback=None,
            )
        )

        self.__send_goal_future_execute_trajectory.add_done_callback(
            self.__response_callback_execute_trajectory
        )
        self.__execution_mutex.release()

    def __response_callback_execute_trajectory(self, response):
        self.__execution_mutex.acquire()
        goal_handle = response.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"Action '{self._execute_trajectory_action_client._action_name}' was rejected."
            )
            self.__is_motion_requested = False
            return

        self.__execution_goal_handle = goal_handle
        self.__is_executing = True
        self.__is_motion_requested = False

        self.__get_result_future_execute_trajectory = goal_handle.get_result_async()
        self.__get_result_future_execute_trajectory.add_done_callback(
            self.__result_callback_execute_trajectory
        )
        self.__execution_mutex.release()

    def __response_callback_with_event_set_execute_trajectory(self, response):
        self.__future_done_event.set()

    def __result_callback_execute_trajectory(self, res):
        self.__execution_mutex.acquire()
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().warn(
                f"Action '{self._execute_trajectory_action_client._action_name}' was unsuccessful: {res.result().status}."
            )
            self.motion_suceeded = False
        else:
            self.motion_suceeded = True

        self.__last_error_code = res.result().result.error_code

        self.__execution_goal_handle = None
        self.__is_executing = False
        self.__execution_mutex.release()

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
        move_action_goal.request.path_constraints = Constraints()
        # move_action_goal.request.trajectory_constraints = "Ignored"
        # move_action_goal.request.reference_trajectories = "Ignored"
        move_action_goal.request.pipeline_id = ""
        move_action_goal.request.planner_id = ""
        move_action_goal.request.group_name = group_name
        move_action_goal.request.num_planning_attempts = 5
        move_action_goal.request.allowed_planning_time = 0.5
        move_action_goal.request.max_velocity_scaling_factor = 0.0
        move_action_goal.request.max_acceleration_scaling_factor = 0.0
        # Note: Attribute was renamed in Iron (https://github.com/ros-planning/moveit_msgs/pull/130)
        if hasattr(move_action_goal.request, "cartesian_speed_limited_link"):
            move_action_goal.request.cartesian_speed_limited_link = end_effector
        else:
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
    def planning_scene(self) -> Optional[PlanningScene]:
        return self.__planning_scene

    @property
    def follow_joint_trajectory_action_client(self) -> str:
        return self.__follow_joint_trajectory_action_client

    @property
    def end_effector_name(self) -> str:
        return self.__end_effector_name

    @property
    def base_link_name(self) -> str:
        return self.__base_link_name

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

    @property
    def cartesian_avoid_collisions(self) -> bool:
        return self.__cartesian_path_request.request.avoid_collisions

    @cartesian_avoid_collisions.setter
    def cartesian_avoid_collisions(self, value: bool):
        self.__cartesian_path_request.avoid_collisions = value

    @property
    def cartesian_jump_threshold(self) -> float:
        return self.__cartesian_path_request.request.jump_threshold

    @cartesian_jump_threshold.setter
    def cartesian_jump_threshold(self, value: float):
        self.__cartesian_path_request.jump_threshold = value

    @property
    def cartesian_prismatic_jump_threshold(self) -> float:
        return self.__cartesian_path_request.request.prismatic_jump_threshold

    @cartesian_prismatic_jump_threshold.setter
    def cartesian_prismatic_jump_threshold(self, value: float):
        self.__cartesian_path_request.prismatic_jump_threshold = value

    @property
    def cartesian_revolute_jump_threshold(self) -> float:
        return self.__cartesian_path_request.request.revolute_jump_threshold

    @cartesian_revolute_jump_threshold.setter
    def cartesian_revolute_jump_threshold(self, value: float):
        self.__cartesian_path_request.revolute_jump_threshold = value

    @property
    def pipeline_id(self) -> int:
        return self.__move_action_goal.request.pipeline_id

    @pipeline_id.setter
    def pipeline_id(self, value: str):
        self.__move_action_goal.request.pipeline_id = value

    @property
    def planner_id(self) -> int:
        return self.__move_action_goal.request.planner_id

    @planner_id.setter
    def planner_id(self, value: str):
        self.__move_action_goal.request.planner_id = value


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


def init_execute_trajectory_goal(
    joint_trajectory: JointTrajectory,
) -> Optional[ExecuteTrajectory.Goal]:
    if joint_trajectory is None:
        return None

    execute_trajectory_goal = ExecuteTrajectory.Goal()

    execute_trajectory_goal.trajectory.joint_trajectory = joint_trajectory

    return execute_trajectory_goal


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
