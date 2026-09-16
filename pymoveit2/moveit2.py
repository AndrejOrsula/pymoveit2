import copy
import threading
import time
from typing import Any, Dict, List, Optional, Tuple, Union

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
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
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2._action_lifecycle import ActionLifecycle, ActionOperation, MoveIt2State
from pymoveit2._diagnostics import describe_error_code, describe_failure
from pymoveit2._mesh import mesh_message
from pymoveit2._planning_scene import SceneClient
from pymoveit2._request_builders import (
    cartesian_request,
    motion_plan_request,
    validate_goal_constraints,
)
from pymoveit2._validation import (
    finite_float,
    finite_vector,
)
from pymoveit2._validation import joint_names as validate_joint_names
from pymoveit2._validation import (
    normalize_joint_state_observation,
    validate_joint_state,
)
from pymoveit2.utils import enum_to_str

__all__ = [
    "MoveIt2",
    "MoveIt2State",
    "init_joint_state",
    "init_execute_trajectory_goal",
    "init_dummy_joint_trajectory_from_state",
]

DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC = 3.0
DEFAULT_JOINT_STATE_TIMEOUT_SEC = 10.0
CANCEL_REISSUE_DELAY_SEC = 0.25


class _Deadline:
    def __init__(self, timeout_sec: Optional[float]):
        self._deadline = (
            None
            if timeout_sec is None
            else time.monotonic() + max(0.0, finite_float(timeout_sec, "timeout_sec"))
        )

    def remaining(self, cap: Optional[float] = None) -> Optional[float]:
        if self._deadline is None:
            return cap
        remaining = max(0.0, self._deadline - time.monotonic())
        if cap is None:
            return remaining
        return min(cap, remaining)

    def expired(self) -> bool:
        return self._deadline is not None and time.monotonic() >= self._deadline


def _reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _best_effort_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class MoveIt2:
    """
    Python interface for MoveIt 2 that enables planning and execution of trajectories.
    For execution, this interface requires that robot utilises JointTrajectoryController.

    Choosing an API:
      1. One-shot convenience — `move_to_pose()` / `move_to_configuration()` build the goal from their arguments, plan (synchronously, unless `use_move_group_action` is enabled) and start the execution asynchronously. Call `wait_until_executed()` afterwards to block until the motion finishes.
      2. Plan, then execute — `plan()` returns the trajectory so that it can be inspected or modified before being passed to `execute()`.
      3. Fully asynchronous — `plan_async()` + `get_trajectory()` to plan without blocking, then `execute()` and `get_execution_future()` to monitor the execution, e.g. for non-blocking pipelines.

    Contracts shared by all methods:
      - The node is never spun internally. An external executor must process callbacks for every blocking method to return.
      - Every goal is tracked as an individual operation. Completion callbacks of an older goal never overwrite the state of a newer one, and cancellation addresses the tracked goal through its goal handle.
      - Goal/constraint setters mutate a shared request template and are not thread-safe with respect to each other. Each submission takes an immutable snapshot of that template, so concurrent modification after submission is harmless.
      - `timeout_sec` arguments of synchronous methods are end-to-end deadlines that cover discovery, joint-state acquisition and the service response. `*_async()` methods may block for at most `wait_for_server_timeout_sec` while waiting for the server to become available.
      - Programming errors (invalid dimensions, missing goals) raise `ValueError`. Runtime failures (missing servers, rejected goals, planner errors) are logged and reported through `None`/`False` return values.
      - All ROS names are relative to the node, so remapping and namespaces apply.
      - Call `destroy()` (or use the instance as a context manager) to release the ROS entities deterministically.
    """

    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link_name: str,
        end_effector_name: str,
        group_name: str = "arm",
        ignore_new_calls_while_executing: bool = False,
        callback_group: Optional[CallbackGroup] = None,
        use_move_group_action: bool = False,
    ):
        """
        Construct an instance of `MoveIt2` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `joint_names` - List of joint names of the robot (can be extracted from URDF)
          - `base_link_name` - Name of the robot base link
          - `end_effector_name` - Name of the robot end effector
          - `group_name` - Name of the planning group for robot arm
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `use_move_group_action` - Flag that enables execution via MoveGroup action (MoveIt 2) ExecuteTrajectory action is employed otherwise together with a separate planning service client
        """

        joint_names = validate_joint_names(joint_names)
        self._node = node
        self._callback_group = callback_group
        self.__closed = False
        self.__resource_mutex = threading.RLock()
        self.__request_mutex = threading.RLock()
        self.__cleanup_mutex = threading.RLock()
        self.__cleanup_pending = None
        self.__joint_state_event = threading.Event()
        self.__cancel_timers: Dict[int, Any] = {}
        self.__pending_reads: Dict[Future, Any] = {}

        try:
            self.__collision_object_publisher = self._node.create_publisher(
                CollisionObject, "collision_object", 10
            )
            self.__attached_collision_object_publisher = self._node.create_publisher(
                AttachedCollisionObject, "attached_collision_object", 10
            )
            self.__trajectory_execution_event_publisher = self._node.create_publisher(
                String, "trajectory_execution_event", 1
            )

            self.__joint_state_mutex = threading.Lock()
            self.__joint_state: Optional[JointState] = None
            self.__joint_state_event = threading.Event()
            self.__new_joint_state_available = False
            self.__move_action_goal = self.__init_move_action_goal(
                frame_id=base_link_name,
                group_name=group_name,
                end_effector=end_effector_name,
            )

            self.__use_move_group_action = use_move_group_action

            self.__joint_names = list(joint_names)
            self.__base_link_name = base_link_name
            self.__end_effector_name = end_effector_name
            self.__group_name = group_name

            self.__lifecycle = ActionLifecycle(
                logger=self._node.get_logger(),
                ignore_new_calls_while_executing=ignore_new_calls_while_executing,
                on_cancel=self.__on_cancel_requested,
            )

            self.__joint_state_subscription = self._node.create_subscription(
                msg_type=JointState,
                topic="joint_states",
                callback=self.__joint_state_callback,
                qos_profile=_best_effort_qos(1),
                callback_group=self._callback_group,
            )

            self.__move_action_client = self.__create_action_client(
                MoveGroup, "move_action"
            )

            self._plan_kinematic_path_service = self._node.create_client(
                srv_type=GetMotionPlan,
                srv_name="plan_kinematic_path",
                qos_profile=_reliable_qos(1),
                callback_group=callback_group,
            )

            self._plan_cartesian_path_service = self._node.create_client(
                srv_type=GetCartesianPath,
                srv_name="compute_cartesian_path",
                qos_profile=_reliable_qos(1),
                callback_group=callback_group,
            )
            self.__cartesian_path_request = GetCartesianPath.Request()
            self.__cartesian_path_request.avoid_collisions = True

            self._execute_trajectory_action_client = self.__create_action_client(
                ExecuteTrajectory, "execute_trajectory"
            )

            self._get_planning_scene_service = self._node.create_client(
                srv_type=GetPlanningScene,
                srv_name="get_planning_scene",
                qos_profile=_reliable_qos(1),
                callback_group=callback_group,
            )
            self._apply_planning_scene_service = self._node.create_client(
                srv_type=ApplyPlanningScene,
                srv_name="apply_planning_scene",
                qos_profile=_reliable_qos(1),
                callback_group=callback_group,
            )

            self.__scene_client = SceneClient(
                node=node,
                callback_group=callback_group,
                get_client=lambda: self._get_planning_scene_service,
                apply_client=lambda: self._apply_planning_scene_service,
            )

            self.__compute_fk_client = None
            self.__compute_ik_client = None

            self.__last_plan_failure: Optional[str] = None
        except Exception:
            self.destroy()
            raise

    def __create_action_client(
        self, action_type: Any, action_name: str
    ) -> ActionClient:
        return ActionClient(
            node=self._node,
            action_type=action_type,
            action_name=action_name,
            goal_service_qos_profile=_reliable_qos(1),
            result_service_qos_profile=_reliable_qos(5),
            cancel_service_qos_profile=_reliable_qos(5),
            feedback_sub_qos_profile=_best_effort_qos(1),
            status_sub_qos_profile=_best_effort_qos(1),
            callback_group=self._callback_group,
        )

    def destroy(self) -> None:
        lifecycle = getattr(self, "_MoveIt2__lifecycle", None)
        if lifecycle is not None:
            lifecycle.close()
        scene_client = getattr(self, "_MoveIt2__scene_client", None)
        if scene_client is not None:
            scene_client.destroy()
        with self.__cleanup_mutex:
            with self.__resource_mutex:
                self.__closed = True
                pending_reads = list(self.__pending_reads.items())
                self.__pending_reads.clear()
                timers = list(self.__cancel_timers.values())
                self.__cancel_timers.clear()
                self.__joint_state_event.set()
                if self.__cleanup_pending is None:
                    self.__cleanup_pending = []
                    for name, cleanup in (
                        (
                            "_MoveIt2__joint_state_subscription",
                            self._node.destroy_subscription,
                        ),
                        (
                            "_MoveIt2__collision_object_publisher",
                            self._node.destroy_publisher,
                        ),
                        (
                            "_MoveIt2__attached_collision_object_publisher",
                            self._node.destroy_publisher,
                        ),
                        (
                            "_MoveIt2__trajectory_execution_event_publisher",
                            self._node.destroy_publisher,
                        ),
                        ("_plan_kinematic_path_service", self._node.destroy_client),
                        ("_plan_cartesian_path_service", self._node.destroy_client),
                        ("_get_planning_scene_service", self._node.destroy_client),
                        ("_apply_planning_scene_service", self._node.destroy_client),
                        ("_MoveIt2__compute_fk_client", self._node.destroy_client),
                        ("_MoveIt2__compute_ik_client", self._node.destroy_client),
                    ):
                        entity = getattr(self, name, None)
                        if entity is not None:
                            self.__cleanup_pending.append((cleanup, (entity,)))
                    for name in (
                        "_MoveIt2__move_action_client",
                        "_execute_trajectory_action_client",
                    ):
                        entity = getattr(self, name, None)
                        if entity is not None:
                            self.__cleanup_pending.append((entity.destroy, ()))
                for timer in timers:
                    self.__cleanup_pending.append((timer.cancel, ()))
                    self.__cleanup_pending.append((self._node.destroy_timer, (timer,)))
            for future, client in pending_reads:
                try:
                    client.remove_pending_request(future)
                except Exception as err:
                    self._node.get_logger().debug(
                        f"Could not detach request on shutdown: {err}"
                    )
                if not future.done():
                    future.cancel()
            remaining = []
            callbacks, self.__cleanup_pending = self.__cleanup_pending, []
            for cleanup, args in callbacks:
                try:
                    cleanup(*args)
                except Exception as err:
                    remaining.append((cleanup, args))
                    self._node.get_logger().warning(
                        f"Resource cleanup failed; destroy() can retry: {err}"
                    )
            self.__cleanup_pending.extend(remaining)

    def __enter__(self) -> "MoveIt2":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.destroy()

    def query_state(self) -> MoveIt2State:
        return self.__lifecycle.query_state()

    def cancel_execution(self) -> bool:
        """
        Cancel the tracked goal.
        """
        return self.__lifecycle.cancel()

    def __on_cancel_requested(self, operation: ActionOperation) -> None:
        with self.__lifecycle.effect_guard(operation) as active:
            if not active:
                return
            with self.__resource_mutex:
                if self.__closed:
                    return
                self.__publish_if_open(
                    self.__trajectory_execution_event_publisher, String(data="stop")
                )
                generation = operation.generation
                if generation in self.__cancel_timers:
                    return

                def reissue() -> None:
                    with self.__lifecycle.effect_guard(operation) as still_active:
                        with self.__resource_mutex:
                            timer = self.__cancel_timers.pop(generation, None)
                            if timer is None:
                                return
                            timer.cancel()
                            self._node.destroy_timer(timer)
                            if still_active and not self.__closed:
                                self.__publish_if_open(
                                    self.__trajectory_execution_event_publisher,
                                    String(data="stop"),
                                )

                try:
                    self.__cancel_timers[generation] = self._node.create_timer(
                        CANCEL_REISSUE_DELAY_SEC,
                        reissue,
                        callback_group=self._callback_group,
                    )
                except (RuntimeError, OSError) as err:
                    self._node.get_logger().debug(
                        f"Could not schedule stop reissue: {err}"
                    )

    def __publish_if_open(self, publisher: Any, message: Any) -> None:
        with self.__resource_mutex:
            if self.__closed:
                self._node.get_logger().warning(
                    "Cannot publish after interface destruction."
                )
                return
            try:
                publisher.publish(message)
            except (RuntimeError, OSError) as err:
                self._node.get_logger().error(f"Publication failed: {err}")

    def stop_all_trajectory_execution(self) -> None:
        self.__publish_if_open(
            self.__trajectory_execution_event_publisher, String(data="stop")
        )

    def get_execution_future(self) -> Optional[Future]:
        return self.__lifecycle.get_result_future()

    def get_last_execution_error_code(self) -> Optional[MoveItErrorCodes]:
        result = self.__lifecycle.last_result
        return getattr(result, "error_code", None)

    def last_failure(self) -> Optional[str]:
        operation = self.__lifecycle.last_operation
        if operation is None or operation.succeeded:
            return None
        return describe_failure(
            status=operation.status,
            result=operation.result,
            reason=operation.reason,
        )

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
        *,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        Plan and execute motion to a Cartesian pose goal.
        """

        deadline = _Deadline(timeout_sec)
        try:
            pose_stamped = self.__to_pose_stamped(pose, position, quat_xyzw, frame_id)

            if self.__use_move_group_action and not cartesian:
                if self.__lifecycle.ignore_new_calls_while_executing and (
                    self.__lifecycle.is_busy()
                ):
                    self._node.get_logger().warning(
                        "Controller is already following a trajectory. Skipping motion."
                    )
                    return False

                goal = self.__snapshot_move_goal()
                constraints = goal.request.goal_constraints[-1]
                constraints.position_constraints.append(
                    self.create_position_constraint(
                        pose_stamped.pose.position,
                        pose_stamped.header.frame_id,
                        target_link,
                        tolerance_position,
                        weight_position,
                    )
                )
                constraints.orientation_constraints.append(
                    self.create_orientation_constraint(
                        pose_stamped.pose.orientation,
                        pose_stamped.header.frame_id,
                        target_link,
                        tolerance_orientation,
                        weight_orientation,
                    )
                )
                return self.__submit_move_action_goal(goal, deadline)

            return self.__execute_before_deadline(
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
                    cartesian_max_step=cartesian_max_step,
                    cartesian_fraction_threshold=cartesian_fraction_threshold,
                    timeout_sec=deadline.remaining(),
                ),
                deadline,
            )
        except ValueError:
            self.__lifecycle.record_failure()
            raise

    def move_to_configuration(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
        *,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """
        Plan and execute motion to a joint configuration goal.
        """

        deadline = _Deadline(timeout_sec)
        try:
            if self.__use_move_group_action:
                if self.__lifecycle.ignore_new_calls_while_executing and (
                    self.__lifecycle.is_busy()
                ):
                    self._node.get_logger().warning(
                        "Controller is already following a trajectory. Skipping motion."
                    )
                    return False

                constraints = self.create_joint_constraints(
                    joint_positions, joint_names, tolerance, weight
                )
                goal = self.__snapshot_move_goal()
                goal.request.goal_constraints[-1].joint_constraints.extend(constraints)
                return self.__submit_move_action_goal(goal, deadline)

            return self.__execute_before_deadline(
                self.plan(
                    joint_positions=joint_positions,
                    joint_names=joint_names,
                    tolerance_joint_position=tolerance,
                    weight_joint_position=weight,
                    timeout_sec=deadline.remaining(),
                ),
                deadline,
            )
        except ValueError:
            self.__lifecycle.record_failure()
            raise

    def __snapshot_move_goal(self) -> MoveGroup.Goal:
        with self.__request_mutex:
            goal = copy.deepcopy(self.__move_action_goal)
            self.clear_goal_constraints()
            self.clear_path_constraints()
            return goal

    def __submit_move_action_goal(
        self, goal: MoveGroup.Goal, deadline: Optional[_Deadline] = None
    ) -> bool:
        joint_state = self.joint_state
        if joint_state is not None:
            goal.request.start_state.joint_state = joint_state
        goal.request.workspace_parameters.header.stamp = (
            self._node.get_clock().now().to_msg()
        )
        if deadline is not None and deadline.expired():
            self.__lifecycle.record_failure()
            return False
        return self.__lifecycle.admit(self.__move_action_client, goal) is not None

    def __execute_before_deadline(
        self, trajectory: Optional[JointTrajectory], deadline: _Deadline
    ) -> bool:
        if deadline.expired():
            self.__lifecycle.record_failure(
                reason="`timeout_sec` expired before execution could start"
            )
            return False
        return self.execute(trajectory)

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
        max_step: Optional[float] = None,
        cartesian_fraction_threshold: float = 0.0,
        timeout_sec: Optional[float] = None,
        cartesian_max_step: Optional[float] = None,
    ) -> Optional[JointTrajectory]:
        """
        Call `plan_async()` and wait for the result.
        """
        deadline = _Deadline(timeout_sec)
        future = self.plan_async(
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            joint_positions=joint_positions,
            joint_names=joint_names,
            frame_id=frame_id,
            target_link=target_link,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
            tolerance_joint_position=tolerance_joint_position,
            weight_position=weight_position,
            weight_orientation=weight_orientation,
            weight_joint_position=weight_joint_position,
            start_joint_state=start_joint_state,
            cartesian=cartesian,
            max_step=max_step,
            cartesian_max_step=cartesian_max_step,
            wait_for_server_timeout_sec=deadline.remaining(
                DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC
            ),
            joint_state_timeout_sec=DEFAULT_JOINT_STATE_TIMEOUT_SEC,
            _deadline=deadline,
        )

        if future is None:
            self.__last_plan_failure = (
                "the planning request was not sent; the planning service of"
                " `move_group` did not become available"
            )
            return None

        if not self._wait_until_future_done(future, timeout_sec=deadline.remaining()):
            self.__last_plan_failure = (
                "planning timed out; raise `timeout_sec`, or"
                " `allowed_planning_time` when the planner itself needs longer"
            )
            self._node.get_logger().warning(
                "Timed out while waiting for the planning future."
            )
            self.__remove_pending_read(
                (
                    self._plan_cartesian_path_service
                    if cartesian
                    else self._plan_kinematic_path_service
                ),
                future,
            )
            return None

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
        cartesian_max_step: Optional[float] = 0.0025,
        wait_for_server_timeout_sec: Optional[
            float
        ] = DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC,
        joint_state_timeout_sec: Optional[float] = DEFAULT_JOINT_STATE_TIMEOUT_SEC,
        *,
        _deadline: Optional[_Deadline] = None,
    ) -> Optional[Future]:
        """
        Plan motion based on previously set goals.
        Cartesian planning requires a pose goal (position and orientation).
        - `cartesian_max_step` - Maximum step between waypoints of Cartesian plans
        """

        if not getattr(self, "_supports_cartesian", True) and (
            cartesian
            or pose is not None
            or position is not None
            or quat_xyzw is not None
        ):
            raise NotImplementedError(
                "This interface supports joint-space planning only."
            )
        if self.__closed:
            self._node.get_logger().warning("Cannot plan after interface destruction.")
            return None
        step = finite_float(cartesian_max_step, "cartesian_max_step", minimum=0.0)
        if cartesian and step == 0.0:
            raise ValueError("`cartesian_max_step` must be positive.")

        request = self.__snapshot_move_goal().request
        constraints = request.goal_constraints[-1]
        if pose is not None:
            stamped = self.__to_pose_stamped(pose, None, None, frame_id)
            position = stamped.pose.position
            quat_xyzw = stamped.pose.orientation
            frame_id = stamped.header.frame_id
        if position is not None:
            constraints.position_constraints.append(
                self.create_position_constraint(
                    position, frame_id, target_link, tolerance_position, weight_position
                )
            )
        if quat_xyzw is not None:
            constraints.orientation_constraints.append(
                self.create_orientation_constraint(
                    quat_xyzw,
                    frame_id,
                    target_link,
                    tolerance_orientation,
                    weight_orientation,
                )
            )
        if joint_positions is not None:
            constraints.joint_constraints.extend(
                self.create_joint_constraints(
                    joint_positions,
                    joint_names,
                    tolerance_joint_position,
                    weight_joint_position,
                )
            )

        validate_goal_constraints(request)

        if start_joint_state is None:
            state_budget = (
                joint_state_timeout_sec
                if _deadline is None
                else _deadline.remaining(joint_state_timeout_sec)
            )
            start_joint_state = self.__wait_for_joint_state(state_budget)
            if start_joint_state is None:
                self._node.get_logger().error(
                    "Cannot plan because no joint states were received within "
                    f"{state_budget} s. Is the node being spun by an "
                    "executor, and is `joint_states` published?"
                )
                return None
        request.start_state.joint_state = self.__validated_start_state(
            start_joint_state
        )
        if _deadline is not None and _deadline.expired():
            self._node.get_logger().warning(
                "Planning deadline expired before submission."
            )
            return None
        server_budget = (
            wait_for_server_timeout_sec
            if _deadline is None
            else _deadline.remaining(wait_for_server_timeout_sec)
        )
        if cartesian:
            return self._plan_cartesian_path(
                request=request,
                max_step=step,
                frame_id=frame_id,
                target_link=target_link,
                wait_for_server_timeout_sec=server_budget,
                _deadline=_deadline,
            )
        return self._plan_kinematic_path(
            request=request,
            wait_for_server_timeout_sec=server_budget,
            _deadline=_deadline,
        )

    def __validated_start_state(
        self, state: Union[JointState, List[float]]
    ) -> JointState:
        if not isinstance(state, JointState):
            state = init_joint_state(self.__joint_names, state)
        validate_joint_state(state, self.__joint_names)
        return copy.deepcopy(state)

    def get_trajectory(
        self,
        future: Future,
        cartesian: bool = False,
        cartesian_fraction_threshold: float = 0.0,
    ) -> Optional[JointTrajectory]:
        """
        Takes in a future returned by plan_async and returns the trajectory if the future is done and planning was successful, else None.
        For cartesian plans, the plan is rejected if the fraction of the path that was completed is less than `cartesian_fraction_threshold`.
        """
        res = self.__future_result(future, "trajectory")
        if res is None:
            self.__last_plan_failure = "the planner returned no response"
            return None

        if cartesian:
            if MoveItErrorCodes.SUCCESS == res.error_code.val:
                if res.fraction >= cartesian_fraction_threshold:
                    self.__last_plan_failure = None
                    return res.solution.joint_trajectory
                else:
                    self.__last_plan_failure = (
                        f"the Cartesian planner reached only {res.fraction} of the"
                        f" path, short of the threshold {cartesian_fraction_threshold};"
                        " lower `cartesian_fraction_threshold`, or move the goal"
                        " closer so a straight path exists"
                    )
                    self._node.get_logger().warning(
                        f"Planning failed! Cartesian planner completed {res.fraction} "
                        f"of the trajectory, less than the threshold {cartesian_fraction_threshold}."
                    )
                    return None
            else:
                self.__last_plan_failure = describe_error_code(res.error_code)
                self._node.get_logger().warning(
                    f"Planning failed! Error code: {enum_to_str(MoveItErrorCodes, res.error_code.val)}"
                )
                return None

        res = res.motion_plan_response
        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            self.__last_plan_failure = None
            return res.trajectory.joint_trajectory
        else:
            self.__last_plan_failure = describe_error_code(res.error_code)
            self._node.get_logger().warning(
                f"Planning failed! Error code: {enum_to_str(MoveItErrorCodes, res.error_code.val)}"
            )
            return None

    def execute(self, joint_trajectory: Optional[JointTrajectory]) -> bool:
        """
        Execute `joint_trajectory` by communicating directly with the controller.
        """

        try:
            execute_trajectory_goal = init_execute_trajectory_goal(
                joint_trajectory=joint_trajectory
            )

            if execute_trajectory_goal is None:
                self._node.get_logger().warning(
                    "Cannot execute motion because the provided/planned trajectory is invalid."
                )
                self.__lifecycle.record_failure(
                    self._execute_trajectory_action_client,
                    reason=self.__last_plan_failure
                    or "there is no trajectory to execute",
                )
                return False

            return self._send_goal_async_execute_trajectory(
                goal=execute_trajectory_goal
            )
        except ValueError:
            self.__lifecycle.record_failure()
            raise

    def wait_until_executed(
        self, timeout_sec: Optional[float] = None, cancel_on_timeout: bool = False
    ) -> bool:
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        If the motion already finished before this call, its outcome is returned once.
        """
        result = self.__lifecycle.wait_until_executed(timeout_sec=timeout_sec)
        if (
            not result
            and cancel_on_timeout
            and self.__lifecycle.query_state() != MoveIt2State.IDLE
        ):
            self.cancel_execution()
        return result

    def _wait_until_future_done(
        self, future: Future, timeout_sec: Optional[float] = None
    ) -> bool:
        if timeout_sec is not None:
            timeout_sec = max(0.0, finite_float(timeout_sec, "timeout_sec"))
        if future.done():
            return True
        if self.__closed:
            return False
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout=timeout_sec)

    def reset_controller(
        self,
        joint_state: Union[JointState, List[float]],
    ) -> bool:
        """
        Reset controller to a given `joint_state` by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        if not isinstance(joint_state, JointState):
            joint_state = init_joint_state(
                joint_names=self.__joint_names,
                joint_positions=list(joint_state),
            )
        validate_joint_state(joint_state, self.__joint_names)
        joint_trajectory = init_dummy_joint_trajectory_from_state(
            copy.deepcopy(joint_state)
        )
        execute_trajectory_goal = init_execute_trajectory_goal(
            joint_trajectory=joint_trajectory
        )

        return self._send_goal_async_execute_trajectory(goal=execute_trajectory_goal)

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
    ) -> None:
        """
        This is direct combination of `set_position_goal()` and `set_orientation_goal()`.
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        pose_stamped = self.__to_pose_stamped(pose, position, quat_xyzw, frame_id)

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

        constraint = PositionConstraint()

        constraint.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        constraint.link_name = (
            target_link if target_link is not None else self.__end_effector_name
        )

        constraint.constraint_region.primitive_poses.append(Pose())
        constraint.constraint_region.primitive_poses[0].position = self.__to_point(
            position
        )

        constraint.constraint_region.primitives.append(SolidPrimitive())
        constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        constraint.constraint_region.primitives[0].dimensions = [
            finite_float(tolerance, "tolerance", minimum=0.0)
        ]

        constraint.weight = finite_float(weight, "weight", minimum=0.0)

        return constraint

    def set_position_goal(
        self,
        position: Union[Point, Tuple[float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> None:
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
        parameterization: int = 0,
    ) -> OrientationConstraint:
        """
        Create a Cartesian orientation constraint of `target_link` with respect to `frame_id`.
          - `frame_id` defaults to the base link
          - `target_link` defaults to end effector
        """

        constraint = OrientationConstraint()

        constraint.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        constraint.link_name = (
            target_link if target_link is not None else self.__end_effector_name
        )

        constraint.orientation = self.__to_quaternion(quat_xyzw)

        if isinstance(tolerance, (int, float)):
            value = finite_float(tolerance, "tolerance", minimum=0.0)
            tolerance_xyz = [value] * 3
        else:
            tolerance_xyz = finite_vector(tolerance, "tolerance", 3)
            if any(value < 0.0 for value in tolerance_xyz):
                raise ValueError("Orientation tolerances must be nonnegative.")
        constraint.absolute_x_axis_tolerance = tolerance_xyz[0]
        constraint.absolute_y_axis_tolerance = tolerance_xyz[1]
        constraint.absolute_z_axis_tolerance = tolerance_xyz[2]

        constraint.parameterization = parameterization

        constraint.weight = finite_float(weight, "weight", minimum=0.0)

        return constraint

    def set_orientation_goal(
        self,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
        target_link: Optional[str] = None,
        tolerance: Union[float, Tuple[float, float, float]] = 0.001,
        weight: float = 1.0,
        parameterization: int = 0,
    ) -> None:
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
        Create joint space constraints.
        """

        constraints = []
        joint_positions = finite_vector(joint_positions, "joint_positions")
        if not joint_positions:
            raise ValueError("`joint_positions` must not be empty.")
        if joint_names is None:
            if len(joint_positions) > len(self.__joint_names):
                raise ValueError("Too many joint positions for configured joints.")
            joint_names = self.__joint_names[: len(joint_positions)]
        else:
            joint_names = validate_joint_names(joint_names)
            if len(joint_positions) != len(joint_names):
                raise ValueError(
                    "Explicit joint names and positions must have equal length."
                )
            if not set(joint_names).issubset(self.__joint_names):
                raise ValueError("Joint goals contain unknown joint names.")
        tolerance = finite_float(tolerance, "tolerance", minimum=0.0)
        weight = finite_float(weight, "weight", minimum=0.0)

        for i in range(len(joint_positions)):
            constraint = JointConstraint()

            constraint.joint_name = joint_names[i]

            constraint.position = float(joint_positions[i])

            constraint.tolerance_above = float(tolerance)
            constraint.tolerance_below = float(tolerance)

            constraint.weight = finite_float(weight, "weight", minimum=0.0)

            constraints.append(constraint)

        return constraints

    def set_joint_goal(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> None:
        """
        Set joint space goal.
        """

        constraints = self.create_joint_constraints(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance=tolerance,
            weight=weight,
        )

        self.__move_action_goal.request.goal_constraints[-1].joint_constraints.extend(
            constraints
        )

    def clear_goal_constraints(self) -> None:
        """
        Clear all goal constraints that were previously set. This function is called automatically after each `plan_async()`.
        """

        self.__move_action_goal.request.goal_constraints = [Constraints()]

    def create_new_goal_constraint(self) -> None:
        """
        Create a new set of goal constraints that will be set together with the request.
        """

        self.__move_action_goal.request.goal_constraints.append(Constraints())

    def set_path_joint_constraint(
        self,
        joint_positions: List[float],
        joint_names: Optional[List[str]] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> None:
        """
        Set joint space path constraints.
        """

        constraints = self.create_joint_constraints(
            joint_positions=joint_positions,
            joint_names=joint_names,
            tolerance=tolerance,
            weight=weight,
        )

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
    ) -> None:
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
        parameterization: int = 0,
    ) -> None:
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

        self.__move_action_goal.request.path_constraints.orientation_constraints.append(
            constraint
        )

    def clear_path_constraints(self) -> None:
        """
        Clear all path constraints that were previously set. This function is called automatically after each `plan_async()`.
        """

        self.__move_action_goal.request.path_constraints = Constraints()

    def compute_fk(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        fk_link_names: Optional[List[str]] = None,
        timeout_sec: Optional[float] = None,
    ) -> Optional[Union[PoseStamped, List[PoseStamped]]]:
        """
        Call `compute_fk_async()` and wait for the result.
        """
        deadline = _Deadline(timeout_sec)
        future = self.compute_fk_async(
            joint_state=joint_state,
            fk_link_names=fk_link_names,
            wait_for_server_timeout_sec=deadline.remaining(
                DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC
            ),
            _deadline=deadline,
        )

        if future is None:
            return None

        if not self._wait_until_future_done(future, timeout_sec=deadline.remaining()):
            self._node.get_logger().warning(
                "Timed out while waiting for the FK future."
            )
            self.__remove_pending_read(self.__compute_fk_client, future)
            return None

        return self.get_compute_fk_result(future, fk_link_names=fk_link_names)

    def get_compute_fk_result(
        self,
        future: Future,
        fk_link_names: Optional[List[str]] = None,
    ) -> Optional[Union[PoseStamped, List[PoseStamped]]]:
        res = self.__future_result(future, "FK result")
        if res is None:
            return None

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            if fk_link_names is None:
                return res.pose_stamped[0] if res.pose_stamped else None
            else:
                return list(res.pose_stamped)
        else:
            self._node.get_logger().warning(
                f"FK computation failed! Error code: {enum_to_str(MoveItErrorCodes, res.error_code.val)}"
            )
            return None

    def compute_fk_async(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        fk_link_names: Optional[List[str]] = None,
        wait_for_server_timeout_sec: Optional[
            float
        ] = DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC,
        *,
        _deadline: Optional[_Deadline] = None,
    ) -> Optional[Future]:
        """
        Compute forward kinematics for all `fk_link_names` in a given `joint_state`.
          - `fk_link_names` defaults to end-effector
          - `joint_state` defaults to the current joint state
        """

        request = GetPositionFK.Request()
        request.header.frame_id = self.__base_link_name
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.fk_link_names = (
            list(fk_link_names)
            if fk_link_names is not None
            else [self.__end_effector_name]
        )
        request.robot_state.is_diff = False

        if joint_state is not None:
            if isinstance(joint_state, JointState):
                request.robot_state.joint_state = self.__validated_start_state(
                    joint_state
                )
            else:
                request.robot_state.joint_state = init_joint_state(
                    joint_names=self.__joint_names,
                    joint_positions=list(joint_state),
                )
        else:
            current_joint_state = self.joint_state
            if current_joint_state is not None:
                request.robot_state.joint_state = current_joint_state

        client = self.__kinematics_client("fk")
        if client is None or not self.__wait_for_service(
            client, wait_for_server_timeout_sec
        ):
            return None
        return self.__call_service_async(client, request, _deadline)

    def compute_ik(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        ik_link_name: Optional[str] = None,
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        constraints: Optional[Constraints] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
        timeout_sec: Optional[float] = None,
    ) -> Optional[JointState]:
        """
        Call `compute_ik_async()` and wait for the result.
        """
        deadline = _Deadline(timeout_sec)
        future = self.compute_ik_async(
            position=position,
            quat_xyzw=quat_xyzw,
            ik_link_name=ik_link_name,
            start_joint_state=start_joint_state,
            constraints=constraints,
            wait_for_server_timeout_sec=deadline.remaining(wait_for_server_timeout_sec),
            _deadline=deadline,
        )

        if future is None:
            return None

        if not self._wait_until_future_done(future, timeout_sec=deadline.remaining()):
            self._node.get_logger().warning(
                "Timed out while waiting for the IK future."
            )
            self.__remove_pending_read(self.__compute_ik_client, future)
            return None

        return self.get_compute_ik_result(future)

    def get_compute_ik_result(
        self,
        future: Future,
    ) -> Optional[JointState]:
        res = self.__future_result(future, "IK result")
        if res is None:
            return None

        if MoveItErrorCodes.SUCCESS == res.error_code.val:
            return res.solution.joint_state
        else:
            self._node.get_logger().warning(
                f"IK computation failed! Error code: {enum_to_str(MoveItErrorCodes, res.error_code.val)}"
            )
            return None

    def compute_ik_async(
        self,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        ik_link_name: Optional[str] = None,
        start_joint_state: Optional[Union[JointState, List[float]]] = None,
        constraints: Optional[Constraints] = None,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
        *,
        _deadline: Optional[_Deadline] = None,
    ) -> Optional[Future]:
        """
        Compute inverse kinematics for the given pose.
          - `ik_link_name` defaults to last link in planning group
          - `start_joint_state` defaults to current joint state
        """

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.__group_name
        request.ik_request.robot_state.is_diff = False
        request.ik_request.avoid_collisions = True
        request.ik_request.pose_stamped.header.frame_id = self.__base_link_name
        request.ik_request.pose_stamped.header.stamp = (
            self._node.get_clock().now().to_msg()
        )
        request.ik_request.pose_stamped.pose.position = self.__to_point(position)
        request.ik_request.pose_stamped.pose.orientation = self.__to_quaternion(
            quat_xyzw
        )

        if ik_link_name is not None:
            request.ik_request.ik_link_name = ik_link_name

        if start_joint_state is not None:
            if isinstance(start_joint_state, JointState):
                request.ik_request.robot_state.joint_state = (
                    self.__validated_start_state(start_joint_state)
                )
            else:
                request.ik_request.robot_state.joint_state = init_joint_state(
                    joint_names=self.__joint_names,
                    joint_positions=list(start_joint_state),
                )
        else:
            current_joint_state = self.joint_state
            if current_joint_state is not None:
                request.ik_request.robot_state.joint_state = current_joint_state

        if constraints is not None:
            request.ik_request.constraints = copy.deepcopy(constraints)

        client = self.__kinematics_client("ik")
        if client is None or not self.__wait_for_service(
            client, wait_for_server_timeout_sec
        ):
            return None
        return self.__call_service_async(client, request, _deadline)

    def reset_new_joint_state_checker(self) -> None:
        with self.__joint_state_mutex:
            self.__new_joint_state_available = False

    def wait_for_joint_state(self, timeout_sec: Optional[float] = None) -> bool:
        return self.__wait_for_joint_state(timeout_sec) is not None

    def force_reset_executing_state(self) -> None:
        self.__lifecycle.force_reset()

    def add_collision_primitive(
        self,
        id: str,
        primitive_type: int,
        dimensions: Tuple[float, ...],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ) -> None:
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

        arity = {
            SolidPrimitive.BOX: 3,
            SolidPrimitive.SPHERE: 1,
            SolidPrimitive.CYLINDER: 2,
            SolidPrimitive.CONE: 2,
        }
        if (
            not isinstance(primitive_type, int)
            or isinstance(primitive_type, bool)
            or primitive_type not in arity
        ):
            raise ValueError(
                "Unsupported primitive_type; expected box, sphere, cylinder or cone."
            )
        dimensions = finite_vector(dimensions, "dimensions", arity[primitive_type])
        if any(value <= 0.0 for value in dimensions):
            raise ValueError("Primitive dimensions must be positive.")
        pose_stamped = self.__to_pose_stamped(pose, position, quat_xyzw, frame_id)

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.primitives.append(
            SolidPrimitive(
                type=primitive_type, dimensions=[float(d) for d in dimensions]
            )
        )

        self.__publish_if_open(self.__collision_object_publisher, msg)

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
    ) -> None:
        """
        Add collision object with a box geometry specified by its size.
        """

        size = finite_vector(size, "size", 3)

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
    ) -> None:
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
    ) -> None:
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
    ) -> None:
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
        max_file_bytes: Optional[int] = None,
        max_vertices: Optional[int] = None,
        max_faces: Optional[int] = None,
    ) -> None:
        """
        Add collision object with a mesh geometry. Either `filepath` must be specified or `mesh` (a `trimesh.Trimesh` or `trimesh.Scene`) must be provided.
        """

        try:
            import trimesh
        except ImportError as err:
            raise ImportError(
                "Python module 'trimesh' is not installed; run "
                "`pip install trimesh` to add mesh collision objects to the "
                "MoveIt 2 planning scene."
            ) from err

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )
        if (filepath is None and mesh is None) or (
            filepath is not None and mesh is not None
        ):
            raise ValueError("Exactly one of `filepath` or `mesh` must be specified!")
        if mesh is not None and not isinstance(mesh, (trimesh.Trimesh, trimesh.Scene)):
            raise ValueError(
                "`mesh` must be an instance of `trimesh.Trimesh` or `trimesh.Scene`!"
            )

        pose_stamped = self.__to_pose_stamped(pose, position, quat_xyzw, frame_id)

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.meshes.append(
            mesh_message(
                trimesh, filepath, mesh, scale, max_file_bytes, max_vertices, max_faces
            )
        )

        self.__publish_if_open(self.__collision_object_publisher, msg)

    def remove_collision_object(self, id: str) -> None:
        """
        Remove collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.id = id
        msg.operation = CollisionObject.REMOVE
        msg.header.stamp = self._node.get_clock().now().to_msg()
        self.__publish_if_open(self.__collision_object_publisher, msg)

    def attach_collision_object(
        self,
        id: str,
        link_name: Optional[str] = None,
        touch_links: Optional[List[str]] = None,
        weight: float = 0.0,
    ) -> None:
        """
        Attach collision object to the robot.
        """

        touch_links = list(touch_links) if touch_links is not None else []

        if link_name is None:
            link_name = self.__end_effector_name

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.ADD)
        )
        msg.link_name = link_name
        msg.touch_links = touch_links
        msg.weight = float(weight)

        self.__publish_if_open(self.__attached_collision_object_publisher, msg)

    def detach_collision_object(self, id: str) -> None:
        """
        Detach collision object from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(id=id, operation=CollisionObject.REMOVE)
        )
        self.__publish_if_open(self.__attached_collision_object_publisher, msg)

    def detach_all_collision_objects(self) -> None:
        """
        Detach all collision objects from the robot.
        """

        msg = AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE)
        )
        self.__publish_if_open(self.__attached_collision_object_publisher, msg)

    def move_collision(
        self,
        id: str,
        position: Union[Point, Tuple[float, float, float]],
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        frame_id: Optional[str] = None,
    ) -> None:
        """
        Move collision object specified by its `id`.
        """

        msg = CollisionObject()
        msg.pose = Pose(
            position=self.__to_point(position),
            orientation=self.__to_quaternion(quat_xyzw),
        )
        msg.id = id
        msg.operation = CollisionObject.MOVE
        msg.header.frame_id = (
            frame_id if frame_id is not None else self.__base_link_name
        )
        msg.header.stamp = self._node.get_clock().now().to_msg()

        self.__publish_if_open(self.__collision_object_publisher, msg)

    def update_planning_scene(self, timeout_sec: Optional[float] = 1.0) -> bool:
        return self.__scene_client.update_planning_scene(timeout_sec)

    def allow_collisions(
        self, id: str, allow: bool, timeout_sec: Optional[float] = 1.0
    ) -> Optional[Future]:
        """
        Set object collision permission.
        """
        if not isinstance(id, str) or not id:
            raise ValueError("Collision object id must be a nonempty string.")
        if not isinstance(allow, bool):
            raise ValueError("`allow` must be a bool.")
        return self.__scene_client.allow_collisions(id, allow, timeout_sec)

    def process_allow_collision_future(self, future: Future) -> bool:
        return self.__scene_client.process_allow_collision_future(future)

    def clear_all_collision_objects(
        self, timeout_sec: Optional[float] = 1.0
    ) -> Optional[Future]:
        return self.__scene_client.clear_all_collision_objects(timeout_sec)

    def cancel_clear_all_collision_objects_future(self, future: Future) -> None:
        self.__scene_client.cancel_clear_all_collision_objects_future(future)

    def process_clear_all_collision_objects_future(self, future: Future) -> bool:
        return self.__scene_client.process_clear_all_collision_objects_future(future)

    def __joint_state_callback(self, msg: JointState) -> None:
        try:
            observation = normalize_joint_state_observation(msg, self.__joint_names)
        except ValueError:
            return
        with self.__joint_state_mutex:
            if self.__closed:
                return
            self.__joint_state = observation
            self.__new_joint_state_available = True
        self.__joint_state_event.set()

    def __wait_for_joint_state(
        self, timeout_sec: Optional[float]
    ) -> Optional[JointState]:
        if timeout_sec is not None:
            timeout_sec = max(0.0, finite_float(timeout_sec, "timeout_sec"))
        if self.__closed:
            return None
        joint_state = self.joint_state
        if joint_state is not None:
            return joint_state
        self._node.get_logger().warning(
            "Joint states are not available yet — waiting..."
        )
        self.__joint_state_event.wait(timeout=timeout_sec)
        return self.joint_state

    def __wait_for_service(self, client: Any, timeout_sec: Optional[float]) -> bool:
        if timeout_sec is not None:
            timeout_sec = max(0.0, finite_float(timeout_sec, "timeout_sec"))
        if self.__closed:
            return False
        try:
            if timeout_sec is None or timeout_sec > 0.0:
                client.wait_for_service(timeout_sec=timeout_sec)
            ready = client.service_is_ready()
        except (RuntimeError, OSError) as err:
            self._node.get_logger().error(f"Service discovery failed: {err}")
            return False
        if not ready:
            self._node.get_logger().warning(
                f"Service '{client.srv_name}' is not yet available. Better luck next time!"
            )
        return ready and not self.__closed

    def __call_service_async(
        self, client: Any, request: Any, deadline: Optional[_Deadline] = None
    ) -> Optional[Future]:
        with self.__resource_mutex:
            if self.__closed or (deadline is not None and deadline.expired()):
                return None
            try:
                future = client.call_async(request)
                self.__pending_reads[future] = client
            except (RuntimeError, OSError) as err:
                self._node.get_logger().error(f"Service submission failed: {err}")
                return None
        future.add_done_callback(self.__read_finished)
        return future

    def __read_finished(self, future: Future) -> None:
        with self.__resource_mutex:
            self.__pending_reads.pop(future, None)

    def __remove_pending_read(self, client: Any, future: Future) -> None:
        with self.__resource_mutex:
            client = self.__pending_reads.pop(future, client)
            if self.__closed or client is None:
                return
            try:
                client.remove_pending_request(future)
            except (RuntimeError, OSError) as err:
                self._node.get_logger().debug(
                    f"Could not detach timed-out request: {err}"
                )
        if not future.done():
            future.cancel()

    def __future_result(self, future: Future, what: str) -> Any:
        if not future.done():
            self._node.get_logger().warning(
                f"Cannot get {what} because future is not done."
            )
            return None
        if future.cancelled():
            self._node.get_logger().warning(
                f"Cannot get {what} because the request was cancelled."
            )
            return None
        try:
            result = future.result()
        except Exception as err:
            self._node.get_logger().error(
                f"Cannot get {what} because the request raised "
                f"{type(err).__name__}: {err}"
            )
            return None
        if result is None:
            self._node.get_logger().warning(
                f"Cannot get {what} because the service returned no response."
            )
        return result

    def _plan_kinematic_path(
        self,
        request: Optional[MotionPlanRequest] = None,
        wait_for_server_timeout_sec: Optional[
            float
        ] = DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC,
        _deadline: Optional[_Deadline] = None,
    ) -> Optional[Future]:
        if request is None:
            with self.__request_mutex:
                request = copy.deepcopy(self.__move_action_goal.request)
        service_request = motion_plan_request(
            request, self._node.get_clock().now().to_msg()
        )

        if not self.__wait_for_service(
            self._plan_kinematic_path_service, wait_for_server_timeout_sec
        ):
            return None

        return self.__call_service_async(
            self._plan_kinematic_path_service, service_request, _deadline
        )

    def _plan_cartesian_path(
        self,
        max_step: float = 0.0025,
        frame_id: Optional[str] = None,
        request: Optional[MotionPlanRequest] = None,
        target_link: Optional[str] = None,
        wait_for_server_timeout_sec: Optional[
            float
        ] = DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC,
        _deadline: Optional[_Deadline] = None,
    ) -> Optional[Future]:
        if request is None:
            with self.__request_mutex:
                request = copy.deepcopy(self.__move_action_goal.request)
        with self.__request_mutex:
            settings = copy.deepcopy(self.__cartesian_path_request)
        service_request = cartesian_request(
            request,
            settings,
            max_step,
            frame_id,
            target_link,
            self.__base_link_name,
            self.__end_effector_name,
            self._node.get_clock().now().to_msg(),
        )
        if not self.__wait_for_service(
            self._plan_cartesian_path_service, wait_for_server_timeout_sec
        ):
            return None

        return self.__call_service_async(
            self._plan_cartesian_path_service, service_request, _deadline
        )

    def _send_goal_async_move_action(self) -> bool:
        goal = copy.deepcopy(self.__move_action_goal)
        goal.request.workspace_parameters.header.stamp = (
            self._node.get_clock().now().to_msg()
        )
        return self.__lifecycle.admit(self.__move_action_client, goal) is not None

    def _send_goal_async_execute_trajectory(
        self,
        goal: ExecuteTrajectory.Goal,
    ) -> bool:
        return (
            self.__lifecycle.admit(self._execute_trajectory_action_client, goal)
            is not None
        )

    @property
    def _execution_lifecycle(self) -> ActionLifecycle:
        return self.__lifecycle

    @property
    def current_operation(self) -> Optional[ActionOperation]:
        return self.__lifecycle.current

    @classmethod
    def __init_move_action_goal(
        cls, frame_id: str, group_name: str, end_effector: str
    ) -> MoveGroup.Goal:
        move_action_goal = MoveGroup.Goal()
        move_action_goal.request.workspace_parameters.header.frame_id = frame_id
        move_action_goal.request.workspace_parameters.min_corner.x = -1.0
        move_action_goal.request.workspace_parameters.min_corner.y = -1.0
        move_action_goal.request.workspace_parameters.min_corner.z = -1.0
        move_action_goal.request.workspace_parameters.max_corner.x = 1.0
        move_action_goal.request.workspace_parameters.max_corner.y = 1.0
        move_action_goal.request.workspace_parameters.max_corner.z = 1.0
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

    def __kinematics_client(self, kind: str) -> Any:
        with self.__resource_mutex:
            if self.__closed:
                return None
            name = f"_MoveIt2__compute_{kind}_client"
            client = getattr(self, name)
            if client is None:
                try:
                    client = self._node.create_client(
                        srv_type=GetPositionFK if kind == "fk" else GetPositionIK,
                        srv_name=f"compute_{kind}",
                        callback_group=self._callback_group,
                    )
                except (RuntimeError, OSError) as err:
                    self._node.get_logger().error(f"Cannot create {kind} client: {err}")
                    return None
                setattr(self, name, client)
            return client

    def __to_point(self, position: Union[Point, Tuple[float, float, float]]) -> Point:
        values = (
            (position.x, position.y, position.z)
            if isinstance(position, Point)
            else position
        )
        x, y, z = finite_vector(values, "position", 3)
        return Point(x=x, y=y, z=z)

    def __to_quaternion(
        self, quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]]
    ) -> Quaternion:
        values = (
            (quat_xyzw.x, quat_xyzw.y, quat_xyzw.z, quat_xyzw.w)
            if isinstance(quat_xyzw, Quaternion)
            else quat_xyzw
        )
        x, y, z, w = finite_vector(values, "quat_xyzw", 4)
        return Quaternion(x=x, y=y, z=z, w=w)

    def __to_pose_stamped(
        self,
        pose: Optional[Union[PoseStamped, Pose]],
        position: Optional[Union[Point, Tuple[float, float, float]]],
        quat_xyzw: Optional[Union[Quaternion, Tuple[float, float, float, float]]],
        frame_id: Optional[str],
    ) -> PoseStamped:
        if isinstance(pose, PoseStamped):
            if (
                frame_id is not None
                and pose.header.frame_id
                and frame_id != pose.header.frame_id
            ):
                raise ValueError("Explicit frame_id conflicts with PoseStamped frame.")
            result = copy.deepcopy(pose)
            result.header.frame_id = (
                pose.header.frame_id or frame_id or self.__base_link_name
            )
            result.pose.position = self.__to_point(pose.pose.position)
            result.pose.orientation = self.__to_quaternion(pose.pose.orientation)
            return result
        header = Header(
            stamp=self._node.get_clock().now().to_msg(),
            frame_id=frame_id if frame_id is not None else self.__base_link_name,
        )
        if isinstance(pose, Pose):
            return PoseStamped(
                header=header,
                pose=Pose(
                    position=self.__to_point(pose.position),
                    orientation=self.__to_quaternion(pose.orientation),
                ),
            )
        if pose is not None:
            raise ValueError("`pose` must be a `Pose` or `PoseStamped`!")
        if position is None or quat_xyzw is None:
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )
        return PoseStamped(
            header=header,
            pose=Pose(
                position=self.__to_point(position),
                orientation=self.__to_quaternion(quat_xyzw),
            ),
        )

    @property
    def planning_scene(self) -> Optional[PlanningScene]:
        return self.__scene_client.planning_scene

    @property
    def planning_scene_cache_dirty(self) -> bool:
        return self.__scene_client.scene_cache_dirty

    @property
    def planning_scene_mutation_quarantined(self) -> bool:
        return self.__scene_client.mutation_quarantined

    @property
    def motion_succeeded(self) -> bool:
        return self.__lifecycle.succeeded

    @motion_succeeded.setter
    def motion_succeeded(self, value: bool) -> None:
        self.__lifecycle.succeeded = bool(value)

    @property
    def end_effector_name(self) -> str:
        return self.__end_effector_name

    @property
    def base_link_name(self) -> str:
        return self.__base_link_name

    @property
    def group_name(self) -> str:
        return self.__group_name

    @property
    def joint_names(self) -> List[str]:
        return list(self.__joint_names)

    @property
    def joint_state(self) -> Optional[JointState]:
        with self.__joint_state_mutex:
            return copy.deepcopy(self.__joint_state)

    @property
    def new_joint_state_available(self) -> bool:
        with self.__joint_state_mutex:
            return self.__new_joint_state_available

    @property
    def max_velocity(self) -> float:
        return self.__move_action_goal.request.max_velocity_scaling_factor

    @max_velocity.setter
    def max_velocity(self, value: float) -> None:
        self.__move_action_goal.request.max_velocity_scaling_factor = float(value)

    @property
    def max_acceleration(self) -> float:
        return self.__move_action_goal.request.max_acceleration_scaling_factor

    @max_acceleration.setter
    def max_acceleration(self, value: float) -> None:
        self.__move_action_goal.request.max_acceleration_scaling_factor = float(value)

    @property
    def num_planning_attempts(self) -> int:
        return self.__move_action_goal.request.num_planning_attempts

    @num_planning_attempts.setter
    def num_planning_attempts(self, value: int) -> None:
        self.__move_action_goal.request.num_planning_attempts = int(value)

    @property
    def allowed_planning_time(self) -> float:
        return self.__move_action_goal.request.allowed_planning_time

    @allowed_planning_time.setter
    def allowed_planning_time(self, value: float) -> None:
        self.__move_action_goal.request.allowed_planning_time = float(value)

    @property
    def cartesian_avoid_collisions(self) -> bool:
        return self.__cartesian_path_request.avoid_collisions

    @cartesian_avoid_collisions.setter
    def cartesian_avoid_collisions(self, value: bool) -> None:
        self.__cartesian_path_request.avoid_collisions = bool(value)

    @property
    def cartesian_jump_threshold(self) -> float:
        return self.__cartesian_path_request.jump_threshold

    @cartesian_jump_threshold.setter
    def cartesian_jump_threshold(self, value: float) -> None:
        self.__cartesian_path_request.jump_threshold = float(value)

    @property
    def cartesian_prismatic_jump_threshold(self) -> float:
        return self.__cartesian_path_request.prismatic_jump_threshold

    @cartesian_prismatic_jump_threshold.setter
    def cartesian_prismatic_jump_threshold(self, value: float) -> None:
        self.__cartesian_path_request.prismatic_jump_threshold = float(value)

    @property
    def cartesian_revolute_jump_threshold(self) -> float:
        return self.__cartesian_path_request.revolute_jump_threshold

    @cartesian_revolute_jump_threshold.setter
    def cartesian_revolute_jump_threshold(self, value: float) -> None:
        self.__cartesian_path_request.revolute_jump_threshold = float(value)

    @property
    def pipeline_id(self) -> str:
        return self.__move_action_goal.request.pipeline_id

    @pipeline_id.setter
    def pipeline_id(self, value: str) -> None:
        self.__move_action_goal.request.pipeline_id = value

    @property
    def planner_id(self) -> str:
        return self.__move_action_goal.request.planner_id

    @planner_id.setter
    def planner_id(self, value: str) -> None:
        self.__move_action_goal.request.planner_id = value

    @property
    def workspace_frame_id(self) -> str:
        return self.__move_action_goal.request.workspace_parameters.header.frame_id

    def set_workspace_parameters(
        self,
        min_corner: Tuple[float, float, float],
        max_corner: Tuple[float, float, float],
        frame_id: Optional[str] = None,
    ) -> None:
        min_corner = tuple(float(v) for v in min_corner)
        max_corner = tuple(float(v) for v in max_corner)
        if len(min_corner) != 3 or len(max_corner) != 3:
            raise ValueError("Workspace corners must contain exactly three values!")
        ws = self.__move_action_goal.request.workspace_parameters
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = min_corner
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = max_corner
        ws.header.frame_id = frame_id if frame_id is not None else self.__base_link_name


def init_joint_state(
    joint_names: List[str],
    joint_positions: Optional[List[float]] = None,
    joint_velocities: Optional[List[float]] = None,
    joint_effort: Optional[List[float]] = None,
) -> JointState:
    joint_names = validate_joint_names(joint_names)
    joint_state = JointState()

    joint_state.name = list(joint_names)
    joint_state.position = (
        finite_vector(joint_positions, "joint_positions", len(joint_names))
        if joint_positions is not None
        else [0.0] * len(joint_names)
    )
    joint_state.velocity = (
        finite_vector(joint_velocities, "joint_velocities")
        if joint_velocities is not None
        else [0.0] * len(joint_names)
    )
    joint_state.effort = (
        finite_vector(joint_effort, "joint_effort")
        if joint_effort is not None
        else [0.0] * len(joint_names)
    )

    validate_joint_state(joint_state)
    return joint_state


def init_execute_trajectory_goal(
    joint_trajectory: Optional[JointTrajectory],
) -> Optional[ExecuteTrajectory.Goal]:
    if joint_trajectory is None:
        return None
    if not isinstance(joint_trajectory, JointTrajectory):
        raise ValueError("`joint_trajectory` must be a JointTrajectory or None.")

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
