import copy
import threading
from typing import Any, List, Optional, Union

from control_msgs.action import GripperCommand as GripperCommandAction
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

from pymoveit2._action_lifecycle import ActionLifecycle, ActionOperation, MoveIt2State
from pymoveit2._diagnostics import describe_failure
from pymoveit2._joint_helpers import (
    classify_gripper_state,
    copy_joint_state,
    finite_float,
    is_uniform,
    joint_state_indices,
    normalize_joint_positions,
    validate_joint_names,
    validate_joint_state,
)
from pymoveit2._validation import normalize_joint_state_observation


class GripperCommand:
    """
    Python interface for Gripper that is controlled by GripperCommand.
    """

    def __init__(
        self,
        node: Node,
        gripper_joint_names: List[str],
        open_gripper_joint_positions: Union[float, List[float]],
        closed_gripper_joint_positions: Union[float, List[float]],
        max_effort: float = 0.0,
        ignore_new_calls_while_executing: bool = True,
        callback_group: Optional[CallbackGroup] = None,
        gripper_command_action_name: str = "gripper_action_controller/gripper_cmd",
    ):
        """
        Construct an instance of `GripperCommand` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `gripper_joint_names` - List of gripper joint names (can be extracted from URDF)
          - `open_gripper_joint_positions` - Configuration of gripper joints when open (a scalar applies to every joint)
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed (a scalar applies to every joint)
          - `max_effort` - Max effort applied when closing
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `gripper_command_action_name` - Name of the action server for the controller
        """

        self._node = node
        self._callback_group = callback_group
        self.__closed = False

        gripper_joint_names = validate_joint_names(gripper_joint_names)
        open_positions = normalize_joint_positions(
            open_gripper_joint_positions,
            len(gripper_joint_names),
            "open_gripper_joint_positions",
        )
        closed_positions = normalize_joint_positions(
            closed_gripper_joint_positions,
            len(gripper_joint_names),
            "closed_gripper_joint_positions",
        )
        max_effort = finite_float(max_effort, "max_effort")
        if not is_uniform(open_positions) or not is_uniform(closed_positions):
            raise ValueError(
                "`GripperCommand` controls one scalar actuator; open and closed "
                "positions must each be uniform across the configured joints. "
                "Use `MoveIt2Gripper` for nonuniform joint goals."
            )

        self.__joint_names = list(gripper_joint_names)
        self.__open_gripper_joint_positions = list(open_positions)
        self.__closed_gripper_joint_positions = list(closed_positions)
        self.__max_effort = max_effort
        self.__open_gripper_command_goal = self.__init_gripper_command_goal(
            position=open_positions[0], max_effort=self.__max_effort
        )
        self.__close_gripper_command_goal = self.__init_gripper_command_goal(
            position=closed_positions[0], max_effort=self.__max_effort
        )
        self.__joint_state_mutex = threading.Lock()
        self.__joint_state: Optional[JointState] = None
        self.__new_joint_state_available = False
        self.__open_tolerance = [
            0.1 * abs(open_positions[i] - closed_positions[i])
            for i in range(len(gripper_joint_names))
        ]
        self.__gripper_joint_indices: Optional[List[int]] = None
        self.__gripper_joint_index_names: Optional[List[str]] = None
        self.__lifecycle = ActionLifecycle(
            logger=self._node.get_logger(),
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            result_success=self.__reached_goal,
        )
        self.__joint_state_subscription = None
        self.__gripper_command_action_client = None
        self.__joint_state_subscription_destroyed = False
        self.__gripper_command_action_client_destroyed = False

        subscription = None
        action_client = None
        try:
            subscription = self._node.create_subscription(
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
            action_client = ActionClient(
                node=self._node,
                action_type=GripperCommandAction,
                action_name=gripper_command_action_name,
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
        except Exception:
            if action_client is not None:
                try:
                    action_client.destroy()
                except Exception:
                    pass
            if subscription is not None:
                try:
                    self._node.destroy_subscription(subscription)
                except Exception:
                    pass
            raise
        self.__joint_state_subscription = subscription
        self.__gripper_command_action_client = action_client

    def destroy(self) -> None:
        self.__closed = True
        errors = []
        try:
            self.__lifecycle.force_reset()
        except Exception as error:
            errors.append(error)

        subscription = self.__joint_state_subscription
        if subscription is not None and not self.__joint_state_subscription_destroyed:
            try:
                self._node.destroy_subscription(subscription)
            except Exception as error:
                errors.append(error)
            else:
                self.__joint_state_subscription_destroyed = True

        action_client = self.__gripper_command_action_client
        if (
            action_client is not None
            and not self.__gripper_command_action_client_destroyed
        ):
            try:
                action_client.destroy()
            except Exception as error:
                errors.append(error)
            else:
                self.__gripper_command_action_client_destroyed = True

        if errors:
            raise errors[0]

    def __enter__(self) -> "GripperCommand":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.destroy()

    def __call__(self) -> bool:
        """
        Callable that is identical to `GripperCommand.toggle()`.
        """

        return self.toggle()

    def toggle(self) -> bool:
        """
        Toggles the gripper between open and closed state.
        """

        if self.is_open:
            return self.close(skip_if_noop=False)
        if self.is_closed:
            return self.open(skip_if_noop=False)
        self._node.get_logger().warning(
            "Cannot toggle the gripper until a complete joint-state observation "
            "establishes whether it is open or closed."
        )
        return False

    def open(self, skip_if_noop: bool = False) -> bool:
        """
        Open the gripper. Returns whether a goal was submitted.
        """

        if skip_if_noop and self.is_open:
            return False

        return self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

    def close(self, skip_if_noop: bool = False) -> bool:
        """
        Close the gripper. Returns whether a goal was submitted.
        """

        if skip_if_noop and self.is_closed:
            return False

        return self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

    def move_to_position(self, position: float) -> bool:
        """
        Move the gripper to a specific position. Returns whether a goal was submitted.
        """

        gripper_cmd_goal = self.__init_gripper_command_goal(
            position=finite_float(position, "position"), max_effort=self.__max_effort
        )
        return self.__send_goal_async_gripper_command(gripper_cmd_goal)

    def reset_open(self, sync: bool = True) -> bool:
        """
        Reset into open configuration using the normal admission policy.
        """
        return self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

    def reset_closed(self, sync: bool = True) -> bool:
        """
        Reset into closed configuration using the normal admission policy.
        """
        return self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

    def force_reset_executing_state(self) -> None:
        self.__lifecycle.force_reset()

    def wait_until_executed(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        """

        return self.__lifecycle.wait_until_executed(
            timeout_sec=timeout_sec, what="gripper motion"
        )

    def query_state(self) -> MoveIt2State:
        return self.__lifecycle.query_state()

    def cancel_execution(self) -> bool:
        """
        Cancel the tracked gripper goal through its goal handle.
        """
        return self.__lifecycle.cancel()

    def get_execution_future(self) -> Optional[Future]:
        return self.__lifecycle.get_result_future()

    def get_last_execution_error_code(self) -> Optional[Any]:
        return None

    def last_failure(self) -> Optional[str]:
        operation = self.__lifecycle.last_operation
        if operation is None or operation.succeeded:
            return None
        short = self.__describe_short_travel(operation.result)
        if short is not None:
            return short
        return describe_failure(
            status=operation.status,
            result=operation.result,
            reason=operation.reason,
        )

    @staticmethod
    def __describe_short_travel(result: Any) -> Optional[str]:
        if result is None or getattr(result, "reached_goal", True):
            return None
        position = getattr(result, "position", None)
        stalled = bool(getattr(result, "stalled", False))
        where = "" if position is None else f" at {position:.4f}"
        why = (
            " It stalled, so something is in the way:"
            " check that the object fits between the fingers,"
            " and raise `max_effort` when it needs a firmer hold."
            if stalled
            else " Check the commanded position against the travel of the gripper."
        )
        return f"The gripper stopped{where} without reaching its goal.{why}"

    def __joint_state_callback(self, msg: JointState) -> None:
        try:
            normalized = normalize_joint_state_observation(msg, self.__joint_names)
        except ValueError:
            self._node.get_logger().warning(
                "Ignoring malformed or incomplete gripper joint state."
            )
            return
        with self.__joint_state_mutex:
            if self.__closed:
                return
            self.__joint_state = normalized
            self.__new_joint_state_available = True

    def reset_new_joint_state_checker(self) -> None:
        with self.__joint_state_mutex:
            self.__new_joint_state_available = False

    def __send_goal_async_gripper_command(
        self,
        goal: GripperCommandAction.Goal,
    ) -> bool:
        if self.__closed:
            self._node.get_logger().warning(
                "Cannot send a gripper command after the interface was destroyed."
            )
            return False
        goal = copy.deepcopy(goal)
        return (
            self.__lifecycle.admit(self.__gripper_command_action_client, goal)
            is not None
        )

    @classmethod
    def __init_gripper_command_goal(
        cls, position: float, max_effort: float
    ) -> GripperCommandAction.Goal:
        gripper_cmd_goal = GripperCommandAction.Goal()
        gripper_cmd_goal.command.position = float(position)
        gripper_cmd_goal.command.max_effort = float(max_effort)

        return gripper_cmd_goal

    @property
    def _execution_lifecycle(self) -> ActionLifecycle:
        return self.__lifecycle

    @property
    def current_operation(self) -> Optional[ActionOperation]:
        return self.__lifecycle.current

    @property
    def motion_succeeded(self) -> bool:
        if not self.__lifecycle.succeeded:
            return False
        result = self.__lifecycle.last_result
        return result is None or self.__reached_goal(result)

    @motion_succeeded.setter
    def motion_succeeded(self, value: bool) -> None:
        self.__lifecycle.succeeded = bool(value)

    @property
    def gripper_command_action_client(self) -> ActionClient:
        return self.__gripper_command_action_client

    @property
    def joint_names(self) -> List[str]:
        return list(self.__joint_names)

    @property
    def open_gripper_joint_positions(self) -> List[float]:
        return list(self.__open_gripper_joint_positions)

    @property
    def closed_gripper_joint_positions(self) -> List[float]:
        return list(self.__closed_gripper_joint_positions)

    @property
    def joint_state(self) -> Optional[JointState]:
        with self.__joint_state_mutex:
            return copy_joint_state(self.__joint_state)

    @property
    def new_joint_state_available(self) -> bool:
        with self.__joint_state_mutex:
            return self.__new_joint_state_available

    @property
    def is_open(self) -> bool:
        """
        Gripper is considered to be open if all of the joints are at their open position.
        """

        return self.__observation_state() is True

    @property
    def is_closed(self) -> bool:
        """
        Gripper is considered closed only when every joint matches its closed target.
        """

        return self.__observation_state() is False

    def __observation_state(self) -> Optional[bool]:
        joint_state = self.joint_state
        if joint_state is None:
            return None
        try:
            validate_joint_state(joint_state, self.__joint_names)
            names = list(joint_state.name)
            if self.__gripper_joint_indices is None or (
                self.__gripper_joint_index_names != names
            ):
                self.__gripper_joint_indices = joint_state_indices(
                    names, self.__joint_names
                )
                self.__gripper_joint_index_names = names
        except ValueError:
            return None
        return classify_gripper_state(
            joint_state,
            self.__gripper_joint_indices,
            self.__open_gripper_joint_positions,
            self.__open_tolerance,
            self.__closed_gripper_joint_positions,
        )

    @staticmethod
    def __reached_goal(result: Any) -> bool:
        return result is not None and bool(getattr(result, "reached_goal", False))
