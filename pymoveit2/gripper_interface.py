import copy
import time
from typing import Any, List, Optional, Type, Union

from control_msgs.action import GripperCommand as GripperCommandAction
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from pymoveit2._action_lifecycle import MoveIt2State
from pymoveit2._joint_helpers import (
    finite_float,
    is_uniform,
    normalize_joint_positions,
    validate_joint_names,
)
from pymoveit2.gripper_command import GripperCommand
from pymoveit2.moveit2_gripper import MoveIt2Gripper

GripperBackend = Union[GripperCommand, MoveIt2Gripper]


class GripperInterface:
    """
    Python interface for a gripper, over `GripperCommand` or `JointTrajectoryController`.
    Exactly one backend (`GripperCommand` or `MoveIt2Gripper`) is constructed once its action server has been discovered, and every public method and property of this class delegates to that backend.
    """

    def __init__(
        self,
        node: Node,
        gripper_joint_names: List[str],
        open_gripper_joint_positions: Union[float, List[float]],
        closed_gripper_joint_positions: Union[float, List[float]],
        gripper_group_name: str = "gripper",
        ignore_new_calls_while_executing: bool = False,
        skip_planning: bool = False,
        skip_planning_fixed_motion_duration: float = 0.5,
        max_effort: float = 0.0,
        callback_group: Optional[CallbackGroup] = None,
        gripper_command_action_name: str = "gripper_action_controller/gripper_cmd",
        use_move_group_action: bool = False,
        interface: Optional[Type[GripperBackend]] = None,
        discovery_timeout_sec: float = 1.0,
    ):
        """
        Combination of `MoveIt2Gripper` and `GripperCommand` interfaces that automatically selects the appropriate interface based on the available actions.
          - `interface` - Force a backend (`GripperCommand` or `MoveIt2Gripper`) instead of discovering it from the available action servers
        The remaining parameters are forwarded to the selected backend.
        """

        gripper_joint_names = validate_joint_names(gripper_joint_names)
        open_gripper_joint_positions = normalize_joint_positions(
            open_gripper_joint_positions,
            len(gripper_joint_names),
            "open_gripper_joint_positions",
        )
        closed_gripper_joint_positions = normalize_joint_positions(
            closed_gripper_joint_positions,
            len(gripper_joint_names),
            "closed_gripper_joint_positions",
        )
        max_effort = finite_float(max_effort, "max_effort")
        skip_planning_fixed_motion_duration = finite_float(
            skip_planning_fixed_motion_duration,
            "skip_planning_fixed_motion_duration",
            minimum=0.0,
        )
        discovery_timeout_sec = finite_float(
            discovery_timeout_sec, "discovery_timeout_sec", minimum=0.0
        )

        if interface is not None and interface not in (GripperCommand, MoveIt2Gripper):
            raise ValueError(
                "`interface` must be `GripperCommand`, `MoveIt2Gripper` or `None`!"
            )
        if interface is GripperCommand and (
            not is_uniform(open_gripper_joint_positions)
            or not is_uniform(closed_gripper_joint_positions)
        ):
            raise ValueError(
                "`GripperCommand` controls one scalar actuator; open and closed "
                "positions must each be uniform across the configured joints. "
                "Use `MoveIt2Gripper` for nonuniform joint goals."
            )

        self._node = node
        self._callback_group = callback_group
        self.__joint_names = list(gripper_joint_names)
        self.__discovery_timeout_sec = discovery_timeout_sec
        self.__forced_interface = interface
        self.__use_move_group_action = bool(use_move_group_action)
        self.__skip_planning = bool(skip_planning)
        self.__gripper_command_action_name = gripper_command_action_name
        self.__gripper_command_kwargs = dict(
            node=node,
            gripper_joint_names=self.__joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            max_effort=max_effort,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            callback_group=callback_group,
            gripper_command_action_name=gripper_command_action_name,
        )
        self.__moveit2_gripper_kwargs = dict(
            node=node,
            gripper_joint_names=self.__joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            gripper_group_name=gripper_group_name,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            skip_planning=skip_planning,
            skip_planning_fixed_motion_duration=skip_planning_fixed_motion_duration,
            callback_group=callback_group,
            use_move_group_action=use_move_group_action,
        )

        self._backend: Optional[GripperBackend] = None
        self._interface: Optional[Type[GripperBackend]] = None
        self.__closed = False
        self.__backend_destroyed = False
        self.__discovery_complete = False

        self.__determine_interface()

    def __determine_interface(self) -> Optional[GripperBackend]:
        if self.__closed:
            return None
        if self._backend is not None:
            return self._backend
        if self.__discovery_complete:
            return None

        selected = self.__forced_interface
        self.__discovery_complete = True
        if selected is None:
            selected = self.__discover_interface()

        if selected is None:
            self._node.get_logger().warning(
                "Unable to determine the appropriate interface for gripper."
            )
            return None

        if selected is GripperCommand:
            self._backend = GripperCommand(**self.__gripper_command_kwargs)
        else:
            self._backend = MoveIt2Gripper(**self.__moveit2_gripper_kwargs)
        self._interface = selected
        return self._backend

    def __discover_interface(self) -> Optional[Type[GripperBackend]]:
        planned_action_name = (
            "move_action"
            if self.__use_move_group_action and not self.__skip_planning
            else "execute_trajectory"
        )
        planned_action_type = (
            MoveGroup
            if self.__use_move_group_action and not self.__skip_planning
            else ExecuteTrajectory
        )
        probes = (
            (GripperCommand, GripperCommandAction, self.__gripper_command_action_name),
            (MoveIt2Gripper, planned_action_type, planned_action_name),
        )
        deadline = time.monotonic() + self.__discovery_timeout_sec
        for backend, action_type, action_name in probes:
            timeout_sec = max(0.0, deadline - time.monotonic())
            client = None
            try:
                client = ActionClient(
                    node=self._node,
                    action_type=action_type,
                    action_name=action_name,
                    callback_group=self._callback_group,
                )
                available = client.wait_for_server(timeout_sec=timeout_sec)
            except Exception as error:
                self._node.get_logger().warning(
                    f"Unable to probe gripper action '{action_name}': {error}"
                )
                available = False
            finally:
                if client is not None:
                    try:
                        client.destroy()
                    except Exception as error:
                        self._node.get_logger().warning(
                            f"Unable to clean up gripper action probe "
                            f"'{action_name}': {error}"
                        )
            if available:
                if backend is GripperCommand and (
                    not is_uniform(
                        self.__gripper_command_kwargs["open_gripper_joint_positions"]
                    )
                    or not is_uniform(
                        self.__gripper_command_kwargs["closed_gripper_joint_positions"]
                    )
                ):
                    self._node.get_logger().warning(
                        "Skipping raw gripper action because the configured joint "
                        "targets are nonuniform; trying planned execution."
                    )
                    continue
                return backend
        return None

    def __require_backend(self, action: str) -> Optional[GripperBackend]:
        if self.__closed:
            self._node.get_logger().warning(
                f"Unable to {action} because the gripper interface was destroyed."
            )
            return None
        backend = self.__determine_interface()
        if backend is None:
            self._node.get_logger().error(
                f"Unable to {action} because the appropriate interface cannot be determined."
            )
        return backend

    @property
    def backend(self) -> Optional[GripperBackend]:
        return self._backend

    @property
    def interface(self) -> Optional[Type[GripperBackend]]:
        return self._interface

    def destroy(self) -> None:
        if self.__backend_destroyed:
            return
        self.__closed = True
        if self._backend is None:
            self.__backend_destroyed = True
            return
        self._backend.destroy()
        self.__backend_destroyed = True

    def __enter__(self) -> "GripperInterface":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.destroy()

    def __call__(self) -> bool:
        """
        Callable that is identical to `GripperInterface.toggle()`.
        """

        return self.toggle()

    def toggle(self) -> bool:
        """
        Toggles the gripper between open and closed state.
        """

        backend = self.__require_backend("toggle the gripper")
        if backend is None:
            return False
        return backend.toggle()

    def open(self, skip_if_noop: bool = False) -> bool:
        """
        Open the gripper. Returns whether a goal was submitted.
        """

        backend = self.__require_backend("open the gripper")
        if backend is None:
            return False
        return backend.open(skip_if_noop=skip_if_noop)

    def close(self, skip_if_noop: bool = False) -> bool:
        """
        Close the gripper. Returns whether a goal was submitted.
        """

        backend = self.__require_backend("close the gripper")
        if backend is None:
            return False
        return backend.close(skip_if_noop=skip_if_noop)

    def move_to_position(self, position: float) -> bool:
        """
        Move the gripper to a specific position. Returns whether a goal was submitted.
        """

        backend = self.__require_backend("move the gripper to a position")
        if backend is None:
            return False
        return backend.move_to_position(position=position)

    def reset_open(self, sync: bool = True) -> bool:
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        backend = self.__require_backend("reset the gripper as open")
        if backend is None:
            return False
        return backend.reset_open(sync=sync)

    def reset_closed(self, sync: bool = True) -> bool:
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        backend = self.__require_backend("reset the gripper as closed")
        if backend is None:
            return False
        return backend.reset_closed(sync=sync)

    def force_reset_executing_state(self) -> None:
        backend = self.__require_backend("reset the executing state")
        if backend is not None:
            backend.force_reset_executing_state()

    def wait_until_executed(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        """

        backend = self.__require_backend("wait until a motion is executed")
        if backend is None:
            return False
        return backend.wait_until_executed(timeout_sec=timeout_sec)

    def query_state(self) -> MoveIt2State:
        if self._backend is None:
            return MoveIt2State.IDLE
        return self._backend.query_state()

    def cancel_execution(self) -> bool:
        """
        Cancel the tracked goal of the selected backend.
        """
        backend = self.__require_backend("cancel the gripper motion")
        if backend is None:
            return False
        return backend.cancel_execution()

    def get_execution_future(self) -> Optional[Future]:
        if self._backend is None:
            self._node.get_logger().warning("Need active goal for future.")
            return None
        return self._backend.get_execution_future()

    def get_last_execution_error_code(self) -> Optional[Any]:
        if self._backend is None:
            return None
        return self._backend.get_last_execution_error_code()

    def last_failure(self) -> Optional[str]:
        if self._backend is None:
            return "No gripper backend was discovered; no goal was ever submitted."
        return self._backend.last_failure()

    def reset_new_joint_state_checker(self) -> None:
        if self._backend is not None:
            self._backend.reset_new_joint_state_checker()

    @property
    def motion_succeeded(self) -> bool:
        """
        Whether the last motion of the selected backend finished successfully.
        """
        return self._backend.motion_succeeded if self._backend is not None else False

    @motion_succeeded.setter
    def motion_succeeded(self, value: bool) -> None:
        if self._backend is not None:
            self._backend.motion_succeeded = value

    @property
    def joint_names(self) -> List[str]:
        return list(self.__joint_names)

    @property
    def joint_state(self) -> Optional[JointState]:
        if self._backend is None:
            return None
        return copy.deepcopy(self._backend.joint_state)

    @property
    def new_joint_state_available(self) -> bool:
        return (
            self._backend.new_joint_state_available
            if self._backend is not None
            else False
        )

    @property
    def is_open(self) -> bool:
        """
        Gripper is considered to be open if all of the joints are at their open position.
        """
        return self._backend.is_open if self._backend is not None else False

    @property
    def is_closed(self) -> bool:
        """
        Gripper is considered closed only when every joint matches its closed target.
        """
        return self._backend.is_closed if self._backend is not None else False

    @property
    def gripper_command_action_client(self) -> Optional[ActionClient]:
        if isinstance(self._backend, GripperCommand):
            return self._backend.gripper_command_action_client
        return None
