import copy
import math
from typing import Any, List, Optional, Sequence, Union

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymoveit2._joint_helpers import (
    classify_gripper_state,
    copy_joint_state,
    finite_float,
    joint_state_indices,
    normalize_joint_positions,
    validate_joint_names,
    validate_joint_state,
)
from pymoveit2.moveit2 import (
    MoveIt2,
    init_dummy_joint_trajectory_from_state,
    init_execute_trajectory_goal,
    init_joint_state,
)


def resolve_gripper_joint_indices(
    joint_state: JointState,
    gripper_joint_names: Sequence[str],
    cached: Optional[List[int]],
    cached_names: Optional[List[str]],
) -> Optional[List[int]]:
    """
    Return the indices of `gripper_joint_names` within `joint_state.name`.
    """
    names = list(joint_state.name)
    if cached is not None and cached_names == names:
        return cached
    try:
        return joint_state_indices(names, gripper_joint_names)
    except ValueError:
        return None


def is_gripper_open(
    joint_state: Optional[JointState],
    indices: Optional[List[int]],
    open_positions: Sequence[float],
    tolerances: Sequence[float],
    closed_positions: Optional[Sequence[float]] = None,
) -> bool:
    """
    Gripper is open when a complete observation matches every open target.
    """
    return (
        classify_gripper_state(
            joint_state, indices, open_positions, tolerances, closed_positions
        )
        is True
    )


def is_gripper_closed(
    joint_state: Optional[JointState],
    indices: Optional[Sequence[int]],
    open_positions: Sequence[float],
    tolerances: Sequence[float],
    closed_positions: Optional[Sequence[float]] = None,
) -> bool:
    """
    Return true only for a complete observation at the closed target.
    """
    return (
        classify_gripper_state(
            joint_state, indices, open_positions, tolerances, closed_positions
        )
        is False
    )


class MoveIt2Gripper(MoveIt2):
    """
    Python interface for a gripper that is controlled by JointTrajectoryController.
    """

    _UNSUPPORTED = (
        "move_to_pose",
        "set_pose_goal",
        "set_position_goal",
        "set_orientation_goal",
        "compute_fk",
        "compute_fk_async",
        "compute_ik",
        "compute_ik_async",
    )
    _supports_cartesian = False

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
        callback_group: Optional[CallbackGroup] = None,
        use_move_group_action: bool = False,
    ):
        """
        Construct an instance of `MoveIt2Gripper` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `gripper_joint_names` - List of gripper joint names (can be extracted from URDF)
          - `open_gripper_joint_positions` - Configuration of gripper joints when open
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed
          - `gripper_group_name` - Name of the planning group for robot gripper
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories while previous is still being executed
          - `skip_planning` - If enabled, planning is skipped and a single joint trajectory point is published for closing or opening. This enables much faster operation, but the collision checking is disabled and the motion smoothness will depend on the controller.
          - `skip_planning_fixed_motion_duration` - Desired duration for the closing and opening motions when `skip_planning` mode is enabled.
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `use_move_group_action` - Flag that enables execution via MoveGroup action (MoveIt 2) ExecuteTrajectory action is employed otherwise together with a separate planning service client
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
        skip_planning_fixed_motion_duration = finite_float(
            skip_planning_fixed_motion_duration,
            "skip_planning_fixed_motion_duration",
            minimum=0.0,
        )

        super().__init__(
            node=node,
            joint_names=gripper_joint_names,
            base_link_name="",
            end_effector_name="",
            group_name=gripper_group_name,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            callback_group=callback_group,
            use_move_group_action=use_move_group_action,
        )

        self.__open_gripper_joint_positions = open_gripper_joint_positions
        self.__closed_gripper_joint_positions = closed_gripper_joint_positions

        self.__skip_planning = bool(skip_planning)
        self.__closed = False
        if skip_planning:
            duration_sec = math.floor(skip_planning_fixed_motion_duration)
            duration_nanosec = int(
                1e9 * (skip_planning_fixed_motion_duration - duration_sec)
            )
            self.__open_dummy_trajectory_goal = init_execute_trajectory_goal(
                init_dummy_joint_trajectory_from_state(
                    init_joint_state(
                        joint_names=gripper_joint_names,
                        joint_positions=open_gripper_joint_positions,
                    ),
                    duration_sec=duration_sec,
                    duration_nanosec=duration_nanosec,
                )
            )
            self.__close_dummy_trajectory_goal = init_execute_trajectory_goal(
                init_dummy_joint_trajectory_from_state(
                    init_joint_state(
                        joint_names=gripper_joint_names,
                        joint_positions=closed_gripper_joint_positions,
                    ),
                    duration_sec=duration_sec,
                    duration_nanosec=duration_nanosec,
                )
            )

        self.__open_tolerance = [
            0.1
            * abs(open_gripper_joint_positions[i] - closed_gripper_joint_positions[i])
            for i in range(len(gripper_joint_names))
        ]
        self.__gripper_joint_indices: Optional[List[int]] = None
        self.__gripper_joint_index_names: Optional[List[str]] = None

    def destroy(self) -> None:
        self.__closed = True
        super().destroy()

    def __call__(self) -> bool:
        """
        Callable that is identical to `MoveIt2Gripper.toggle()`.
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

        if self.__closed:
            return False
        if skip_if_noop and self.is_open:
            return False

        if self.__skip_planning:
            return self.__open_without_planning()
        return self.move_to_configuration(
            joint_positions=self.__open_gripper_joint_positions
        )

    def close(self, skip_if_noop: bool = False) -> bool:
        """
        Close the gripper. Returns whether a goal was submitted.
        """

        if self.__closed:
            return False
        if skip_if_noop and self.is_closed:
            return False

        if self.__skip_planning:
            return self.__close_without_planning()
        return self.move_to_configuration(
            joint_positions=self.__closed_gripper_joint_positions
        )

    def move_to_position(self, position: float) -> bool:
        """
        Move the gripper to a specific position (applied to every gripper joint).
        """

        scalar = finite_float(position, "position")
        joint_positions = [scalar for _ in self.joint_names]
        return self.move_to_configuration(joint_positions=joint_positions)

    def reset_open(self, sync: bool = True) -> bool:
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        return self.reset_controller(joint_state=self.__open_gripper_joint_positions)

    def reset_closed(self, sync: bool = True) -> bool:
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        return self.reset_controller(joint_state=self.__closed_gripper_joint_positions)

    def __open_without_planning(self) -> bool:
        if self.__closed:
            return False
        return self._send_goal_async_execute_trajectory(
            goal=copy.deepcopy(self.__open_dummy_trajectory_goal),
        )

    def __close_without_planning(self) -> bool:
        if self.__closed:
            return False
        return self._send_goal_async_execute_trajectory(
            goal=copy.deepcopy(self.__close_dummy_trajectory_goal),
        )

    def __unsupported(self, name: str) -> Any:
        raise NotImplementedError(
            f"`MoveIt2Gripper.{name}()` is not supported: a gripper only exposes "
            "joint-space goals (`open()`, `close()`, `move_to_position()`, "
            "`move_to_configuration()`)."
        )

    def move_to_pose(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("move_to_pose")

    def set_pose_goal(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("set_pose_goal")

    def set_position_goal(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("set_position_goal")

    def set_orientation_goal(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("set_orientation_goal")

    def compute_fk(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("compute_fk")

    def compute_fk_async(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("compute_fk_async")

    def compute_ik(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("compute_ik")

    def compute_ik_async(self, *args: Any, **kwargs: Any) -> Any:
        return self.__unsupported("compute_ik_async")

    @property
    def open_gripper_joint_positions(self) -> List[float]:
        return list(self.__open_gripper_joint_positions)

    @property
    def closed_gripper_joint_positions(self) -> List[float]:
        return list(self.__closed_gripper_joint_positions)

    @property
    def joint_names(self) -> List[str]:
        return list(super().joint_names)

    @property
    def joint_state(self) -> Optional[JointState]:
        return copy_joint_state(super().joint_state)

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
            validate_joint_state(joint_state, super().joint_names)
            names = list(joint_state.name)
            if self.__gripper_joint_indices is None or (
                self.__gripper_joint_index_names != names
            ):
                self.__gripper_joint_indices = joint_state_indices(
                    names, super().joint_names
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
