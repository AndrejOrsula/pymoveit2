from typing import List, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from .gripper_command import GripperCommand
from .moveit2_gripper import MoveIt2Gripper


class GripperInterface(MoveIt2Gripper, GripperCommand):
    """
    Python interface for MoveIt 2 Gripper that is controlled either by GripperCommand or JointTrajectoryController.
    The appropriate interface is automatically selected based on the available action.
    """

    def __init__(
        self,
        node: Node,
        gripper_joint_names: List[str],
        open_gripper_joint_positions: List[float],
        closed_gripper_joint_positions: List[float],
        gripper_group_name: str = "gripper",
        execute_via_moveit: bool = False,
        ignore_new_calls_while_executing: bool = False,
        skip_planning: bool = False,
        skip_planning_fixed_motion_duration: float = 0.5,
        max_effort: float = 0.0,
        callback_group: Optional[CallbackGroup] = None,
        follow_joint_trajectory_action_name: str = "DEPRECATED",
        gripper_command_action_name: str = "gripper_action_controller/gripper_cmd",
        use_move_group_action: bool = False,
    ):
        """
        Combination of `MoveIt2Gripper` and `GripperCommand` interfaces that automatically
        selects the appropriate interface based on the available actions.
        """

        # Check for deprecated parameters
        if execute_via_moveit:
            node.get_logger().warn(
                "Parameter `execute_via_moveit` is deprecated. Please use `use_move_group_action` instead."
            )
            use_move_group_action = True
        if follow_joint_trajectory_action_name != "DEPRECATED":
            node.get_logger().warn(
                "Parameter `follow_joint_trajectory_action_name` is deprecated. `MoveIt2` uses the `execute_trajectory` action instead."
            )

        MoveIt2Gripper.__init__(
            self=self,
            node=node,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            gripper_group_name=gripper_group_name,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            skip_planning=skip_planning,
            skip_planning_fixed_motion_duration=skip_planning_fixed_motion_duration,
            callback_group=callback_group,
            use_move_group_action=use_move_group_action,
        )

        GripperCommand.__init__(
            self=self,
            node=node,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            max_effort=max_effort,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            callback_group=callback_group,
            gripper_command_action_name=gripper_command_action_name,
        )

        self.__determine_interface()

    def __determine_interface(self, timeout_sec=1.0):
        if self.gripper_command_action_client.wait_for_server(timeout_sec=timeout_sec):
            self._interface = GripperCommand
        elif self._execute_trajectory_action_client.wait_for_server(
            timeout_sec=timeout_sec
        ):
            self._interface = MoveIt2Gripper
        else:
            self._interface = None

        if self._interface is None:
            self._node.get_logger().warn(
                f"Unable to determine the appropriate interface for gripper."
            )

    def __call__(self):
        """
        Callable that is identical to `MoveIt2Gripper.toggle()`.
        """

        self.toggle()

    def toggle(self):
        """
        Toggles the gripper between open and closed state.
        """

        if self.is_open:
            self.close(skip_if_noop=False)
        else:
            self.open(skip_if_noop=False)

    def open(self, skip_if_noop: bool = False):
        """
        Open the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is already open.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to close the gripper because the appropriate interface cannot be determined."
            )
            return

        self._interface.open(self=self, skip_if_noop=skip_if_noop)

    def close(self, skip_if_noop: bool = False):
        """
        Close the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is not open.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to close the gripper because the appropriate interface cannot be determined."
            )
            return

        self._interface.close(self=self, skip_if_noop=skip_if_noop)

    def move_to_position(self, position: float):
        """
        Move the gripper to a specific position.
        - `position` - Desired position of the gripper.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                "Unable to move the gripper to a position because the appropriate "
                "interface cannot be determined."
            )
            return

        self._interface.move_to_position(self=self, position=position)

    def reset_open(self, sync: bool = True):
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to reset the gripper as open because the appropriate interface cannot be determined."
            )
            return

        self._interface.reset_open(self=self, sync=sync)

    def reset_closed(self, sync: bool = True):
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to reset the gripper as closed because the appropriate interface cannot be determined."
            )
            return

        self._interface.reset_closed(self=self, sync=sync)

    def force_reset_executing_state(self):
        """
        Force reset of internal states that block execution while `ignore_new_calls_while_executing` is being
        used. This function is applicable only in a very few edge-cases, so it should almost never be used.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to reset the executing state because the appropriate interface cannot be determined."
            )
            return

        self._interface.force_reset_executing_state(self=self)

    def wait_until_executed(self) -> bool:
        """
        Wait until the previously requested motion is finalised through either a success or failure.
        """

        if self._interface is None:
            self.__determine_interface()
        if self._interface is None:
            self._node.get_logger().error(
                f"Unable to wait until a motion is executed because the appropriate interface cannot be determined."
            )
            return False

        return self._interface.wait_until_executed(self=self)
