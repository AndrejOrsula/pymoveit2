from typing import List, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from .moveit2_gripper import MoveIt2Gripper


class GripperInterface(MoveIt2Gripper):
    """
    Python interface for MoveIt 2 Gripper that is controlled by JointTrajectoryController.
    """

    def __init__(
        self,
        node: Node,
        gripper_joint_names: List[str],
        open_gripper_joint_positions: List[float],
        closed_gripper_joint_positions: List[float],
        gripper_group_name: str = "gripper",
        ignore_new_calls_while_executing: bool = False,
        skip_planning: bool = False,
        skip_planning_fixed_motion_duration: float = 0.5,
        callback_group: Optional[CallbackGroup] = None,
        execute_via_moveit: bool = True,
        follow_joint_trajectory_action_name: str = "joint_trajectory_controller/follow_joint_trajectory",
    ):
        """
        Combination of `MoveIt2Gripper` and additional control interfaces.
        """
        super().__init__(
            node=node,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=open_gripper_joint_positions,
            closed_gripper_joint_positions=closed_gripper_joint_positions,
            gripper_group_name=gripper_group_name,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            skip_planning=skip_planning,
            skip_planning_fixed_motion_duration=skip_planning_fixed_motion_duration,
            callback_group=callback_group,
            execute_via_moveit=execute_via_moveit,
            follow_joint_trajectory_action_name=follow_joint_trajectory_action_name,
        )

    def open(self, skip_if_noop: bool = False):
        """Open the gripper."""
        if skip_if_noop and self.is_open:
            self._node.get_logger().warn("Gripper is already opened")
            return
        self.move_to_configuration(self._MoveIt2Gripper__open_gripper_joint_positions)

    def close(self, skip_if_noop: bool = False):
        """Close the gripper."""
        if skip_if_noop and self.is_closed:
            self._node.get_logger().warn("Gripper is already closed")
            return
        self.move_to_configuration(self._MoveIt2Gripper__closed_gripper_joint_positions)

    def toggle(self):
        """Toggle between open and closed states."""
        if self.is_open:
            self.close()
            self._node.get_logger().info("Closing gripper")
        else:
            self.open()
            self._node.get_logger().info("Opening gripper")

    def move_to_position(self, position: float):
        """Move the gripper to a specific position."""
        joint_positions = [position for _ in self.joint_names]
        self.move_to_configuration(joint_positions=joint_positions)
