from typing import List, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from .moveit2 import MoveIt2


class MoveIt2Gripper(MoveIt2):
    """
    Python interface for MoveIt 2 Gripper that is controlled by JointTrajectoryController.
    This implementation builds on MoveIt2 to reuse code (while keeping MoveIt2 standalone).
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
        callback_group: Optional[CallbackGroup] = None,
        follow_joint_trajectory_action_name: str = "gripper_trajectory_controller/follow_joint_trajectory",
    ):
        """
        Construct an instance of `MoveIt2Gripper` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `gripper_joint_names` - List of gripper joint names (can be extracted from URDF)
          - `open_gripper_joint_positions` - Configuration of gripper joints when open
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed
          - `gripper_group_name` - Name of the planning group for robot gripper
          - `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed othewise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `follow_joint_trajectory_action_name` - Name of the action server for the controller
        """

        super().__init__(
            node=node,
            joint_names=gripper_joint_names,
            base_link_name="",
            end_effector_name="",
            group_name=gripper_group_name,
            execute_via_moveit=execute_via_moveit,
            ignore_new_calls_while_executing=ignore_new_calls_while_executing,
            callback_group=callback_group,
            follow_joint_trajectory_action_name=follow_joint_trajectory_action_name,
        )
        self.__del_redundant_attributes()

        assert (
            len(gripper_joint_names)
            == len(open_gripper_joint_positions)
            == len(closed_gripper_joint_positions)
        )
        self.__open_gripper_joint_positions = open_gripper_joint_positions
        self.__closed_gripper_joint_positions = closed_gripper_joint_positions

        # Tolerance used for checking whether the gripper is open or closed
        self.__open_tolerance = [
            0.1
            * abs(open_gripper_joint_positions[i] - closed_gripper_joint_positions[i])
            for i in range(len(gripper_joint_names))
        ]

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

    def open(self, skip_if_noop: bool = True):
        """
        Open the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is already open.
        """

        if skip_if_noop and self.is_open:
            return

        self.move_to_configuration(joint_positions=self.__open_gripper_joint_positions)

    def close(self, skip_if_noop: bool = True):
        """
        Close the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is not open.
        """

        if skip_if_noop and self.is_closed:
            return

        self.move_to_configuration(
            joint_positions=self.__closed_gripper_joint_positions
        )

    def reset_open(self):
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.reset_controller(joint_state=self.__open_gripper_joint_positions)

    def reset_closed(self):
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.reset_controller(joint_state=self.__closed_gripper_joint_positions)

    def __del_redundant_attributes(self):

        self.move_to_pose = None
        self.set_pose_goal = None
        self.set_position_goal = None
        self.set_orientation_goal = None
        self.compute_fk = None
        self.compute_ik = None

    @property
    def is_open(self) -> bool:
        """
        Gripper is considered to be open if all of the joints are at their open position.
        """

        for i in range(len(self.joint_names)):

            if (
                abs(
                    self.joint_state.position[
                        self.joint_state.name.index(self.joint_names[i])
                    ]
                    - self.__open_gripper_joint_positions[i]
                )
                > self.__open_tolerance[i]
            ):
                return False

        return True

    @property
    def is_closed(self) -> bool:
        """
        Gripper is considered to be closed if any of the joints is outside of their open position.
        """

        return not self.is_open
