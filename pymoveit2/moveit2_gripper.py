import math
from typing import List, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from .moveit2 import *


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
        skip_planning: bool = False,
        skip_planning_fixed_motion_duration: float = 0.5,
        callback_group: Optional[CallbackGroup] = None,
        follow_joint_trajectory_action_name: str = "DEPRECATED",
        use_move_group_action: bool = False,
    ):
        """
        Construct an instance of `MoveIt2Gripper` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `gripper_joint_names` - List of gripper joint names (can be extracted from URDF)
          - `open_gripper_joint_positions` - Configuration of gripper joints when open
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed
          - `gripper_group_name` - Name of the planning group for robot gripper
          - [DEPRECATED] `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed otherwise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `skip_planning` - If enabled, planning is skipped and a single joint trajectory point is published
                              for closing or opening. This enables much faster operation, but the collision
                              checking is disabled and the motion smoothness will depend on the controller.
          - `skip_planning_fixed_motion_duration` - Desired duration for the closing and opening motions when
                                                    `skip_planning` mode is enabled.
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - [DEPRECATED] `follow_joint_trajectory_action_name` - Name of the action server for the controller
          - `use_move_group_action` - Flag that enables execution via MoveGroup action (MoveIt 2)
                               ExecuteTrajectory action is employed otherwise
                               together with a separate planning service client
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
        self.__del_redundant_attributes()

        assert (
            len(gripper_joint_names)
            == len(open_gripper_joint_positions)
            == len(closed_gripper_joint_positions)
        )
        self.__open_gripper_joint_positions = open_gripper_joint_positions
        self.__closed_gripper_joint_positions = closed_gripper_joint_positions

        self.__skip_planning = skip_planning
        if skip_planning:
            duration_sec = math.floor(skip_planning_fixed_motion_duration)
            duration_nanosec = int(
                10e8 * (skip_planning_fixed_motion_duration - duration_sec)
            )
            self.__open_dummy_trajectory_goal = init_follow_joint_trajectory_goal(
                init_dummy_joint_trajectory_from_state(
                    init_joint_state(
                        joint_names=gripper_joint_names,
                        joint_positions=open_gripper_joint_positions,
                    ),
                    duration_sec=duration_sec,
                    duration_nanosec=duration_nanosec,
                )
            )
            self.__close_dummy_trajectory_goal = init_follow_joint_trajectory_goal(
                init_dummy_joint_trajectory_from_state(
                    init_joint_state(
                        joint_names=gripper_joint_names,
                        joint_positions=closed_gripper_joint_positions,
                    ),
                    duration_sec=duration_sec,
                    duration_nanosec=duration_nanosec,
                )
            )

        # Tolerance used for checking whether the gripper is open or closed
        self.__open_tolerance = [
            0.1
            * abs(open_gripper_joint_positions[i] - closed_gripper_joint_positions[i])
            for i in range(len(gripper_joint_names))
        ]
        # Indices of gripper joint within the joint state message topic.
        # It is assumed that the order of these does not change during execution.
        self.__gripper_joint_indices: Optional[List[int]] = None

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

        if skip_if_noop and self.is_open:
            return

        if self.__skip_planning:
            self.__open_without_planning()
        else:
            self.move_to_configuration(
                joint_positions=self.__open_gripper_joint_positions
            )

    def close(self, skip_if_noop: bool = False):
        """
        Close the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is not open.
        """

        if skip_if_noop and self.is_closed:
            return

        if self.__skip_planning:
            self.__close_without_planning()
        else:
            self.move_to_configuration(
                joint_positions=self.__closed_gripper_joint_positions
            )

    def move_to_position(self, position: float):
        """
        Move the gripper to a specific position.
        - `position` - Desired position of the gripper
        """

        joint_positions = [position for _ in self.joint_names]
        self.move_to_configuration(joint_positions=joint_positions)

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

    def __open_without_planning(self):
        self._send_goal_async_follow_joint_trajectory(
            goal=self.__open_dummy_trajectory_goal,
            wait_until_response=False,
        )

    def __close_without_planning(self):
        self._send_goal_async_follow_joint_trajectory(
            goal=self.__close_dummy_trajectory_goal,
            wait_until_response=False,
        )

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

        joint_state = self.joint_state

        # Assume the gripper is open if there are no joint state readings yet
        if joint_state is None:
            return True

        # For the sake of performance, find the indices of joints only once.
        # This is especially useful for robots with many joints.
        if self.__gripper_joint_indices is None:
            self.__gripper_joint_indices: List[int] = []
            for joint_name in self.joint_names:
                self.__gripper_joint_indices.append(joint_state.name.index(joint_name))

        for local_joint_index, joint_state_index in enumerate(
            self.__gripper_joint_indices
        ):
            if (
                abs(
                    joint_state.position[joint_state_index]
                    - self.__open_gripper_joint_positions[local_joint_index]
                )
                > self.__open_tolerance[local_joint_index]
            ):
                return False

        return True

    @property
    def is_closed(self) -> bool:
        """
        Gripper is considered to be closed if any of the joints is outside of their open position.
        """

        return not self.is_open
