from typing import List, Optional, Union

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class GripperCommand:
    """
    Python interface for Gripper that is controlled by GripperCommand.
    """

    def __init__(
        self,
        node: Node,
        open_gripper_joint_positions: Union[float, List[float]],
        closed_gripper_joint_positions: Union[float, List[float]],
        max_effort: float = 0.0,
        ignore_new_calls_while_executing: bool = True,
        callback_group: Optional[CallbackGroup] = None,
        gripper_command_action_name: str = "gripper_action_controller/gripper_command",
        **kwargs,
    ):
        """
        Construct an instance of `MoveIt2Gripper` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `open_gripper_joint_positions` - Configuration of gripper joints when open
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed
          - `max_effort` - Max effort applied when closing
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `gripper_command_action_name` - Name of the action server for the controller
        """

        self._node = node

        # Create action client for move action
        self.__gripper_command_action_client = ActionClient(
            node=self._node,
            action_type=GripperCommand,
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
            callback_group=callback_group,
        )

        self.__open_gripper_command_goal = self.__init_gripper_command_goal(
            position=open_gripper_joint_positions, max_effort=max_effort
        )
        self.__close_gripper_command_goal = self.__init_gripper_command_goal(
            position=closed_gripper_joint_positions, max_effort=max_effort
        )

        # Flag that determines whether a new goal can be send while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing
        self.__is_executing = False

        # Initialize additional variables
        self.__is_open = True

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
            self.close()
        else:
            self.open()

    def open(self, skip_if_noop: bool = True):
        """
        Open the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is already open.
        """

        if skip_if_noop and self.is_open:
            return

        if self.__ignore_new_calls_while_executing and self.__is_executing:
            return

        self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

    def close(self, skip_if_noop: bool = True):
        """
        Close the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is not open.
        """

        if skip_if_noop and self.is_closed:
            return

        if self.__ignore_new_calls_while_executing and self.__is_executing:
            return

        self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

    def reset_open(self):
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

        self.__is_open = True

    def reset_closed(self):
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

        self.__is_open = False

    def __toggle_internal_gripper_state(self):

        self.__is_open = not self.__is_open

    def __send_goal_async_gripper_command(
        self, goal: GripperCommand.Goal, wait_for_server_timeout_sec: float = 1.0
    ):

        if not self.__gripper_command_action_client.wait_for_server(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Action server {self.__gripper_command_action_client._action_name} is not yet ready. Better luck next time!"
            )
            return

        action_result = self.__gripper_command_action_client.send_goal_async(
            goal=goal,
            feedback_callback=None,
        )

        action_result.add_done_callback(self.__response_callback_gripper_command)

    def __response_callback_gripper_command(self, response):

        goal_handle = response.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn(
                f"Action '{self.__gripper_command_action_client._action_name}' was rejected"
            )
            return

        if self.__ignore_new_calls_while_executing:
            self.__is_executing = True

        self.__get_result_future_gripper_command = goal_handle.get_result_async()
        self.__get_result_future_gripper_command.add_done_callback(
            self.__result_callback_gripper_command
        )

    def __result_callback_gripper_command(self, res):

        if res.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.__toggle_internal_gripper_state()
        else:
            self._node.get_logger().error(
                f"Action '{self.__gripper_command_action_client._action_name}' was unsuccessful: {res.result().status}"
            )

        if self.__ignore_new_calls_while_executing:
            self.__is_executing = False

    @classmethod
    def __init_gripper_command_goal(
        cls, position: Union[float, List[float]], max_effort: float
    ) -> GripperCommand.Goal:

        if hasattr(position, "__getitem__"):
            position = position[0]

        gripper_cmd_goal = GripperCommand.Goal()
        gripper_cmd_goal.command.position = position
        gripper_cmd_goal.command.max_effort = max_effort

        return gripper_cmd_goal

    @property
    def is_open(self) -> bool:

        return self.__is_open

    @property
    def is_closed(self) -> bool:

        return not self.is_open
