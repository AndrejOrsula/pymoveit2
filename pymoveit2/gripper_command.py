import threading
from typing import List, Optional, Union

from action_msgs.msg import GoalStatus
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
from sensor_msgs.msg import JointState


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
        gripper_command_action_name: str = "gripper_action_controller/gripper_command",
    ):
        """
        Construct an instance of `GripperCommand` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `gripper_joint_names` - List of gripper joint names (can be extracted from URDF)
          - `open_gripper_joint_positions` - Configuration of gripper joints when open
          - `closed_gripper_joint_positions` - Configuration of gripper joints when fully closed
          - `max_effort` - Max effort applied when closing
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `gripper_command_action_name` - Name of the action server for the controller
        """

        self._node = node
        self._callback_group = callback_group

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
        self.__gripper_command_action_client = ActionClient(
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

        # Initialise command goals for opening/closing
        self.__open_gripper_joint_positions = open_gripper_joint_positions
        self.__open_gripper_command_goal = self.__init_gripper_command_goal(
            position=open_gripper_joint_positions, max_effort=max_effort
        )
        self.__close_gripper_command_goal = self.__init_gripper_command_goal(
            position=closed_gripper_joint_positions, max_effort=max_effort
        )

        # Initialise internals for determining whether the gripper is open or closed
        self.__joint_state_mutex = threading.Lock()
        self.__joint_state = None
        self.__new_joint_state_available = False
        # Tolerance used for checking whether the gripper is open or closed
        self.__open_tolerance = [
            0.1
            * abs(open_gripper_joint_positions[i] - closed_gripper_joint_positions[i])
            for i in range(len(open_gripper_joint_positions))
        ]
        # Indices of gripper joint within the joint state message topic.
        # It is assumed that the order of these does not change during execution.
        self.__gripper_joint_indices: Optional[List[int]] = None

        # Flag that determines whether a new goal can be send while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing

        # Store additional variables for later use
        self.__joint_names = gripper_joint_names

        # Internal states that monitor the current motion requests and execution
        self.__is_motion_requested = False
        self.__is_executing = False
        self.__wait_until_executed_rate = self._node.create_rate(1000.0)

    def __call__(self):
        """
        Callable that is identical to `GripperCommand.toggle()`.
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

        if self.__ignore_new_calls_while_executing and self.__is_executing:
            return
        self.__is_motion_requested = True

        self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

    def close(self, skip_if_noop: bool = False):
        """
        Close the gripper.
        - `skip_if_noop` - No action will be performed if the gripper is not open.
        """

        if skip_if_noop and self.is_closed:
            return

        if self.__ignore_new_calls_while_executing and self.__is_executing:
            return
        self.__is_motion_requested = True

        self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

    def reset_open(self, **kwargs):
        """
        Reset into open configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.force_reset_executing_state()
        self.__send_goal_async_gripper_command(self.__open_gripper_command_goal)

    def reset_closed(self, **kwargs):
        """
        Reset into closed configuration by sending a dummy joint trajectory.
        This is useful for simulated robots that allow instantaneous reset of joints.
        """

        self.force_reset_executing_state()
        self.__send_goal_async_gripper_command(self.__close_gripper_command_goal)

    def force_reset_executing_state(self):
        """
        Force reset of internal states that block execution while `ignore_new_calls_while_executing` is being
        used. This function is applicable only in a very few edge-cases, so it should almost never be used.
        """

        self.__is_motion_requested = False
        self.__is_executing = False

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
        return True

    def __joint_state_callback(self, msg: JointState):
        # Update only if all relevant joints are included in the message
        for joint_name in self.joint_names:
            if not joint_name in msg.name:
                return

        self.__joint_state_mutex.acquire()
        self.__joint_state = msg
        self.__new_joint_state_available = True
        self.__joint_state_mutex.release()

    def __send_goal_async_gripper_command(
        self,
        goal: GripperCommandAction.Goal,
        wait_for_server_timeout_sec: Optional[float] = 1.0,
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
                f"Action '{self.__gripper_command_action_client._action_name}' was rejected."
            )
            self.__is_motion_requested = False
            return

        self.__is_executing = True
        self.__is_motion_requested = False

        self.__get_result_future_gripper_command = goal_handle.get_result_async()
        self.__get_result_future_gripper_command.add_done_callback(
            self.__result_callback_gripper_command
        )

    def __result_callback_gripper_command(self, res):
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().error(
                f"Action '{self.__gripper_command_action_client._action_name}' was unsuccessful: {res.result().status}"
            )

        self.__is_executing = False

    @classmethod
    def __init_gripper_command_goal(
        cls, position: Union[float, List[float]], max_effort: float
    ) -> GripperCommandAction.Goal:
        if hasattr(position, "__getitem__"):
            position = position[0]

        gripper_cmd_goal = GripperCommandAction.Goal()
        gripper_cmd_goal.command.position = position
        gripper_cmd_goal.command.max_effort = max_effort

        return gripper_cmd_goal

    @property
    def gripper_command_action_client(self) -> str:
        return self.__gripper_command_action_client

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
