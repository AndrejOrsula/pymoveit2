from copy import deepcopy
from typing import Optional, Tuple

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import SetBool


class MoveIt2Servo:
    """
    Python interface for MoveIt 2 Servo that enables real-time control in Cartesian Space.
    This implementation is just a thin wrapper around TwistStamped message publisher.
    """

    def __init__(
        self,
        node: Node,
        frame_id: str,
        namespace: str = "",
        linear_speed: float = 1.0,
        angular_speed: float = 1.0,
        enable_at_init: bool = True,
        callback_group: Optional[CallbackGroup] = None,
    ):
        """
        Construct an instance of `MoveIt2Servo` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `namespace` - Namespace for the node
          - `frame_id` - Reference frame in which to publish command messages
          - `linear_speed` - Factor that can be used to scale all input linear twist commands
          - `angular_speed` - Factor that can be used to scale all input angular twist commands
          - `enable_at_init` - Flag that enables initialisation of MoveIt 2 Servo during initialisation
                               Otherwise, `MoveIt2Servo.enable()` must be called explicitly
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
        """

        self._node = node
        self.namespace = namespace

        # Create publishers
        self.__twist_pub = self._node.create_publisher(
            msg_type=TwistStamped,
            topic=self.namespace + "/servo_node/delta_twist_cmds",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=callback_group,
        )
        self.__jog_pub = self._node.create_publisher(
            msg_type=JointJog,
            topic="/servo_node/delta_joint_cmds",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=callback_group,
        )

        # Create service clients
        self.__pause_service = self._node.create_client(
            srv_type=SetBool,
            srv_name=self.namespace + "/servo_node/pause_servo",
            callback_group=callback_group,
        )
        self.__command_type_service = self._node.create_client(
            srv_type=ServoCommandType,
            srv_name=self.namespace + "/servo_node/switch_command_type",
            callback_group=callback_group,
        )
        self.__enable_req = SetBool.Request(data=False)
        self.__disable_req = SetBool.Request(data=True)
        self.__twist_command_type_req = ServoCommandType.Request(
            command_type=ServoCommandType.Request.TWIST
        )
        self.__is_enabled = False

        # Initialize message based on passed arguments
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = frame_id
        self.__twist_msg.twist.linear.x = linear_speed
        self.__twist_msg.twist.linear.y = linear_speed
        self.__twist_msg.twist.linear.z = linear_speed
        self.__twist_msg.twist.angular.x = angular_speed
        self.__twist_msg.twist.angular.y = angular_speed
        self.__twist_msg.twist.angular.z = angular_speed

        # Enable servo immediately, if desired
        if enable_at_init:
            self.enable()

    def __del__(self):
        """
        Try to stop MoveIt 2 Servo during destruction.
        """

        try:
            if self.is_enabled:
                self.__pause_service.call_async(self.__disable_req)
        except:
            pass

    def __call__(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """
        Callable that is identical to `MoveIt2Servo.servo()`.
        """

        self.servo(linear=linear, angular=angular)

    def servo(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        enable_if_disabled: bool = True,
    ):
        """
        Apply linear and angular twist using MoveIt 2 Servo.
        Input is scaled by `linear_speed` and `angular_speed`, respectively.
        """

        if not self.__ensure_enabled(enable_if_disabled):
            return

        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self._node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]
        self.__twist_pub.publish(twist_msg)

    def servo_jog(
        self,
        joint_names: Tuple[str, ...] = tuple(),
        velocities: Tuple[float, ...] = tuple(),
        enable_if_disabled: bool = True,
    ):
        """
        Apply velocity commands to joints listed in joints_names.
        Units are in rad/s.
        """

        if not self.__ensure_enabled(enable_if_disabled):
            return

        jog_msg = JointJog()
        jog_msg.header.stamp = self._node.get_clock().now().to_msg()
        jog_msg.header.frame_id = self.frame_id
        jog_msg.joint_names = joint_names
        jog_msg.velocities = [float(v) for v in velocities]
        self.__jog_pub.publish(jog_msg)

    def __ensure_enabled(self, enable_if_disabled: bool) -> bool:
        if self.is_enabled:
            return True

        self._node.get_logger().warn(
            "Command failed because MoveIt Servo is not yet enabled."
        )
        if enable_if_disabled:
            self._node.get_logger().warn(
                f"Calling '{self.__start_service.srv_name}' service to enable MoveIt Servo..."
            )
            if self.enable():
                return True
        return False

    def enable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Enable MoveIt 2 Servo server via async service call.
        """

        while not self.__pause_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__pause_service.srv_name}' is not yet available..."
            )
            return False
        while not self.__command_type_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__command_type_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result: SetBool.Response = self.__pause_service.call(self.__enable_req)
            if not result or not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be enabled. ({result.message})"
                )
                self.__is_enabled = False
                return False
            switch_cmd_result: ServoCommandType.Response = (
                self.__command_type_service.call(self.__twist_command_type_req)
            )
            if not switch_cmd_result or not switch_cmd_result.success:
                self._node.get_logger().error(
                    "MoveIt Servo could not be switched to TWIST command type."
                )
                self.__is_enabled = False
                return False
            self.__is_enabled = True
            return True
        else:
            start_service_future = self.__pause_service.call_async(self.__enable_req)
            start_service_future.add_done_callback(self.__enable_done_callback)
            return True

    def disable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Disable MoveIt 2 Servo server via async service call.
        """

        while not self.__pause_service.wait_for_service(
            timeout_sec=wait_for_server_timeout_sec
        ):
            self._node.get_logger().warn(
                f"Service '{self.__pause_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result: SetBool.Response = self.__pause_service.call(self.__disable_req)
            if not result or not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be disabled. ({result.message})"
                )
            self.__is_enabled = not result.success
            return result.success
        else:
            pause_service_future = self.__pause_service.call_async(self.__disable_req)
            pause_service_future.add_done_callback(self.__disable_done_callback)
            return True

    def __enable_done_callback(self, future: Future):
        result: SetBool.Response = future.result()

        if not result.success:
            self._node.get_logger().error(
                f"MoveIt Servo could not be enabled. ({result.message})"
            )
            self.__is_enabled = False
            return

        switch_cmd_future = self.__command_type_service.call_async(
            self.__twist_command_type_req
        )
        switch_cmd_future.add_done_callback(self.__switch_command_type_done_callback)

    def __switch_command_type_done_callback(self, future: Future):
        result: ServoCommandType.Response = future.result()

        if not result.success:
            self._node.get_logger().error(
                "MoveIt Servo could not be switched to TWIST command type."
            )

        self.__is_enabled = result.success

    def __disable_done_callback(self, future: Future):
        result: SetBool.Response = future.result()

        if not result.success:
            self._node.get_logger().error(
                f"MoveIt Servo could not be disabled. ({result.message})"
            )

        self.__is_enabled = not result.success

    @property
    def is_enabled(self) -> bool:
        return self.__is_enabled

    @property
    def frame_id(self) -> str:
        return self.__twist_msg.header.frame_id

    @frame_id.setter
    def frame_id(self, value: str):
        self.__twist_msg.header.frame_id = value

    @property
    def linear_speed(self) -> float:
        return self.__twist_msg.twist.linear.x

    @linear_speed.setter
    def linear_speed(self, value: float):
        self.__twist_msg.twist.linear.x = value
        self.__twist_msg.twist.linear.y = value
        self.__twist_msg.twist.linear.z = value

    @property
    def angular_speed(self) -> float:
        return self.__twist_msg.twist.angular.x

    @angular_speed.setter
    def angular_speed(self, value: float):
        self.__twist_msg.twist.angular.x = value
        self.__twist_msg.twist.angular.y = value
        self.__twist_msg.twist.angular.z = value
