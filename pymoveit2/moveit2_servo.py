import math
import threading
import time
from copy import deepcopy
from typing import Any, Optional, Tuple

from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger

from ._validation import finite_float, finite_vector

try:
    from moveit_msgs.srv import ServoCommandType
except ImportError:
    ServoCommandType = None


def _normalize_namespace(namespace: str) -> str:
    if not isinstance(namespace, str):
        raise ValueError("`namespace` must be a string")
    absolute = namespace.startswith("/")
    parts = [part for part in namespace.split("/") if part]
    if absolute:
        return "/" + "/".join(parts) if parts else "/"
    return "/".join(parts)


class _ServoTransition:
    def __init__(self, kind: str, target: Any, generation: int):
        self.kind = kind
        self.target = target
        self.generation = generation
        self.future: Optional[Future] = None


class MoveIt2Servo:
    """
    Python interface for MoveIt 2 Servo, which enables real-time Cartesian control.

    - Humble uses the legacy ``start_servo``/``stop_servo`` Trigger services.
    - Jazzy and newer use ``pause_servo`` and ``switch_command_type``.
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
        legacy_interface: Optional[bool] = None,
    ):
        """
        Construct a MoveIt 2 Servo interface.
        """

        self._node = node
        self.namespace = _normalize_namespace(namespace)
        if legacy_interface is None:
            legacy_interface = ServoCommandType is None
        elif not legacy_interface and ServoCommandType is None:
            raise ValueError(
                "The current servo interface requires `moveit_msgs/srv/ServoCommandType`, "
                "which is not available in this installation; use `legacy_interface=True`."
            )
        self.__legacy = bool(legacy_interface)

        self.__lock = threading.Lock()
        self.__state_changed = threading.Event()
        self.__closed = False
        self.__remote_state_unknown = False
        self.__desired_enabled = False
        self.__confirmed_enabled = False
        self.__lifecycle_known = False
        self.__is_enabled = False
        self.__desired_command_type: Optional[int] = None
        self.__active_command_type: Optional[int] = None
        self.__transition: Optional[_ServoTransition] = None
        self.__lifecycle_generation = 0
        self.__command_type_generation = 0

        self.__pending_lifecycle_future: Optional[Future] = None
        self.__pending_lifecycle_enable: Optional[bool] = None
        self.__pending_command_type: Optional[int] = None
        self.__pending_command_type_future: Optional[Future] = None

        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.__twist_pub = self._node.create_publisher(
            msg_type=TwistStamped,
            topic=self.__endpoint("servo_node/delta_twist_cmds"),
            qos_profile=qos,
            callback_group=callback_group,
        )
        self.__jog_pub = self._node.create_publisher(
            msg_type=JointJog,
            topic=self.__endpoint("servo_node/delta_joint_cmds"),
            qos_profile=qos,
            callback_group=callback_group,
        )

        self.__command_type_service = None
        if self.__legacy:
            self.__start_service = self._node.create_client(
                srv_type=Trigger,
                srv_name=self.__endpoint("servo_node/start_servo"),
                callback_group=callback_group,
            )
            self.__stop_service = self._node.create_client(
                srv_type=Trigger,
                srv_name=self.__endpoint("servo_node/stop_servo"),
                callback_group=callback_group,
            )
            self.__trigger_req = Trigger.Request()
            self.__enable_service = self.__start_service
            self.__disable_service = self.__stop_service
            self.__enable_req = self.__trigger_req
            self.__disable_req = self.__trigger_req
        else:
            self.__pause_service = self._node.create_client(
                srv_type=SetBool,
                srv_name=self.__endpoint("servo_node/pause_servo"),
                callback_group=callback_group,
            )
            self.__command_type_service = self._node.create_client(
                srv_type=ServoCommandType,
                srv_name=self.__endpoint("servo_node/switch_command_type"),
                callback_group=callback_group,
            )
            self.__unpause_req = SetBool.Request(data=False)
            self.__pause_req = SetBool.Request(data=True)
            self.__enable_service = self.__pause_service
            self.__disable_service = self.__pause_service
            self.__enable_req = self.__unpause_req
            self.__disable_req = self.__pause_req

        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = frame_id
        linear_speed = finite_float(linear_speed, "linear_speed")
        angular_speed = finite_float(angular_speed, "angular_speed")
        self.__twist_msg.twist.linear.x = linear_speed
        self.__twist_msg.twist.linear.y = linear_speed
        self.__twist_msg.twist.linear.z = linear_speed
        self.__twist_msg.twist.angular.x = angular_speed
        self.__twist_msg.twist.angular.y = angular_speed
        self.__twist_msg.twist.angular.z = angular_speed

        if enable_at_init:
            self.enable()

    def __endpoint(self, suffix: str) -> str:
        if self.namespace == "/":
            return "/" + suffix
        if self.namespace:
            return self.namespace + "/" + suffix
        return suffix

    def destroy(self) -> None:
        with self.__lock:
            if self.__closed:
                return
            self.__closed = True
            self.__desired_enabled = False
            self.__desired_command_type = None
            self.__is_enabled = False
            self.__active_command_type = None
            transition = self.__transition
            cleanup_service = None
            cleanup_request = None
            if transition is None and self.__confirmed_enabled:
                cleanup_service = self.__disable_service
                cleanup_request = self.__disable_req
            self.__transition = None
            self.__pending_lifecycle_future = None
            self.__pending_lifecycle_enable = None
            self.__pending_command_type = None
            self.__pending_command_type_future = None
            self.__state_changed.set()

        if cleanup_service is not None:
            try:
                if cleanup_service.service_is_ready():
                    cleanup_service.call_async(cleanup_request)
            except Exception as err:
                self._log("debug", f"Servo best-effort disable failed: {err}")

        for publisher in (self.__twist_pub, self.__jog_pub):
            try:
                self._node.destroy_publisher(publisher)
            except (AttributeError, RuntimeError, TypeError):
                pass
        clients = {
            id(client): client
            for client in (
                self.__enable_service,
                self.__disable_service,
                self.__command_type_service,
            )
            if client is not None
        }
        for client in clients.values():
            try:
                self._node.destroy_client(client)
            except (AttributeError, RuntimeError, TypeError):
                pass

    def __enter__(self) -> "MoveIt2Servo":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.destroy()

    def __del__(self) -> None:
        try:
            if getattr(self, "_MoveIt2Servo__closed", True):
                return
            self.destroy()
        except Exception:
            pass

    def __call__(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> bool:
        return self.servo(linear=linear, angular=angular)

    def servo(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        enable_if_disabled: bool = True,
    ) -> bool:
        """
        Publish a twist only after all required remote state is acknowledged.
        """

        linear_values = finite_vector(linear, "linear", length=3)
        angular_values = finite_vector(angular, "angular", length=3)
        command_type = None
        if not self.__legacy:
            command_type = ServoCommandType.Request.TWIST
        if not self.__ready_to_publish(enable_if_disabled, command_type):
            return False

        with self.__lock:
            if self.__closed or not self.__is_enabled:
                return False
            if command_type is not None and self.__active_command_type != command_type:
                return False
            twist_pub = self.__twist_pub
            twist_msg = deepcopy(self.__twist_msg)
            try:
                twist_msg.header.stamp = self._node.get_clock().now().to_msg()
                twist_msg.twist.linear.x *= linear_values[0]
                twist_msg.twist.linear.y *= linear_values[1]
                twist_msg.twist.linear.z *= linear_values[2]
                twist_msg.twist.angular.x *= angular_values[0]
                twist_msg.twist.angular.y *= angular_values[1]
                twist_msg.twist.angular.z *= angular_values[2]
                twist_pub.publish(twist_msg)
            except (RuntimeError, ValueError, TypeError) as err:
                publish_error = err
            else:
                publish_error = None
        if publish_error is not None:
            self._log("error", f"MoveIt Servo twist publish failed: {publish_error}")
            return False
        return True

    def servo_jog(
        self,
        joint_names: Tuple[str, ...] = tuple(),
        velocities: Tuple[float, ...] = tuple(),
        enable_if_disabled: bool = True,
    ) -> bool:
        """
        Publish finite joint velocities only after acknowledged readiness.
        """

        if isinstance(joint_names, (str, bytes)):
            raise ValueError("`joint_names` must be a sequence of strings")
        try:
            names = tuple(joint_names)
        except TypeError as err:
            raise ValueError("`joint_names` must be a sequence of strings") from err
        velocity_values = finite_vector(velocities, "velocities")
        if len(names) != len(velocity_values):
            raise ValueError(
                "`joint_names` and `velocities` must have the same length!"
            )
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("`joint_names` must contain non-empty strings")
        command_type = None
        if not self.__legacy:
            command_type = ServoCommandType.Request.JOINT_JOG
        if not self.__ready_to_publish(enable_if_disabled, command_type):
            return False

        with self.__lock:
            if self.__closed or not self.__is_enabled:
                return False
            if command_type is not None and self.__active_command_type != command_type:
                return False
            jog_pub = self.__jog_pub
            frame_id = self.__twist_msg.header.frame_id
            try:
                jog_msg = JointJog()
                jog_msg.header.stamp = self._node.get_clock().now().to_msg()
                jog_msg.header.frame_id = frame_id
                jog_msg.joint_names = list(names)
                jog_msg.velocities = list(velocity_values)
                jog_pub.publish(jog_msg)
            except (RuntimeError, ValueError, TypeError) as err:
                publish_error = err
            else:
                publish_error = None
        if publish_error is not None:
            self._log(
                "error", f"MoveIt Servo joint-jog publish failed: {publish_error}"
            )
            return False
        return True

    def enable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Request enable; ``sync=True`` waits for the acknowledgement.
        """

        return self.__request_lifecycle(
            enable=True,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            sync=sync,
        )

    def disable(
        self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False
    ) -> bool:
        """
        Request disable and close command admission immediately.
        """

        return self.__request_lifecycle(
            enable=False,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            sync=sync,
        )

    def wait_until_ready(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait for confirmed enable only.
        On the modern protocol this may return ``True`` while a command-type switch is still pending. Use ``wait_until_command_ready`` when publishing a particular command type must be synchronized explicitly.
        """

        deadline = self.__deadline(timeout_sec)
        while True:
            with self.__lock:
                if (
                    self.__closed
                    or self.__remote_state_unknown
                    or not self.__desired_enabled
                ):
                    return False
                if self.__is_enabled:
                    return True
                transition = self.__transition
                future = (
                    transition.future
                    if transition is not None and transition.kind == "lifecycle"
                    else None
                )
                if transition is None or transition.kind != "lifecycle":
                    return False
            remaining = self.__remaining(deadline)
            if remaining == 0.0:
                return False
            if future is not None:
                self.__wait_future(future, remaining)
            else:
                self.__state_changed.wait(
                    0.01 if remaining is None else min(0.01, remaining)
                )
                self.__state_changed.clear()

    def wait_until_command_ready(
        self, command_type: Optional[int] = None, timeout_sec: Optional[float] = None
    ) -> bool:
        """
        Wait for enable and an acknowledged modern command-type selection.
        """

        if self.__legacy:
            return self.wait_until_ready(timeout_sec=timeout_sec)
        if command_type is None:
            with self.__lock:
                command_type = self.__desired_command_type
        if command_type is None:
            return self.wait_until_ready(timeout_sec=timeout_sec)
        if command_type not in (
            ServoCommandType.Request.TWIST,
            ServoCommandType.Request.JOINT_JOG,
        ):
            raise ValueError(f"Unsupported Servo command type: {command_type}")
        deadline = self.__deadline(timeout_sec)
        while True:
            with self.__lock:
                if self.__closed or self.__remote_state_unknown:
                    return False
                if not self.__is_enabled:
                    if not self.__desired_enabled:
                        return False
                    ready_future = None
                    transition = self.__transition
                    if (
                        transition is not None
                        and transition.kind == "lifecycle"
                        and transition.target is True
                    ):
                        ready_future = transition.future
                elif (
                    self.__active_command_type == command_type
                    and self.__transition is None
                ):
                    return True
                else:
                    self.__desired_command_type = command_type
                    ready_future = (
                        self.__transition.future if self.__transition else None
                    )
            remaining = self.__remaining(deadline)
            if remaining == 0.0:
                return False
            if ready_future is None:
                self.__ensure_transition(0.0, sync=False)
                with self.__lock:
                    ready_future = (
                        self.__transition.future if self.__transition else None
                    )
            remaining = self.__remaining(deadline)
            if remaining == 0.0:
                return False
            if ready_future is not None:
                self.__wait_future(ready_future, remaining)
            else:
                self.__state_changed.wait(
                    0.01 if remaining is None else min(0.01, remaining)
                )
                self.__state_changed.clear()

    def shutdown(self, timeout_sec: float = 1.0) -> bool:
        deadline = self.__deadline(timeout_sec, require_value=True)
        with self.__lock:
            if self.__closed or self.__remote_state_unknown:
                return False
            self.__desired_enabled = False
            self.__desired_command_type = None
            self.__is_enabled = False
            self.__active_command_type = None
            self.__state_changed.set()

        while True:
            with self.__lock:
                if self.__remote_state_unknown or self.__closed:
                    return False
                transition = self.__transition
                if transition is None and not self.__confirmed_enabled:
                    return True
                future = transition.future if transition is not None else None
            remaining = self.__remaining(deadline)
            if remaining == 0.0:
                self.__quarantine(transition)
                return False
            if transition is None:
                if not self.__ensure_transition(remaining, sync=False):
                    self.__quarantine(None)
                    return False
                continue
            if future is None:
                self.__state_changed.wait(min(0.01, remaining))
                self.__state_changed.clear()
                continue
            self.__wait_future(future, remaining)
            if not future.done():
                self.__quarantine(transition)
                return False

    def __request_lifecycle(
        self, enable: bool, wait_for_server_timeout_sec: Optional[float], sync: bool
    ) -> bool:
        if wait_for_server_timeout_sec is not None:
            wait_for_server_timeout_sec = finite_float(
                wait_for_server_timeout_sec,
                "wait_for_server_timeout_sec",
                minimum=0.0,
            )
        with self.__lock:
            if self.__closed or self.__remote_state_unknown:
                return False
            self.__desired_enabled = enable
            if not enable:
                self.__desired_command_type = None
                self.__active_command_type = None
                if (
                    self.__transition is not None
                    and self.__transition.kind == "command_type"
                ):
                    self.__pending_command_type = None
                    self.__pending_command_type_future = None
                self.__is_enabled = False
            else:
                self.__is_enabled = self.__confirmed_enabled and (
                    self.__transition is None or self.__transition.kind != "lifecycle"
                )
            self.__state_changed.set()

        accepted = self.__ensure_transition(wait_for_server_timeout_sec, sync=sync)
        if not sync:
            return accepted
        if not accepted:
            return False
        return self.__wait_for_enabled_target(enable)

    def __ensure_transition(
        self, wait_for_server_timeout_sec: Optional[float], sync: bool
    ) -> bool:
        with self.__lock:
            if self.__closed or self.__remote_state_unknown:
                return False
            if self.__transition is not None:
                return True
            transition = self.__next_transition_locked()
            if transition is None:
                return True
            self.__transition = transition
            self.__set_pending_locked(transition)
            self.__state_changed.set()
        return self.__dispatch_transition(transition, wait_for_server_timeout_sec, sync)

    def __next_transition_locked(self) -> Optional[_ServoTransition]:
        if self.__desired_enabled != self.__confirmed_enabled or (
            not self.__desired_enabled
            and not self.__confirmed_enabled
            and not self.__lifecycle_known
        ):
            self.__lifecycle_generation += 1
            return _ServoTransition(
                "lifecycle", self.__desired_enabled, self.__lifecycle_generation
            )
        if (
            not self.__legacy
            and self.__desired_enabled
            and self.__desired_command_type is not None
            and self.__active_command_type != self.__desired_command_type
        ):
            self.__command_type_generation += 1
            return _ServoTransition(
                "command_type",
                self.__desired_command_type,
                self.__command_type_generation,
            )
        return None

    def __set_pending_locked(self, transition: _ServoTransition) -> None:
        if transition.kind == "lifecycle":
            self.__pending_lifecycle_future = None
            self.__pending_lifecycle_enable = bool(transition.target)
        else:
            self.__pending_command_type = int(transition.target)
            self.__pending_command_type_future = None

    def __clear_pending_locked(self, transition: _ServoTransition) -> None:
        if transition.kind == "lifecycle":
            self.__pending_lifecycle_future = None
            self.__pending_lifecycle_enable = None
        else:
            self.__pending_command_type = None
            self.__pending_command_type_future = None

    def __dispatch_transition(
        self,
        transition: _ServoTransition,
        wait_for_server_timeout_sec: Optional[float],
        sync: bool,
    ) -> bool:
        if transition.kind == "lifecycle":
            client = (
                self.__enable_service if transition.target else self.__disable_service
            )
            request = self.__enable_req if transition.target else self.__disable_req
            verb = "enabled" if transition.target else "disabled"
        else:
            client = self.__command_type_service
            request = ServoCommandType.Request(command_type=transition.target)
            verb = "switched command type"

        try:
            ready = client.wait_for_service(timeout_sec=wait_for_server_timeout_sec)
        except Exception as err:
            self.__transition_failed(
                transition, unknown=False, message=f"could not discover service: {err}"
            )
            return False
        if not ready:
            self.__transition_failed(
                transition,
                unknown=False,
                message=f"Service '{client.srv_name}' is not yet available",
            )
            return False

        if sync:
            try:
                result = client.call(request)
            except Exception as err:
                self.__transition_failed(
                    transition,
                    unknown=True,
                    message=f"could not be {verb}: {type(err).__name__}: {err}",
                )
                return False
            success = self.__response_success(result)
            self.__settle_transition(
                transition,
                success=success,
                unknown=False,
                message=self.__response_message(result),
                remote_settled=True,
            )
            return success

        try:
            future = client.call_async(request)
        except Exception as err:
            self.__transition_failed(
                transition,
                unknown=False,
                message=f"could not be {verb}: {type(err).__name__}: {err}",
            )
            return False
        with self.__lock:
            if self.__closed or self.__transition is not transition:
                return False
            transition.future = future
            if transition.kind == "lifecycle":
                self.__pending_lifecycle_future = future
            else:
                self.__pending_command_type_future = future
            self.__state_changed.set()
        try:
            future.add_done_callback(
                lambda done_future, transition=transition: self.__transition_done_callback(
                    done_future, transition
                )
            )
        except Exception as err:
            self.__transition_failed(
                transition,
                unknown=True,
                message=f"could not track {verb} response: {err}",
            )
            return False
        if future.done():
            self.__transition_done_callback(future, transition)
            with self.__lock:
                if self.__remote_state_unknown:
                    return False
        return True

    def __transition_done_callback(
        self, future: Future, transition: _ServoTransition
    ) -> None:
        with self.__lock:
            if self.__closed or self.__transition is not transition:
                return
        if future.cancelled():
            self.__settle_transition(
                transition,
                success=False,
                unknown=True,
                message="response future was cancelled",
                remote_settled=False,
            )
            return
        try:
            result = future.result()
        except Exception as err:
            self.__settle_transition(
                transition,
                success=False,
                unknown=True,
                message=f"{type(err).__name__}: {err}",
                remote_settled=False,
            )
            return
        self.__settle_transition(
            transition,
            success=self.__response_success(result),
            unknown=False,
            message=self.__response_message(result),
            remote_settled=True,
        )

    def __settle_transition(
        self,
        transition: _ServoTransition,
        success: bool,
        unknown: bool,
        message: str,
        remote_settled: bool = False,
    ) -> Optional[bool]:
        with self.__lock:
            if self.__closed or self.__transition is not transition:
                return None
            self.__transition = None
            self.__clear_pending_locked(transition)
            if unknown:
                self.__remote_state_unknown = True
                self.__confirmed_enabled = False
                self.__is_enabled = False
                self.__active_command_type = None
            elif transition.kind == "lifecycle":
                if remote_settled:
                    self.__lifecycle_known = True
                if success:
                    self.__confirmed_enabled = bool(transition.target)
                elif transition.target:
                    self.__confirmed_enabled = False
                self.__is_enabled = self.__confirmed_enabled and self.__desired_enabled
                if not transition.target:
                    self.__active_command_type = None
            else:
                if (
                    success
                    and self.__confirmed_enabled
                    and self.__desired_enabled
                    and self.__desired_command_type == transition.target
                ):
                    self.__active_command_type = int(transition.target)
                else:
                    self.__active_command_type = None
                self.__is_enabled = self.__confirmed_enabled and self.__desired_enabled
            self.__state_changed.set()
            successor_required = success or (
                not unknown
                and transition.kind == "command_type"
                and (
                    (
                        self.__desired_enabled
                        and self.__desired_command_type != transition.target
                    )
                    or (not self.__desired_enabled and self.__confirmed_enabled)
                )
            )
        if unknown:
            self._log("error", f"MoveIt Servo transition state is unknown: {message}")
        elif not success:
            self._log("error", f"MoveIt Servo transition failed: {message}")
        if successor_required:
            self.__ensure_transition(0.0, sync=False)
        return success

    def __transition_failed(
        self, transition: _ServoTransition, unknown: bool, message: str
    ) -> None:
        self.__settle_transition(
            transition,
            success=False,
            unknown=unknown,
            message=message,
            remote_settled=False,
        )

    def __ready_to_publish(
        self, enable_if_disabled: bool, command_type: Optional[int]
    ) -> bool:
        with self.__lock:
            if self.__closed or self.__remote_state_unknown:
                return False
            enabled = self.__is_enabled
            desired_enabled = self.__desired_enabled
            transition = self.__transition
        if not enabled:
            if not enable_if_disabled:
                self._log(
                    "warning", "Command refused because MoveIt Servo is not enabled"
                )
                return False
            if (
                not desired_enabled
                or transition is None
                or transition.kind != "lifecycle"
            ):
                self._log(
                    "warning",
                    "Command dropped because MoveIt Servo is not enabled; requesting enable",
                )
                self.enable()
            return False
        if command_type is None:
            return True
        return self.__ensure_command_type(command_type)

    def __ensure_command_type(self, command_type: int) -> bool:
        with self.__lock:
            if (
                self.__closed
                or self.__remote_state_unknown
                or not self.__is_enabled
                or not self.__desired_enabled
            ):
                return False
            self.__desired_command_type = command_type
            if self.__active_command_type == command_type and self.__transition is None:
                return True
            if self.__transition is not None:
                return False
            self.__state_changed.set()
        self.__ensure_transition(0.0, sync=False)
        return False

    def __wait_for_enabled_target(self, enable: bool) -> bool:
        while True:
            with self.__lock:
                if self.__closed or self.__remote_state_unknown:
                    return False
                if self.__desired_enabled != enable:
                    return False
                if enable and self.__is_enabled and self.__desired_enabled:
                    return True
                if (
                    not enable
                    and not self.__confirmed_enabled
                    and self.__transition is None
                ):
                    return True
                transition = self.__transition
                future = transition.future if transition is not None else None
            if future is not None:
                self.__wait_future(future, None)
            else:
                self.__state_changed.wait(0.01)
                self.__state_changed.clear()

    @staticmethod
    def __wait_future(future: Future, timeout_sec: Optional[float]) -> bool:
        event = threading.Event()
        try:
            future.add_done_callback(lambda _: event.set())
        except Exception:
            return False
        if future.done():
            event.set()
        event.wait(timeout=timeout_sec)
        return future.done()

    @staticmethod
    def __response_success(result: Any) -> bool:
        return result is not None and bool(getattr(result, "success", False))

    @staticmethod
    def __response_message(result: Any) -> str:
        return str(getattr(result, "message", "no response"))

    @staticmethod
    def __deadline(
        timeout_sec: Optional[float], require_value: bool = False
    ) -> Optional[float]:
        if timeout_sec is None:
            if require_value:
                raise ValueError("`timeout_sec` must be finite and non-negative")
            return None
        try:
            timeout = float(timeout_sec)
        except (TypeError, ValueError, OverflowError) as err:
            raise ValueError("`timeout_sec` must be finite and non-negative") from err
        if not math.isfinite(timeout) or timeout < 0.0:
            raise ValueError("`timeout_sec` must be finite and non-negative")
        return time.monotonic() + timeout

    @staticmethod
    def __remaining(deadline: Optional[float]) -> Optional[float]:
        if deadline is None:
            return None
        return max(0.0, deadline - time.monotonic())

    def __quarantine(self, transition: Optional[_ServoTransition]) -> None:
        with self.__lock:
            if self.__closed:
                return
            if transition is not None and self.__transition is not transition:
                return
            self.__remote_state_unknown = True
            self.__transition = None
            if transition is not None:
                self.__clear_pending_locked(transition)
            self.__confirmed_enabled = False
            self.__is_enabled = False
            self.__active_command_type = None
            self.__state_changed.set()
        self._log("error", "MoveIt Servo transition timed out; remote state is unknown")

    def _log(self, level: str, message: str) -> None:
        with self.__lock:
            if self.__closed:
                return
        try:
            getattr(self._node.get_logger(), level)(message)
        except Exception:
            pass

    @property
    def uses_legacy_interface(self) -> bool:
        return self.__legacy

    @property
    def is_enabled(self) -> bool:
        with self.__lock:
            return self.__is_enabled

    @property
    def active_command_type(self) -> Optional[int]:
        with self.__lock:
            return self.__active_command_type

    @property
    def remote_state_unknown(self) -> bool:
        with self.__lock:
            return self.__remote_state_unknown

    @property
    def is_closed(self) -> bool:
        with self.__lock:
            return self.__closed

    @property
    def frame_id(self) -> str:
        with self.__lock:
            return self.__twist_msg.header.frame_id

    @frame_id.setter
    def frame_id(self, value: str) -> None:
        with self.__lock:
            self.__twist_msg.header.frame_id = value

    @property
    def linear_speed(self) -> float:
        with self.__lock:
            return self.__twist_msg.twist.linear.x

    @linear_speed.setter
    def linear_speed(self, value: float) -> None:
        value = finite_float(value, "linear_speed")
        with self.__lock:
            self.__twist_msg.twist.linear.x = value
            self.__twist_msg.twist.linear.y = value
            self.__twist_msg.twist.linear.z = value

    @property
    def angular_speed(self) -> float:
        with self.__lock:
            return self.__twist_msg.twist.angular.x

    @angular_speed.setter
    def angular_speed(self, value: float) -> None:
        value = finite_float(value, "angular_speed")
        with self.__lock:
            self.__twist_msg.twist.angular.x = value
            self.__twist_msg.twist.angular.y = value
            self.__twist_msg.twist.angular.z = value
