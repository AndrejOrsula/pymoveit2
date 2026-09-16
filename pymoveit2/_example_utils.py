"""Helpers shared by the examples."""

import sys
import time
from threading import Event, Thread
from typing import Any, Callable, Optional


def wait_for_future(future: Any, timeout_sec: float) -> bool:
    if future.done():
        return True
    done = Event()
    future.add_done_callback(lambda _: done.set())
    return future.done() or done.wait(timeout=max(0.0, timeout_sec))


def wait_for_motion(
    moveit2: Any,
    cancel_after_secs: float,
    timeout_sec: float,
    node: Any,
    idle_state: Any,
    *,
    deadline: Optional[float] = None,
) -> bool:
    deadline = (
        time.monotonic() + max(0.0, timeout_sec) if deadline is None else deadline
    )
    if cancel_after_secs > 0.0:
        cancel_deadline = min(deadline, time.monotonic() + max(0.0, cancel_after_secs))
        if moveit2.wait_until_executed(
            timeout_sec=max(0.0, cancel_deadline - time.monotonic())
        ):
            return True
        if moveit2.query_state() == idle_state:
            node.get_logger().error("Motion failed before cancellation")
            return False
        node.get_logger().info("Cancelling motion")
        if not moveit2.cancel_execution():
            node.get_logger().error("Failed to submit motion cancellation")
            return False

    success = moveit2.wait_until_executed(
        timeout_sec=max(0.0, deadline - time.monotonic())
    )
    if not success:
        node.get_logger().error("Motion did not complete successfully")
    return success


def end_effector_pose(
    moveit2: Any, joint_positions: Any, timeout_sec: float, node: Any
) -> Optional[Any]:
    pose = moveit2.compute_fk(joint_positions, timeout_sec=max(0.0, timeout_sec))
    if isinstance(pose, list):
        pose = pose[0] if pose else None
    if pose is None:
        node.get_logger().error("Failed to compute the pose of the end effector")
    return pose


def complete_pose(
    moveit2: Any,
    joint_positions: Any,
    timeout_sec: float,
    node: Any,
    position: Optional[Any] = None,
    quat_xyzw: Optional[Any] = None,
) -> tuple:
    if position is not None and quat_xyzw is not None:
        return list(position), list(quat_xyzw)
    pose = end_effector_pose(moveit2, joint_positions, timeout_sec, node)
    if pose is None:
        raise RuntimeError("Pass `position` and `quat_xyzw` instead")
    point, orientation = pose.pose.position, pose.pose.orientation
    return (
        list(position) if position is not None else [point.x, point.y, point.z],
        (
            list(quat_xyzw)
            if quat_xyzw is not None
            else [orientation.x, orientation.y, orientation.z, orientation.w]
        ),
    )


def _shutdown_executor(executor: Any, label: str) -> tuple[bool, bool]:
    try:
        shutdown_result = executor.shutdown(timeout_sec=1.0)
    except TypeError:
        try:
            shutdown_result = executor.shutdown()
        except Exception as error:
            print(
                f"Failed to stop {label} executor: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
            return True, False
    except Exception as error:
        print(
            f"Failed to stop {label} executor: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return True, False
    if shutdown_result is False:
        print(f"Failed to stop {label} executor", file=sys.stderr)
        return True, False
    return False, True


def _join_executor_thread(thread: Thread, label: str) -> tuple[bool, bool]:
    try:
        thread.join(timeout=2.0)
        if thread.is_alive():
            print(
                f"{label.capitalize()} executor thread did not stop",
                file=sys.stderr,
            )
            return True, False
    except Exception as error:
        print(
            f"Failed to join {label} executor thread: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return True, False
    return False, True


def _destroy_interface(interface: Any, label: str) -> bool:
    try:
        interface.destroy()
    except Exception as error:
        print(
            f"Failed to destroy {label} interface: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return True
    return False


def _destroy_node(node: Any, label: str) -> bool:
    try:
        destroy_result = node.destroy_node()
    except Exception as error:
        print(
            f"Failed to destroy {label} node: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return True
    if destroy_result is False:
        print(f"Failed to destroy {label} node", file=sys.stderr)
        return True
    return False


def cleanup(
    interface: Any,
    executor: Any,
    executor_thread: Optional[Thread],
    label: str,
    *,
    node: Any = None,
    ros_ok: Callable[[], bool],
    ros_shutdown: Callable[[], None],
    acknowledged_shutdown: Optional[Callable[[float], bool]] = None,
    shutdown_timeout_sec: float = 1.0,
) -> bool:
    failed = False
    if interface is not None and acknowledged_shutdown is not None:
        try:
            if not acknowledged_shutdown(timeout_sec=shutdown_timeout_sec):
                failed = True
                print(f"{label} shutdown was not acknowledged", file=sys.stderr)
        except Exception as error:
            failed = True
            print(
                f"Failed to shut down {label}: {type(error).__name__}: {error}",
                file=sys.stderr,
            )

    executor_quiescent = executor is None
    if executor is not None:
        shutdown_failed, executor_quiescent = _shutdown_executor(executor, label)
        failed = shutdown_failed or failed

    thread_quiescent = executor_thread is None
    if executor_thread is not None:
        join_failed, thread_quiescent = _join_executor_thread(executor_thread, label)
        failed = join_failed or failed

    if executor_quiescent and thread_quiescent:
        if interface is not None:
            failed = _destroy_interface(interface, label) or failed
        if node is not None:
            failed = _destroy_node(node, label) or failed
    else:
        failed = True
        print(
            f"Skipped destroying {label} resources while executor quiescence "
            "was not confirmed",
            file=sys.stderr,
        )

    try:
        if ros_ok():
            ros_shutdown()
    except Exception as error:
        failed = True
        print(
            f"Failed to shut down ROS: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
    return failed


DEFAULT_DESCRIPTION_NODE_NAME: str = "move_group"
DEFAULT_DESCRIPTION_NODE_PARAMETER: str = "robot_description_node"
DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER: str = "robot_description_timeout_sec"
DEFAULT_DESCRIPTION_TIMEOUT_SEC: float = 10.0

_ARM_PARAMETERS: tuple = (
    ("group_name", "str"),
    ("joint_names", "str[]"),
    ("base_link_name", "str"),
    ("end_effector_name", "str"),
)
_GRIPPER_PARAMETERS: tuple = (
    ("gripper_group_name", "str"),
    ("gripper_joint_names", "str[]"),
    ("open_gripper_joint_positions", "float[]"),
    ("closed_gripper_joint_positions", "float[]"),
)


def _parameter_type(kind: str) -> Any:
    from rclpy.parameter import Parameter

    return {
        "str": Parameter.Type.STRING,
        "str[]": Parameter.Type.STRING_ARRAY,
        "float[]": Parameter.Type.DOUBLE_ARRAY,
    }[kind]


def parameter_value(node: Any, name: str) -> Optional[Any]:
    try:
        value = node.get_parameter(name).value
    except Exception:
        return None
    if value is None or (isinstance(value, (list, tuple)) and len(value) == 0):
        return None
    if isinstance(value, str) and not value:
        return None
    return value


def declare_robot_parameters(
    node: Any,
    *,
    arm: bool = True,
    gripper: bool = False,
    frame_id: bool = False,
) -> None:
    node.declare_parameter(
        DEFAULT_DESCRIPTION_NODE_PARAMETER, DEFAULT_DESCRIPTION_NODE_NAME
    )
    node.declare_parameter(
        DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER, DEFAULT_DESCRIPTION_TIMEOUT_SEC
    )
    declared: list = []
    if arm:
        declared.extend(_ARM_PARAMETERS)
    if gripper:
        declared.extend(_GRIPPER_PARAMETERS)
    if frame_id:
        declared.append(("frame_id", "str"))
    for name, kind in declared:
        node.declare_parameter(name, _parameter_type(kind))


class RobotConfiguration:
    def __init__(self, node: Any, callback_group: Any = None) -> None:
        self._node = node
        self._callback_group = callback_group
        self._description: Optional[Any] = None

    @property
    def description(self) -> Any:
        if self._description is None:
            from pymoveit2.robot_description import RobotDescription

            remote_node = (
                parameter_value(self._node, DEFAULT_DESCRIPTION_NODE_PARAMETER)
                or DEFAULT_DESCRIPTION_NODE_NAME
            )
            timeout_sec = parameter_value(
                self._node, DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER
            )
            self._node.get_logger().info(
                f"Discovering the robot configuration from '{remote_node}'"
            )
            try:
                self._description = RobotDescription.from_node(
                    self._node,
                    remote_node_name=remote_node,
                    timeout_sec=(
                        DEFAULT_DESCRIPTION_TIMEOUT_SEC
                        if timeout_sec is None
                        else float(timeout_sec)
                    ),
                    callback_group=self._callback_group,
                )
            except Exception as error:
                raise RuntimeError(
                    f"Unable to read the robot description from '{remote_node}': "
                    f"{type(error).__name__}: {error} Start MoveIt 2 first, or set "
                    "`robot_description_node` to the node that has the URDF and SRDF."
                ) from error
            self._node.get_logger().info(
                f"Discovered '{self._description.name}' with groups "
                f"{self._description.group_names}"
            )
        return self._description

    def _resolve(self, parameters: tuple, discover: Callable[[Optional[str]], dict]):
        overrides = {name: parameter_value(self._node, name) for name, _ in parameters}
        if all(value is not None for value in overrides.values()):
            return {
                key: list(value) if isinstance(value, (list, tuple)) else value
                for key, value in overrides.items()
            }
        group_name = overrides[parameters[0][0]]
        resolved = dict(discover(group_name))
        for name, value in overrides.items():
            if value is not None:
                resolved[name] = (
                    list(value) if isinstance(value, (list, tuple)) else value
                )
        return resolved

    def moveit2_kwargs(self) -> dict:
        return self._resolve(
            _ARM_PARAMETERS,
            lambda group_name: self.description.moveit2_kwargs(group_name),
        )

    def joint_positions(
        self,
        parameter_name: str = "joint_positions",
        state_name: Optional[str] = None,
        default: Optional[Any] = None,
    ) -> list:
        override = parameter_value(self._node, parameter_name)
        if override is not None:
            return [float(value) for value in override]
        if default is not None:
            return [float(value) for value in default]
        group_name = str(self.moveit2_kwargs()["group_name"])
        try:
            positions = self.description.joint_positions(state_name, group_name)
        except ValueError as error:
            raise ValueError(f"{error} Pass `{parameter_name}` instead.") from error
        return [float(value) for value in positions]

    def gripper_kwargs(self) -> dict:
        return self._resolve(
            _GRIPPER_PARAMETERS,
            lambda group_name: self.description.moveit2_gripper_kwargs(group_name),
        )

    def frame_id(self) -> str:
        override = parameter_value(self._node, "frame_id")
        if override is not None:
            return str(override)
        return str(self.moveit2_kwargs()["base_link_name"])
