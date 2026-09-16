import threading
import time
from typing import Any, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2.moveit2 import MoveIt2
from pymoveit2.robot_description import DEFAULT_DESCRIPTION_NODE_NAME, RobotDescription

DEFAULT_NODE_NAME = "pymoveit2"
DEFAULT_TIMEOUT_SEC = 10.0


class RobotSession:
    """
    Self-contained session for controlling a robot: node, executor, description and interfaces
    """

    def __init__(
        self,
        node_name: str = DEFAULT_NODE_NAME,
        *,
        namespace: Optional[str] = None,
        group_name: Optional[str] = None,
        description_node: str = DEFAULT_DESCRIPTION_NODE_NAME,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        num_threads: int = 2,
        args: Optional[List[str]] = None,
    ):
        self._owns_rclpy = False
        self._closed = False
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._arm: Optional[MoveIt2] = None
        self._gripper: Optional[Any] = None
        self._servo: Optional[Any] = None

        try:
            if not rclpy.ok():
                rclpy.init(args=args)
                self._owns_rclpy = True
            self._node = rclpy.create_node(
                node_name, **({"namespace": namespace} if namespace else {})
            )
            self._callback_group = ReentrantCallbackGroup()

            self._executor = MultiThreadedExecutor(num_threads)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()

            try:
                self._description = RobotDescription.from_node(
                    self._node,
                    remote_node_name=description_node,
                    timeout_sec=timeout_sec,
                    callback_group=self._callback_group,
                )
            except Exception as error:
                raise RuntimeError(
                    f"Unable to read the robot description from '{description_node}':"
                    f" {type(error).__name__}: {error} Start MoveIt 2 first, or pass"
                    " `description_node` naming the node that has the URDF and SRDF."
                ) from error

            self._group_name = group_name
            self._arm = MoveIt2(
                node=self._node,
                callback_group=self._callback_group,
                **self._description.moveit2_kwargs(group_name),
            )
        except Exception:
            self.close()
            raise

    @property
    def node(self) -> Node:
        return self._require(self._node, "node")

    @property
    def executor(self) -> MultiThreadedExecutor:
        return self._require(self._executor, "executor")

    @property
    def description(self) -> RobotDescription:
        return self._description

    @property
    def arm(self) -> MoveIt2:
        return self._require(self._arm, "arm")

    @property
    def gripper(self) -> Any:
        if self._gripper is None:
            from pymoveit2.gripper_interface import GripperInterface

            self._gripper = GripperInterface(
                node=self.node,
                callback_group=self._callback_group,
                **self._description.moveit2_gripper_kwargs(),
            )
        return self._gripper

    @property
    def servo(self) -> Any:
        if self._servo is None:
            from pymoveit2.moveit2_servo import MoveIt2Servo

            self._servo = MoveIt2Servo(
                node=self.node,
                frame_id=str(
                    self._description.moveit2_kwargs(self._group_name)["base_link_name"]
                ),
                callback_group=self._callback_group,
            )
        return self._servo

    def move_to_pose(
        self,
        position: Any = None,
        quat_xyzw: Any = None,
        *,
        wait: bool = True,
        timeout_sec: Optional[float] = None,
        **kwargs: Any,
    ) -> bool:
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        submitted = self.arm.move_to_pose(
            position=position, quat_xyzw=quat_xyzw, timeout_sec=timeout_sec, **kwargs
        )
        return self._settle(submitted, wait, deadline)

    def move_to_configuration(
        self,
        joint_positions: Optional[List[float]] = None,
        *,
        wait: bool = True,
        timeout_sec: Optional[float] = None,
        **kwargs: Any,
    ) -> bool:
        if joint_positions is None:
            joint_positions = self.joint_positions()
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        submitted = self.arm.move_to_configuration(
            joint_positions, timeout_sec=timeout_sec, **kwargs
        )
        return self._settle(submitted, wait, deadline)

    def joint_positions(self, state_name: Optional[str] = None) -> List[float]:
        return list(
            self._description.joint_positions(state_name, str(self.arm.group_name))
        )

    def last_failure(self) -> Optional[str]:
        return self.arm.last_failure()

    def _settle(self, submitted: bool, wait: bool, deadline: Optional[float]) -> bool:
        if not submitted or not wait:
            return submitted
        remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
        return self.arm.wait_until_executed(timeout_sec=remaining)

    def close(self) -> bool:
        if self._closed:
            return True
        self._closed = True
        clean = self._disable_servo()
        quiescent = self._stop_executor()
        if quiescent:
            clean = self._destroy_entities() and clean
        else:
            clean = False
            self._warn("Left ROS entities intact; the executor did not stop.")
        return self._stop_ros() and clean

    def _disable_servo(self) -> bool:
        servo = self._servo
        if servo is None:
            return True
        try:
            return bool(servo.shutdown(timeout_sec=1.0))
        except Exception as error:
            self._warn(f"Servo shutdown failed: {type(error).__name__}: {error}")
            return False

    def _stop_executor(self) -> bool:
        quiescent = True
        if self._executor is not None:
            try:
                if self._executor.shutdown(timeout_sec=1.0) is False:
                    quiescent = False
            except Exception as error:
                quiescent = False
                self._warn(f"Executor shutdown failed: {type(error).__name__}: {error}")
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                quiescent = False
                self._warn("The executor thread did not stop.")
        return quiescent

    def _destroy_entities(self) -> bool:
        clean = True
        owners = [self._servo, self._gripper, self._arm]
        for owner in owners:
            if owner is None:
                continue
            try:
                owner.destroy()
            except Exception as error:
                clean = False
                self._warn(f"Cleanup failed: {type(error).__name__}: {error}")
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception as error:
                clean = False
                self._warn(f"Cleanup failed: {type(error).__name__}: {error}")
        return clean

    def _stop_ros(self) -> bool:
        if not self._owns_rclpy:
            return True
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            self._warn(f"ROS shutdown failed: {type(error).__name__}: {error}")
            return False
        return True

    def __enter__(self) -> "RobotSession":
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)
        arm = self.__dict__.get("_arm")
        if arm is None:
            raise AttributeError(name)
        try:
            return getattr(arm, name)
        except AttributeError:
            raise AttributeError(
                f"Neither `RobotSession` nor `MoveIt2` has '{name}'."
            ) from None

    def _require(self, value: Any, what: str) -> Any:
        if value is None:
            raise RuntimeError(f"This session has no {what}; it is closed or failed.")
        return value

    def _warn(self, message: str) -> None:
        if self._node is not None:
            self._node.get_logger().warning(message)
        else:
            print(message)


def connect(
    node_name: str = DEFAULT_NODE_NAME,
    *,
    namespace: Optional[str] = None,
    group_name: Optional[str] = None,
    description_node: str = DEFAULT_DESCRIPTION_NODE_NAME,
    timeout_sec: float = DEFAULT_TIMEOUT_SEC,
    num_threads: int = 2,
    args: Optional[List[str]] = None,
) -> RobotSession:
    return RobotSession(
        node_name,
        namespace=namespace,
        group_name=group_name,
        description_node=description_node,
        timeout_sec=timeout_sec,
        num_threads=num_threads,
        args=args,
    )
