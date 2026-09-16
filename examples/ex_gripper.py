#!/usr/bin/env python3
"""
Open, close or toggle the gripper.
- ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"   # or "open", "close"
- ros2 run pymoveit2 ex_gripper.py --ros-args -p gripper_command_action_name:="gripper_controller/gripper_cmd"
- ros2 run pymoveit2 ex_gripper.py --ros-args -p gripper_joint_names:="[finger_joint]" -p open_gripper_joint_positions:="[0.0]" -p closed_gripper_joint_positions:="[0.8]" -p gripper_group_name:="gripper"
"""

import sys
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import GripperInterface
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
)


def _run_finite_action(gripper_interface, action: str, deadline: float, node) -> bool:
    submitted = (
        gripper_interface.open() if action == "open" else gripper_interface.close()
    )
    if not submitted:
        node.get_logger().error(f"Failed to submit gripper {action} action")
        return False
    success = gripper_interface.wait_until_executed(
        timeout_sec=max(0.0, deadline - time.monotonic())
    )
    if not success:
        node.get_logger().error(f"Gripper {action} action failed")
    return success


def _run_toggle(gripper_interface, timeout_sec: float, node):
    completed = False
    try:
        while rclpy.ok():
            if not gripper_interface.toggle():
                node.get_logger().error("Failed to submit gripper toggle action")
                return False, False
            if not gripper_interface.wait_until_executed(timeout_sec=timeout_sec):
                node.get_logger().error("Gripper toggle action failed")
                return False, False
            completed = True
            time.sleep(1.0)
    except KeyboardInterrupt:
        return completed, True
    return completed and not rclpy.ok(), False


def main() -> int:
    rclpy.init()
    node = None
    gripper_interface = None
    executor = None
    executor_thread = None
    status = 1
    action = ""
    completed_toggle = False

    try:
        # Node for this example
        node = Node("ex_gripper")

        # Declare parameter for gripper action
        node.declare_parameter(
            "action",
            "toggle",
        )

        # The gripper joints and their open/closed positions come from the SRDF group
        # states of `move_group`. Every value can still be set as a ROS parameter.
        node.declare_parameter(
            "gripper_command_action_name",
            "gripper_action_controller/gripper_cmd",
        )
        declare_robot_parameters(node, arm=False, gripper=True)
        # Seconds to wait for each gripper motion and for the first joint state
        node.declare_parameter("timeout_sec", 10.0)

        # Let callbacks run in parallel, so a blocking call does not stall its own reply
        callback_group = ReentrantCallbackGroup()

        action = node.get_parameter("action").get_parameter_value().string_value
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )

        # Spin the node in the background. Discovery below needs it.
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Build the gripper interface
        robot = RobotConfiguration(node, callback_group=callback_group)
        gripper_interface = GripperInterface(
            node=node,
            callback_group=callback_group,
            gripper_command_action_name=node.get_parameter(
                "gripper_command_action_name"
            )
            .get_parameter_value()
            .string_value,
            discovery_timeout_sec=min(1.0, timeout_sec),
            **robot.gripper_kwargs(),
        )

        # Without a joint state, open/closed is unknown
        deadline = time.monotonic() + timeout_sec
        if gripper_interface.backend is None:
            raise RuntimeError("no gripper action server is available")
        while (
            rclpy.ok()
            and gripper_interface.joint_state is None
            and time.monotonic() < deadline
        ):
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        if gripper_interface.joint_state is None:
            node.get_logger().error(
                "Timed out waiting for the first gripper joint state"
            )
            raise RuntimeError("gripper joint state is unavailable")

        # Run the action. Toggle keeps going until Ctrl-C and stops on the first failure.
        node.get_logger().info(f'Performing gripper action "{action}"')
        if action in ("open", "close"):
            status = int(
                not _run_finite_action(gripper_interface, action, deadline, node)
            )
        elif action == "toggle":
            completed_toggle, interrupted = _run_toggle(
                gripper_interface, timeout_sec, node
            )
            status = int(not completed_toggle)
            if interrupted and completed_toggle:
                status = 0
        else:
            node.get_logger().error(
                f"Unknown gripper action '{action}'; expected open, close, or toggle"
            )
    except KeyboardInterrupt:
        # Ctrl-C during setup or a finite action counts as a failure.
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Gripper example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Gripper example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            gripper_interface,
            executor,
            executor_thread,
            "gripper",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
