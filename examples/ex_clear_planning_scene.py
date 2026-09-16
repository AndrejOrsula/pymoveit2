#!/usr/bin/env python3
"""
Remove everything from the planning scene.
- ros2 run pymoveit2 ex_clear_planning_scene.py
- ros2 run pymoveit2 ex_clear_planning_scene.py --ros-args -p cancel_after:=0.0
"""

import sys
import time
from threading import Thread

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
    wait_for_future,
)

_wait_for_future = wait_for_future


def main() -> int:
    rclpy.init()
    node = None
    moveit2 = None
    executor = None
    executor_thread = None
    status = 1

    try:
        # Node for this example
        node = Node("ex_clear_planning_scene")

        node.declare_parameter(
            "cancel_after",
            -1.0,
            ParameterDescriptor(
                name="cancel_after",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "The number of seconds after which the service call to clear the "
                    "planning scene should be cancelled. If negative (default), don't cancel."
                ),
                read_only=True,
            ),
        )
        # Seconds to wait for the planning-scene update to be confirmed
        node.declare_parameter("timeout_sec", 5.0)

        # Let callbacks run in parallel, so a blocking call does not stall its own reply
        callback_group = ReentrantCallbackGroup()

        # The robot configuration comes from the URDF and SRDF that `move_group` is
        # running with. Every value can still be set as a ROS parameter.
        declare_robot_parameters(node)

        # Spin the node in the background. Discovery below needs it.
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Build the interface from what was discovered
        robot = RobotConfiguration(node, callback_group=callback_group)
        moveit2 = MoveIt2(
            node=node,
            callback_group=callback_group,
            **robot.moveit2_kwargs(),
        )

        # Get parameters
        cancel_after = (
            node.get_parameter("cancel_after").get_parameter_value().double_value
        )
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )

        # Clear planning scene
        deadline = time.monotonic() + timeout_sec
        future = moveit2.clear_all_collision_objects(
            timeout_sec=max(0.0, deadline - time.monotonic())
        )
        success = False
        if future is None:
            node.get_logger().error("Failed to submit clear planning scene request")
        else:
            wait_deadline = deadline
            if cancel_after >= 0.0:
                wait_deadline = min(deadline, time.monotonic() + cancel_after)
            if _wait_for_future(future, wait_deadline - time.monotonic()):
                success = bool(
                    moveit2.process_clear_all_collision_objects_future(future)
                )
            elif future.done():
                success = bool(
                    moveit2.process_clear_all_collision_objects_future(future)
                )
            elif cancel_after >= 0.0 and time.monotonic() >= wait_deadline:
                moveit2.cancel_clear_all_collision_objects_future(future)
                node.get_logger().info("Cancelled clear planning scene service call")
            else:
                node.get_logger().error("Timed out while clearing the planning scene")
            if success:
                node.get_logger().info("Successfully cleared planning scene")
            elif not (cancel_after >= 0.0 and time.monotonic() >= wait_deadline):
                node.get_logger().error("Failed to clear planning scene")
        status = int(not success)
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Clear planning-scene example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Clear planning-scene example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "planning-scene",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
