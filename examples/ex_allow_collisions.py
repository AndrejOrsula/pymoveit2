#!/usr/bin/env python3
"""
Allow or forbid collisions between the robot and one object.
- ros2 run pymoveit2 ex_allow_collisions.py --ros-args -p id:="sphere" -p allow:=true
- ros2 run pymoveit2 ex_allow_collisions.py --ros-args -p id:="sphere" -p allow:=false
"""

import sys
import time
from threading import Thread

import rclpy
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
        node = Node("ex_allow_collisions")

        # Object to allow or forbid collisions with
        node.declare_parameter(
            "id",
            "box",
        )
        node.declare_parameter(
            "allow",
            True,
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
        object_id = node.get_parameter("id").get_parameter_value().string_value
        allow = node.get_parameter("allow").get_parameter_value().bool_value
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )

        # (Dis)allow collisions
        deadline = time.monotonic() + timeout_sec
        future = moveit2.allow_collisions(
            object_id,
            allow,
            timeout_sec=max(0.0, deadline - time.monotonic()),
        )
        success = False
        if future is None:
            node.get_logger().error("Failed to submit the planning scene update")
        elif _wait_for_future(future, deadline - time.monotonic()):
            success = bool(moveit2.process_allow_collision_future(future))
        else:
            node.get_logger().error(
                "Timed out while waiting for the planning scene update"
            )
        if success:
            node.get_logger().info(
                f"{'Allow' if allow else 'Disallow'}ed collisions between all objects and '{object_id}' successfully"
            )
            status = 0
        else:
            node.get_logger().error(
                f"Failed to {'allow' if allow else 'disallow'} collisions between all objects and '{object_id}'"
            )
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Collision permission example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Collision permission example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "collision",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
