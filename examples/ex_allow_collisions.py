#!/usr/bin/env python3
"""
Example of (dis)allowing collisions between the robot object and an object with
the specified ID.
- ros2 run pymoveit2 ex_allow_collisions.py --ros-args -p id:="sphere" -p allow:=true
- ros2 run pymoveit2 ex_allow_collisions.py --ros-args -p id:="sphere" -p allow:=false
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_allow_collisions")

    # Declare parameter for joint positions
    node.declare_parameter(
        "id",
        "box",
    )
    node.declare_parameter(
        "allow",
        True,
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    object_id = node.get_parameter("id").get_parameter_value().string_value
    allow = node.get_parameter("allow").get_parameter_value().bool_value

    # (Dis)allow collisions
    moveit2.allow_collisions(object_id, allow)
    node.get_logger().info(
        f"{'Allow' if allow else 'Disallow'}ed collisions between all objects and '{object_id}'"
    )

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
