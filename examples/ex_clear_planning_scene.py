#!/usr/bin/env python3
"""
Example of clearing the planning scene.
- ros2 run pymoveit2 ex_clear_planning_scene.py"
- ros2 run pymoveit2 ex_clear_planning_scene.py --ros-args -p cancel_after:=0.0"
"""

from os import path
from threading import Thread

import rclpy
import trimesh
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda as robot


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_clear_planning_scene")

    # Declare parameter for joint positions
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

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    cancel_after = node.get_parameter("cancel_after").get_parameter_value().double_value

    # Clear planning scene
    future = moveit2.clear_all_collision_objects()
    start_time = node.get_clock().now()
    if future is None:
        node.get_logger().error("Failed to clear planning scene")
    else:
        rate = node.create_rate(10)
        while rclpy.ok() and not future.done():
            if (
                cancel_after >= 0.0
                and (node.get_clock().now() - start_time).nanoseconds / 1e9
                >= cancel_after
            ):
                moveit2.cancel_clear_all_collision_objects_future(future)
                node.get_logger().info("Cancelled clear planning scene service call")
                break
            rate.sleep()
        if future.cancelled():
            node.get_logger().info("Cancelled clear planning scene service call")
        if future.done():
            success = moveit2.process_clear_all_collision_objects_future(future)
            if success:
                node.get_logger().info("Successfully cleared planning scene")
            else:
                node.get_logger().error("Failed to clear planning scene")

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
