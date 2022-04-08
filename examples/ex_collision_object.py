#!/usr/bin/env python3
"""
Example of adding a collision object
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p object_mesh_file:="/home/user/test.stl"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


def main(args=None):

    rclpy.init(args=args)

    # Create node for this example
    node = Node("ex_collision_object")
    # Declare parameter for joint positions
    node.declare_parameter(
        "object_mesh_file",
        "test.stl",
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

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    filename = node.get_parameter("object_mesh_file").get_parameter_value().string_value

    moveit2.add_collision_mesh(
        filename=filename,
        id=filename,
        position=[0,0,0],
        quat_xyzw=[0,0,0,0])

    # moveit2.remove_collision_mesh(id=filename)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
