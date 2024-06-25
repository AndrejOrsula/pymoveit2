#!/usr/bin/env python3
"""
Example of computing Forward Kinematics.
- ros2 run pymoveit2 ex_fk.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
- ros2 run pymoveit2 ex_fk.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]" -p synchronous:=False
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import panda as robot


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_fk")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            -0.7853981633974483,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ],
    )
    node.declare_parameter("synchronous", True)

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
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value

    # Move to joint configuration
    node.get_logger().info(
        f"Computing FK for {{joint_positions: {list(joint_positions)}}}"
    )
    retval = None
    if synchronous:
        retval = moveit2.compute_fk(joint_positions)
    else:
        future = moveit2.compute_fk_async(joint_positions)
        if future is not None:
            rate = node.create_rate(10)
            while not future.done():
                rate.sleep()
            retval = moveit2.get_compute_fk_result(future)
    if retval is None:
        print("Failed.")
    else:
        print("Succeeded. Result: " + str(retval))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
