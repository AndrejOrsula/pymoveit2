#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
- ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import kinova


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            -1.47568,
            2.92779,
            1.00845,
            -2.0847,
            1.43588,
            1.32575
        ],
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=kinova.joint_names(),
        base_link_name=kinova.base_link_name(),
        end_effector_name="forkTip",
        group_name=kinova.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    rate = node.create_rate(10)
    while moveit2.query_state() != MoveIt2State.EXECUTING:
        rate.sleep()
    print("Current State: " + str(moveit2.query_state()))
    print("Cancelling goal")
    future = moveit2.cancel_execution()
    print("Current State: " + str(moveit2.query_state()))
    while not future.done():
        rate.sleep()
    print("Cancellation result: " + str(future.result().return_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
