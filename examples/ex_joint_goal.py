#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
- ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
- ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]" -p synchronous:=False -p cancel_after_secs:=1.0
- ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]" -p synchronous:=False -p cancel_after_secs:=0.0
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
    node = Node("ex_joint_goal")

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
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

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
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
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

    # Get parameters
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    if synchronous:
        # Note: the same functionality can be achieved by setting
        # `synchronous:=false` and `cancel_after_secs` to a negative value.
        moveit2.wait_until_executed()
    else:
        # Wait for the request to get accepted (i.e., for execution to start)
        print("Current State: " + str(moveit2.query_state()))
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        # Get the future
        print("Current State: " + str(moveit2.query_state()))
        future = moveit2.get_execution_future()

        # Cancel the goal
        if cancel_after_secs > 0.0:
            # Sleep for the specified time
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            # Cancel the goal
            print("Cancelling goal")
            moveit2.cancel_execution()

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
