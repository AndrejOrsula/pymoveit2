#!/usr/bin/env python3
"""
Example of moving to a joint configuration with orientation path constraints.
- ros2 run pymoveit2 ex_orientation_path_constraints.py --ros-args -p use_orientation_constraint:=True
- ros2 run pymoveit2 ex_orientation_path_constraints.py --ros-args -p use_orientation_constraint:=False
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
    node = Node("ex_orientation_path_constraints")

    # Declare parameter for joint positions
    node.declare_parameter(
        "initial_joint_positions",
        [1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854],
    )
    node.declare_parameter(
        "goal_joint_positions",
        [
            1.1967347888074664,
            -1.3973650345565432,
            0.1342628652196778,
            -2.508581053909259,
            -2.412084037045761,
            0.5267181456160516,
            1.3027041597804059,
        ],
    )
    node.declare_parameter("use_orientation_constraint", True)
    node.declare_parameter(
        "orientation_constraint_quaternion",
        [
            0.5,
            -0.5,
            0.5,
            0.5,
        ],
    )
    node.declare_parameter(
        "orientation_constraint_tolerance",
        [
            3.14159,
            0.5,
            0.5,
        ],
    )
    node.declare_parameter("orientation_constraint_parameterization", 1)

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

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Get parameters
    initial_joint_positions = (
        node.get_parameter("initial_joint_positions")
        .get_parameter_value()
        .double_array_value
    )
    goal_joint_positions = (
        node.get_parameter("goal_joint_positions")
        .get_parameter_value()
        .double_array_value
    )
    use_orientation_constraint = (
        node.get_parameter("use_orientation_constraint")
        .get_parameter_value()
        .bool_value
    )
    orientation_constraint_quaternion = (
        node.get_parameter("orientation_constraint_quaternion")
        .get_parameter_value()
        .double_array_value
    )
    orientation_constraint_tolerance = (
        node.get_parameter("orientation_constraint_tolerance")
        .get_parameter_value()
        .double_array_value
    )
    orientation_constraint_parameterization = (
        node.get_parameter("orientation_constraint_parameterization")
        .get_parameter_value()
        .integer_value
    )

    # Move to initial joint configuration
    node.get_logger().info(
        f"Moving to {{joint_positions: {list(initial_joint_positions)}}}"
    )
    moveit2.move_to_configuration(initial_joint_positions)
    moveit2.wait_until_executed()

    # Set orientation path constraint
    if use_orientation_constraint:
        node.get_logger().info(f"Setting orientation path constraint")
        moveit2.set_path_orientation_constraint(
            quat_xyzw=orientation_constraint_quaternion,
            tolerance=orientation_constraint_tolerance,
            parameterization=orientation_constraint_parameterization,
        )

    # Move to goal joint configuration
    node.get_logger().info(
        f"Moving to {{joint_positions: {list(goal_joint_positions)}}}"
    )
    moveit2.move_to_configuration(goal_joint_positions)
    moveit2.wait_until_executed()

    # Shutdown
    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
