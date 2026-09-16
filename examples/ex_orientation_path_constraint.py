#!/usr/bin/env python3
"""
Move to a joint configuration while keeping the end effector orientation fixed.
- ros2 run pymoveit2 ex_orientation_path_constraint.py --ros-args -p use_orientation_constraint:=True
- ros2 run pymoveit2 ex_orientation_path_constraint.py --ros-args -p use_orientation_constraint:=False
"""

import sys
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from pymoveit2 import MoveIt2
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
    end_effector_pose,
    parameter_value,
)


def _held_orientation(moveit2, joint_positions, deadline: float, node):
    pose = end_effector_pose(
        moveit2, joint_positions, deadline - time.monotonic(), node
    )
    if pose is None:
        return None
    orientation = pose.pose.orientation
    return [orientation.x, orientation.y, orientation.z, orientation.w]


def _run_motion(moveit2, joint_positions, deadline: float, node) -> bool:
    if not moveit2.move_to_configuration(
        joint_positions, timeout_sec=max(0.0, deadline - time.monotonic())
    ):
        node.get_logger().error("Failed to submit constrained joint motion")
        return False
    success = moveit2.wait_until_executed(
        timeout_sec=max(0.0, deadline - time.monotonic())
    )
    if not success:
        node.get_logger().error("Constrained joint motion failed")
    return success


def main() -> int:
    rclpy.init()
    node = None
    moveit2 = None
    executor = None
    executor_thread = None
    status = 1
    try:
        node = Node("ex_orientation_path_constraint")
        # Both configurations default to values derived from the robot description.
        node.declare_parameter("initial_joint_positions", Parameter.Type.DOUBLE_ARRAY)
        node.declare_parameter("goal_joint_positions", Parameter.Type.DOUBLE_ARRAY)
        # Radians to turn the first joint by, when there is no goal configuration
        node.declare_parameter("goal_joint_offset", 0.5)
        node.declare_parameter("use_orientation_constraint", True)
        # Orientation to hold. Empty keeps the one the robot starts the motion with.
        node.declare_parameter(
            "orientation_constraint_quaternion", Parameter.Type.DOUBLE_ARRAY
        )
        node.declare_parameter("orientation_constraint_tolerance", [3.14159, 0.5, 0.5])
        node.declare_parameter("orientation_constraint_parameterization", 1)
        node.declare_parameter("timeout_sec", 30.0)

        callback_group = ReentrantCallbackGroup()
        # The robot configuration comes from the URDF and SRDF that `move_group` is
        # running with. Every value can still be set as a ROS parameter.
        declare_robot_parameters(node)
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

        initial_joint_positions = robot.joint_positions("initial_joint_positions")
        # Without a goal, turn the first joint, which every arm can do
        offset = node.get_parameter("goal_joint_offset").value
        goal_joint_positions = robot.joint_positions(
            "goal_joint_positions",
            default=[
                position + float(offset) if index == 0 else position
                for index, position in enumerate(initial_joint_positions)
            ],
        )
        use_orientation_constraint = (
            node.get_parameter("use_orientation_constraint")
            .get_parameter_value()
            .bool_value
        )
        orientation_constraint_quaternion = parameter_value(
            node, "orientation_constraint_quaternion"
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
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )
        moveit2.max_velocity = 0.5
        moveit2.max_acceleration = 0.5
        deadline = time.monotonic() + timeout_sec

        node.get_logger().info(
            f"Moving to {{joint_positions: {list(initial_joint_positions)}}}"
        )
        success = _run_motion(moveit2, initial_joint_positions, deadline, node)
        if success and use_orientation_constraint:
            if orientation_constraint_quaternion is None:
                orientation_constraint_quaternion = _held_orientation(
                    moveit2, initial_joint_positions, deadline, node
                )
            success = orientation_constraint_quaternion is not None
            if success:
                node.get_logger().info(
                    "Holding the end effector orientation "
                    f"{list(orientation_constraint_quaternion)}"
                )
                moveit2.set_path_orientation_constraint(
                    quat_xyzw=orientation_constraint_quaternion,
                    tolerance=orientation_constraint_tolerance,
                    parameterization=orientation_constraint_parameterization,
                )
        if success:
            node.get_logger().info(
                f"Moving to {{joint_positions: {list(goal_joint_positions)}}}"
            )
            success = _run_motion(moveit2, goal_joint_positions, deadline, node)
        status = int(not success)
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Orientation-constraint example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Orientation-constraint example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "orientation-constraint",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
