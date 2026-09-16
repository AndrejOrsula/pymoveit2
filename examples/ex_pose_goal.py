#!/usr/bin/env python3
"""
Move to a pose.
- ros2 run pymoveit2 ex_pose_goal.py
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.3, 0.0, 0.3]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p cartesian:=True
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p synchronous:=False -p cancel_after_secs:=1.0
"""

import sys
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    complete_pose,
    declare_robot_parameters,
    parameter_value,
)
from pymoveit2._example_utils import wait_for_motion as _wait_for_motion_impl


def _wait_for_motion(
    moveit2,
    cancel_after_secs: float,
    timeout_sec: float,
    node,
    *,
    deadline=None,
) -> bool:
    return _wait_for_motion_impl(
        moveit2,
        cancel_after_secs,
        timeout_sec,
        node,
        MoveIt2State.IDLE,
        deadline=deadline,
    )


def main() -> int:
    rclpy.init()
    node = None
    moveit2 = None
    executor = None
    executor_thread = None
    status = 1

    try:
        # Node for this example
        node = Node("ex_pose_goal")

        # Target pose. Without it, the pose of a group state of the SRDF is used.
        node.declare_parameter("position", Parameter.Type.DOUBLE_ARRAY)
        node.declare_parameter("quat_xyzw", Parameter.Type.DOUBLE_ARRAY)
        node.declare_parameter("synchronous", True)
        # If non-positive, don't cancel. Only used if synchronous is False
        node.declare_parameter("cancel_after_secs", 0.0)
        # Total budget for planning, execution and cancellation
        node.declare_parameter("timeout_sec", 30.0)
        # Planner ID. Empty uses the default planner of `move_group`.
        node.declare_parameter("planner_id", "")
        # Declare parameters for cartesian planning
        node.declare_parameter("cartesian", False)
        node.declare_parameter("cartesian_max_step", 0.0025)
        node.declare_parameter("cartesian_fraction_threshold", 0.0)
        node.declare_parameter("cartesian_jump_threshold", 0.0)
        node.declare_parameter("cartesian_avoid_collisions", True)

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
        planner_id = node.get_parameter("planner_id").get_parameter_value().string_value
        if planner_id:
            moveit2.planner_id = planner_id

        # Scale down velocity and acceleration of joints (percentage of maximum)
        moveit2.max_velocity = 0.5
        moveit2.max_acceleration = 0.5

        # Get parameters
        position = parameter_value(node, "position")
        quat_xyzw = parameter_value(node, "quat_xyzw")
        synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
        cancel_after_secs = (
            node.get_parameter("cancel_after_secs").get_parameter_value().double_value
        )
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )
        cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
        cartesian_max_step = (
            node.get_parameter("cartesian_max_step").get_parameter_value().double_value
        )
        cartesian_fraction_threshold = (
            node.get_parameter("cartesian_fraction_threshold")
            .get_parameter_value()
            .double_value
        )
        cartesian_jump_threshold = (
            node.get_parameter("cartesian_jump_threshold")
            .get_parameter_value()
            .double_value
        )
        cartesian_avoid_collisions = (
            node.get_parameter("cartesian_avoid_collisions")
            .get_parameter_value()
            .bool_value
        )

        # Set parameters for cartesian planning
        moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
        moveit2.cartesian_jump_threshold = cartesian_jump_threshold

        deadline = time.monotonic() + timeout_sec

        # Without a target, use the pose that the arm has in a state of the SRDF
        position, quat_xyzw = complete_pose(
            moveit2,
            robot.joint_positions(),
            deadline - time.monotonic(),
            node,
            position,
            quat_xyzw,
        )

        # Move to pose
        node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        submitted = moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
            timeout_sec=max(0.0, deadline - time.monotonic()),
        )
        if not submitted:
            node.get_logger().error("Failed to submit pose motion")
        elif synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            status = int(
                not moveit2.wait_until_executed(
                    timeout_sec=max(0.0, deadline - time.monotonic())
                )
            )
        else:
            status = int(
                not _wait_for_motion(
                    moveit2,
                    cancel_after_secs,
                    timeout_sec,
                    node,
                    deadline=deadline,
                )
            )
        if status == 0:
            node.get_logger().info("Pose motion completed successfully")
        else:
            node.get_logger().error("Pose motion failed")
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Pose-goal example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Pose-goal example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "pose-goal",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
