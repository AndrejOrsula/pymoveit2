#!/usr/bin/env python3
"""
Check state validity and collisions: joint positions in, validity and contacts out.
- ros2 run pymoveit2 ex_check_state_validity.py
- ros2 run pymoveit2 ex_check_state_validity.py --ros-args -p joint_positions:="[0.0, 1.0]" -p synchronous:=False
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
        node = Node("ex_check_state_validity")

        # Target joint positions. Without them, a group state of the SRDF is used.
        node.declare_parameter("joint_positions", Parameter.Type.DOUBLE_ARRAY)
        node.declare_parameter("synchronous", True)
        node.declare_parameter("timeout_sec", 10.0)

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
        joint_positions = robot.joint_positions()
        synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
        timeout_sec = node.get_parameter("timeout_sec").value

        # Check state validity
        node.get_logger().info(
            f"Checking state validity for {{joint_positions: {list(joint_positions)}}}"
        )
        timeout_sec = max(0.0, float(timeout_sec))
        deadline = time.monotonic() + timeout_sec
        if synchronous:
            retval = moveit2.check_state_validity(
                robot_state=joint_positions,
                timeout_sec=max(0.0, deadline - time.monotonic()),
            )
        else:
            future = moveit2.check_state_validity_async(
                robot_state=joint_positions,
                wait_for_server_timeout_sec=max(0.0, deadline - time.monotonic()),
            )
            if future is None:
                retval = None
            elif not _wait_for_future(future, deadline - time.monotonic()):
                node.get_logger().error(
                    "Timed out while waiting for the check_state_validity result"
                )
                retval = None
            else:
                try:
                    retval = moveit2.get_check_state_validity_result(future)
                except Exception as error:
                    node.get_logger().error(
                        f"Failed to process state validity result: {type(error).__name__}: {error}"
                    )
                    retval = None
        if retval is None:
            node.get_logger().error("Failed to check state validity")
            print("Failed.")
        else:
            print(f"Succeeded. Valid: {retval.valid}")
            for contact in retval.contacts:
                print(
                    f"Contact between {contact.contact_body_1} & {contact.contact_body_2}"
                )
            status = 0
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"State validity example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"State validity example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "State validity",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
