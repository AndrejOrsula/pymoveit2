#!/usr/bin/env python3
"""
Move the end effector in a circle with MoveIt 2 Servo.
- ros2 run pymoveit2 ex_servo.py
- ros2 run pymoveit2 ex_servo.py --ros-args -p frame_id:="base_link" -p namespace:="/robot1"
"""

import sys
import time
from math import cos, sin
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2Servo
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
)


def main() -> int:
    rclpy.init()
    node = None
    moveit2_servo = None
    executor = None
    executor_thread = None
    status = 1
    published_command = False
    interrupted = False
    startup_timeout_sec = 5.0
    shutdown_timeout_sec = 2.0

    try:
        # Node for this example
        node = Node("ex_servo")

        # Servo commands use the base link of the arm group by default.
        # Set `frame_id` to use another frame.
        declare_robot_parameters(node, frame_id=True)
        node.declare_parameter("namespace", "")
        node.declare_parameter("linear_speed", 1.0)
        node.declare_parameter("angular_speed", 1.0)
        node.declare_parameter("startup_timeout_sec", 5.0)
        node.declare_parameter("command_failure_limit", 3)
        node.declare_parameter("shutdown_timeout_sec", 2.0)

        # Let callbacks run in parallel, so a blocking call does not stall its own reply
        callback_group = ReentrantCallbackGroup()

        startup_timeout_sec = max(
            0.0,
            float(node.get_parameter("startup_timeout_sec").value),
        )
        command_failure_limit = max(
            1, int(node.get_parameter("command_failure_limit").value)
        )
        shutdown_timeout_sec = max(
            0.0,
            float(node.get_parameter("shutdown_timeout_sec").value),
        )

        # Spin the node in the background. Discovery needs it, and so does the Servo
        # shutdown in the `finally` block, which waits for the servo node to confirm.
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Create MoveIt 2 Servo interface
        robot = RobotConfiguration(node, callback_group=callback_group)
        moveit2_servo = MoveIt2Servo(
            node=node,
            frame_id=robot.frame_id(),
            namespace=node.get_parameter("namespace").value,
            linear_speed=node.get_parameter("linear_speed").value,
            angular_speed=node.get_parameter("angular_speed").value,
            callback_group=callback_group,
            enable_at_init=False,
        )

        startup_deadline = time.monotonic() + startup_timeout_sec
        if not moveit2_servo.enable(
            wait_for_server_timeout_sec=max(0.0, startup_deadline - time.monotonic())
        ):
            node.get_logger().error("Failed to submit MoveIt Servo enable request")
        elif not moveit2_servo.wait_until_ready(
            timeout_sec=max(0.0, startup_deadline - time.monotonic())
        ):
            node.get_logger().error("Timed out waiting for MoveIt Servo readiness")
        else:
            consecutive_failures = 0
            while rclpy.ok():
                now_sec = time.monotonic()
                try:
                    published = moveit2_servo(
                        linear=(sin(now_sec), cos(now_sec), 0.0),
                        angular=(0.0, 0.0, 0.0),
                    )
                except Exception as error:
                    published = False
                    node.get_logger().error(
                        f"Servo command failed: {type(error).__name__}: {error}"
                    )
                if published:
                    published_command = True
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    startup_expired = (
                        not published_command and time.monotonic() >= startup_deadline
                    )
                    runtime_failures_exceeded = (
                        published_command
                        and consecutive_failures >= command_failure_limit
                    )
                    if startup_expired or runtime_failures_exceeded:
                        node.get_logger().error(
                            "MoveIt Servo published no command within the "
                            "startup and failure limits"
                        )
                        break
                time.sleep(0.2)
            else:
                status = int(not published_command)
    except KeyboardInterrupt:
        interrupted = True
        if published_command:
            status = 0
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Servo example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Servo example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        # The executor still runs, so `shutdown` can wait for the servo node to confirm
        # it is off. `destroy` then releases the local resources.
        cleanup_failed = cleanup(
            moveit2_servo,
            executor,
            executor_thread,
            "MoveIt Servo",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
            acknowledged_shutdown=(
                moveit2_servo.shutdown if moveit2_servo is not None else None
            ),
            shutdown_timeout_sec=shutdown_timeout_sec,
        )
        if interrupted and published_command:
            status = 0
        if cleanup_failed:
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
