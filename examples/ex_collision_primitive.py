#!/usr/bin/env python3
"""
Add, move or remove a box, sphere, cone or cylinder in the planning scene.
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="cylinder" -p position:="[0.2, 0.0, -0.045]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p dimensions:="[0.04, 0.02]"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p action:="remove" -p shape:="sphere"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p action:="move" -p shape:="sphere" -p position:="[0.2, 0.0, 0.2]"
"""

import sys
import time
from functools import partial
from threading import Thread
from typing import Callable

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
)


def wait_for_scene_object(
    moveit2: MoveIt2,
    object_id: str,
    present: bool,
    timeout_sec: float,
    apply_change: Callable[[], None],
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        apply_change()
        if moveit2.update_planning_scene(timeout_sec=min(1.0, remaining)):
            ids = {obj.id for obj in moveit2.planning_scene.world.collision_objects}
            if (object_id in ids) == present:
                return True
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        time.sleep(min(0.1, remaining))
    return False


def main() -> int:
    rclpy.init()
    node = None
    moveit2 = None
    executor = None
    executor_thread = None
    status = 1
    try:
        # Node for this example
        node = Node("ex_collision_primitive")

        node.declare_parameter("shape", "box")
        node.declare_parameter("action", "add")
        node.declare_parameter("position", [0.5, 0.0, 0.5])
        node.declare_parameter("quat_xyzw", [0.0, 0.0, -0.7071, 0.7071])
        node.declare_parameter("dimensions", [0.1, 0.1, 0.1])
        # Seconds to wait for the planning scene to show the change
        node.declare_parameter("timeout_sec", 5.0)

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

        shape = node.get_parameter("shape").get_parameter_value().string_value
        action = node.get_parameter("action").get_parameter_value().string_value
        position = (
            node.get_parameter("position").get_parameter_value().double_array_value
        )
        quat_xyzw = (
            node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        )
        dimensions = (
            node.get_parameter("dimensions").get_parameter_value().double_array_value
        )
        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )
        object_id = shape

        if action == "add":
            node.get_logger().info(
                f"Adding collision primitive of type '{shape}' "
                f"{{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}, dimensions: {list(dimensions)}}}"
            )
            if shape == "box":
                apply_change = partial(
                    moveit2.add_collision_box,
                    id=object_id,
                    position=position,
                    quat_xyzw=quat_xyzw,
                    size=dimensions,
                )
            elif shape == "sphere":
                apply_change = partial(
                    moveit2.add_collision_sphere,
                    id=object_id,
                    position=position,
                    radius=dimensions[0],
                )
            elif shape == "cylinder":
                apply_change = partial(
                    moveit2.add_collision_cylinder,
                    id=object_id,
                    position=position,
                    quat_xyzw=quat_xyzw,
                    height=dimensions[0],
                    radius=dimensions[1],
                )
            elif shape == "cone":
                apply_change = partial(
                    moveit2.add_collision_cone,
                    id=object_id,
                    position=position,
                    quat_xyzw=quat_xyzw,
                    height=dimensions[0],
                    radius=dimensions[1],
                )
            else:
                raise ValueError(f"Unknown shape '{shape}'")
            expect_present = True
        elif action == "remove":
            node.get_logger().info(
                f"Removing collision primitive with ID '{object_id}'"
            )
            apply_change = partial(moveit2.remove_collision_object, id=object_id)
            expect_present = False
        elif action == "move":
            node.get_logger().info(
                f"Moving collision primitive with ID '{object_id}' to "
                f"{{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            apply_change = partial(
                moveit2.move_collision,
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
            )
            expect_present = True
        else:
            raise ValueError(
                f"Unknown action '{action}'. Valid values are 'add', 'remove', 'move'"
            )

        success = wait_for_scene_object(
            moveit2, object_id, expect_present, timeout_sec, apply_change
        )
        if success:
            node.get_logger().info("Planning scene updated successfully")
            status = 0
        else:
            node.get_logger().error(
                f"Planning scene did not reflect the change within {timeout_sec} s"
            )
    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Primitive collision example failed: {type(error).__name__}: {error}"
            )
        else:
            print(
                f"Primitive collision example failed: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
    finally:
        if cleanup(
            moveit2,
            executor,
            executor_thread,
            "collision",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
