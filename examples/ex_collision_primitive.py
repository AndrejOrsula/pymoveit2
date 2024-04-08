#!/usr/bin/env python3
"""
Example of adding and removing a collision object with a primitive geometry.
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="cylinder" -p position:="[0.2, 0.0, -0.045]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p dimensions:="[0.04, 0.02]"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p action:="remove" -p shape:="sphere"
- ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p action:="move" -p shape:="sphere" -p position:="[0.2, 0.0, 0.2]"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_primitive")

    # Declare parameter for joint positions
    node.declare_parameter(
        "shape",
        "box",
    )
    node.declare_parameter(
        "action",
        "add",
    )
    node.declare_parameter("position", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, -0.7071, 0.7071])
    node.declare_parameter("dimensions", [0.1, 0.1, 0.1])

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

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    shape = node.get_parameter("shape").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    dimensions = (
        node.get_parameter("dimensions").get_parameter_value().double_array_value
    )

    # Use the name of the primitive shape as the ID
    object_id = shape

    if action == "add":
        # Add collision primitive
        node.get_logger().info(
            f"Adding collision primitive of type '{shape}' "
            f"{{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}, dimensions: {list(dimensions)}}}"
        )
        if shape == "box":
            moveit2.add_collision_box(
                id=object_id, position=position, quat_xyzw=quat_xyzw, size=dimensions
            )
        elif shape == "sphere":
            moveit2.add_collision_sphere(
                id=object_id, position=position, radius=dimensions[0]
            )
        elif shape == "cylinder":
            moveit2.add_collision_cylinder(
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
                height=dimensions[0],
                radius=dimensions[1],
            )
        elif shape == "cone":
            moveit2.add_collision_cone(
                id=object_id,
                position=position,
                quat_xyzw=quat_xyzw,
                height=dimensions[0],
                radius=dimensions[1],
            )
        else:
            raise ValueError(f"Unknown shape '{shape}'")
    elif action == "remove":
        # Remove collision primitive
        node.get_logger().info(f"Removing collision primitive with ID '{object_id}'")
        moveit2.remove_collision_object(id=object_id)
    elif action == "move":
        # Move collision primitive
        node.get_logger().info(
            f"Moving collision primitive with ID '{object_id}' to "
            f"{{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_collision(id=object_id, position=position, quat_xyzw=quat_xyzw)
    else:
        raise ValueError(
            f"Unknown action '{action}'. Valid values are 'add', 'remove', 'move'"
        )

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
