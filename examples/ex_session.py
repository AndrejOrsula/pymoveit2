#!/usr/bin/env python3
"""
Move a robot through a `RobotSession`, the one-call interface of `connect()`.
- ros2 run pymoveit2 ex_session.py
- ros2 run pymoveit2 ex_session.py --ros-args -p position:="[0.3, 0.0, 0.3]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
- ros2 run pymoveit2 ex_session.py --ros-args -p gripper:=True -p timeout_sec:=60.0
"""

import sys
import time
from typing import Optional

from pymoveit2 import connect
from pymoveit2._example_utils import parameter_value


def _declare_parameters(node) -> None:
    from rclpy.parameter import Parameter

    node.declare_parameter("position", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("quat_xyzw", Parameter.Type.DOUBLE_ARRAY)
    node.declare_parameter("gripper", False)
    node.declare_parameter("timeout_sec", 30.0)


def _move(session, deadline: float) -> bool:
    node = session.node
    position = parameter_value(node, "position")
    quat_xyzw = parameter_value(node, "quat_xyzw")

    if position is not None and quat_xyzw is not None:
        node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        return session.move_to_pose(
            list(position),
            list(quat_xyzw),
            timeout_sec=max(0.0, deadline - time.monotonic()),
        )

    node.get_logger().info("Moving to a group state of the SRDF")
    return session.move_to_configuration(
        timeout_sec=max(0.0, deadline - time.monotonic())
    )


def _actuate_gripper(session) -> bool:
    node = session.node
    if not node.get_parameter("gripper").get_parameter_value().bool_value:
        return True
    if session.description.gripper_group_name is None:
        node.get_logger().warning("The SRDF defines no gripper; skipping it")
        return True
    node.get_logger().info("Closing the gripper")
    session.gripper.close()
    return bool(session.gripper.wait_until_executed())


def main() -> int:
    status = 1
    session: Optional[object] = None

    try:
        session = connect("ex_session")
        _declare_parameters(session.node)
        timeout_sec = max(
            0.0,
            float(
                session.node.get_parameter("timeout_sec")
                .get_parameter_value()
                .double_value
            ),
        )
        deadline = time.monotonic() + timeout_sec

        if not _move(session, deadline):
            session.node.get_logger().error(f"Motion failed: {session.last_failure()}")
        elif not _actuate_gripper(session):
            session.node.get_logger().error("Gripper motion failed")
        else:
            session.node.get_logger().info("Session example completed successfully")
            status = 0
    except Exception as error:
        message = f"Session example failed: {type(error).__name__}: {error}"
        if session is not None:
            session.node.get_logger().error(message)
        else:
            print(message, file=sys.stderr)
    finally:
        if session is not None and not session.close():
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
