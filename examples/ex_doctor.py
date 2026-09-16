#!/usr/bin/env python3
"""
Check what an example needs, one link of the chain at a time.
- ros2 run pymoveit2 ex_doctor.py
- ros2 run pymoveit2 ex_doctor.py --ros-args -p robot_description_node:="move_group"
- ros2 run pymoveit2 ex_doctor.py --ros-args -p timeout_sec:=5.0
"""

import sys
import time
from threading import Event, Thread

import rclpy
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymoveit2._example_utils import (
    RobotConfiguration,
    cleanup,
    declare_robot_parameters,
)

OK, FAIL, SKIP = "ok", "FAIL", "skip"


def report(check: str, status: str, detail: str) -> bool:
    print(f"{check:<24}{status:<6}{detail}")
    return status != FAIL


def check_description(robot, deadline: float) -> tuple:
    try:
        description = robot.description
    except Exception as error:
        return (
            None,
            report("robot description", FAIL, str(error)),
        )
    return description, report(
        "robot description",
        OK,
        f"'{description.name}', groups {description.group_names}",
    )


def check_group(robot, deadline: float) -> tuple:
    try:
        kwargs = robot.moveit2_kwargs()
    except Exception as error:
        return (
            None,
            report(
                "planning group",
                FAIL,
                f"{error} Pass `group_name` to pick the group yourself.",
            ),
        )
    return kwargs, report(
        "planning group",
        OK,
        f"'{kwargs['group_name']}', {len(kwargs['joint_names'])} joints,"
        f" {kwargs['base_link_name']} to {kwargs['end_effector_name']}",
    )


def check_joint_states(node: Node, kwargs, callback_group, deadline: float) -> bool:
    received = Event()
    seen: list = []

    def remember(message: JointState) -> None:
        seen.append(list(message.name))
        received.set()

    subscription = node.create_subscription(
        JointState, "joint_states", remember, 10, callback_group=callback_group
    )
    try:
        if not received.wait(timeout=max(0.0, deadline - time.monotonic())):
            return report(
                "joint states",
                FAIL,
                "no message on `joint_states`. Start the controllers:"
                " `ros2 control list_controllers`.",
            )
        if kwargs is None:
            return report(
                "joint states",
                OK,
                f"{len(seen[0])} joints published; not checked against a group",
            )
        missing = [name for name in kwargs["joint_names"] if name not in seen[0]]
        if missing:
            return report(
                "joint states",
                FAIL,
                f"{len(missing)} joint(s) of the group are absent: {missing}."
                " Check that every controller of the group is active.",
            )
        return report("joint states", OK, f"{len(seen[0])} joints published")
    finally:
        node.destroy_subscription(subscription)


def check_actions(node: Node, callback_group, deadline: float) -> bool:
    missing = []
    for action_type, name in (
        (MoveGroup, "move_action"),
        (ExecuteTrajectory, "execute_trajectory"),
    ):
        client = ActionClient(node, action_type, name, callback_group=callback_group)
        try:
            if not client.wait_for_server(
                timeout_sec=max(0.0, deadline - time.monotonic())
            ):
                missing.append(name)
        finally:
            client.destroy()
    if missing:
        return report(
            "action servers",
            FAIL,
            f"unavailable: {missing}. Check that MoveIt 2 runs in this namespace:"
            " `ros2 action list`.",
        )
    return report("action servers", OK, "move_action, execute_trajectory")


def check_services(node: Node, callback_group, deadline: float) -> bool:
    missing = []
    for service_type, name in (
        (GetMotionPlan, "plan_kinematic_path"),
        (GetPositionIK, "compute_ik"),
        (GetPositionFK, "compute_fk"),
    ):
        client = node.create_client(service_type, name, callback_group=callback_group)
        try:
            if not client.wait_for_service(
                timeout_sec=max(0.0, deadline - time.monotonic())
            ):
                missing.append(name)
        finally:
            node.destroy_client(client)
    if missing:
        return report(
            "planning services",
            FAIL,
            f"unavailable: {missing}. Check the `move_group` log for a load failure.",
        )
    return report("planning services", OK, "planning, IK and FK are served")


def main() -> int:
    rclpy.init()
    node = None
    executor = None
    executor_thread = None
    status = 1

    try:
        node = Node("ex_doctor")
        # Budget for each individual check, not for the run as a whole.
        node.declare_parameter("timeout_sec", 5.0)
        callback_group = ReentrantCallbackGroup()
        declare_robot_parameters(node)

        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        timeout_sec = max(
            0.0,
            float(node.get_parameter("timeout_sec").get_parameter_value().double_value),
        )
        robot = RobotConfiguration(node, callback_group=callback_group)

        passed = []
        description, ok = check_description(robot, time.monotonic() + timeout_sec)
        passed.append(ok)
        kwargs, ok = (
            check_group(robot, time.monotonic() + timeout_sec)
            if description is not None
            else (None, report("planning group", SKIP, "no description was read"))
        )
        passed.append(ok)
        passed.append(
            check_joint_states(
                node, kwargs, callback_group, time.monotonic() + timeout_sec
            )
        )
        passed.append(
            check_actions(node, callback_group, time.monotonic() + timeout_sec)
        )
        passed.append(
            check_services(node, callback_group, time.monotonic() + timeout_sec)
        )
        status = int(not all(passed))
        print(
            "\nEverything an example needs is in place."
            if status == 0
            else "\nFix the failed checks above, then run an example again."
        )
    except Exception as error:
        print(f"Doctor failed: {type(error).__name__}: {error}", file=sys.stderr)
    finally:
        if cleanup(
            None,
            executor,
            executor_thread,
            "doctor",
            node=node,
            ros_ok=rclpy.ok,
            ros_shutdown=rclpy.shutdown,
        ):
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
