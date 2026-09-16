from action_msgs.msg import GoalStatus
from conftest import GRIPPER_JOINTS, FakeActionClient, complete
from control_msgs.action import GripperCommand as GripperCommandAction
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory

from pymoveit2 import GripperCommand


def _trajectory() -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = ["panda_joint1"]
    return trajectory


def _result(code: int) -> ExecuteTrajectory.Result:
    result = ExecuteTrajectory.Result()
    result.error_code.val = code
    return result


def test_nothing_submitted_yet_has_no_failure(moveit2):
    assert moveit2.last_failure() is None


def test_a_succeeded_goal_has_no_failure(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result(MoveItErrorCodes.SUCCESS))
    assert moveit2.motion_succeeded is True
    assert moveit2.last_failure() is None


def test_a_controller_failure_names_the_controller_check(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(
        handle, GoalStatus.STATUS_ABORTED, _result(MoveItErrorCodes.CONTROL_FAILED)
    )
    described = moveit2.last_failure()
    assert described.startswith("CONTROL_FAILED: ")
    assert "ros2 control list_controllers" in described


def test_a_rejected_goal_says_it_was_rejected(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    fake_execute_client.reject()
    assert moveit2.last_failure() == "The goal was rejected."


def test_an_empty_trajectory_says_planning_produced_none(moveit2):
    assert moveit2.execute(None) is False
    assert "no trajectory to execute" in moveit2.last_failure()


def test_a_planning_failure_survives_into_the_explanation(moveit2, monkeypatch):
    monkeypatch.setattr(moveit2, "plan", lambda *args, **kwargs: None, raising=False)
    moveit2._MoveIt2__last_plan_failure = "PLANNING_FAILED: no collision-free path."
    assert moveit2.execute(None) is False
    assert moveit2.last_failure() == "PLANNING_FAILED: no collision-free path."


def test_a_late_completion_of_an_older_goal_does_not_rewrite_the_reason(
    moveit2, fake_execute_client
):
    moveit2.execute(_trajectory())
    first = fake_execute_client.accept(0)
    moveit2.execute(_trajectory())
    second = fake_execute_client.accept(1)
    complete(
        second, GoalStatus.STATUS_ABORTED, _result(MoveItErrorCodes.GOAL_IN_COLLISION)
    )
    complete(first, GoalStatus.STATUS_ABORTED, _result(MoveItErrorCodes.CONTROL_FAILED))
    assert moveit2.last_failure().startswith("GOAL_IN_COLLISION: ")


def test_a_gripper_that_stalls_short_says_so_not_check_the_log(rclpy_node):
    gripper = GripperCommand(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        open_gripper_joint_positions=[0.04, 0.04],
        closed_gripper_joint_positions=[0.0, 0.0],
    )
    try:
        client = FakeActionClient("gripper_action_controller/gripper_cmd")
        gripper._GripperCommand__gripper_command_action_client = client
        gripper.close()
        handle = client.accept()
        result = GripperCommandAction.Result()
        result.reached_goal = False
        result.stalled = True
        result.position = 0.021
        complete(handle, GoalStatus.STATUS_SUCCEEDED, result)

        described = gripper.last_failure()
        assert "stopped at 0.0210 without reaching its goal" in described
        assert "max_effort" in described
        assert "move_group" not in described
    finally:
        gripper.destroy()


def test_a_gripper_short_of_its_goal_without_stalling_points_at_the_travel(
    rclpy_node,
):
    gripper = GripperCommand(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        open_gripper_joint_positions=[0.04, 0.04],
        closed_gripper_joint_positions=[0.0, 0.0],
    )
    try:
        client = FakeActionClient("gripper_action_controller/gripper_cmd")
        gripper._GripperCommand__gripper_command_action_client = client
        gripper.open()
        handle = client.accept()
        result = GripperCommandAction.Result()
        result.reached_goal = False
        result.stalled = False
        complete(handle, GoalStatus.STATUS_SUCCEEDED, result)

        described = gripper.last_failure()
        assert "travel of the gripper" in described
        assert "stalled" not in described
    finally:
        gripper.destroy()


def test_a_gripper_goal_reports_its_own_failure(rclpy_node):
    gripper = GripperCommand(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        open_gripper_joint_positions=[0.04, 0.04],
        closed_gripper_joint_positions=[0.0, 0.0],
    )
    try:
        client = FakeActionClient("gripper_action_controller/gripper_cmd")
        gripper._GripperCommand__gripper_command_action_client = client
        assert gripper.last_failure() is None
        gripper.open()
        client.reject()
        assert gripper.last_failure() == "The goal was rejected."

        gripper.open()
        handle = client.accept()
        result = GripperCommandAction.Result()
        result.reached_goal = True
        complete(handle, GoalStatus.STATUS_SUCCEEDED, result)
        assert gripper.last_failure() is None
    finally:
        gripper.destroy()
