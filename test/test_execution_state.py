import time

import pytest
from action_msgs.msg import GoalStatus
from conftest import complete
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory


def _trajectory() -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = ["panda_joint1"]
    return trajectory


def _result(code: int = MoveItErrorCodes.SUCCESS) -> ExecuteTrajectory.Result:
    result = ExecuteTrajectory.Result()
    result.error_code.val = code
    return result


def test_wait_until_executed_no_motion_returns_false(moveit2):
    assert moveit2.wait_until_executed(timeout_sec=1.0) is False


def test_wait_until_executed_times_out(moveit2, fake_execute_client):
    assert moveit2.execute(_trajectory()) is True
    fake_execute_client.accept()
    start = time.monotonic()
    assert moveit2.wait_until_executed(timeout_sec=0.2) is False
    elapsed = time.monotonic() - start
    assert 0.15 <= elapsed < 2.0

    from pymoveit2 import MoveIt2State

    assert moveit2.query_state() == MoveIt2State.EXECUTING

    moveit2.force_reset_executing_state()
    assert moveit2.query_state() == MoveIt2State.IDLE


def test_wait_until_executed_cancel_on_timeout(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    assert (
        moveit2.wait_until_executed(timeout_sec=0.05, cancel_on_timeout=True) is False
    )
    assert handle.cancel_requests == 1


def test_no_stale_success_after_failed_send(moveit2):
    moveit2.motion_succeeded = True
    assert moveit2.execute(joint_trajectory=None) is False
    assert moveit2.wait_until_executed(timeout_sec=1.0) is False
    assert moveit2.motion_succeeded is False


def test_invalid_attempt_invalidates_completed_success(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())

    assert moveit2.execute(joint_trajectory=None) is False
    assert moveit2.get_execution_future() is None
    assert moveit2.wait_until_executed(timeout_sec=0.1) is False
    assert moveit2.motion_succeeded is False


def test_unavailable_server_invalidates_completed_success(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())

    fake_execute_client.ready = False
    assert moveit2.execute(_trajectory()) is False
    assert moveit2.get_execution_future() is None
    assert moveit2.wait_until_executed(timeout_sec=0.1) is False
    assert moveit2.motion_succeeded is False


def test_no_stale_success_when_server_missing(moveit2, fake_execute_client):
    fake_execute_client.ready = False
    moveit2.motion_succeeded = True
    assert moveit2.execute(_trajectory()) is False
    assert moveit2.motion_succeeded is False


def test_motion_succeeded_property(moveit2):
    moveit2.motion_succeeded = True
    assert moveit2.motion_succeeded is True


def test_motion_suceeded_deprecated_alias_still_works_and_warns(
    moveit2,
):
    with pytest.warns(DeprecationWarning):
        moveit2.motion_suceeded = True
    assert moveit2.motion_succeeded is True
    with pytest.warns(DeprecationWarning):
        assert moveit2.motion_suceeded is True


def test_fast_completion_before_wait_is_reported_once(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.wait_until_executed(timeout_sec=0.1) is True

    assert moveit2.wait_until_executed(timeout_sec=0.1) is False
    assert moveit2.get_last_execution_error_code().val == MoveItErrorCodes.SUCCESS
