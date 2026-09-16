import pytest
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes

from pymoveit2._diagnostics import (
    ERROR_FIXES,
    NON_FAILURE_VALUES,
    describe_error_code,
    describe_failure,
    describe_status,
)


def test_every_documented_code_exists_in_the_message():
    for name in ERROR_FIXES:
        assert hasattr(MoveItErrorCodes, name), name


def test_success_is_resolved_without_naming_a_missing_constant():
    assert MoveItErrorCodes.SUCCESS in NON_FAILURE_VALUES


def test_the_common_failures_are_all_documented():
    for name in (
        "PLANNING_FAILED",
        "CONTROL_FAILED",
        "GOAL_IN_COLLISION",
        "START_STATE_IN_COLLISION",
        "NO_IK_SOLUTION",
        "INVALID_GROUP_NAME",
        "TIMED_OUT",
    ):
        assert name in ERROR_FIXES, name


def test_a_failure_names_the_code_and_what_to_do():
    described = describe_error_code(
        MoveItErrorCodes(val=MoveItErrorCodes.PLANNING_FAILED)
    )
    assert described is not None
    assert described.startswith("PLANNING_FAILED: ")
    assert "allowed_planning_time" in described


def test_success_and_an_unset_code_describe_nothing():
    assert describe_error_code(MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)) is None
    assert describe_error_code(None) is None
    for value in NON_FAILURE_VALUES:
        assert describe_error_code(MoveItErrorCodes(val=value)) is None


def test_every_failure_code_of_this_distribution_is_documented():
    present = {
        name
        for name in dir(MoveItErrorCodes)
        if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
    }
    undocumented = present - set(ERROR_FIXES) - {"SUCCESS", "UNDEFINED"}
    assert not undocumented, sorted(undocumented)


def test_an_unknown_code_still_reports_its_value():
    described = describe_error_code(MoveItErrorCodes(val=-424242))
    assert described is not None
    assert "-424242" in described


def test_the_detail_moveit_supplies_is_kept():
    code = MoveItErrorCodes(val=MoveItErrorCodes.CONTROL_FAILED)
    if not hasattr(code, "message"):
        pytest.skip("this distribution has no `message` field on MoveItErrorCodes")
    code.message = "controller 'arm' is inactive"
    code.source = "trajectory_execution_manager"
    described = describe_error_code(code)
    assert "controller 'arm' is inactive." in described
    assert "trajectory_execution_manager" in described


def test_a_rejected_goal_reports_what_the_interface_saw():
    described = describe_failure(reason="the goal was rejected")
    assert described == "The goal was rejected."


def test_the_error_code_leads_and_the_observation_follows():
    described = describe_failure(
        status=GoalStatus.STATUS_ABORTED,
        result=type(
            "Result",
            (),
            {"error_code": MoveItErrorCodes(val=MoveItErrorCodes.CONTROL_FAILED)},
        )(),
        reason="the goal was aborted",
    )
    assert described.startswith("CONTROL_FAILED: ")
    assert "ros2 control list_controllers" in described
    assert "The goal was aborted." in described


def test_a_status_without_a_code_is_still_explained():
    described = describe_failure(status=GoalStatus.STATUS_ABORTED)
    assert described.startswith("STATUS_ABORTED: ")


def test_a_silent_failure_says_so_rather_than_nothing():
    assert "no reason" in describe_failure()


def test_a_successful_status_describes_nothing():
    assert describe_status(GoalStatus.STATUS_SUCCEEDED) is None
    assert describe_status(None) is None
