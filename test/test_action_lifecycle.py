import threading

import pytest
from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
from conftest import FakeActionClient, FakeGoalHandle, complete
from rclpy.task import Future

from pymoveit2 import MoveIt2State
from pymoveit2._action_lifecycle import ActionLifecycle

NAME = "fake_action"


class RecordingLogger:
    def __init__(self):
        self.records = []

    def warning(self, message):
        self.records.append(("warning", message))

    def error(self, message):
        self.records.append(("error", message))

    def debug(self, message):
        self.records.append(("debug", message))


class CountingGoalHandle(FakeGoalHandle):
    def __init__(self, accepted=True, on_result_request=None, on_cancel_request=None):
        super().__init__(accepted=accepted)
        self.result_requests = 0
        self.on_result_request = on_result_request
        self.on_cancel_request = on_cancel_request

    def get_result_async(self):
        self.result_requests += 1
        if self.on_result_request is not None:
            return self.on_result_request()
        return self.result_future

    def cancel_goal_async(self):
        future = super().cancel_goal_async()
        if self.on_cancel_request is not None:
            self.on_cancel_request()
        return future


class RefusingFuture(Future):
    def add_done_callback(self, callback):
        raise RuntimeError("no callbacks")


class RegisterThenRaiseFuture(Future):
    def add_done_callback(self, callback):
        super().add_done_callback(callback)
        raise RuntimeError("registry full")


def _raise(error):
    raise error


def _lifecycle(**kwargs):
    logger = RecordingLogger()
    lifecycle = ActionLifecycle(
        logger=logger, ignore_new_calls_while_executing=False, **kwargs
    )
    return lifecycle, logger


def _accept(client, handle, index=-1):
    client.response_futures[index].set_result(handle)
    return handle


def _cancel_response(return_code=CancelGoal.Response.ERROR_NONE, canceling=()):
    response = CancelGoal.Response()
    response.return_code = return_code
    response.goals_canceling = list(canceling)
    return response


# Goal response


@pytest.mark.parametrize(
    "respond, failure",
    [
        (
            lambda future: future.cancel(),
            "the goal request was cancelled before a response arrived",
        ),
        (
            lambda future: future.set_exception(RuntimeError("boom")),
            "the goal request raised RuntimeError: boom",
        ),
        (
            lambda future: future.set_result(None),
            "the goal request returned no goal handle",
        ),
        (
            lambda future: future.set_result(FakeGoalHandle(accepted=False)),
            "the goal was rejected",
        ),
    ],
    ids=["cancelled", "raised", "no-handle", "rejected"],
)
def test_unexecuted_goal_settles_with_its_failure(respond, failure):
    settled = []
    lifecycle, logger = _lifecycle(on_settled=settled.append)
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")

    respond(client.response_futures[-1])

    assert logger.records == [
        ("warning", f"Action '{NAME}' was not executed: {failure}.")
    ]
    assert settled == [op]
    assert op.reason == failure
    assert op.goal_handle is None
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert lifecycle.wait_until_executed(timeout_sec=0.1) is False


def test_accepted_goal_tracks_its_result_without_cancelling():
    lifecycle, logger = _lifecycle()
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")
    handle = _accept(client, CountingGoalHandle())

    assert op.goal_handle is handle
    assert op.result_future is handle.result_future
    assert handle.result_requests == 1
    assert handle.cancel_requests == 0
    assert lifecycle.query_state() == MoveIt2State.EXECUTING
    assert lifecycle.get_result_future() is handle.result_future
    assert logger.records == []


def test_deferred_cancel_is_sent_once_the_goal_is_accepted():
    cancelled = []
    lifecycle, _ = _lifecycle(on_cancel=cancelled.append)
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")
    assert lifecycle.cancel() is True

    handle = _accept(client, CountingGoalHandle())

    assert handle.cancel_requests == 1
    assert cancelled == [op]
    assert lifecycle.query_state() == MoveIt2State.EXECUTING


def test_deferred_cancel_of_a_superseded_goal_is_not_sent():
    lifecycle, _ = _lifecycle()
    client = FakeActionClient(NAME)
    lifecycle.admit(client, "first")
    assert lifecycle.cancel() is True
    newer = lifecycle.admit(client, "second")

    older_handle = _accept(client, CountingGoalHandle(), index=0)

    assert older_handle.result_requests == 1
    assert older_handle.cancel_requests == 0
    assert lifecycle.current is newer


def test_goal_accepted_after_force_reset_is_not_tracked():
    lifecycle, logger = _lifecycle()
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")
    lifecycle.force_reset()

    handle = _accept(client, CountingGoalHandle())

    assert handle.result_requests == 0
    assert op.goal_handle is None
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert logger.records == []


def test_superseded_goal_accepted_after_close_is_not_tracked():
    lifecycle, _ = _lifecycle()
    client = FakeActionClient(NAME)
    older = lifecycle.admit(client, "first")
    lifecycle.admit(client, "second")
    lifecycle.close()
    assert not older.abandoned and not older.terminal

    handle = _accept(client, CountingGoalHandle(), index=0)

    assert handle.result_requests == 0
    assert older.goal_handle is None


def test_goal_accepted_after_its_operation_settled_is_not_tracked():
    settled = []
    lifecycle, logger = _lifecycle(on_settled=settled.append)
    client = FakeActionClient(NAME)
    client.response_futures.append(RegisterThenRaiseFuture())
    client.send_goal_async = lambda goal, feedback_callback=None: (
        client.response_futures[-1]
    )
    assert lifecycle.admit(client, "goal") is None
    op = lifecycle.last_operation
    assert settled == [op]
    assert logger.records == [
        ("error", f"Action '{NAME}' response callback failed: registry full")
    ]

    handle = _accept(client, CountingGoalHandle())

    assert handle.result_requests == 0
    assert op.goal_handle is None
    assert settled == [op]
    assert lifecycle.query_state() == MoveIt2State.IDLE


def test_goal_reset_while_its_result_is_requested_is_not_tracked():
    lifecycle, logger = _lifecycle()
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")
    assert lifecycle.cancel() is True
    handle = CountingGoalHandle(
        on_result_request=lambda: (lifecycle.force_reset(), handle.result_future)[1]
    )

    _accept(client, handle)

    assert handle.result_requests == 1
    assert op.result_future is None
    assert handle.cancel_requests == 0
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert logger.records == []


@pytest.mark.parametrize(
    "on_result_request, log, reason",
    [
        (
            lambda: _raise(RuntimeError("gone")),
            ("error", f"Action '{NAME}' result could not be requested: gone"),
            f"the result of '{NAME}' could not be requested: gone",
        ),
        (
            lambda: None,
            ("error", f"Action '{NAME}' returned no result future."),
            f"'{NAME}' accepted the goal but returned no result future",
        ),
        (
            lambda: RefusingFuture(),
            ("error", f"Action '{NAME}' result callback failed: no callbacks"),
            f"the result callback of '{NAME}' failed: no callbacks",
        ),
    ],
    ids=["request-raised", "no-result-future", "callback-refused"],
)
def test_accepted_goal_without_a_usable_result_settles_failed(
    on_result_request, log, reason
):
    settled = []
    lifecycle, logger = _lifecycle(on_settled=settled.append)
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")

    _accept(client, CountingGoalHandle(on_result_request=on_result_request))

    assert logger.records == [log]
    assert settled == [op]
    assert op.reason == reason
    assert op.succeeded is False
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert lifecycle.wait_until_executed(timeout_sec=0.1) is False


# Result


def _executing(**kwargs):
    lifecycle, logger = _lifecycle(**kwargs)
    client = FakeActionClient(NAME)
    op = lifecycle.admit(client, "goal")
    handle = client.accept()
    return lifecycle, logger, op, handle


def test_succeeded_result_settles_successfully():
    lifecycle, logger, op, handle = _executing()
    payload = object()

    complete(handle, GoalStatus.STATUS_SUCCEEDED, payload)

    assert (op.succeeded, op.status, op.result, op.reason) == (
        True,
        GoalStatus.STATUS_SUCCEEDED,
        payload,
        None,
    )
    assert lifecycle.last_result is payload
    assert lifecycle.wait_until_executed(timeout_sec=0.1) is True
    assert logger.records == []


def test_aborted_result_settles_unsuccessfully_without_validation():
    validated = []
    lifecycle, logger, op, handle = _executing(
        result_success=lambda result: validated.append(result) or True
    )
    payload = object()

    complete(handle, GoalStatus.STATUS_ABORTED, payload)

    assert validated == []
    assert (op.succeeded, op.status, op.result, op.reason) == (
        False,
        GoalStatus.STATUS_ABORTED,
        payload,
        None,
    )
    assert logger.records == [
        ("warning", f"Action '{NAME}' was unsuccessful: STATUS_ABORTED.")
    ]
    assert lifecycle.wait_until_executed(timeout_sec=0.1) is False


def test_result_without_status_settles_unsuccessfully():
    lifecycle, logger, op, handle = _executing()

    handle.result_future.set_result(object())

    assert (op.succeeded, op.status, op.result, op.reason) == (
        False,
        None,
        None,
        None,
    )
    assert logger.records == [
        ("warning", f"Action '{NAME}' was unsuccessful: None :UNKNOWN NAME.")
    ]


def test_succeeded_result_that_meets_the_goal_settles_successfully():
    validated = []
    lifecycle, logger, op, handle = _executing(
        result_success=lambda result: validated.append(result) or 1
    )
    payload = object()

    complete(handle, GoalStatus.STATUS_SUCCEEDED, payload)

    assert validated == [payload]
    assert op.succeeded is True
    assert op.reason is None
    assert logger.records == []


def test_succeeded_result_that_misses_the_goal_settles_unsuccessfully():
    lifecycle, logger, op, handle = _executing(result_success=lambda result: 0)

    complete(handle, GoalStatus.STATUS_SUCCEEDED, object())

    assert op.succeeded is False
    assert op.status == GoalStatus.STATUS_SUCCEEDED
    assert op.reason == (
        f"'{NAME}' reported success, but its result does not meet the goal"
    )
    assert logger.records == [
        ("warning", f"Action '{NAME}' was unsuccessful: STATUS_SUCCEEDED.")
    ]
    assert lifecycle.wait_until_executed(timeout_sec=0.1) is False


def test_succeeded_result_whose_validation_raises_settles_unsuccessfully():
    lifecycle, logger, op, handle = _executing(
        result_success=lambda result: _raise(ValueError("bad"))
    )

    complete(handle, GoalStatus.STATUS_SUCCEEDED, object())

    assert op.succeeded is False
    assert op.status == GoalStatus.STATUS_SUCCEEDED
    assert op.reason == (
        f"the result of '{NAME}' could not be validated: ValueError: bad"
    )
    assert logger.records == [
        ("error", f"Action '{NAME}' result validation raised ValueError: bad"),
        ("warning", f"Action '{NAME}' was unsuccessful: STATUS_SUCCEEDED."),
    ]


@pytest.mark.parametrize(
    "finish, log, reason",
    [
        (
            lambda future: future.cancel(),
            (
                "warning",
                f"Action '{NAME}' result future was cancelled before completion.",
            ),
            f"the result of '{NAME}' was cancelled before it arrived",
        ),
        (
            lambda future: future.set_exception(RuntimeError("boom")),
            ("error", f"Action '{NAME}' result raised RuntimeError: boom"),
            f"the result of '{NAME}' raised RuntimeError: boom",
        ),
        (
            lambda future: future.set_result(None),
            ("warning", f"Action '{NAME}' returned no result."),
            f"'{NAME}' returned no result",
        ),
    ],
    ids=["cancelled", "raised", "none"],
)
def test_missing_result_settles_with_its_reason(finish, log, reason):
    settled = []
    lifecycle, logger, op, handle = _executing(on_settled=settled.append)

    finish(handle.result_future)

    assert logger.records == [log]
    assert settled == [op]
    assert (op.succeeded, op.status, op.result, op.reason) == (
        False,
        None,
        None,
        reason,
    )
    assert lifecycle.query_state() == MoveIt2State.IDLE


# Cancel


def test_cancel_requests_the_goal_and_runs_the_cancel_hook():
    cancelled = []
    lifecycle, logger, op, handle = _executing(on_cancel=cancelled.append)

    assert lifecycle.cancel() is True

    assert handle.cancel_requests == 1
    assert cancelled == [op]
    assert lifecycle.query_state() == MoveIt2State.EXECUTING
    assert logger.records == []


def test_cancel_without_a_cancel_hook_only_requests_the_goal():
    lifecycle, logger, _op, handle = _executing()

    assert lifecycle.cancel() is True

    assert handle.cancel_requests == 1
    assert logger.records == []


@pytest.mark.parametrize(
    "respond, records",
    [
        (
            lambda future: future.set_result(_cancel_response(canceling=[GoalInfo()])),
            [],
        ),
        (
            lambda future: future.set_result(
                _cancel_response(CancelGoal.Response.ERROR_GOAL_TERMINATED)
            ),
            [
                (
                    "debug",
                    f"Action '{NAME}' cancel request arrived after the goal "
                    "terminated.",
                )
            ],
        ),
        (
            lambda future: future.set_result(
                _cancel_response(CancelGoal.Response.ERROR_REJECTED)
            ),
            [("warning", f"Action '{NAME}' did not accept the cancel request.")],
        ),
        (
            lambda future: future.set_result(None),
            [("warning", f"Action '{NAME}' did not accept the cancel request.")],
        ),
        (
            lambda future: future.set_exception(RuntimeError("boom")),
            [("error", f"Action '{NAME}' cancel request failed: boom")],
        ),
    ],
    ids=["canceling", "terminated", "rejected", "none", "raised"],
)
def test_cancel_response_is_logged_by_outcome(respond, records):
    lifecycle, logger, _op, handle = _executing()
    assert lifecycle.cancel() is True

    respond(handle.cancel_future)

    assert logger.records == records


def test_cancel_that_cannot_be_sent_skips_the_cancel_hook():
    cancelled = []
    lifecycle, logger = _lifecycle(on_cancel=cancelled.append)
    client = FakeActionClient(NAME)
    lifecycle.admit(client, "goal")
    handle = _accept(client, CountingGoalHandle())
    handle.cancel_goal_async = lambda: _raise(RuntimeError("offline"))

    assert lifecycle.cancel() is True

    assert cancelled == []
    assert logger.records == [
        ("error", f"Action '{NAME}' could not be cancelled: offline")
    ]
    assert lifecycle.query_state() == MoveIt2State.EXECUTING


def test_cancel_response_callback_failure_still_runs_the_cancel_hook():
    cancelled = []
    lifecycle, logger, op, handle = _executing(on_cancel=cancelled.append)
    handle.cancel_future = RefusingFuture()

    assert lifecycle.cancel() is True

    assert handle.cancel_requests == 1
    assert cancelled == [op]
    assert logger.records == [
        ("error", f"Action '{NAME}' cancel callback failed: no callbacks")
    ]


def test_cancel_hook_failure_is_logged():
    lifecycle, logger, _op, handle = _executing(
        on_cancel=lambda op: _raise(RuntimeError("hook"))
    )

    assert lifecycle.cancel() is True

    assert handle.cancel_requests == 1
    assert logger.records == [("error", "Action cancel callback failed: hook")]


def test_cancel_hook_is_skipped_when_the_goal_is_reset_during_the_request():
    cancelled = []
    lifecycle, logger = _lifecycle(on_cancel=cancelled.append)
    client = FakeActionClient(NAME)
    lifecycle.admit(client, "goal")
    handle = _accept(
        client, CountingGoalHandle(on_cancel_request=lambda: lifecycle.force_reset())
    )

    assert lifecycle.cancel() is True

    assert handle.cancel_requests == 1
    assert cancelled == []
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert logger.records == []


def test_cancel_that_loses_the_race_to_force_reset_sends_nothing():
    cancelled = []
    lifecycle, logger, op, handle = _executing(on_cancel=cancelled.append)
    results = []

    with lifecycle.effect_guard(op) as active:
        assert active is True
        canceller = threading.Thread(
            target=lambda: results.append(lifecycle.cancel()), daemon=True
        )
        canceller.start()
        for _ in range(1000):
            with lifecycle.lock:
                if op.cancel_requested:
                    break
            threading.Event().wait(0.001)
        assert op.cancel_requested is True
        lifecycle.force_reset()

    canceller.join(timeout=1.0)
    assert not canceller.is_alive()
    assert results == [True]
    assert handle.cancel_requests == 0
    assert cancelled == []
    assert logger.records == []
