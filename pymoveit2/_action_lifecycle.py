"""Helpers for the lifecycle of individual ROS 2 action goals."""

import threading
from contextlib import contextmanager
from enum import Enum
from typing import Any, Callable, Iterator, Optional, cast

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.task import Future

from pymoveit2.utils import enum_to_str


class MoveIt2State(Enum):
    """
    An enum the represents the current execution state of the MoveIt2 interface.
    - IDLE: No motion is being requested or executed
    - REQUESTING: Execution has been requested, but the request has not yet been accepted.
    - EXECUTING: Execution has been requested and accepted, and has not yet been completed.
    """

    IDLE = 0
    REQUESTING = 1
    EXECUTING = 2


class ActionOperation:
    __slots__ = (
        "generation",
        "client",
        "send_goal_future",
        "goal_handle",
        "result_future",
        "status",
        "result",
        "reason",
        "succeeded",
        "cancel_requested",
        "terminal",
        "abandoned",
        "reported",
        "done",
    )

    def __init__(self, generation: int, client: ActionClient):
        self.generation = generation
        self.client = client
        self.send_goal_future: Optional[Future] = None
        self.goal_handle: Any = None
        self.result_future: Optional[Future] = None
        self.status: Optional[int] = None
        self.result: Any = None
        self.reason: Optional[str] = None
        self.succeeded: bool = False
        self.cancel_requested: bool = False
        self.terminal: bool = False
        self.abandoned: bool = False
        self.reported: bool = False
        self.done = threading.Event()


class ActionLifecycle:
    def __init__(
        self,
        logger: Any,
        ignore_new_calls_while_executing: bool,
        on_settled: Optional[Callable[[ActionOperation], None]] = None,
        on_cancel: Optional[Callable[[ActionOperation], None]] = None,
        result_success: Optional[Callable[[Any], bool]] = None,
    ):
        self._logger = logger
        self._ignore_new_calls_while_executing = ignore_new_calls_while_executing
        self._on_settled = on_settled
        self._result_success = result_success
        self._on_cancel = on_cancel

        self._lock = threading.Lock()
        self._effect_lock = threading.RLock()
        self._generation = 0
        self._current: Optional[ActionOperation] = None
        self._last: Optional[ActionOperation] = None
        self._succeeded = False
        self._closed = False

    @property
    def lock(self) -> threading.Lock:
        return self._lock

    @property
    def ignore_new_calls_while_executing(self) -> bool:
        return self._ignore_new_calls_while_executing

    @property
    def succeeded(self) -> bool:
        with self._lock:
            return self._succeeded

    @succeeded.setter
    def succeeded(self, value: bool) -> None:
        with self._lock:
            self._succeeded = bool(value)

    def query_state(self) -> MoveIt2State:
        with self._lock:
            return self._query_state_locked()

    def _query_state_locked(self) -> MoveIt2State:
        op = self._current
        if op is None:
            return MoveIt2State.IDLE
        if op.goal_handle is None:
            return MoveIt2State.REQUESTING
        return MoveIt2State.EXECUTING

    def is_busy(self) -> bool:
        with self._lock:
            return self._current is not None

    @property
    def current(self) -> Optional[ActionOperation]:
        with self._lock:
            return self._current

    def is_current(self, operation: ActionOperation) -> bool:
        with self._lock:
            return (
                not self._closed
                and self._current is operation
                and not operation.terminal
                and not operation.abandoned
            )

    @property
    def last_result(self) -> Any:
        with self._lock:
            return self._last.result if self._last is not None else None

    @property
    def last_operation(self) -> Optional[ActionOperation]:
        with self._lock:
            return self._last

    def get_result_future(self) -> Optional[Future]:
        with self._lock:
            op = self._current
            if op is not None:
                if op.result_future is None:
                    self._logger.warning(
                        "The current goal has not been accepted yet; no result "
                        "future exists. Poll `query_state()` until it reports "
                        "EXECUTING."
                    )
                    return None
                return op.result_future

            last = self._last
            if last is not None and last.result_future is not None:
                return last.result_future

            self._logger.warning("Need active goal for future.")
            return None

    def admit(self, client: ActionClient, goal: Any) -> Optional[ActionOperation]:
        with self._effect_lock:
            with self._lock:
                if self._closed:
                    self._logger.warning("Action lifecycle is closed. Skipping motion.")
                    return None
                if self._ignore_new_calls_while_executing and self._current is not None:
                    self._logger.warning(
                        "Controller is already following a trajectory. Skipping motion."
                    )
                    return None

            try:
                ready = client.server_is_ready()
            except Exception as err:
                self._logger.error(
                    f"Action server '{client._action_name}' readiness failed: {err}"
                )
                with self._lock:
                    if self._current is not None:
                        return None
                    op = self._record_failed_attempt_locked(client)
                op.done.set()
                self._notify_settled(op)
                return None

            if not ready:
                self._logger.warning(
                    f"Action server '{client._action_name}' is not yet available. "
                    "Better luck next time!"
                )
                with self._lock:
                    if self._current is not None:
                        return None
                    op = self._record_failed_attempt_locked(client)
                op.done.set()
                self._notify_settled(op)
                return None

            with self._lock:
                if self._closed:
                    return None
                if self._ignore_new_calls_while_executing and self._current is not None:
                    self._logger.warning(
                        "Controller is already following a trajectory. Skipping motion."
                    )
                    return None
                self._generation += 1
                op = ActionOperation(self._generation, client)
                self._current = op
                self._last = None
                self._succeeded = False

            try:
                send_goal_future = client.send_goal_async(
                    goal=goal, feedback_callback=None
                )
            except Exception as err:
                self._logger.error(
                    f"Failed to send goal to '{client._action_name}': {err}"
                )
                self._settle(op, succeeded=False)
                return None

            if send_goal_future is None:
                self._logger.error(
                    f"Action '{client._action_name}' returned no goal future."
                )
                self._settle(op, succeeded=False)
                return None

            with self._lock:
                op.send_goal_future = send_goal_future

        try:
            send_goal_future.add_done_callback(
                lambda future, op=op: self._on_goal_response(op, future)
            )
        except Exception as err:
            self._logger.error(
                f"Action '{client._action_name}' response callback failed: {err}"
            )
            self._settle(op, succeeded=False)
            return None
        return op

    def record_failure(
        self, client: Optional[ActionClient] = None, reason: Optional[str] = None
    ) -> bool:
        with self._effect_lock:
            with self._lock:
                if self._closed or self._current is not None:
                    return False
                op = self._record_failed_attempt_locked(client, reason)
            op.done.set()
            self._notify_settled(op)
        return True

    def _record_failed_attempt_locked(
        self, client: Optional[ActionClient], reason: Optional[str] = None
    ) -> ActionOperation:
        self._generation += 1
        op = ActionOperation(self._generation, cast(ActionClient, client))
        op.terminal = True
        op.reason = reason
        self._last = op
        self._succeeded = False
        return op

    def _on_goal_response(self, op: ActionOperation, future: Future) -> None:
        name = op.client._action_name
        goal_handle: Any = None
        failure: Optional[str] = None
        if future.cancelled():
            failure = "the goal request was cancelled before a response arrived"
        else:
            try:
                goal_handle = future.result()
            except Exception as err:
                failure = f"the goal request raised {type(err).__name__}: {err}"
        if failure is None and goal_handle is None:
            failure = "the goal request returned no goal handle"
        if failure is None and not goal_handle.accepted:
            failure = "the goal was rejected"

        if failure is not None:
            self._logger.warning(f"Action '{name}' was not executed: {failure}.")
            self._settle(op, succeeded=False, reason=failure)
            return

        with self._lock:
            if self._closed or op.abandoned or op.terminal:
                return
            op.goal_handle = goal_handle

        try:
            result_future = goal_handle.get_result_async()
        except Exception as err:
            self._logger.error(f"Action '{name}' result could not be requested: {err}")
            self._settle(
                op,
                succeeded=False,
                reason=f"the result of '{name}' could not be requested: {err}",
            )
            return

        if result_future is None:
            self._logger.error(f"Action '{name}' returned no result future.")
            self._settle(
                op,
                succeeded=False,
                reason=f"'{name}' accepted the goal but returned no result future",
            )
            return

        with self._lock:
            if self._closed or op.abandoned or op.terminal:
                return
            op.result_future = result_future
            cancel_now = (
                self._current is op
                and not op.terminal
                and not op.abandoned
                and op.cancel_requested
            )

        if cancel_now:
            self._request_cancel(op)
        try:
            result_future.add_done_callback(
                lambda result, op=op: self._on_result(op, result)
            )
        except Exception as err:
            self._logger.error(f"Action '{name}' result callback failed: {err}")
            self._settle(
                op,
                succeeded=False,
                reason=f"the result callback of '{name}' failed: {err}",
            )

    def _on_result(self, op: ActionOperation, future: Future) -> None:
        name = op.client._action_name
        status: Optional[int] = None
        result: Any = None
        reason: Optional[str] = None
        succeeded = False
        if future.cancelled():
            reason = f"the result of '{name}' was cancelled before it arrived"
            self._logger.warning(
                f"Action '{name}' result future was cancelled before completion."
            )
        else:
            try:
                wrapped = future.result()
            except Exception as err:
                wrapped = None
                reason = f"the result of '{name}' raised {type(err).__name__}: {err}"
                self._logger.error(
                    f"Action '{name}' result raised {type(err).__name__}: {err}"
                )
            if wrapped is None:
                if not future.cancelled() and reason is None:
                    reason = f"'{name}' returned no result"
                    self._logger.warning(f"Action '{name}' returned no result.")
            else:
                status = getattr(wrapped, "status", None)
                result = getattr(wrapped, "result", None)
                succeeded = status == GoalStatus.STATUS_SUCCEEDED
                if succeeded and self._result_success is not None:
                    try:
                        succeeded = bool(self._result_success(result))
                    except Exception as err:
                        self._logger.error(
                            f"Action '{name}' result validation raised "
                            f"{type(err).__name__}: {err}"
                        )
                        succeeded = False
                        reason = (
                            f"the result of '{name}' could not be validated:"
                            f" {type(err).__name__}: {err}"
                        )
                    else:
                        if not succeeded:
                            reason = (
                                f"'{name}' reported success, but its result does"
                                " not meet the goal"
                            )
                if not succeeded:
                    self._logger.warning(
                        f"Action '{name}' was unsuccessful: "
                        f"{enum_to_str(GoalStatus, status)}."
                    )
        self._settle(
            op, succeeded=succeeded, status=status, result=result, reason=reason
        )

    def _settle(
        self,
        op: ActionOperation,
        succeeded: bool,
        status: Optional[int] = None,
        result: Any = None,
        reason: Optional[str] = None,
    ) -> None:
        notify = False
        with self._effect_lock:
            with self._lock:
                if op.terminal:
                    return
                op.succeeded = succeeded
                op.status = status
                op.result = result
                op.reason = reason
                op.terminal = True
                if self._current is op:
                    self._current = None
                    self._last = op
                    self._succeeded = succeeded
                    notify = True
                elif not op.abandoned:
                    self._logger.debug(
                        f"Ignoring stale completion of generation {op.generation}."
                    )
            op.done.set()
            if notify:
                self._notify_settled(op)

    def _notify_settled(self, op: ActionOperation) -> None:
        callback = self._on_settled
        if callback is None:
            return
        try:
            callback(op)
        except Exception as err:
            self._logger.error(f"Action settlement callback failed: {err}")

    def cancel(self) -> bool:
        with self._lock:
            op = self._current
            if op is None or self._closed or op.terminal or op.abandoned:
                self._logger.warning("Attempted to cancel without active goal.")
                return False
            already_requested = op.cancel_requested
            op.cancel_requested = True
            has_handle = op.goal_handle is not None
        if has_handle and not already_requested:
            self._request_cancel(op)
        return True

    def _request_cancel(self, op: ActionOperation) -> bool:
        with self._effect_lock:
            if not self._cancel_effect_valid_locked(op):
                return False
            with self._lock:
                goal_handle = op.goal_handle

            name = op.client._action_name
            try:
                cancel_future = goal_handle.cancel_goal_async()
            except Exception as err:
                self._logger.error(f"Action '{name}' could not be cancelled: {err}")
                return False

            def _log_cancel_response(future: Future) -> None:
                try:
                    response = future.result()
                except Exception as err:
                    self._logger.error(f"Action '{name}' cancel request failed: {err}")
                    return
                if response is None or not response.goals_canceling:
                    if response is not None and response.return_code == 3:
                        self._logger.debug(
                            f"Action '{name}' cancel request arrived after the "
                            "goal terminated."
                        )
                    else:
                        self._logger.warning(
                            f"Action '{name}' did not accept the cancel request."
                        )

            try:
                cancel_future.add_done_callback(_log_cancel_response)
            except Exception as err:
                self._logger.error(f"Action '{name}' cancel callback failed: {err}")

            if self._on_cancel is not None and self._cancel_effect_valid_locked(op):
                try:
                    self._on_cancel(op)
                except Exception as err:
                    self._logger.error(f"Action cancel callback failed: {err}")
            return True

    def _cancel_effect_valid_locked(self, op: ActionOperation) -> bool:
        with self._lock:
            return (
                not self._closed
                and self._current is op
                and not op.abandoned
                and not op.terminal
                and op.goal_handle is not None
            )

    @contextmanager
    def effect_guard(self, operation: ActionOperation) -> Iterator[bool]:
        with self._effect_lock:
            yield self.is_current(operation)

    def run_if_current(
        self, operation: ActionOperation, callback: Callable[[], None]
    ) -> bool:
        with self._effect_lock:
            if not self.is_current(operation):
                return False
            try:
                callback()
            except Exception as err:
                self._logger.error(f"Guarded action callback failed: {err}")
                return False
            return True

    def force_reset(self) -> None:
        with self._effect_lock:
            with self._lock:
                op = self._current
                if op is not None:
                    op.abandoned = True
                    op.terminal = True
                    op.succeeded = False
                    self._current = None
                self._last = None
                self._succeeded = False
            if op is not None:
                op.done.set()

    def close(self) -> None:
        with self._effect_lock:
            with self._lock:
                if self._closed:
                    return
                self._closed = True
                op = self._current
                if op is not None:
                    op.abandoned = True
                    op.terminal = True
                    op.succeeded = False
                    self._current = None
                self._last = None
                self._succeeded = False
            if op is not None:
                op.done.set()

    def wait_until_executed(
        self, timeout_sec: Optional[float] = None, what: str = "motion"
    ) -> bool:
        with self._lock:
            op = self._current
            if op is None:
                last = self._last
                if last is not None and not last.reported:
                    last.reported = True
                    return last.succeeded
                self._logger.warning(
                    f"Cannot wait until {what} is executed (no {what} is in progress)."
                )
                return False

        if not op.done.wait(timeout=timeout_sec):
            self._logger.warning(
                f"Timed out while waiting for the {what} to finish. The goal may "
                "still be in flight; call `cancel_execution()` to stop it or "
                "`force_reset_executing_state()` to abandon it."
            )
            return False

        with self._lock:
            op.reported = True
            return op.succeeded and not op.abandoned
