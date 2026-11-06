import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from conftest import PANDA_JOINTS, FakeActionClient, complete
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory

from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2._action_lifecycle import ActionLifecycle


def _trajectory() -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = ["panda_joint1"]
    return trajectory


def _result(code: int = MoveItErrorCodes.SUCCESS) -> ExecuteTrajectory.Result:
    result = ExecuteTrajectory.Result()
    result.error_code.val = code
    return result


def test_state_transitions(moveit2, fake_execute_client):
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.execute(_trajectory()) is True
    assert moveit2.query_state() == MoveIt2State.REQUESTING
    handle = fake_execute_client.accept()
    assert moveit2.query_state() == MoveIt2State.EXECUTING
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is True


def test_older_completion_does_not_idle_newer_goal(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    first = fake_execute_client.accept(0)
    moveit2.execute(_trajectory())
    second = fake_execute_client.accept(1)
    assert len(fake_execute_client.sent_goals) == 2

    complete(first, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.query_state() == MoveIt2State.EXECUTING
    assert moveit2.motion_succeeded is False
    assert moveit2.get_execution_future() is second.result_future

    complete(second, GoalStatus.STATUS_ABORTED, _result(MoveItErrorCodes.FAILURE))
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is False
    assert moveit2.wait_until_executed(timeout_sec=0.1) is False
    assert moveit2.get_last_execution_error_code().val == MoveItErrorCodes.FAILURE


def test_out_of_order_callbacks(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    first = fake_execute_client.accept(0)
    moveit2.execute(_trajectory())
    second = fake_execute_client.accept(1)
    complete(second, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is True
    complete(first, GoalStatus.STATUS_ABORTED, _result(MoveItErrorCodes.FAILURE))
    assert moveit2.motion_succeeded is True
    assert moveit2.get_last_execution_error_code().val == MoveItErrorCodes.SUCCESS


def test_rejected_goal_settles_idle(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    fake_execute_client.reject()
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.wait_until_executed(timeout_sec=0.1) is False


def test_exceptional_response_future_settles_idle(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    fake_execute_client.response_futures[-1].set_exception(RuntimeError("boom"))
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is False


def test_exceptional_result_future_settles_idle(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    handle.result_future.set_exception(RuntimeError("boom"))
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is False


def test_cancelled_and_null_results_settle_idle(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    handle.result_future.cancel()
    assert moveit2.query_state() == MoveIt2State.IDLE

    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    handle.result_future.set_result(None)
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.motion_succeeded is False

    moveit2.execute(_trajectory())
    fake_execute_client.response_futures[-1].set_result(None)
    assert moveit2.query_state() == MoveIt2State.IDLE


def test_get_execution_future_returns_retained_future(moveit2, fake_execute_client):
    assert moveit2.get_execution_future() is None
    moveit2.execute(_trajectory())
    assert moveit2.get_execution_future() is None
    handle = fake_execute_client.accept()
    assert moveit2.get_execution_future() is handle.result_future
    assert moveit2.get_execution_future() is handle.result_future


def test_get_execution_future_returns_completed_future(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.get_execution_future() is handle.result_future


def test_get_goal_future_exists_before_the_goal_is_accepted(
    moveit2, fake_execute_client
):
    assert moveit2.get_goal_future() is None
    moveit2.execute(_trajectory())
    assert moveit2.query_state() == MoveIt2State.REQUESTING
    assert moveit2.get_goal_future() is fake_execute_client.response_futures[-1]
    assert moveit2.get_execution_future() is None


def test_get_goal_future_resolves_to_the_goal_handle(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    goal_future = moveit2.get_goal_future()
    handle = fake_execute_client.accept()
    assert goal_future.result() is handle
    assert moveit2.get_goal_future() is goal_future
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.get_goal_future() is goal_future


def test_get_goal_future_returns_a_rejected_goal(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    goal_future = moveit2.get_goal_future()
    handle = fake_execute_client.reject()
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.get_goal_future() is goal_future
    assert goal_future.result() is handle
    assert moveit2.get_execution_future() is None


def test_get_goal_future_follows_the_newest_goal(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    moveit2.execute(_trajectory())
    assert moveit2.get_goal_future() is fake_execute_client.response_futures[1]


def test_get_goal_future_is_none_after_force_reset(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    fake_execute_client.accept()
    moveit2.force_reset_executing_state()
    assert moveit2.get_goal_future() is None


def test_get_goal_future_is_none_when_no_goal_was_sent(moveit2, fake_execute_client):
    fake_execute_client.ready = False
    assert moveit2.execute(_trajectory()) is False
    assert moveit2.get_goal_future() is None


def test_caller_spun_executor_follows_motions_through_the_goal_future(rclpy_node):
    accept = [True]

    def finish(goal_handle):
        goal_handle.succeed()
        return MoveGroup.Result(error_code=MoveItErrorCodes(val=1))

    server_node = rclpy.create_node("pymoveit2_fake_move_group")
    server = ActionServer(
        server_node,
        MoveGroup,
        "/move_action",
        finish,
        goal_callback=lambda _: (
            GoalResponse.ACCEPT if accept[0] else GoalResponse.REJECT
        ),
    )
    server_executor = SingleThreadedExecutor()
    server_executor.add_node(server_node)
    server_thread = threading.Thread(target=server_executor.spin, daemon=True)
    server_thread.start()

    client_node = rclpy.create_node("pymoveit2_goal_future_client")
    executor = SingleThreadedExecutor()
    executor.add_node(client_node)
    moveit2 = MoveIt2(
        node=client_node,
        joint_names=PANDA_JOINTS,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        use_move_group_action=True,
        ignore_new_calls_while_executing=True,
    )

    def move():
        assert moveit2.move_to_configuration([0.0] * 7) is True
        goal_future = moveit2.get_goal_future()
        executor.spin_until_future_complete(goal_future, timeout_sec=10.0)
        assert goal_future.done()
        deadline = time.monotonic() + 10.0
        while moveit2.query_state() != MoveIt2State.IDLE:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)
        return goal_future.result().accepted

    try:
        assert moveit2._MoveIt2__move_action_client.wait_for_server(timeout_sec=10.0)
        for _ in range(2):
            assert move() is True
            assert moveit2.motion_succeeded is True
            assert moveit2.get_last_execution_error_code().val == 1
        accept[0] = False
        assert move() is False
        assert moveit2.motion_succeeded is False
        assert "rejected" in moveit2.last_failure()
    finally:
        moveit2.destroy()
        executor.shutdown()
        client_node.destroy_node()
        server_executor.shutdown()
        server_thread.join(timeout=5.0)
        server.destroy()
        server_node.destroy_node()


def test_cancel_uses_goal_handle_and_stop_event(moveit2, fake_execute_client):
    class RecordingPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publisher = RecordingPublisher()
    moveit2._MoveIt2__trajectory_execution_event_publisher = publisher
    assert moveit2.cancel_execution() is False
    assert publisher.messages == []
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    assert moveit2.cancel_execution() is True
    assert handle.cancel_requests == 1
    assert [m.data for m in publisher.messages] == ["stop"]
    moveit2.stop_all_trajectory_execution()
    assert len(publisher.messages) == 2


def test_cancel_before_acceptance_is_deferred(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    assert moveit2.cancel_execution() is True
    handle = fake_execute_client.accept()
    assert handle.cancel_requests == 1


def test_deferred_cancel_is_invalidated_by_reset_before_late_acceptance(
    moveit2, fake_execute_client
):
    class RecordingPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publisher = RecordingPublisher()
    moveit2._MoveIt2__trajectory_execution_event_publisher = publisher

    moveit2.execute(_trajectory())
    assert moveit2.cancel_execution() is True
    moveit2.force_reset_executing_state()

    assert moveit2.execute(_trajectory()) is True
    fresh = fake_execute_client.accept(1)
    stale = fake_execute_client.accept(0)

    assert stale.cancel_requests == 0
    assert publisher.messages == []
    assert moveit2.query_state() == MoveIt2State.EXECUTING
    assert fresh.cancel_requests == 0


def test_cancel_effect_is_serialized_with_force_reset():
    client = FakeActionClient()
    lifecycle = None
    entered = threading.Event()
    release = threading.Event()
    reset_done = threading.Event()
    validity = []

    class Logger:
        def warning(self, _message):
            pass

        def error(self, _message):
            pass

        def debug(self, _message):
            pass

    def on_cancel(operation):
        entered.set()
        assert release.wait(timeout=1.0)
        validity.append(lifecycle.is_current(operation))

    lifecycle = ActionLifecycle(
        logger=Logger(),
        ignore_new_calls_while_executing=False,
        on_cancel=on_cancel,
    )
    lifecycle.admit(client, _trajectory())
    client.accept()

    cancel_thread = threading.Thread(target=lifecycle.cancel, daemon=True)
    cancel_thread.start()
    assert entered.wait(timeout=1.0)

    reset_thread = threading.Thread(
        target=lambda: (lifecycle.force_reset(), reset_done.set()), daemon=True
    )
    reset_thread.start()
    assert not reset_done.wait(timeout=0.05)

    release.set()
    cancel_thread.join(timeout=1.0)
    reset_thread.join(timeout=1.0)
    assert not cancel_thread.is_alive()
    assert not reset_thread.is_alive()
    assert validity == [True]
    assert lifecycle.query_state() == MoveIt2State.IDLE


def test_settled_callback_can_read_lifecycle_state():
    client = FakeActionClient()
    lifecycle = None
    observed_states = []

    class Logger:
        def warning(self, _message):
            pass

        def error(self, _message):
            pass

        def debug(self, _message):
            pass

    def on_settled(_operation):
        observed_states.append(lifecycle.query_state())

    lifecycle = ActionLifecycle(
        logger=Logger(),
        ignore_new_calls_while_executing=False,
        on_settled=on_settled,
    )
    lifecycle.admit(client, _trajectory())
    handle = client.accept()

    completion = threading.Thread(
        target=complete,
        args=(handle, GoalStatus.STATUS_SUCCEEDED, _result()),
        daemon=True,
    )
    completion.start()
    completion.join(timeout=1.0)
    assert not completion.is_alive()
    assert observed_states == [MoveIt2State.IDLE]


def test_result_success_predicate_settles_before_waiter_observes_outcome():
    client = FakeActionClient()
    predicate_entered = threading.Event()
    release_predicate = threading.Event()
    waiter_done = threading.Event()
    waiter_result = []

    class Logger:
        def warning(self, _message):
            pass

        def error(self, _message):
            pass

        def debug(self, _message):
            pass

    def result_success(_result):
        predicate_entered.set()
        assert release_predicate.wait(timeout=1.0)
        return False

    lifecycle = ActionLifecycle(
        logger=Logger(),
        ignore_new_calls_while_executing=False,
        result_success=result_success,
    )
    lifecycle.admit(client, _trajectory())
    handle = client.accept()

    completion = threading.Thread(
        target=complete,
        args=(handle, GoalStatus.STATUS_SUCCEEDED, _result()),
        daemon=True,
    )
    completion.start()
    assert predicate_entered.wait(timeout=1.0)

    def wait_for_result():
        waiter_result.append(lifecycle.wait_until_executed(timeout_sec=1.0))
        waiter_done.set()

    waiter = threading.Thread(target=wait_for_result, daemon=True)
    waiter.start()
    assert not waiter_done.wait(timeout=0.05)

    release_predicate.set()
    completion.join(timeout=1.0)
    waiter.join(timeout=1.0)
    assert not completion.is_alive()
    assert not waiter.is_alive()
    assert waiter_result == [False]
    assert lifecycle.succeeded is False


def test_force_reset_ignores_late_callbacks(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    stale = fake_execute_client.accept()
    assert moveit2.get_execution_future() is stale.result_future
    moveit2.force_reset_executing_state()
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.get_execution_future() is None

    moveit2.execute(_trajectory())
    fresh = fake_execute_client.accept()
    complete(stale, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.query_state() == MoveIt2State.EXECUTING
    assert moveit2.motion_succeeded is False
    complete(fresh, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.motion_succeeded is True


def test_force_reset_invalidates_completed_future(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
    assert moveit2.get_execution_future() is handle.result_future

    moveit2.force_reset_executing_state()
    assert moveit2.get_execution_future() is None
    assert moveit2.wait_until_executed(timeout_sec=0.1) is False


def test_ignore_new_calls_guard_covers_every_send_path(rclpy_node):
    moveit2 = MoveIt2(
        node=rclpy_node,
        joint_names=PANDA_JOINTS,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        ignore_new_calls_while_executing=True,
        callback_group=ReentrantCallbackGroup(),
    )
    try:
        execute_client = FakeActionClient("execute_trajectory")
        move_client = FakeActionClient("move_action")
        moveit2._execute_trajectory_action_client = execute_client
        moveit2._MoveIt2__move_action_client = move_client

        assert moveit2.execute(_trajectory()) is True
        first = moveit2.current_operation

        assert moveit2.execute(_trajectory()) is False
        assert moveit2.reset_controller([0.0] * 7) is False
        assert moveit2._send_goal_async_move_action() is False
        assert len(execute_client.sent_goals) == 1
        assert len(move_client.sent_goals) == 0

        handle = execute_client.accept()
        assert moveit2.current_operation is first
        assert moveit2.get_execution_future() is handle.result_future
        assert moveit2.reset_controller([0.0] * 7) is False
        complete(handle, GoalStatus.STATUS_SUCCEEDED, _result())
        assert moveit2.reset_controller([0.0] * 7) is True
        assert len(execute_client.sent_goals) == 2
    finally:
        moveit2.destroy()


def test_move_group_action_goal_is_snapshotted(rclpy_node):
    moveit2 = MoveIt2(
        node=rclpy_node,
        joint_names=PANDA_JOINTS,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        use_move_group_action=True,
        callback_group=ReentrantCallbackGroup(),
    )
    try:
        move_client = FakeActionClient("move_action")
        moveit2._MoveIt2__move_action_client = move_client
        assert moveit2.move_to_configuration([0.1] * 7) is True
        sent = move_client.sent_goals[0]
        assert len(sent.request.goal_constraints[0].joint_constraints) == 7

        moveit2.set_joint_goal([0.5] * 7)
        assert sent.request.goal_constraints[0].joint_constraints[0].position == 0.1
        move_client.accept()
        assert moveit2.query_state() == MoveIt2State.EXECUTING
    finally:
        moveit2.destroy()


def test_destroy_is_idempotent_and_abandons_goal(moveit2, fake_execute_client):
    moveit2.execute(_trajectory())
    handle = fake_execute_client.accept()
    assert moveit2.get_execution_future() is handle.result_future
    moveit2.destroy()
    moveit2.destroy()
    assert fake_execute_client.destroyed is True
    assert moveit2.query_state() == MoveIt2State.IDLE
    assert moveit2.get_execution_future() is None


def test_context_manager(rclpy_node):
    with MoveIt2(
        node=rclpy_node,
        joint_names=PANDA_JOINTS,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
    ) as moveit2:
        assert moveit2.query_state() == MoveIt2State.IDLE


class _RecordingLogger:
    def __init__(self):
        self.messages = []

    def warning(self, message):
        self.messages.append(("warning", message))

    def error(self, message):
        self.messages.append(("error", message))

    def debug(self, message):
        pass


def _lifecycle(ignore_new_calls_while_executing=False):
    settled = []
    lifecycle = ActionLifecycle(
        logger=_RecordingLogger(),
        ignore_new_calls_while_executing=ignore_new_calls_while_executing,
        on_settled=settled.append,
    )
    return lifecycle, settled


class _RaisingReadinessClient(FakeActionClient):
    def server_is_ready(self):
        raise RuntimeError("discovery failed")


def test_admit_records_one_failed_attempt_when_readiness_raises():
    lifecycle, settled = _lifecycle()
    client = _RaisingReadinessClient()
    assert lifecycle.admit(client, _trajectory()) is None
    assert client.sent_goals == []
    assert lifecycle.query_state() == MoveIt2State.IDLE
    assert len(settled) == 1 and settled[0].terminal
    assert lifecycle.wait_until_executed(timeout_sec=0.0) is False


def test_admit_records_one_failed_attempt_when_server_is_missing():
    lifecycle, settled = _lifecycle()
    client = FakeActionClient("fake_action", ready=False)
    assert lifecycle.admit(client, _trajectory()) is None
    assert client.sent_goals == []
    assert len(settled) == 1 and settled[0].terminal
    assert lifecycle.wait_until_executed(timeout_sec=0.0) is False
    assert any(
        level == "warning" and "'fake_action' is not yet available" in message
        for level, message in lifecycle._logger.messages
    )


def test_admit_logs_nothing_when_the_server_is_ready():
    lifecycle, _ = _lifecycle()
    assert lifecycle.admit(FakeActionClient(), _trajectory()) is not None
    assert lifecycle._logger.messages == []


def test_admit_failure_while_busy_keeps_the_tracked_goal():
    for failing in (_RaisingReadinessClient(), FakeActionClient(ready=False)):
        lifecycle, settled = _lifecycle()
        client = FakeActionClient()
        tracked = lifecycle.admit(client, _trajectory())
        client.accept()
        assert lifecycle.admit(failing, _trajectory()) is None
        assert lifecycle.current is tracked
        assert lifecycle.query_state() == MoveIt2State.EXECUTING
        assert settled == []


def test_admit_refuses_without_sending_once_closed_or_disabled():
    for shut, settles in (("close", 0), ("disable", 1)):
        lifecycle, settled = _lifecycle()
        client = FakeActionClient()
        getattr(lifecycle, shut)()
        assert lifecycle.admit(client, _trajectory()) is None
        assert client.sent_goals == []
        assert len(settled) == settles
        assert lifecycle.query_state() == MoveIt2State.IDLE


def test_a_disabled_refusal_supersedes_an_unreported_success():
    lifecycle, settled = _lifecycle()
    client = FakeActionClient()
    assert lifecycle.admit(client, _trajectory()) is not None
    complete(client.accept(), GoalStatus.STATUS_SUCCEEDED, _result())
    lifecycle.disable()
    assert lifecycle.admit(client, _trajectory()) is None
    assert len(client.sent_goals) == 1
    assert settled[-1].reason == "Interface is disabled. Skipping motion."
    assert lifecycle.succeeded is False
    assert lifecycle.wait_until_executed(timeout_sec=0.0) is False


def test_admit_rechecks_admission_after_the_readiness_probe():
    for shut in ("close", "disable"):
        lifecycle, settled = _lifecycle()

        class ShuttingClient(FakeActionClient):
            def server_is_ready(self, shut=shut, lifecycle=lifecycle):
                getattr(lifecycle, shut)()
                return True

        client = ShuttingClient()
        assert lifecycle.admit(client, _trajectory()) is None
        assert client.sent_goals == []
        assert len(settled) == (shut == "disable")


def test_admit_rechecks_busy_after_the_readiness_probe():
    lifecycle, _ = _lifecycle(ignore_new_calls_while_executing=True)
    other = FakeActionClient("other")

    class RacingClient(FakeActionClient):
        def server_is_ready(self):
            assert lifecycle.admit(other, _trajectory()) is not None
            return True

    client = RacingClient()
    assert lifecycle.admit(client, _trajectory()) is None
    assert client.sent_goals == []
    assert len(other.sent_goals) == 1
    assert lifecycle.query_state() == MoveIt2State.REQUESTING


def test_admit_refuses_while_busy_when_ignoring_new_calls():
    lifecycle, _ = _lifecycle(ignore_new_calls_while_executing=True)
    client = FakeActionClient()
    tracked = lifecycle.admit(client, _trajectory())
    assert lifecycle.admit(client, _trajectory()) is None
    assert len(client.sent_goals) == 1
    assert lifecycle.current is tracked


def test_admit_settles_a_goal_whose_send_fails():
    class RaisingSendClient(FakeActionClient):
        def send_goal_async(self, goal, feedback_callback=None):
            raise RuntimeError("transport down")

    class NoFutureClient(FakeActionClient):
        def send_goal_async(self, goal, feedback_callback=None):
            return None

    class Unwatchable:
        def add_done_callback(self, _callback):
            raise RuntimeError("callback rejected")

    class UnwatchableClient(FakeActionClient):
        def send_goal_async(self, goal, feedback_callback=None):
            return Unwatchable()

    for client in (RaisingSendClient(), NoFutureClient(), UnwatchableClient()):
        lifecycle, settled = _lifecycle()
        assert lifecycle.admit(client, _trajectory()) is None
        assert lifecycle.query_state() == MoveIt2State.IDLE
        assert len(settled) == 1 and settled[0].succeeded is False
        assert lifecycle.wait_until_executed(timeout_sec=0.0) is False
        assert lifecycle.admit(FakeActionClient(), _trajectory()) is not None


def test_admit_tracks_the_send_future_and_its_response():
    lifecycle, _ = _lifecycle()
    client = FakeActionClient()
    op = lifecycle.admit(client, _trajectory())
    assert op is lifecycle.current
    assert op.send_goal_future is client.response_futures[0]
    assert lifecycle.query_state() == MoveIt2State.REQUESTING
    handle = client.accept()
    assert lifecycle.query_state() == MoveIt2State.EXECUTING
    complete(handle, GoalStatus.STATUS_SUCCEEDED)
    assert lifecycle.query_state() == MoveIt2State.IDLE
    second = lifecycle.admit(client, _trajectory())
    assert second.generation > op.generation
