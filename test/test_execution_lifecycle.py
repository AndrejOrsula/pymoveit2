import threading

from action_msgs.msg import GoalStatus
from conftest import PANDA_JOINTS, FakeActionClient, complete
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.callback_groups import ReentrantCallbackGroup
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
