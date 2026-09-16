import math
import threading
import time

import pytest
from conftest import FakeServiceClient
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger

from pymoveit2 import MoveIt2Servo

try:
    from moveit_msgs.srv import ServoCommandType
except ImportError:
    ServoCommandType = None


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_servo(rclpy_node, **kwargs):
    return MoveIt2Servo(
        node=rclpy_node, frame_id="panda_link0", enable_at_init=False, **kwargs
    )


def _lifecycle_response(servo, success: bool):
    if servo.uses_legacy_interface:
        return Trigger.Response(success=success)
    return SetBool.Response(success=success)


def _install_fakes(servo):
    enable = FakeServiceClient("enable")
    disable = (
        enable if not servo.uses_legacy_interface else FakeServiceClient("disable")
    )
    servo._MoveIt2Servo__enable_service = enable
    servo._MoveIt2Servo__disable_service = disable
    twist_pub = RecordingPublisher()
    jog_pub = RecordingPublisher()
    servo._MoveIt2Servo__twist_pub = twist_pub
    servo._MoveIt2Servo__jog_pub = jog_pub
    command_type = None
    if not servo.uses_legacy_interface:
        command_type = FakeServiceClient("switch_command_type")
        servo._MoveIt2Servo__command_type_service = command_type
    return enable, disable, command_type, twist_pub, jog_pub


def test_servo_constructs_and_reports_mode(rclpy_node):
    servo = make_servo(rclpy_node)

    assert servo.uses_legacy_interface is (ServoCommandType is None)
    servo.destroy()


def test_servo_explicit_protocol_override(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    assert servo.uses_legacy_interface is True
    assert servo._MoveIt2Servo__enable_service.srv_name.endswith("start_servo")
    servo.destroy()
    if ServoCommandType is None:
        with pytest.raises(ValueError):
            make_servo(rclpy_node, legacy_interface=False)
    else:
        servo = make_servo(rclpy_node, legacy_interface=False)
        assert servo.uses_legacy_interface is False
        servo.destroy()


def test_servo_namespaced_topics(rclpy_node):
    servo = make_servo(rclpy_node, namespace="/robot1")
    assert servo._MoveIt2Servo__twist_pub.topic_name.startswith("/robot1/")
    assert servo._MoveIt2Servo__jog_pub.topic_name.startswith("/robot1/")
    servo.destroy()


def test_servo_namespace_normalizes_relative_and_absolute_overrides(rclpy_node):
    relative = make_servo(rclpy_node, namespace="sub//", legacy_interface=True)
    absolute = make_servo(rclpy_node, namespace="/root//", legacy_interface=True)
    root = make_servo(rclpy_node, namespace="///", legacy_interface=True)
    try:
        assert relative._MoveIt2Servo__twist_pub.topic_name.endswith(
            "/sub/servo_node/delta_twist_cmds"
        )
        assert absolute._MoveIt2Servo__twist_pub.topic_name == (
            "/root/servo_node/delta_twist_cmds"
        )
        assert root._MoveIt2Servo__twist_pub.topic_name == (
            "/servo_node/delta_twist_cmds"
        )
    finally:
        relative.destroy()
        absolute.destroy()
        root.destroy()


def test_servo_default_endpoints_follow_real_node_namespace():
    import rclpy

    nodes = [
        rclpy.create_node("servo_a", namespace="/robot_a"),
        rclpy.create_node("servo_b", namespace="/robot_b"),
    ]
    servos = [
        MoveIt2Servo(
            node=node, frame_id="base", enable_at_init=False, legacy_interface=True
        )
        for node in nodes
    ]
    try:
        assert servos[0]._MoveIt2Servo__twist_pub.topic_name == (
            "/robot_a/servo_node/delta_twist_cmds"
        )
        assert servos[1]._MoveIt2Servo__twist_pub.topic_name == (
            "/robot_b/servo_node/delta_twist_cmds"
        )

        assert servos[0]._MoveIt2Servo__enable_service.srv_name == (
            "servo_node/start_servo"
        )
        assert servos[1]._MoveIt2Servo__enable_service.srv_name == (
            "servo_node/start_servo"
        )
    finally:
        for servo, node in zip(servos, nodes, strict=True):
            servo.destroy()
            node.destroy_node()


def test_servo_destroy_invalidates_pending_enable_and_delayed_commands(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, _, _, twist_pub, _ = _install_fakes(servo)

    assert servo.enable() is True
    assert len(enable.requests) == 1
    servo.destroy()
    enable.futures[0].set_result(Trigger.Response(success=True))

    assert servo.is_enabled is False
    assert servo.servo(linear=(1.0, 0.0, 0.0)) is False
    assert twist_pub.messages == []


def test_servo_calls_without_server_do_not_raise(rclpy_node):
    servo = make_servo(rclpy_node)
    assert servo.enable(wait_for_server_timeout_sec=0.1) is False
    assert servo.disable(wait_for_server_timeout_sec=0.1) is False
    assert servo.servo(linear=(1.0, 0.0, 0.0), enable_if_disabled=False) is False
    assert (
        servo.servo_jog(
            joint_names=("panda_joint1",), velocities=(0.1,), enable_if_disabled=False
        )
        is False
    )
    with pytest.raises(ValueError):
        servo.servo_jog(joint_names=("a", "b"), velocities=(0.1,))
    servo.destroy()


def test_servo_publishes_only_after_confirmed_enable(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, twist_pub, _ = _install_fakes(servo)

    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert len(enable.requests) == 1
    assert twist_pub.messages == []

    enable.futures[0].set_result(_lifecycle_response(servo, False))
    assert servo.is_enabled is False
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert len(enable.requests) == 2

    enable.futures[1].set_result(_lifecycle_response(servo, True))
    assert servo.is_enabled is True

    if servo.uses_legacy_interface:
        assert servo(linear=(0.1, 0.0, 0.0)) is True
        assert len(twist_pub.messages) == 1
    else:
        assert servo(linear=(0.1, 0.0, 0.0)) is False
        assert servo(linear=(0.1, 0.0, 0.0)) is False
        assert len(command_type.requests) == 1
        assert command_type.requests[0].command_type == ServoCommandType.Request.TWIST
        assert twist_pub.messages == []
        command_type.futures[0].set_result(ServoCommandType.Response(success=True))
        assert servo.active_command_type == ServoCommandType.Request.TWIST
        assert servo(linear=(0.1, 0.0, 0.0)) is True
        assert len(twist_pub.messages) == 1
    assert twist_pub.messages[0].twist.linear.x == 0.1
    servo.destroy()


def test_servo_disable_supersedes_pending_enable(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, disable, _, twist_pub, _ = _install_fakes(servo)
    servo.enable()
    assert servo.disable() is True
    assert servo.is_enabled is False
    requests_before_ack = 1 if disable is enable else 0
    assert len(disable.requests) == requests_before_ack
    enable.futures[0].set_result(_lifecycle_response(servo, True))
    assert servo.is_enabled is False
    assert servo.servo(linear=(0.1, 0.0, 0.0), enable_if_disabled=False) is False
    assert twist_pub.messages == []

    assert len(disable.requests) == requests_before_ack + 1
    disable.futures[-1].set_result(_lifecycle_response(servo, True))
    assert servo.is_enabled is False
    servo.destroy()


def test_servo_rapid_enable_disable_alternation(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, disable, _, _, _ = _install_fakes(servo)
    servo.enable()
    servo.disable()
    servo.enable()

    enable.futures[0].set_result(_lifecycle_response(servo, True))
    assert servo.is_enabled is True
    assert len(disable.requests) == (1 if disable is enable else 0)

    servo.disable()
    disable.futures[-1].set_result(_lifecycle_response(servo, False))
    assert servo.is_enabled is False
    servo.destroy()


def test_servo_sync_enable_reports_confirmation(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, _, _, _ = _install_fakes(servo)
    enable.sync_response = _lifecycle_response(servo, False)
    assert servo.enable(sync=True) is False
    assert servo.is_enabled is False
    enable.sync_response = _lifecycle_response(servo, True)
    assert servo.enable(sync=True) is True
    assert servo.is_enabled is True
    assert servo.wait_until_ready(timeout_sec=0.1) is True
    servo.destroy()


def test_servo_reenable_waits_for_pending_disable_ack(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, disable, _, twist_pub, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(Trigger.Response(success=True))
    assert servo.disable() is True
    assert servo.enable() is True
    assert servo.is_enabled is False
    assert servo.servo(linear=(0.1, 0.0, 0.0), enable_if_disabled=False) is False
    assert twist_pub.messages == []
    assert len(disable.futures) == 1

    disable.futures[0].set_result(Trigger.Response(success=True))
    assert len(enable.futures) == 2
    enable.futures[1].set_result(Trigger.Response(success=True))
    assert servo.is_enabled is True
    assert servo.servo(linear=(0.1, 0.0, 0.0)) is True
    servo.destroy()


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_failed_type_switch_still_orders_pending_disable(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))
    assert servo(linear=(0.1, 0.0, 0.0)) is False

    result = []
    thread = threading.Thread(target=lambda: result.append(servo.disable(sync=True)))
    thread.start()
    try:
        deadline = time.monotonic() + 1.0
        while len(command_type.futures) < 1 and time.monotonic() < deadline:
            time.sleep(0.001)
        command_type.futures[0].set_result(ServoCommandType.Response(success=False))
        deadline = time.monotonic() + 1.0
        while len(enable.futures) < 2 and time.monotonic() < deadline:
            time.sleep(0.001)
        assert len(enable.futures) == 2
        enable.futures[1].set_result(SetBool.Response(success=True))
        thread.join(timeout=1.0)
        assert result == [True]
    finally:
        servo.destroy()
        thread.join(timeout=1.0)


def test_sync_enable_reports_superseded_target(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, disable, _, _, _ = _install_fakes(servo)
    entered = threading.Event()
    release = threading.Event()

    def blocking_call(request):
        entered.set()
        release.wait(timeout=1.0)
        return Trigger.Response(success=True)

    enable.call = blocking_call
    result = []
    thread = threading.Thread(target=lambda: result.append(servo.enable(sync=True)))
    thread.start()
    assert entered.wait(timeout=1.0)
    assert servo.disable() is True
    release.set()
    try:
        deadline = time.monotonic() + 1.0
        while len(disable.futures) < 1 and time.monotonic() < deadline:
            time.sleep(0.001)
        assert len(disable.futures) == 1
        disable.futures[0].set_result(Trigger.Response(success=True))
        thread.join(timeout=1.0)
        assert result == [False]
    finally:
        servo.destroy()
        thread.join(timeout=1.0)


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_command_type_switch_modern_interface(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, twist_pub, jog_pub = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))

    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert servo.servo_jog(joint_names=("panda_joint1",), velocities=(0.1,)) is False
    assert [r.command_type for r in command_type.requests] == [
        ServoCommandType.Request.TWIST
    ]
    command_type.futures[0].set_result(ServoCommandType.Response(success=True))
    assert servo.active_command_type is None
    assert [r.command_type for r in command_type.requests] == [
        ServoCommandType.Request.TWIST,
        ServoCommandType.Request.JOINT_JOG,
    ]
    command_type.futures[1].set_result(ServoCommandType.Response(success=True))
    assert servo.active_command_type == ServoCommandType.Request.JOINT_JOG
    assert servo.servo_jog(joint_names=("panda_joint1",), velocities=(0.1,)) is True
    assert len(jog_pub.messages) == 1
    assert twist_pub.messages == []

    assert servo(linear=(0.1, 0.0, 0.0)) is False
    command_type.futures[-1].set_result(ServoCommandType.Response(success=False))
    assert servo.active_command_type is None
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert len(command_type.requests) == 4

    command_type.ready = False
    command_type.futures[-1].set_result(ServoCommandType.Response(success=False))
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert len(command_type.requests) == 4
    servo.destroy()


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_servo_type_failure_still_orders_changed_successor(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert servo.servo_jog(joint_names=("joint",), velocities=(0.1,)) is False
    command_type.futures[0].set_result(ServoCommandType.Response(success=False))
    assert [request.command_type for request in command_type.requests] == [
        ServoCommandType.Request.TWIST,
        ServoCommandType.Request.JOINT_JOG,
    ]
    command_type.futures[1].set_result(ServoCommandType.Response(success=True))
    assert servo.active_command_type == ServoCommandType.Request.JOINT_JOG
    servo.destroy()


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_command_ready_does_not_ignore_pending_switch(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    command_type.futures[0].set_result(ServoCommandType.Response(success=True))
    assert servo(linear=(0.1, 0.0, 0.0)) is True
    assert servo.servo_jog(joint_names=("joint",), velocities=(0.1,)) is False
    assert (
        servo.wait_until_command_ready(ServoCommandType.Request.TWIST, timeout_sec=0.01)
        is False
    )
    servo.destroy()


def test_servo_destroy_is_idempotent(rclpy_node):
    servo = make_servo(rclpy_node)
    servo.destroy()
    servo.destroy()
    with make_servo(rclpy_node) as servo:
        assert servo.is_enabled is False


def test_servo_validates_finite_vectors_before_transport(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, _, _, twist_pub, jog_pub = _install_fakes(servo)

    for values in ((1.0, 2.0), (math.nan, 0.0, 0.0), (math.inf, 0.0, 0.0)):
        with pytest.raises(ValueError):
            servo.servo(linear=values)
    with pytest.raises(ValueError):
        servo.servo_jog(joint_names=("joint",), velocities=(math.nan,))

    assert enable.requests == []
    assert twist_pub.messages == []
    assert jog_pub.messages == []
    servo.destroy()


def test_servo_rejects_scalar_joint_name_input(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    _, _, _, _, jog_pub = _install_fakes(servo)
    with pytest.raises(ValueError):
        servo.servo_jog(joint_names="abc", velocities=(0.1, 0.2, 0.3))
    assert jog_pub.messages == []
    servo.destroy()


def test_servo_rejects_nonfinite_service_discovery_timeout(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, _, _, _, _ = _install_fakes(servo)
    with pytest.raises(ValueError):
        servo.enable(wait_for_server_timeout_sec=math.nan)
    assert enable.requests == []
    servo.destroy()


def test_servo_transport_exception_releases_transition_for_retry(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    failing = FakeServiceClient("enable")

    def raise_call_async(request):
        raise RuntimeError("transport unavailable")

    failing.call_async = raise_call_async
    servo._MoveIt2Servo__enable_service = failing
    assert servo.enable() is False
    assert servo._MoveIt2Servo__transition is None
    assert servo.remote_state_unknown is False

    replacement = FakeServiceClient("enable")
    servo._MoveIt2Servo__enable_service = replacement
    assert servo.enable() is True
    replacement.futures[0].set_result(Trigger.Response(success=True))
    assert servo.is_enabled is True
    servo.destroy()


def test_servo_wait_ready_handles_transition_claim_before_future(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, _, _, _, _ = _install_fakes(servo)
    entered = threading.Event()
    release = threading.Event()

    def delayed_call_async(request):
        enable.requests.append(request)
        entered.set()
        release.wait(timeout=1.0)
        future = Future()
        enable.futures.append(future)
        return future

    enable.call_async = delayed_call_async
    sender = threading.Thread(target=servo.enable)
    sender.start()
    assert entered.wait(timeout=1.0)
    result = []
    waiter = threading.Thread(
        target=lambda: result.append(servo.wait_until_ready(timeout_sec=1.0))
    )
    waiter.start()
    try:
        time.sleep(0.05)
        assert result == []
        release.set()
        deadline = time.monotonic() + 1.0
        while len(enable.futures) < 1 and time.monotonic() < deadline:
            time.sleep(0.001)
        enable.futures[0].set_result(Trigger.Response(success=True))
        sender.join(timeout=1.0)
        waiter.join(timeout=1.0)
        assert result == [True]
    finally:
        release.set()
        servo.destroy()
        sender.join(timeout=1.0)
        waiter.join(timeout=1.0)


def test_servo_completed_future_exception_quarantines_remote_state(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, _, _, _, _ = _install_fakes(servo)

    assert servo.enable() is True
    enable.futures[0].set_exception(RuntimeError("response transport failed"))
    assert servo.is_enabled is False
    assert servo.remote_state_unknown is True
    assert servo.enable() is False
    assert servo.disable() is False
    servo.destroy()


def test_servo_shutdown_waits_for_ack_and_then_allows_destroy(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, disable, _, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(Trigger.Response(success=True))

    result = []
    worker = threading.Thread(target=lambda: result.append(servo.shutdown(1.0)))
    worker.start()
    deadline = time.monotonic() + 1.0
    while len(disable.futures) < 1 and time.monotonic() < deadline:
        time.sleep(0.001)
    assert len(disable.futures) == 1
    disable.futures[0].set_result(Trigger.Response(success=True))
    worker.join(timeout=1.0)

    assert result == [True]
    assert servo.is_enabled is False
    assert servo.remote_state_unknown is False
    servo.destroy()


def test_servo_immediately_completed_transport_exception_is_reported(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    failing = FakeServiceClient("enable")

    def completed_exception(request):
        future = Future()
        future.set_exception(RuntimeError("response failed before return"))
        return future

    failing.call_async = completed_exception
    servo._MoveIt2Servo__enable_service = failing
    assert servo.enable() is False
    assert servo.remote_state_unknown is True
    assert servo._MoveIt2Servo__transition is None
    servo.destroy()


def test_servo_shutdown_timeout_quarantines_and_late_response_is_inert(rclpy_node):
    servo = make_servo(rclpy_node, legacy_interface=True)
    enable, disable, _, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(Trigger.Response(success=True))

    assert servo.shutdown(0.001) is False
    assert servo.remote_state_unknown is True
    assert servo.enable() is False
    assert servo.disable() is False
    disable.futures[0].set_result(Trigger.Response(success=True))
    assert servo.is_enabled is False
    servo.destroy()


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_servo_disable_invalidates_pending_type_before_ack(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, disable, command_type, twist_pub, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))
    assert servo(linear=(0.1, 0.0, 0.0)) is False
    assert len(command_type.futures) == 1

    assert servo.disable() is True
    assert servo.is_enabled is False
    assert servo.active_command_type is None
    assert servo._MoveIt2Servo__pending_command_type is None
    command_type.futures[0].set_result(ServoCommandType.Response(success=True))
    assert servo.active_command_type is None
    assert len(disable.futures) == 2
    assert twist_pub.messages == []
    disable.futures[1].set_result(SetBool.Response(success=True))
    servo.destroy()


@pytest.mark.skipif(ServoCommandType is None, reason="legacy (Humble) interface")
def test_wait_until_ready_is_enable_only(rclpy_node):
    servo = make_servo(rclpy_node)
    enable, _, command_type, _, _ = _install_fakes(servo)
    servo.enable()
    enable.futures[0].set_result(SetBool.Response(success=True))
    assert servo.wait_until_ready(timeout_sec=0.01) is True
    assert servo.servo(linear=(0.1, 0.0, 0.0)) is False
    assert servo.wait_until_ready(timeout_sec=0.01) is True
    assert (
        servo.wait_until_command_ready(ServoCommandType.Request.TWIST, timeout_sec=0.01)
        is False
    )
    command_type.futures[0].set_result(ServoCommandType.Response(success=True))
    assert (
        servo.wait_until_command_ready(ServoCommandType.Request.TWIST, timeout_sec=0.01)
        is True
    )
    servo.destroy()
