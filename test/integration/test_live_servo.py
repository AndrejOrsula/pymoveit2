import os
import threading
import time

import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory

from pymoveit2 import MoveIt2Servo

try:
    from moveit_msgs.srv import ServoCommandType
except ImportError:
    ServoCommandType = None


pytestmark = pytest.mark.skipif(
    os.environ.get("PYMOVEIT2_INTEGRATION") != "1",
    reason="integration environment not available",
)

READY_TIMEOUT_SEC = 10.0


def _skip_or_fail(reason: str) -> None:
    if os.environ.get("PYMOVEIT2_REQUIRE_SERVO") == "1":
        pytest.fail(reason)
    pytest.skip(reason)


def _wait_until(predicate, timeout_sec: float = READY_TIMEOUT_SEC) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


def _service_types(node):
    return {name: set(types) for name, types in node.get_service_names_and_types()}


def _servo_namespace() -> str:
    return os.environ.get("PYMOVEIT2_SERVO_NAMESPACE", "").rstrip("/")


def _servo_service_base(namespace: str) -> str:
    return f"{namespace}/servo_node" if namespace else "/servo_node"


def _publish_until_output(servo, output_event) -> bool:
    output_event.clear()
    accepted = False
    deadline = time.monotonic() + READY_TIMEOUT_SEC
    while time.monotonic() < deadline:
        accepted = (
            servo.servo(
                linear=(0.05, 0.0, 0.0),
                angular=(0.0, 0.0, 0.0),
                enable_if_disabled=False,
            )
            or accepted
        )
        output_event.wait(0.05)
        if output_event.is_set():
            break
    return accepted


def _wait_for_output_quiescence(
    output_messages, stable_sec: float = 0.2, timeout_sec: float = READY_TIMEOUT_SEC
) -> bool:
    deadline = time.monotonic() + timeout_sec
    observed_count = len(output_messages)
    stable_since = time.monotonic()
    while time.monotonic() < deadline:
        current_count = len(output_messages)
        now = time.monotonic()
        if current_count != observed_count:
            observed_count = current_count
            stable_since = now
        elif now - stable_since >= stable_sec:
            return True
        time.sleep(0.01)
    return False


@pytest.fixture(scope="module")
def servo_probe_node():
    rclpy.init()
    node = rclpy.create_node(f"pymoveit2_servo_probe_{os.getpid()}")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output_messages = []
    output_event = threading.Event()
    output_topic = os.environ.get(
        "PYMOVEIT2_SERVO_OUTPUT_TOPIC", "/panda_arm_controller/joint_trajectory"
    )

    def record_output(message):
        output_messages.append(message)
        output_event.set()

    output_subscription = node.create_subscription(
        JointTrajectory,
        output_topic,
        record_output,
        10,
    )
    try:
        yield node, output_messages, output_event
    finally:
        node.destroy_subscription(output_subscription)
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("legacy", [True, False], ids=["legacy", "modern"])
def test_live_servo_acknowledges_requested_protocol(servo_probe_node, legacy):
    servo_probe_node, output_messages, output_event = servo_probe_node
    namespace = _servo_namespace()
    base = _servo_service_base(namespace)
    requested = os.environ.get("PYMOVEIT2_SERVO_INTERFACE", "").strip().lower()
    protocol_name = "legacy" if legacy else "modern"
    if requested and requested not in (protocol_name, "both"):
        pytest.skip(f"PYMOVEIT2_SERVO_INTERFACE={requested!r} selects another protocol")

    if legacy:
        required = {
            f"{base}/start_servo": "std_srvs/srv/Trigger",
            f"{base}/stop_servo": "std_srvs/srv/Trigger",
        }
    else:
        if ServoCommandType is None:
            _skip_or_fail(
                "actual modern MoveIt Servo unavailable: "
                "moveit_msgs/srv/ServoCommandType is absent"
            )
        required = {
            f"{base}/pause_servo": "std_srvs/srv/SetBool",
            f"{base}/switch_command_type": "moveit_msgs/srv/ServoCommandType",
        }

    if not _wait_until(
        lambda: all(
            expected in _service_types(servo_probe_node).get(name, set())
            for name, expected in required.items()
        ),
        timeout_sec=(
            READY_TIMEOUT_SEC
            if os.environ.get("PYMOVEIT2_REQUIRE_SERVO") == "1"
            else 2.0
        ),
    ):
        _skip_or_fail(
            "actual MoveIt Servo backend unavailable; required services/types: "
            + ", ".join(
                f"{name} ({service_type})" for name, service_type in required.items()
            )
        )

    servo = MoveIt2Servo(
        node=servo_probe_node,
        frame_id="panda_link0",
        namespace=namespace,
        enable_at_init=False,
        callback_group=ReentrantCallbackGroup(),
        legacy_interface=legacy,
    )
    try:
        assert (
            servo.enable(
                wait_for_server_timeout_sec=READY_TIMEOUT_SEC,
                sync=True,
            )
            is True
        )
        assert servo.wait_until_ready(timeout_sec=READY_TIMEOUT_SEC) is True

        if legacy:
            assert _publish_until_output(servo, output_event) is True
        else:
            assert (
                servo.servo(
                    linear=(0.0, 0.0, 0.0),
                    angular=(0.0, 0.0, 0.0),
                    enable_if_disabled=False,
                )
                is False
            )
            assert _wait_until(
                lambda: servo.active_command_type == ServoCommandType.Request.TWIST,
                timeout_sec=READY_TIMEOUT_SEC,
            )
            assert _publish_until_output(servo, output_event) is True

        assert output_event.is_set()
        assert output_messages
        output_event.clear()

        assert (
            servo.disable(
                wait_for_server_timeout_sec=READY_TIMEOUT_SEC,
                sync=True,
            )
            is True
        )
        assert servo.is_enabled is False
        assert servo.servo(enable_if_disabled=False) is False
        assert _wait_for_output_quiescence(output_messages) is True

        assert (
            servo.enable(
                wait_for_server_timeout_sec=READY_TIMEOUT_SEC,
                sync=True,
            )
            is True
        )
        assert servo.wait_until_ready(timeout_sec=READY_TIMEOUT_SEC) is True
        assert servo.shutdown(timeout_sec=READY_TIMEOUT_SEC) is True
        assert servo.is_enabled is False
        assert servo.remote_state_unknown is False
        assert servo.servo(enable_if_disabled=False) is False
    finally:
        servo.destroy()
