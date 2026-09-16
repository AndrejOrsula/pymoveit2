import threading

import pytest
from conftest import PANDA_JOINTS, FakeActionClient, FakeServiceClient
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2.moveit2 import init_joint_state


@pytest.mark.parametrize("pose", [False, True])
def test_move_group_does_not_submit_after_preparation_deadline(moveit2, pose):
    client = FakeActionClient("move_action")
    moveit2._MoveIt2__move_action_client = client
    moveit2._MoveIt2__use_move_group_action = True
    if pose:
        submitted = moveit2.move_to_pose(
            position=(0.3, 0.0, 0.5),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            timeout_sec=0.0,
        )
    else:
        submitted = moveit2.move_to_configuration([0.1] * 7, timeout_sec=0.0)
    assert not submitted
    assert not client.sent_goals


def test_planning_expiry_prevents_late_execution(moveit2, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("pymoveit2.moveit2.time.monotonic", lambda: clock[0])
    client = FakeActionClient("execute_trajectory")
    moveit2._execute_trajectory_action_client = client

    def late_plan(**kwargs):
        clock[0] += 0.2
        return JointTrajectory(
            joint_names=PANDA_JOINTS,
            points=[JointTrajectoryPoint(positions=[0.1] * 7)],
        )

    monkeypatch.setattr(moveit2, "plan", late_plan)
    assert not moveit2.move_to_configuration([0.1] * 7, timeout_sec=0.1)
    assert not client.sent_goals


def test_constructor_failure_releases_created_entities(rclpy_node, monkeypatch):
    from pymoveit2 import MoveIt2

    def counts():
        return tuple(
            len(list(getattr(rclpy_node, name)))
            for name in ("publishers", "subscriptions", "clients", "waitables")
        )

    before = counts()

    def fail_client(**kwargs):
        raise RuntimeError("client construction failed")

    monkeypatch.setattr(rclpy_node, "create_client", fail_client)
    with pytest.raises(RuntimeError, match="construction failed"):
        MoveIt2(rclpy_node, PANDA_JOINTS, "panda_link0", "panda_hand")
    assert counts() == before


def test_destroy_attempts_remaining_entities_and_retries_failure(moveit2, monkeypatch):
    node = moveit2._node
    original = node.destroy_publisher
    attempted = []
    failed = []

    def fail_once(publisher):
        attempted.append(publisher)
        if not failed:
            failed.append(publisher)
            raise RuntimeError("transient publisher cleanup failure")
        return original(publisher)

    monkeypatch.setattr(node, "destroy_publisher", fail_once)
    moveit2.destroy()
    assert len(attempted) == 3
    moveit2.destroy()
    assert attempted.count(failed[0]) == 2
    assert failed[0] not in list(node.publishers)


def test_empty_plan_rejected_before_joint_wait_or_transport(moveit2, monkeypatch):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    waited = []
    monkeypatch.setattr(
        moveit2,
        "_MoveIt2__wait_for_joint_state",
        lambda timeout: waited.append(timeout),
    )
    with pytest.raises(ValueError, match="goal"):
        moveit2.plan_async()
    assert not waited
    assert not planner.requests


def test_failed_state_wait_consumes_goal_and_path_constraints(moveit2):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    moveit2.set_path_joint_constraint([0.3] * 7)
    assert (
        moveit2.plan_async(joint_positions=[0.1] * 7, joint_state_timeout_sec=0.0)
        is None
    )
    moveit2.plan_async(joint_positions=[0.2] * 7, start_joint_state=[0.0] * 7)
    request = planner.requests[-1].motion_plan_request
    assert len(request.goal_constraints[-1].joint_constraints) == 7
    assert {c.position for c in request.goal_constraints[-1].joint_constraints} == {0.2}
    assert not request.path_constraints.joint_constraints


def test_stored_cartesian_frame_is_not_reinterpreted(moveit2):
    client = FakeServiceClient("compute_cartesian_path")
    moveit2._plan_cartesian_path_service = client

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose = Pose(
        position=Point(x=1.3, y=-0.8, z=2.5), orientation=Quaternion(w=1.0)
    )
    moveit2.set_pose_goal(pose=pose, target_link="panda_link8")
    moveit2.plan_async(cartesian=True, start_joint_state=[0.0] * 7)
    request = client.requests[-1]
    assert request.header.frame_id == "world"
    assert request.link_name == "panda_link8"
    assert request.waypoints[0] == pose.pose


@pytest.mark.parametrize("mismatch", ["frame", "link"])
def test_mixed_cartesian_constraints_rejected_before_send(moveit2, mismatch):
    client = FakeServiceClient("compute_cartesian_path")
    moveit2._plan_cartesian_path_service = client
    moveit2.set_position_goal(
        (0.3, 0.0, 0.5), frame_id="world", target_link="panda_hand"
    )
    moveit2.set_orientation_goal(
        (0.0, 0.0, 0.0, 1.0),
        frame_id="other" if mismatch == "frame" else "world",
        target_link="panda_link8" if mismatch == "link" else "panda_hand",
    )
    with pytest.raises(ValueError, match="frame|link"):
        moveit2.plan_async(cartesian=True, start_joint_state=[0.0] * 7)
    assert not client.requests


def test_plan_recomputes_discovery_budget_after_joint_wait(moveit2, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr("pymoveit2.moveit2.time.monotonic", lambda: clock[0])
    budgets = []

    def joints(timeout_sec):
        clock[0] += 0.08
        return init_joint_state(PANDA_JOINTS, [0.0] * 7)

    class SlowDiscovery(FakeServiceClient):
        def wait_for_service(self, timeout_sec=None):
            budgets.append(timeout_sec)
            clock[0] += timeout_sec
            return False

    moveit2._plan_kinematic_path_service = SlowDiscovery(ready=False)
    monkeypatch.setattr(moveit2, "_MoveIt2__wait_for_joint_state", joints)
    assert moveit2.plan(joint_positions=[0.1] * 7, timeout_sec=0.1) is None
    assert budgets == pytest.approx([0.02])
    assert clock[0] == pytest.approx(100.1)


@pytest.mark.parametrize("kind", ["fk", "ik", "plan", "cartesian"])
@pytest.mark.parametrize("stage", ["discovery", "send"])
def test_immediate_transport_exception_is_runtime_failure(moveit2, kind, stage):
    class BrokenClient(FakeServiceClient):
        def wait_for_service(self, timeout_sec=None):
            if stage == "discovery":
                raise RuntimeError("transport closed during discovery")
            return True

        def call_async(self, request):
            raise RuntimeError("transport closed during send")

    client = BrokenClient()
    if kind == "fk":
        moveit2._MoveIt2__compute_fk_client = client
        result = moveit2.compute_fk_async(joint_state=[0.0] * 7)
    elif kind == "ik":
        moveit2._MoveIt2__compute_ik_client = client
        result = moveit2.compute_ik_async((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
    elif kind == "plan":
        moveit2._plan_kinematic_path_service = client
        result = moveit2.plan_async(
            joint_positions=[0.1] * 7, start_joint_state=[0.0] * 7
        )
    else:
        moveit2._plan_cartesian_path_service = client
        result = moveit2.plan_async(
            position=(0.3, 0.0, 0.5),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            cartesian=True,
            start_joint_state=[0.0] * 7,
        )
    assert result is None


@pytest.mark.parametrize("kind", ["fk", "ik"])
def test_concurrent_first_kinematics_call_owns_one_client(moveit2, monkeypatch, kind):
    first_started = threading.Event()
    second_created = threading.Event()
    release = threading.Event()
    clients = []
    failures = []

    def create(**kwargs):
        client = FakeServiceClient(kwargs["srv_name"])
        clients.append(client)
        if len(clients) == 1:
            first_started.set()
            assert release.wait(2.0)
        else:
            second_created.set()
        return client

    monkeypatch.setattr(moveit2._node, "create_client", create)

    def run():
        try:
            if kind == "fk":
                moveit2.compute_fk_async(joint_state=[0.0] * 7)
            else:
                moveit2.compute_ik_async((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0))
        except Exception as exc:
            failures.append(exc)

    first, second = threading.Thread(target=run), threading.Thread(target=run)
    first.start()
    assert first_started.wait(1.0)
    second.start()
    second_created.wait(0.1)
    release.set()
    first.join(2.0)
    second.join(2.0)
    assert not first.is_alive() and not second.is_alive()
    assert not failures
    assert len(clients) == 1
    assert len(clients[0].requests) == 2


@pytest.mark.parametrize(
    "dimensions", [(1.0,), (1.0, 2.0), (1.0, 2.0, float("nan")), (1.0, 2.0, -1.0), None]
)
def test_primitive_dimensions_validated_with_valid_pose(moveit2, dimensions):
    with pytest.raises(ValueError):
        moveit2.add_collision_primitive(
            id="bad",
            primitive_type=SolidPrimitive.BOX,
            dimensions=dimensions,
            position=(0.0, 0.0, 0.0),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )


@pytest.mark.parametrize("positions", [[0.0], [0.0, float("nan")]])
def test_joint_state_name_position_correspondence(positions):
    with pytest.raises(ValueError):
        init_joint_state(["j1", "j2"], positions)


def test_incomplete_observation_does_not_signal_ready(moveit2):
    state = JointState(name=PANDA_JOINTS, position=[0.1])
    moveit2._MoveIt2__joint_state_callback(state)
    assert moveit2.joint_state is None
    assert not moveit2.wait_for_joint_state(timeout_sec=0.0)


@pytest.mark.parametrize("field", ["velocity", "effort"])
def test_unknown_optional_sensor_readings_preserve_position_readiness(moveit2, field):
    state = JointState(name=PANDA_JOINTS, position=[0.1] * 7)
    setattr(state, field, [float("nan")] + [0.0] * 6)
    moveit2._MoveIt2__joint_state_callback(state)
    assert moveit2.wait_for_joint_state(timeout_sec=0.0)
    assert list(moveit2.joint_state.position) == [0.1] * 7
    assert list(getattr(moveit2.joint_state, field)) == []
    assert len(getattr(state, field)) == 7


@pytest.mark.parametrize("values", [[float("nan")], [float("inf")] * 7])
def test_malformed_optional_sensor_readings_do_not_establish_readiness(moveit2, values):
    state = JointState(name=PANDA_JOINTS, position=[0.1] * 7, effort=values)
    moveit2._MoveIt2__joint_state_callback(state)
    assert not moveit2.wait_for_joint_state(timeout_sec=0.0)


def test_observation_and_joint_names_are_defensive_copies(moveit2):
    state = init_joint_state(PANDA_JOINTS, [0.1] * 7)
    moveit2._MoveIt2__joint_state_callback(state)
    moveit2.joint_names.clear()
    observed = moveit2.joint_state
    observed.position[0] = 9.0
    state.position[1] = 8.0
    assert moveit2.joint_names == PANDA_JOINTS
    assert list(moveit2.joint_state.position) == [0.1] * 7


def test_concurrent_planning_does_not_mix_convenience_goals(moveit2, monkeypatch):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    waiting = threading.Event()
    release = threading.Event()
    errors = []

    def wait_state(timeout_sec):
        waiting.set()
        assert release.wait(2.0)
        return init_joint_state(PANDA_JOINTS, [0.0] * 7)

    monkeypatch.setattr(moveit2, "_MoveIt2__wait_for_joint_state", wait_state)

    def first():
        try:
            moveit2.plan_async(joint_positions=[0.1] * 7)
        except Exception as error:
            errors.append(error)

    thread = threading.Thread(target=first)
    thread.start()
    assert waiting.wait(1.0)
    try:
        moveit2.plan_async(joint_positions=[0.2] * 7, start_joint_state=[0.0] * 7)

        moveit2.set_joint_goal([0.3] * 7)
    finally:
        release.set()
        thread.join(2.0)
    assert not thread.is_alive() and not errors
    moveit2.plan_async(start_joint_state=[0.0] * 7)
    positions = [
        [
            c.position
            for c in request.motion_plan_request.goal_constraints[-1].joint_constraints
        ]
        for request in planner.requests
    ]
    assert positions == [[0.2] * 7, [0.1] * 7, [0.3] * 7]


def test_partial_goals_and_empty_optional_state_fields_remain_valid(moveit2):
    assert len(moveit2.create_joint_constraints([0.1, 0.2])) == 2
    constraints = moveit2.create_joint_constraints([0.4], [PANDA_JOINTS[-1]])
    assert constraints[0].joint_name == PANDA_JOINTS[-1]
    state = init_joint_state(PANDA_JOINTS, [0.0] * 7, [], [])
    assert not state.velocity and not state.effort
    with pytest.raises(ValueError):
        moveit2.create_joint_constraints([0.1], PANDA_JOINTS)
    with pytest.raises(ValueError):
        moveit2.create_joint_constraints([0.1], ["unknown"])


def test_timed_out_reads_release_client_pending_requests(moveit2):
    client = FakeServiceClient("compute_fk")
    moveit2._MoveIt2__compute_fk_client = client
    for _ in range(3):
        assert moveit2.compute_fk(joint_state=[0.0] * 7, timeout_sec=0.001) is None
    assert client.removed_requests == client.futures
    assert all(future.cancelled() for future in client.futures)
    assert not moveit2._MoveIt2__pending_reads


@pytest.mark.parametrize("field", ["position", "velocity", "effort"])
def test_nonfinite_joint_state_fields_never_establish_readiness(moveit2, field):
    state = init_joint_state(PANDA_JOINTS, [0.1] * 7)
    setattr(state, field, [float("inf")] * 7)
    moveit2._MoveIt2__joint_state_callback(state)
    assert moveit2.joint_state is None


@pytest.mark.parametrize("value", [float("nan"), float("inf"), "0.1"])
def test_invalid_timeout_rejected(moveit2, value):
    with pytest.raises(ValueError):
        moveit2.plan(joint_positions=[0.1] * 7, timeout_sec=value)


def test_cancel_timer_destroy_restores_node_baseline(moveit2, fake_execute_client):
    from trajectory_msgs.msg import JointTrajectory

    baseline = len(list(moveit2._node.timers))
    assert moveit2.execute(JointTrajectory(joint_names=PANDA_JOINTS))
    fake_execute_client.accept()
    assert moveit2.cancel_execution()
    assert len(list(moveit2._node.timers)) == baseline + 1
    moveit2.destroy()
    assert len(list(moveit2._node.timers)) == baseline


def test_old_timer_callback_cannot_stop_new_operation(moveit2, fake_execute_client):
    from trajectory_msgs.msg import JointTrajectory

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    publisher = Publisher()
    moveit2._MoveIt2__trajectory_execution_event_publisher = publisher
    moveit2.execute(JointTrajectory(joint_names=PANDA_JOINTS))
    fake_execute_client.accept()
    moveit2.cancel_execution()
    timer = next(iter(moveit2._MoveIt2__cancel_timers.values()))
    moveit2.force_reset_executing_state()
    moveit2.execute(JointTrajectory(joint_names=PANDA_JOINTS))
    fake_execute_client.accept()
    timer.callback()
    assert len(publisher.messages) == 1
    assert not moveit2._MoveIt2__cancel_timers


def test_destroy_detaches_pending_read_and_prevents_new_client(moveit2, monkeypatch):
    client = FakeServiceClient("compute_fk")
    moveit2._MoveIt2__compute_fk_client = client
    future = moveit2.compute_fk_async(joint_state=[0.0] * 7)
    moveit2.destroy()
    assert future.cancelled()
    assert client.removed_requests == [future]
    assert not moveit2._MoveIt2__pending_reads

    def no_client(**kwargs):
        pytest.fail("closed interface must not create a lazy client")

    monkeypatch.setattr(moveit2._node, "create_client", no_client)
    assert moveit2.compute_ik_async((0.3, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0)) is None
