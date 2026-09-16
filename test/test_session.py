import threading
import time
import types

import pytest

from pymoveit2 import session as session_module


class FakeLogger:
    def warning(self, message):
        self.last = message

    def info(self, message):
        pass


class FakeNode:
    def __init__(self, name, namespace=None):
        self.name = name
        self.namespace = namespace
        self.destroyed = False
        self._logger = FakeLogger()

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        self.destroyed = True


class FakeExecutor:
    def __init__(self, num_threads=None, events=None):
        self.num_threads = num_threads
        self.nodes = []
        self.events = events
        self.spinning = threading.Event()

    def add_node(self, node):
        self.nodes.append(node)

    def spin(self):
        self.spinning.set()

    def shutdown(self, timeout_sec=None):
        if self.events is not None:
            self.events.append("executor_shutdown")
        return True


class FakeMoveIt2:
    def __init__(self, node=None, callback_group=None, events=None, **kwargs):
        self.kwargs = kwargs
        self.events = events
        self.destroyed = False
        self.submitted = True
        self.executed = True
        self.waits = 0
        self.wait_timeouts = []
        self.planning_delay = 0.0
        self.group_name = kwargs.get("group_name", "arm")

    def move_to_pose(self, **kwargs):
        self.pose_call = kwargs
        time.sleep(self.planning_delay)
        return self.submitted

    def move_to_configuration(self, joint_positions, **kwargs):
        self.configuration_call = (joint_positions, kwargs)
        return self.submitted

    def wait_until_executed(self, timeout_sec=None):
        self.waits += 1
        self.wait_timeouts.append(timeout_sec)
        return self.executed

    def last_failure(self):
        return None if self.executed else "PLANNING_FAILED: no path."

    def add_collision_box(self, **kwargs):
        return "box added"

    def destroy(self):
        self.destroyed = True
        if self.events is not None:
            self.events.append("arm_destroy")


class FakeDescription:
    name = "test_robot"
    group_names = ["arm"]

    def moveit2_kwargs(self, group_name=None):
        return {
            "joint_names": ["j1", "j2"],
            "base_link_name": "base",
            "end_effector_name": "tip",
            "group_name": group_name or "arm",
        }

    def joint_positions(self, state_name=None, group_name=None):
        return [0.1, 0.2]


@pytest.fixture()
def fake_ros(monkeypatch):
    events = []
    state = {"initialized": True, "shutdown": False, "nodes": []}

    def create_node(name, **kwargs):
        node = FakeNode(name, **kwargs)
        state["nodes"].append(node)
        return node

    executors = []

    def build_executor(num_threads):
        executor = FakeExecutor(num_threads, events)
        executors.append(executor)
        return executor

    def discover(cls, node, **kwargs):
        state["spinning_at_discovery"] = executors[-1].spinning.wait(timeout=5.0)
        return FakeDescription()

    fake_rclpy = types.SimpleNamespace(
        ok=lambda: state["initialized"] and not state["shutdown"],
        init=lambda args=None: state.update(initialized=True),
        shutdown=lambda: state.update(shutdown=True),
        create_node=create_node,
    )
    monkeypatch.setattr(session_module, "rclpy", fake_rclpy)
    monkeypatch.setattr(session_module, "MultiThreadedExecutor", build_executor)
    monkeypatch.setattr(session_module, "ReentrantCallbackGroup", lambda: "cbg")
    monkeypatch.setattr(
        session_module,
        "MoveIt2",
        lambda **kwargs: FakeMoveIt2(events=events, **kwargs),
    )
    monkeypatch.setattr(
        session_module.RobotDescription, "from_node", classmethod(discover)
    )
    return types.SimpleNamespace(events=events, state=state)


def test_connect_spins_the_node_before_it_reads_the_description(fake_ros):
    with session_module.connect() as arm:
        assert fake_ros.state["spinning_at_discovery"] is True
        assert arm.executor.nodes == [arm.node]
        assert arm.description.name == "test_robot"
        assert arm.arm.kwargs["group_name"] == "arm"


def test_a_motion_returns_once_the_robot_arrived(fake_ros):
    with session_module.connect() as arm:
        assert arm.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0])
        assert arm.arm.waits == 1
        assert arm.arm.pose_call["position"] == [0.4, 0.0, 0.4]


def test_no_wait_reports_only_that_the_goal_was_sent(fake_ros):
    with session_module.connect() as arm:
        assert arm.move_to_pose([0.4, 0.0, 0.4], wait=False)
        assert arm.arm.waits == 0


def test_a_rejected_goal_is_never_waited_on(fake_ros):
    with session_module.connect() as arm:
        arm.arm.submitted = False
        assert not arm.move_to_pose([0.4, 0.0, 0.4])
        assert arm.arm.waits == 0


def test_a_configuration_without_a_target_uses_a_group_state(fake_ros):
    with session_module.connect() as arm:
        assert arm.move_to_configuration()
        assert arm.arm.configuration_call[0] == [0.1, 0.2]


def test_the_timeout_is_one_budget_for_planning_and_arrival(fake_ros):
    with session_module.connect() as arm:
        arm.arm.planning_delay = 0.05
        assert arm.move_to_pose([0.4, 0.0, 0.4], timeout_sec=1.0)
        spent = arm.arm.wait_timeouts[0]
        assert spent < 1.0, spent
        assert spent > 0.5, spent


def test_an_expired_budget_still_waits_without_a_negative_timeout(fake_ros):
    with session_module.connect() as arm:
        arm.arm.planning_delay = 0.05
        assert arm.move_to_pose([0.4, 0.0, 0.4], timeout_sec=0.0)
        assert arm.arm.wait_timeouts[0] == 0.0


def test_no_timeout_stays_unbounded(fake_ros):
    with session_module.connect() as arm:
        assert arm.move_to_pose([0.4, 0.0, 0.4])
        assert arm.arm.wait_timeouts[0] is None


def test_a_failed_motion_explains_itself(fake_ros):
    with session_module.connect() as arm:
        arm.arm.executed = False
        assert not arm.move_to_pose([0.4, 0.0, 0.4])
        assert arm.last_failure() == "PLANNING_FAILED: no path."


def test_the_rest_of_the_interface_stays_reachable(fake_ros):
    with session_module.connect() as arm:
        assert arm.add_collision_box(id="box") == "box added"


def test_a_name_that_exists_nowhere_names_both_classes(fake_ros):
    unknown = "move_to_nowhere"
    with session_module.connect() as arm:
        with pytest.raises(AttributeError, match="RobotSession"):
            getattr(arm, unknown)


def test_closing_stops_the_executor_before_it_destroys_anything(fake_ros):
    arm = session_module.connect()
    node = arm.node
    assert arm.close()
    assert fake_ros.events == ["executor_shutdown", "arm_destroy"]
    assert node.destroyed
    assert fake_ros.state["shutdown"] is False


def test_closing_twice_is_harmless(fake_ros):
    arm = session_module.connect()
    assert arm.close()
    assert arm.close()
    assert fake_ros.events.count("arm_destroy") == 1


def test_a_session_that_started_ros_also_stops_it(fake_ros):
    fake_ros.state["initialized"] = False
    arm = session_module.connect()
    assert arm.close()
    assert fake_ros.state["shutdown"] is True


def test_a_missing_move_group_says_what_to_start(fake_ros, monkeypatch):
    def unavailable(cls, node, **kwargs):
        raise RuntimeError("timed out")

    monkeypatch.setattr(
        session_module.RobotDescription, "from_node", classmethod(unavailable)
    )
    with pytest.raises(RuntimeError, match="Start MoveIt 2 first"):
        session_module.connect(timeout_sec=0.1)

    assert fake_ros.state["nodes"][-1].destroyed


def test_the_exported_names_are_importable():
    import pymoveit2

    assert pymoveit2.connect is session_module.connect
    assert pymoveit2.RobotSession is session_module.RobotSession
