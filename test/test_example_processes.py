import ast
import os
import shutil
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).parent.parent
EXAMPLES = sorted((REPO / "examples").glob("ex_*.py"))


def _run_fake_process(body: str) -> subprocess.CompletedProcess[str]:
    fake_imports = """
import sys
import types

rclpy = types.ModuleType("rclpy")
rclpy.ok = lambda: True
rclpy.executors = types.SimpleNamespace(MultiThreadedExecutor=object)
rclpy.callback_groups = types.ModuleType("rclpy.callback_groups")
rclpy.callback_groups.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
rclpy.node = types.ModuleType("rclpy.node")
rclpy.node.Node = type("Node", (), {})

class ParameterTypeSentinel:
    def __init__(self, name):
        self.name = name


rclpy.parameter = types.ModuleType("rclpy.parameter")
rclpy.parameter.Parameter = type(
    "Parameter",
    (),
    {
        "Type": types.SimpleNamespace(
            STRING=ParameterTypeSentinel("string"),
            STRING_ARRAY=ParameterTypeSentinel("string_array"),
            DOUBLE_ARRAY=ParameterTypeSentinel("double_array"),
        )
    },
)
sys.modules["rclpy"] = rclpy
sys.modules["rclpy.callback_groups"] = rclpy.callback_groups
sys.modules["rclpy.node"] = rclpy.node
sys.modules["rclpy.parameter"] = rclpy.parameter

pymoveit2 = types.ModuleType("pymoveit2")
pymoveit2.__path__ = [__HELPER_PATH__]
pymoveit2.MoveIt2 = type("MoveIt2", (), {})
pymoveit2.MoveIt2Servo = type("MoveIt2Servo", (), {})
pymoveit2.MoveIt2State = types.SimpleNamespace(IDLE="IDLE", EXECUTING="EXECUTING")
sys.modules["pymoveit2"] = pymoveit2
pymoveit2.GripperInterface = type("GripperInterface", (), {})

class FakeRobotDescription:
    name = "fake_robot"
    group_names = ["arm", "gripper"]

    @classmethod
    def from_node(cls, node, **kwargs):
        return cls()

    def moveit2_kwargs(self, group_name=None):
        return {
            "joint_names": [],
            "base_link_name": "base",
            "end_effector_name": "tip",
            "group_name": group_name or "arm",
        }

    def moveit2_gripper_kwargs(self, group_name=None):
        return {
            "gripper_joint_names": [],
            "open_gripper_joint_positions": [],
            "closed_gripper_joint_positions": [],
            "gripper_group_name": group_name or "gripper",
        }

    def joint_positions(self, state_name=None, group_name=None):
        return [0.0, 0.0]


robot_description = types.ModuleType("pymoveit2.robot_description")
robot_description.RobotDescription = FakeRobotDescription
robot_description.DEFAULT_DESCRIPTION_NODE_NAME = "move_group"
sys.modules["pymoveit2.robot_description"] = robot_description
"""
    fake_imports = fake_imports.replace(
        "__HELPER_PATH__", repr(str(REPO / "pymoveit2"))
    )
    script = fake_imports + "\n" + textwrap.dedent(body)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(REPO)
    return subprocess.run(
        [sys.executable, "-c", script],
        cwd=REPO,
        env=env,
        capture_output=True,
        text=True,
        timeout=3.0,
        check=False,
    )


def test_async_fk_wait_handles_fast_and_never_completing_futures():
    result = _run_fake_process("""
        import time
        from examples import ex_fk

        class Future:
            def __init__(self, complete=False):
                self._complete = complete
            def done(self):
                return self._complete
            def add_done_callback(self, callback):
                if self._complete:
                    callback(self)

        assert ex_fk._wait_for_future(Future(True), 0.1)
        started = time.monotonic()
        assert not ex_fk._wait_for_future(Future(False), 0.05)
        assert time.monotonic() - started < 1.0
        """)
    assert result.returncode == 0, result.stderr


def test_joint_wait_handles_rejection_and_never_completing_motion():
    result = _run_fake_process("""
        import time
        from examples import ex_joint_goal

        class Logger:
            def error(self, message):
                pass
            def info(self, message):
                pass

        class Node:
            def get_logger(self):
                return Logger()

        class Motion:
            def __init__(self):
                self.cancels = 0
            def wait_until_executed(self, timeout_sec):
                return False
            def query_state(self):
                return "EXECUTING"
            def cancel_execution(self):
                self.cancels += 1
                return True

        motion = Motion()
        started = time.monotonic()
        assert not ex_joint_goal._wait_for_motion(motion, 0.01, 0.05, Node())
        assert motion.cancels == 1
        assert time.monotonic() - started < 1.0
        """)
    assert result.returncode == 0, result.stderr


def test_scene_wait_confirms_remote_edit_and_times_out_on_frozen_backend():
    result = _run_fake_process("""
        from types import SimpleNamespace
        from examples import ex_collision_primitive

        class Scene:
            def __init__(self):
                self.world = SimpleNamespace(collision_objects=[])

        class MoveIt:
            def __init__(self, present):
                self.planning_scene = Scene()
                self.present = present
            def update_planning_scene(self, timeout_sec):
                if self.present:
                    self.planning_scene.world.collision_objects = [
                        SimpleNamespace(id="box")
                    ]
                    return True
                return False

        applied = []
        assert ex_collision_primitive.wait_for_scene_object(
            MoveIt(True), "box", True, 0.2, lambda: applied.append(True)
        )
        assert applied
        assert not ex_collision_primitive.wait_for_scene_object(
            MoveIt(False), "box", True, 0.03, lambda: None
        )
        """)
    assert result.returncode == 0, result.stderr


def _run_fake_main(example: str, mode: str, expected: int):
    body = r"""
import importlib
import time

MODE = "__MODE__"
EXPECTED = __EXPECTED__
TIMEOUT_SEC = 2.0 if MODE == "delayed_discovery" else 0.05
ros_state = {"ok": True}

rclpy.ok = lambda: ros_state["ok"]
rclpy.init = lambda: None
rclpy.shutdown = lambda: ros_state.update(ok=False)


class Executor:
    last = None

    def __init__(self, *args, **kwargs):
        self.shutdown_called = False
        Executor.last = self

    def add_node(self, node):
        self.node = node

    def spin(self):
        return None

    def shutdown(self, timeout_sec=None):
        self.shutdown_called = True
        return True


rclpy.executors.MultiThreadedExecutor = Executor


class Logger:
    def info(self, message):
        pass

    def error(self, message):
        pass


class ParameterValue:
    def __init__(self, value):
        self.bool_value = value if isinstance(value, bool) else False
        self.integer_value = value if isinstance(value, int) and not isinstance(value, bool) else 0
        self.double_value = value if isinstance(value, (int, float)) and not isinstance(value, bool) else 0.0
        self.string_value = value if isinstance(value, str) else ""
        self.double_array_value = list(value) if isinstance(value, (list, tuple)) else []


class Parameter:
    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        return ParameterValue(self.value)


_UNSET = object()


class FakeNode:
    last = None
    overrides = {
        "synchronous": MODE == "delayed_discovery",
        "timeout_sec": TIMEOUT_SEC,
        "cancel_after_secs": 0.0,
    }

    def __init__(self, name):
        self._params = {}
        self._logger = Logger()
        self.destroyed = False
        FakeNode.last = self

    def declare_parameter(self, name, default, *args, **kwargs):
        if type(default).__name__ == "ParameterTypeSentinel":

            value = self.overrides.get(name, _UNSET)
        else:
            value = self.overrides.get(name, default)
        self._params[name] = value
        return Parameter(None if value is _UNSET else value)

    def get_parameter(self, name):
        value = self._params[name]
        if value is _UNSET:
            raise RuntimeError(f"parameter '{name}' is not initialized")
        return Parameter(value)

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        self.destroyed = True


class Future:
    def __init__(self):
        self._done = MODE in ("fast", "failed_result", "exception")

    def done(self):
        return self._done

    def add_done_callback(self, callback):
        if self._done:
            callback(self)


class FakeVector:
    x = y = z = 0.0
    w = 1.0


class FakePose:
    pose = type(
        "Pose", (), {"position": FakeVector(), "orientation": FakeVector()}
    )()


class FakeMoveIt2:
    last = None

    def __init__(self, *args, **kwargs):
        self.plan_timeouts = []
        self.wait_timeouts = []
        self.future_timeouts = []
        self.destroyed = False
        FakeMoveIt2.last = self

    def compute_fk(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("timeout_sec"))

        return FakePose()

    def compute_fk_async(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("wait_for_server_timeout_sec"))
        if MODE == "rejected":
            return None
        return Future()

    def get_compute_fk_result(self, future):
        if MODE == "failed_result":
            return None
        if MODE == "exception":
            raise RuntimeError("fake FK result failure")
        return object()

    def compute_ik(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("timeout_sec"))

        if MODE == "delayed_discovery" and kwargs.get("wait_for_server_timeout_sec", 1.0) < 1.5:
            return None
        return object()

    def compute_ik_async(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("wait_for_server_timeout_sec"))
        if MODE == "rejected":
            return None
        return Future()

    def get_compute_ik_result(self, future):
        if MODE == "failed_result":
            return None
        if MODE == "exception":
            raise RuntimeError("fake IK result failure")
        return object()

    def move_to_configuration(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("timeout_sec"))
        return MODE != "rejected"

    def move_to_pose(self, *args, **kwargs):
        self.plan_timeouts.append(kwargs.get("timeout_sec"))
        return MODE != "rejected"

    def wait_until_executed(self, timeout_sec=None):
        self.wait_timeouts.append(timeout_sec)
        if MODE == "fast":
            return True
        if MODE == "exception":
            raise RuntimeError("fake execution failure")
        if MODE == "never":
            time.sleep(min(timeout_sec or 0.0, TIMEOUT_SEC))
        return False

    def query_state(self):
        return "IDLE"

    def cancel_execution(self):
        return True

    def destroy(self):
        self.destroyed = True


module = importlib.import_module("examples.__EXAMPLE__")
module.Node = FakeNode
module.MoveIt2 = FakeMoveIt2

if hasattr(module, "_wait_for_future"):
    original_wait_for_future = module._wait_for_future

    def tracked_wait_for_future(future, timeout_sec):
        FakeMoveIt2.last.future_timeouts.append(timeout_sec)
        return original_wait_for_future(future, timeout_sec)

    module._wait_for_future = tracked_wait_for_future

started = time.monotonic()
try:
    sys.exit(module.main())
except SystemExit as error:
    assert error.code == EXPECTED, error.code
    assert FakeMoveIt2.last is not None
    assert FakeMoveIt2.last.destroyed
    assert FakeNode.last is not None and FakeNode.last.destroyed
    assert Executor.last is not None and Executor.last.shutdown_called
    assert FakeMoveIt2.last.plan_timeouts
    assert all(
        value is not None and 0.0 <= value <= TIMEOUT_SEC
        for value in FakeMoveIt2.last.plan_timeouts
    )
    assert all(
        value is not None and 0.0 <= value <= TIMEOUT_SEC
        for value in FakeMoveIt2.last.wait_timeouts
    )
    assert all(
        value is not None and 0.0 <= value <= TIMEOUT_SEC
        for value in FakeMoveIt2.last.future_timeouts
    )
    assert time.monotonic() - started < 1.0
"""
    body = body.replace("__EXAMPLE__", example)
    body = body.replace("__MODE__", mode)
    body = body.replace("__EXPECTED__", str(expected))
    return _run_fake_process(body)


@pytest.mark.parametrize(
    ("example", "mode", "expected"),
    [
        ("ex_fk", "fast", 0),
        ("ex_fk", "rejected", 1),
        ("ex_fk", "failed_result", 1),
        ("ex_fk", "exception", 1),
        ("ex_fk", "never", 1),
        ("ex_ik", "fast", 0),
        ("ex_ik", "delayed_discovery", 0),
        ("ex_ik", "rejected", 1),
        ("ex_ik", "failed_result", 1),
        ("ex_ik", "exception", 1),
        ("ex_ik", "never", 1),
        ("ex_joint_goal", "fast", 0),
        ("ex_joint_goal", "rejected", 1),
        ("ex_joint_goal", "failed_result", 1),
        ("ex_joint_goal", "exception", 1),
        ("ex_joint_goal", "never", 1),
        ("ex_pose_goal", "fast", 0),
        ("ex_pose_goal", "rejected", 1),
        ("ex_pose_goal", "failed_result", 1),
        ("ex_pose_goal", "exception", 1),
        ("ex_pose_goal", "never", 1),
    ],
)
def test_real_example_main_process_status_and_bounded_deadline(example, mode, expected):
    result = _run_fake_main(example, mode, expected)
    assert result.returncode == 0, result.stderr


def test_cleanup_stops_executor_before_destroying_ros_entities():
    result = _run_fake_process("""
        from pymoveit2._example_utils import cleanup

        events = []

        class Interface:
            def shutdown(self, timeout_sec=1.0):
                events.append("ack")
                return True
            def destroy(self):
                events.append("interface_destroy")

        class Executor:
            def shutdown(self, timeout_sec=None):
                events.append("executor_shutdown")
                return True

        class ExecutorThread:
            def join(self, timeout=None):
                events.append("join")
            def is_alive(self):
                events.append("is_alive")
                return False

        class Node:
            def destroy_node(self):
                events.append("node_destroy")

        interface = Interface()
        assert not cleanup(
            interface,
            Executor(),
            ExecutorThread(),
            "fake",
            node=Node(),
            ros_ok=lambda: True,
            ros_shutdown=lambda: events.append("ros_shutdown"),
            acknowledged_shutdown=interface.shutdown,
        )
        assert events == [
            "ack",
            "executor_shutdown",
            "join",
            "is_alive",
            "interface_destroy",
            "node_destroy",
            "ros_shutdown",
        ], events
        """)
    assert result.returncode == 0, result.stderr


def test_cleanup_does_not_destroy_entities_when_executor_stays_active():
    result = _run_fake_process("""
        from pymoveit2._example_utils import cleanup

        class Interface:
            def __init__(self):
                self.destroyed = False
            def destroy(self):
                self.destroyed = True

        class Executor:
            def shutdown(self, timeout_sec=None):
                return False

        class ExecutorThread:
            def join(self, timeout=None):
                pass
            def is_alive(self):
                return True

        class Node:
            def __init__(self):
                self.destroyed = False
            def destroy_node(self):
                self.destroyed = True

        interface = Interface()
        node = Node()
        assert cleanup(
            interface,
            Executor(),
            ExecutorThread(),
            "fake",
            node=node,
            ros_ok=lambda: True,
            ros_shutdown=lambda: None,
        )
        assert not interface.destroyed
        assert not node.destroyed
        """)
    assert result.returncode == 0, result.stderr


def test_cleanup_reports_executor_failure_without_unsafe_destroy():
    result = _run_fake_process("""
        from pymoveit2._example_utils import cleanup

        class Interface:
            def __init__(self):
                self.destroyed = False
            def destroy(self):
                self.destroyed = True

        class Executor:
            def shutdown(self, timeout_sec=None):
                return False

        class Node:
            def __init__(self):
                self.destroyed = False
            def destroy_node(self):
                self.destroyed = True

        interface = Interface()
        node = Node()
        assert cleanup(
            interface,
            Executor(),
            None,
            "fake",
            node=node,
            ros_ok=lambda: True,
            ros_shutdown=lambda: None,
        )
        assert not interface.destroyed
        assert not node.destroyed
        """)
    assert result.returncode == 0, result.stderr


def test_servo_startup_failures_honor_startup_deadline():
    result = _run_fake_process("""
        import time
        from types import SimpleNamespace
        from examples import ex_servo

        state = {"ok": True}
        rclpy.init = lambda: None
        rclpy.ok = lambda: state["ok"]
        rclpy.shutdown = lambda: state.update(ok=False)

        class Logger:
            def info(self, message):
                pass
            def error(self, message):
                pass

        class Node:
            def __init__(self, name):
                self.params = {}
                self.logger = Logger()
                self.destroyed = False
            def declare_parameter(self, name, default):
                self.params[name] = {
                    "startup_timeout_sec": 0.1,
                    "command_failure_limit": 1,
                    "shutdown_timeout_sec": 0.01,
                }.get(name, default)
            def get_parameter(self, name):
                return SimpleNamespace(value=self.params[name])
            def get_logger(self):
                return self.logger
            def destroy_node(self):
                self.destroyed = True

        class Executor:
            def __init__(self, *args, **kwargs):
                self.shutdown_called = False
            def add_node(self, node):
                self.node = node
            def spin(self):
                return None
            def shutdown(self, timeout_sec=None):
                self.shutdown_called = True
                return True

        class Servo:
            last = None
            def __init__(self, *args, **kwargs):
                self.calls = 0
                self.destroyed = False
                Servo.last = self
            def enable(self, wait_for_server_timeout_sec=None):
                return True
            def wait_until_ready(self, timeout_sec=None):
                return True
            def __call__(self, **kwargs):
                self.calls += 1
                return False
            def shutdown(self, timeout_sec=1.0):
                return True
            def destroy(self):
                self.destroyed = True

        rclpy.executors.MultiThreadedExecutor = Executor
        ex_servo.Node = Node
        ex_servo.MoveIt2Servo = Servo
        started = time.monotonic()
        try:
            sys.exit(ex_servo.main())
        except SystemExit as error:
            assert error.code == 1
            assert time.monotonic() - started >= 0.1
            assert time.monotonic() - started < 1.0
            assert Servo.last.calls >= 2
            assert Servo.last.destroyed
        """)
    assert result.returncode == 0, result.stderr


def test_all_examples_return_process_status_and_use_wall_clock_bounds():
    for path in EXAMPLES:
        tree = ast.parse(path.read_text(), filename=str(path))
        main_functions = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "main"
        ]
        assert main_functions, path.name
        assert "time.monotonic" in path.read_text(), path.name
        assert "finally" in path.read_text(), path.name
        assert "sys.exit(main())" in path.read_text(), path.name
        assert "EXECUTING" not in path.read_text(), path.name
        assert any(
            isinstance(node, ast.Return) for node in ast.walk(main_functions[0])
        ), path.name


def _run_installed_example(example: str):
    if os.environ.get("PYMOVEIT2_INSTALLED_EXAMPLES") != "1":
        pytest.skip(
            "requires an installed ROS package and an isolated no-server container"
        )
    ros2 = shutil.which("ros2")
    assert ros2 is not None, "ros2 is required to run the installed examples"
    return subprocess.run(
        [
            ros2,
            "run",
            "pymoveit2",
            example,
            "--ros-args",
            "-p",
            "timeout_sec:=0.5",
            "-p",
            "robot_description_timeout_sec:=0.5",
        ],
        text=True,
        capture_output=True,
        timeout=10.0,
        check=False,
    )


@pytest.mark.parametrize(
    "example",
    [
        "ex_doctor.py",
        "ex_fk.py",
        "ex_ik.py",
        "ex_joint_goal.py",
        "ex_pose_goal.py",
        "ex_gripper.py",
    ],
)
def test_installed_examples_without_a_server_exit_one(example):
    result = _run_installed_example(example)
    output = result.stdout + result.stderr
    assert result.returncode == 1, output
    assert "Unable to read the robot description" in output, output
    assert "robot_description_node" in output, output
