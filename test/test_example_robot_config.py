import pytest

from pymoveit2 import _example_utils
from pymoveit2._example_utils import (
    DEFAULT_DESCRIPTION_NODE_NAME,
    DEFAULT_DESCRIPTION_NODE_PARAMETER,
    DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER,
    RobotConfiguration,
    declare_robot_parameters,
)
from pymoveit2.robot_description import (
    DEFAULT_DESCRIPTION_NODE_NAME as DESCRIPTION_NODE_NAME,
)

ARM_KWARGS = {
    "joint_names": ["joint1", "joint2"],
    "base_link_name": "base",
    "end_effector_name": "tip",
    "group_name": "arm",
}
GRIPPER_KWARGS = {
    "gripper_joint_names": ["finger"],
    "open_gripper_joint_positions": [0.04],
    "closed_gripper_joint_positions": [0.0],
    "gripper_group_name": "hand",
}


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, overrides=None):
        self.declared = {}
        self.overrides = dict(overrides or {})
        self._logger = _Logger()

    def declare_parameter(self, name, value):
        self.declared[name] = value
        return _Parameter(self.overrides.get(name))

    def get_parameter(self, name):
        if name not in self.declared:
            raise KeyError(name)
        if name in self.overrides:
            return _Parameter(self.overrides[name])
        default = self.declared[name]

        if type(default).__name__ == "Type":
            raise RuntimeError(f"parameter '{name}' is not initialized")
        return _Parameter(default)

    def get_logger(self):
        return self._logger


class _Description:
    name = "fake_robot"
    group_names = ["arm", "hand"]

    def __init__(self):
        self.arm_calls = []
        self.gripper_calls = []
        self.state_calls = []

    def moveit2_kwargs(self, group_name=None):
        self.arm_calls.append(group_name)
        return dict(ARM_KWARGS)

    def moveit2_gripper_kwargs(self, group_name=None):
        self.gripper_calls.append(group_name)
        return dict(GRIPPER_KWARGS)

    def joint_positions(self, state_name=None, group_name=None):
        self.state_calls.append((state_name, group_name))
        if state_name == "unknown":
            raise ValueError(f"Group '{group_name}' has no state 'unknown'.")
        return [0.1, 0.2]


@pytest.fixture
def discovery(monkeypatch):
    calls = []
    description = _Description()

    class _RobotDescription:
        @staticmethod
        def from_node(node, **kwargs):
            calls.append(kwargs)
            return description

    monkeypatch.setattr(
        "pymoveit2.robot_description.RobotDescription", _RobotDescription
    )
    return calls, description


def test_local_description_node_name_matches_the_library_default():
    assert DEFAULT_DESCRIPTION_NODE_NAME == DESCRIPTION_NODE_NAME


def test_declare_robot_parameters_declares_typed_overrides():
    from rclpy.parameter import Parameter

    node = _Node()
    declare_robot_parameters(node, gripper=True, frame_id=True)

    assert node.declared[DEFAULT_DESCRIPTION_NODE_PARAMETER] == (
        DEFAULT_DESCRIPTION_NODE_NAME
    )
    assert node.declared[DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER] == 10.0
    assert node.declared["joint_names"] is Parameter.Type.STRING_ARRAY
    assert node.declared["group_name"] is Parameter.Type.STRING
    assert node.declared["open_gripper_joint_positions"] is Parameter.Type.DOUBLE_ARRAY
    assert node.declared["frame_id"] is Parameter.Type.STRING


def test_arm_configuration_is_discovered_when_nothing_is_overridden(discovery):
    calls, description = discovery
    node = _Node()
    declare_robot_parameters(node)

    kwargs = RobotConfiguration(node).moveit2_kwargs()

    assert kwargs == ARM_KWARGS
    assert description.arm_calls == [None]
    assert calls[0]["remote_node_name"] == DEFAULT_DESCRIPTION_NODE_NAME
    assert calls[0]["timeout_sec"] == 10.0


def test_discovery_uses_the_configured_node_and_timeout(discovery):
    calls, _ = discovery
    node = _Node(
        {
            DEFAULT_DESCRIPTION_NODE_PARAMETER: "/robot/move_group",
            DEFAULT_DESCRIPTION_TIMEOUT_PARAMETER: 2.5,
        }
    )
    declare_robot_parameters(node)

    RobotConfiguration(node).moveit2_kwargs()

    assert calls[0]["remote_node_name"] == "/robot/move_group"
    assert calls[0]["timeout_sec"] == 2.5


def test_complete_overrides_skip_discovery(discovery):
    calls, _ = discovery
    node = _Node(
        {
            "joint_names": ["a", "b"],
            "base_link_name": "root",
            "end_effector_name": "hand",
            "group_name": "manipulator",
        }
    )
    declare_robot_parameters(node)

    kwargs = RobotConfiguration(node).moveit2_kwargs()

    assert kwargs == {
        "joint_names": ["a", "b"],
        "base_link_name": "root",
        "end_effector_name": "hand",
        "group_name": "manipulator",
    }
    assert calls == []


def test_partial_override_selects_the_group_and_wins_over_discovery(discovery):
    _, description = discovery
    node = _Node({"group_name": "manipulator", "end_effector_name": "grip"})
    declare_robot_parameters(node)

    kwargs = RobotConfiguration(node).moveit2_kwargs()

    assert description.arm_calls == ["manipulator"]
    assert kwargs["end_effector_name"] == "grip"
    assert kwargs["joint_names"] == ARM_KWARGS["joint_names"]


def test_description_is_discovered_once_per_configuration(discovery):
    calls, _ = discovery
    node = _Node()
    declare_robot_parameters(node, gripper=True)
    configuration = RobotConfiguration(node)

    configuration.moveit2_kwargs()
    configuration.gripper_kwargs()

    assert len(calls) == 1


def test_gripper_configuration_is_discovered_and_overridable(discovery):
    _, description = discovery
    node = _Node({"closed_gripper_joint_positions": [0.01]})
    declare_robot_parameters(node, arm=False, gripper=True)

    kwargs = RobotConfiguration(node).gripper_kwargs()

    assert description.gripper_calls == [None]
    assert kwargs["closed_gripper_joint_positions"] == [0.01]
    assert kwargs["gripper_joint_names"] == GRIPPER_KWARGS["gripper_joint_names"]


def test_frame_id_falls_back_to_the_discovered_base_link(discovery):
    _, _ = discovery
    node = _Node()
    declare_robot_parameters(node, frame_id=True)

    assert RobotConfiguration(node).frame_id() == ARM_KWARGS["base_link_name"]


def test_frame_id_override_skips_discovery(discovery):
    calls, _ = discovery
    node = _Node({"frame_id": "world"})
    declare_robot_parameters(node, frame_id=True)

    assert RobotConfiguration(node).frame_id() == "world"
    assert calls == []


def test_joint_positions_fall_back_to_a_group_state(discovery):
    _, description = discovery
    node = _Node()
    declare_robot_parameters(node)

    assert RobotConfiguration(node).joint_positions() == [0.1, 0.2]
    assert description.state_calls == [(None, "arm")]


def test_joint_positions_prefer_the_parameter_and_then_the_default(discovery):
    _, description = discovery
    node = _Node({"joint_positions": [1.0, 2.0]})
    declare_robot_parameters(node)
    node.declare_parameter("joint_positions", [])
    configuration = RobotConfiguration(node)

    assert configuration.joint_positions() == [1.0, 2.0]
    assert configuration.joint_positions("goal_joint_positions", default=[3.0]) == [3.0]
    assert description.state_calls == []


def test_joint_positions_name_the_parameter_to_pass_instead(discovery):
    node = _Node()
    declare_robot_parameters(node)

    with pytest.raises(ValueError, match="Pass `initial_joint_positions` instead."):
        RobotConfiguration(node).joint_positions(
            "initial_joint_positions", state_name="unknown"
        )


def test_failed_discovery_names_the_node_to_start(monkeypatch):
    class _Failing:
        @staticmethod
        def from_node(node, **kwargs):
            raise TimeoutError("Service 'move_group/get_parameters' is not available.")

    monkeypatch.setattr("pymoveit2.robot_description.RobotDescription", _Failing)
    node = _Node()
    declare_robot_parameters(node)

    with pytest.raises(RuntimeError, match="robot_description_node"):
        RobotConfiguration(node).moveit2_kwargs()


class _Pose:
    class _Vector:
        def __init__(self, **values):
            self.__dict__.update(values)

    def __init__(self):
        self.pose = _Pose._Vector(
            position=_Pose._Vector(x=0.1, y=0.2, z=0.3),
            orientation=_Pose._Vector(x=0.0, y=0.0, z=0.0, w=1.0),
        )


_DEFAULT_POSE = _Pose()


class _MoveIt2:
    def __init__(self, pose=_DEFAULT_POSE):
        self.pose = pose
        self.calls = []

    def compute_fk(self, joint_positions, timeout_sec=None):
        self.calls.append((list(joint_positions), timeout_sec))
        return self.pose


def test_complete_pose_keeps_a_pose_that_was_given():
    moveit2 = _MoveIt2()

    assert _example_utils.complete_pose(
        moveit2, [0.0], 1.0, _Node(), [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]
    ) == ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    assert moveit2.calls == []


def test_complete_pose_fills_missing_values_from_forward_kinematics():
    moveit2 = _MoveIt2()

    position, quat_xyzw = _example_utils.complete_pose(
        moveit2, [0.5], 2.0, _Node(), None, [0.0, 0.0, 1.0, 0.0]
    )

    assert position == [0.1, 0.2, 0.3]
    assert quat_xyzw == [0.0, 0.0, 1.0, 0.0]
    assert moveit2.calls == [([0.5], 2.0)]

    assert _example_utils.complete_pose(moveit2, [0.5], 2.0, _Node()) == (
        [0.1, 0.2, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    )


def test_complete_pose_reports_a_failed_lookup():
    with pytest.raises(RuntimeError, match="Pass `position` and `quat_xyzw` instead"):
        _example_utils.complete_pose(_MoveIt2(pose=None), [0.0], 1.0, _Node())


@pytest.mark.parametrize("value", [None, "", [], ()])
def test_empty_parameter_values_are_treated_as_unset(value):
    node = _Node({"group_name": value})
    node.declared["group_name"] = "unused"

    assert _example_utils.parameter_value(node, "group_name") is None


def test_undeclared_parameter_is_treated_as_unset():
    assert _example_utils.parameter_value(_Node(), "group_name") is None
