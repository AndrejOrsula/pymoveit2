import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

import pymoveit2.robot_description as robot_description_module
from pymoveit2 import MoveIt2, RobotDescription
from pymoveit2.robots import panda

URDF = """
<robot name="panda">
  <link name="panda_link0"/>
  <link name="panda_link1"/>
  <link name="panda_link2"/>
  <link name="panda_link3"/>
  <link name="panda_link4"/>
  <link name="panda_link5"/>
  <link name="panda_link6"/>
  <link name="panda_link7"/>
  <link name="panda_link8"/>
  <link name="panda_hand"/>
  <link name="panda_hand_tcp"/>
  <link name="panda_leftfinger"/>
  <link name="panda_rightfinger"/>
  <joint name="panda_joint1" type="revolute">
    <parent link="panda_link0"/><child link="panda_link1"/>
  </joint>
  <joint name="panda_joint2" type="revolute">
    <parent link="panda_link1"/><child link="panda_link2"/>
  </joint>
  <joint name="panda_joint3" type="revolute">
    <parent link="panda_link2"/><child link="panda_link3"/>
  </joint>
  <joint name="panda_joint4" type="revolute">
    <parent link="panda_link3"/><child link="panda_link4"/>
  </joint>
  <joint name="panda_joint5" type="revolute">
    <parent link="panda_link4"/><child link="panda_link5"/>
  </joint>
  <joint name="panda_joint6" type="revolute">
    <parent link="panda_link5"/><child link="panda_link6"/>
  </joint>
  <joint name="panda_joint7" type="revolute">
    <parent link="panda_link6"/><child link="panda_link7"/>
  </joint>
  <joint name="panda_joint8" type="fixed">
    <parent link="panda_link7"/><child link="panda_link8"/>
  </joint>
  <joint name="panda_hand_joint" type="fixed">
    <parent link="panda_link8"/><child link="panda_hand"/>
  </joint>
  <joint name="panda_hand_tcp_joint" type="fixed">
    <parent link="panda_hand"/><child link="panda_hand_tcp"/>
  </joint>
  <joint name="panda_finger_joint1" type="prismatic">
    <parent link="panda_hand"/><child link="panda_leftfinger"/>
  </joint>
  <joint name="panda_finger_joint2" type="prismatic">
    <parent link="panda_hand"/><child link="panda_rightfinger"/>
    <mimic joint="panda_finger_joint1"/>
  </joint>
</robot>
"""

SRDF = """
<robot name="panda">
  <group name="panda_arm">
    <chain base_link="panda_link0" tip_link="panda_link8"/>
  </group>
  <group name="hand">
    <link name="panda_hand"/>
    <link name="panda_leftfinger"/>
    <link name="panda_rightfinger"/>
    <joint name="panda_finger_joint1"/>
    <passive_joint name="panda_finger_joint2"/>
  </group>
  <group name="panda_arm_hand">
    <group name="panda_arm"/>
    <group name="hand"/>
  </group>
  <group_state group="panda_arm" name="ready">
    <joint name="panda_joint2" value="-0.785"/>
    <joint name="panda_joint4" value="-2.356"/>
    <joint name="panda_joint6" value="1.571"/>
    <joint name="panda_joint7" value="0.785"/>
  </group_state>
  <group_state group="hand" name="open">
    <joint name="panda_finger_joint1" value="0.035"/>
    <joint name="panda_finger_joint2" value="0.035"/>
  </group_state>
  <group_state group="hand" name="close">
    <joint name="panda_finger_joint1" value="0"/>
    <joint name="panda_finger_joint2" value="0"/>
  </group_state>
  <end_effector group="hand" name="hand" parent_group="panda_arm" parent_link="panda_link8"/>
</robot>
"""


@pytest.fixture()
def description() -> RobotDescription:
    return RobotDescription(URDF, SRDF)


def test_groups_are_parsed(description):
    assert description.name == "panda"
    assert sorted(description.group_names) == ["hand", "panda_arm", "panda_arm_hand"]


def test_chain_group_matches_the_preset(description):
    arm = description.group("panda_arm")
    assert arm.joint_names == panda.joint_names()
    assert arm.base_link_name == panda.base_link_name()
    assert arm.end_effector_name == "panda_link8"
    assert arm.link_names[0] == "panda_link0"
    assert arm.link_names[-1] == "panda_link8"
    assert not arm.passive_joint_names


def test_link_and_joint_group_separates_passive_joints(description):
    hand = description.group("hand")
    assert hand.joint_names == ["panda_finger_joint1"]
    assert hand.passive_joint_names == ["panda_finger_joint2"]
    assert hand.movable_joint_names == panda.gripper_joint_names()
    assert hand.base_link_name == "panda_hand"


def test_subgroups_are_flattened_in_kinematic_order(description):
    combined = description.group("panda_arm_hand")
    assert combined.subgroup_names == ["panda_arm", "hand"]
    assert combined.joint_names == [*panda.joint_names(), "panda_finger_joint1"]
    assert combined.passive_joint_names == ["panda_finger_joint2"]


def test_arm_and_gripper_groups_are_detected(description):
    assert description.arm_group_name == "panda_arm"
    assert description.gripper_group_name == "hand"


def test_groups_are_detected_without_an_end_effector_element():
    srdf = """
    <robot name="panda">
      <group name="manipulator">
        <chain base_link="panda_link0" tip_link="panda_link8"/>
      </group>
      <group name="gripper"><joint name="panda_finger_joint1"/></group>
    </robot>
    """
    description = RobotDescription(URDF, srdf)
    assert description.gripper_group_name == "gripper"
    assert description.arm_group_name == "manipulator"


def test_moveit2_kwargs_configure_the_interface(description, rclpy_node):
    kwargs = description.moveit2_kwargs()
    assert kwargs == {
        "joint_names": panda.joint_names(),
        "base_link_name": "panda_link0",
        "end_effector_name": "panda_link8",
        "group_name": "panda_arm",
    }
    interface = MoveIt2(
        node=rclpy_node, callback_group=ReentrantCallbackGroup(), **kwargs
    )
    try:
        assert interface.joint_names == panda.joint_names()
    finally:
        interface.destroy()


def test_gripper_kwargs_use_the_srdf_states(description):
    assert description.moveit2_gripper_kwargs() == {
        "gripper_joint_names": panda.gripper_joint_names(),
        "open_gripper_joint_positions": [0.035, 0.035],
        "closed_gripper_joint_positions": [0.0, 0.0],
        "gripper_group_name": "hand",
    }


def test_robot_level_passive_joints_are_honoured():
    srdf = """
    <robot name="panda">
      <group name="arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
      <passive_joint name="panda_joint7"/>
    </robot>
    """
    group = RobotDescription(URDF, srdf).group("arm")
    assert "panda_joint7" not in group.joint_names
    assert group.passive_joint_names == ["panda_joint7"]


def test_srdf_virtual_joints_in_a_group_are_ignored():
    srdf = """
    <robot name="panda">
      <virtual_joint name="virtual_joint" type="fixed" parent_frame="world"
                     child_link="panda_link0"/>
      <group name="arm">
        <joint name="virtual_joint"/>
        <chain base_link="panda_link0" tip_link="panda_link8"/>
      </group>
    </robot>
    """
    group = RobotDescription(URDF, srdf).group("arm")
    assert group.joint_names == panda.joint_names()
    assert "virtual_joint" not in group.movable_joint_names
    assert group.base_link_name == "panda_link0"
    assert group.end_effector_name == "panda_link8"


def test_named_state_positions_default_to_zero(description):
    assert description.group("panda_arm").state_positions("ready") == [
        0.0,
        -0.785,
        0.0,
        -2.356,
        0.0,
        1.571,
        0.785,
    ]


def test_joint_positions_use_a_default_srdf_state(description):
    assert description.joint_positions() == description.group(
        "panda_arm"
    ).state_positions("ready")
    assert description.joint_positions("ready", "panda_arm") == (
        description.joint_positions()
    )


def test_joint_positions_use_the_only_state_of_the_group():
    srdf = """
    <robot name="panda">
      <group name="arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
      <group_state group="arm" name="folded">
        <joint name="panda_joint2" value="-1.0"/>
      </group_state>
    </robot>
    """
    positions = RobotDescription(URDF, srdf).joint_positions()
    assert positions == [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_joint_positions_reject_an_ambiguous_choice():
    srdf = """
    <robot name="panda">
      <group name="arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
      <group_state group="arm" name="left">
        <joint name="panda_joint1" value="-1.0"/>
      </group_state>
      <group_state group="arm" name="right">
        <joint name="panda_joint1" value="1.0"/>
      </group_state>
    </robot>
    """
    description = RobotDescription(URDF, srdf)
    with pytest.raises(ValueError, match="no default state"):
        description.joint_positions()
    assert description.joint_positions("left") == [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_joint_positions_report_an_srdf_without_states():
    srdf = """
    <robot name="panda">
      <group name="arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
    </robot>
    """
    with pytest.raises(ValueError, match="defines no state for group 'arm'"):
        RobotDescription(URDF, srdf).joint_positions()


def test_public_model_observations_are_defensive_copies(description):
    group = description.group("hand")
    group.joint_names.append("caller_mutation")
    group.named_states["open"]["panda_finger_joint1"] = 99.0
    assert description.group("hand").joint_names == ["panda_finger_joint1"]
    assert (
        description.group("hand").named_states["open"]["panda_finger_joint1"] == 0.035
    )

    groups = description.groups
    groups["hand"].passive_joint_names.clear()
    groups["hand"].named_states["close"].clear()
    assert description.group("hand").passive_joint_names == ["panda_finger_joint2"]
    assert description.group("hand").named_states["close"] == {
        "panda_finger_joint1": 0.0,
        "panda_finger_joint2": 0.0,
    }

    kwargs = description.moveit2_gripper_kwargs()
    kwargs["gripper_joint_names"].clear()
    kwargs["open_gripper_joint_positions"][0] = 123.0
    assert description.moveit2_gripper_kwargs()["gripper_joint_names"] == [
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]
    assert description.moveit2_gripper_kwargs()["open_gripper_joint_positions"] == [
        0.035,
        0.035,
    ]


def test_joint_order_is_kinematic_and_xml_independent():
    urdf = """
    <robot name="branched">
      <link name="base"/><link name="a"/><link name="b"/>
      <link name="a_tip"/><link name="b_tip"/>
      <joint name="z_tip" type="revolute">
        <parent link="b"/><child link="b_tip"/>
      </joint>
      <joint name="a_tip_joint" type="revolute">
        <parent link="a"/><child link="a_tip"/>
      </joint>
      <joint name="b" type="revolute">
        <parent link="base"/><child link="b"/>
      </joint>
      <joint name="a" type="revolute">
        <parent link="base"/><child link="a"/>
      </joint>
    </robot>
    """
    reversed_urdf = """
    <robot name="branched">
      <link name="base"/><link name="a"/><link name="b"/>
      <link name="a_tip"/><link name="b_tip"/>
      <joint name="a" type="revolute">
        <parent link="base"/><child link="a"/>
      </joint>
      <joint name="z_tip" type="revolute">
        <parent link="b"/><child link="b_tip"/>
      </joint>
      <joint name="b" type="revolute">
        <parent link="base"/><child link="b"/>
      </joint>
      <joint name="a_tip_joint" type="revolute">
        <parent link="a"/><child link="a_tip"/>
      </joint>
    </robot>
    """
    srdf = """
    <robot name="branched"><group name="all">
      <joint name="z_tip"/><joint name="a_tip_joint"/>
      <joint name="b"/><joint name="a"/>
    </group></robot>
    """
    expected = ["a", "b", "a_tip_joint", "z_tip"]
    assert RobotDescription(urdf, srdf).group("all").joint_names == expected
    assert RobotDescription(reversed_urdf, srdf).group("all").joint_names == expected


@pytest.mark.parametrize(
    "urdf, message",
    [
        (
            "<robot><link name='base'/><link name='base'/></robot>",
            "duplicate link",
        ),
        (
            "<robot><link name='base'/><joint name='j' type='fixed'>"
            "<parent link='missing'/><child link='base'/></joint></robot>",
            "unknown parent link",
        ),
        (
            "<robot><link name='base'/><link name='tip'/><link name='other'/>"
            "<joint name='a' type='fixed'><parent link='base'/><child link='tip'/></joint>"
            "<joint name='b' type='fixed'><parent link='other'/><child link='tip'/></joint></robot>",
            "multiple parent",
        ),
        (
            "<robot><link name='root'/><link name='a'/><link name='b'/>"
            "<joint name='ab' type='fixed'><parent link='a'/><child link='b'/></joint>"
            "<joint name='ba' type='fixed'><parent link='b'/><child link='a'/></joint></robot>",
            "cycle",
        ),
        (
            "<robot><link name='base'/><link name='tip'/>"
            "<joint name='j' type='fixed'><parent link='base'/><child link='tip'/>"
            "<mimic joint='missing'/></joint></robot>",
            "unknown joint",
        ),
        (
            "<robot><link name='base'/><link name='a'/><link name='b'/>"
            "<joint name='ja' type='fixed'><parent link='base'/><child link='a'/>"
            "<mimic joint='jb'/></joint>"
            "<joint name='jb' type='fixed'><parent link='a'/><child link='b'/>"
            "<mimic joint='ja'/></joint></robot>",
            "mimic graph contains a cycle",
        ),
    ],
)
def test_malformed_urdf_graphs_fail_without_hanging(urdf, message):
    with pytest.raises(ValueError, match=message):
        RobotDescription(urdf, "<robot><group name='arm'/></robot>")


def test_multiple_default_gripper_groups_require_explicit_selection():
    srdf = """
    <robot name="panda">
      <group name="arm"><chain base_link="panda_link0" tip_link="panda_link8"/></group>
      <group name="left_gripper"><joint name="panda_finger_joint1"/></group>
      <group name="right_gripper"><joint name="panda_finger_joint2"/></group>
      <end_effector group="left_gripper" name="left" parent_group="arm"
                    parent_link="panda_link8"/>
      <end_effector group="right_gripper" name="right" parent_group="arm"
                    parent_link="panda_link8"/>
      <group_state group="left_gripper" name="open">
        <joint name="panda_finger_joint1" value="0.1"/>
      </group_state>
      <group_state group="left_gripper" name="close">
        <joint name="panda_finger_joint1" value="0.0"/>
      </group_state>
    </robot>
    """
    description = RobotDescription(URDF, srdf)
    with pytest.raises(ValueError, match="pass group_name explicitly"):
        _ = description.gripper_group_name
    assert description.arm_group_name == "arm"
    assert (
        description.moveit2_gripper_kwargs(group_name="left_gripper")[
            "gripper_group_name"
        ]
        == "left_gripper"
    )


@pytest.mark.parametrize(
    "call, message",
    [
        (lambda desc: desc.group("nonexistent"), "Unknown group"),
        (
            lambda desc: desc.group("panda_arm").state_positions("folded"),
            "no state 'folded'",
        ),
        (
            lambda desc: desc.moveit2_gripper_kwargs(group_name="panda_arm"),
            "no open state",
        ),
        (
            lambda desc: desc.moveit2_gripper_kwargs(open_state="ajar"),
            "no state 'ajar'",
        ),
    ],
)
def test_invalid_lookups_are_rejected(description, call, message):
    with pytest.raises(ValueError, match=message):
        call(description)


@pytest.mark.parametrize(
    "urdf, srdf, message",
    [
        ("<robot", SRDF, "Unable to parse the URDF"),
        ("<nonrobot/>", SRDF, "root element is 'nonrobot'"),
        (URDF, "<robot name='x'/>", "does not define any planning group"),
        (
            URDF,
            "<robot name='x'><group name='a'><group name='b'/></group></robot>",
            "unknown subgroup 'b'",
        ),
        (
            URDF,
            "<robot name='x'>"
            "<group name='a'><group name='b'/><group name='b'/></group>"
            "<group name='b'/></robot>",
            "repeats subgroup 'b'",
        ),
        (
            URDF,
            "<robot name='x'>"
            "<group name='a'><group name='b'/></group>"
            "<group name='b'><group name='a'/></group>"
            "</robot>",
            "contains itself as a subgroup",
        ),
        (
            URDF,
            "<robot name='x'><group name='a'>"
            "<chain base_link='panda_hand' tip_link='panda_link0'/>"
            "</group></robot>",
            "is not a descendant of",
        ),
        (
            URDF,
            "<robot name='x'><group name='a'><joint name='nope'/></group></robot>",
            "does not define joint 'nope'",
        ),
    ],
)
def test_invalid_descriptions_are_rejected(urdf, srdf, message):
    with pytest.raises(ValueError, match=message):
        RobotDescription(urdf, srdf)


class _FakeParameterClient:
    def __init__(self, values, ready: bool = True):
        self.values = values
        self.ready = ready
        self.requests = []
        self.removed_requests = []

    def wait_for_service(self, timeout_sec=None) -> bool:
        return self.ready

    def call_async(self, request) -> Future:
        self.requests.append(request)
        future = Future()
        future.set_result(type("Response", (), {"values": list(self.values)})())
        return future

    def remove_pending_request(self, future) -> None:
        self.removed_requests.append(future)


class _FakeNode:
    def __init__(self, client):
        self.client = client
        self.destroyed = False

    def create_client(self, srv_type, srv_name, callback_group=None):
        self.service_name = srv_name
        return self.client

    def destroy_client(self, client) -> None:
        self.destroyed = True


def _string_value(value: str) -> ParameterValue:
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)


def test_from_node_reads_the_parameters_of_move_group():
    client = _FakeParameterClient([_string_value(URDF), _string_value(SRDF)])
    node = _FakeNode(client)

    description = RobotDescription.from_node(node)

    assert node.service_name == "move_group/get_parameters"
    assert client.requests[0].names == [
        "robot_description",
        "robot_description_semantic",
    ]
    assert description.arm_group_name == "panda_arm"
    assert node.destroyed


def test_from_node_accepts_unbounded_none_timeout():
    client = _FakeParameterClient([_string_value(URDF), _string_value(SRDF)])
    node = _FakeNode(client)

    description = RobotDescription.from_node(node, timeout_sec=None)

    assert description.name == "panda"
    assert client.requests
    assert node.destroyed


@pytest.mark.parametrize("timeout_sec", [float("inf"), float("-inf"), float("nan")])
def test_from_node_rejects_nonfinite_timeout(timeout_sec):
    client = _FakeParameterClient([_string_value(URDF), _string_value(SRDF)])
    node = _FakeNode(client)

    with pytest.raises(ValueError, match="finite"):
        RobotDescription.from_node(node, timeout_sec=timeout_sec)

    assert client.requests == []
    assert not node.destroyed


@pytest.mark.parametrize(
    "client, error, message",
    [
        (
            _FakeParameterClient([], ready=False),
            TimeoutError,
            "is not available",
        ),
        (
            _FakeParameterClient([_string_value(URDF)]),
            RuntimeError,
            "Invalid response",
        ),
        (
            _FakeParameterClient([_string_value(URDF), ParameterValue()]),
            RuntimeError,
            "is not set",
        ),
    ],
)
def test_from_node_rejects_unusable_parameters(client, error, message):
    node = _FakeNode(client)
    with pytest.raises(error, match=message):
        RobotDescription.from_node(node, timeout_sec=0.0)
    assert node.destroyed


class _LogicalClock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value


class _PendingParameterClient(_FakeParameterClient):
    def __init__(self, clock, discovery_delay):
        super().__init__([], ready=True)
        self.clock = clock
        self.discovery_delay = discovery_delay
        self.wait_timeouts = []

    def wait_for_service(self, timeout_sec=None) -> bool:
        self.wait_timeouts.append(timeout_sec)
        self.clock.value += self.discovery_delay
        return True

    def call_async(self, request) -> Future:
        self.requests.append(request)
        return Future()


class _RecordingEvent:
    waits = []

    def __init__(self):
        self.set_called = False

    def set(self):
        self.set_called = True

    def wait(self, timeout=None):
        self.waits.append(timeout)
        return False


def test_from_node_uses_one_monotonic_budget_for_discovery_and_response(monkeypatch):
    clock = _LogicalClock()
    client = _PendingParameterClient(clock, discovery_delay=0.08)
    node = _FakeNode(client)
    _RecordingEvent.waits = []
    monkeypatch.setattr(robot_description_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(robot_description_module.threading, "Event", _RecordingEvent)

    with pytest.raises(TimeoutError, match="Timed out while calling"):
        RobotDescription.from_node(node, timeout_sec=0.1)

    assert client.wait_timeouts == [pytest.approx(0.1)]
    assert _RecordingEvent.waits == [pytest.approx(0.02)]
    assert len(client.requests) == 1
    assert len(client.removed_requests) == 1
    assert node.destroyed


def test_from_node_does_not_send_after_positive_budget_expires(monkeypatch):
    clock = _LogicalClock()
    client = _PendingParameterClient(clock, discovery_delay=0.11)
    node = _FakeNode(client)
    monkeypatch.setattr(robot_description_module.time, "monotonic", clock.monotonic)

    with pytest.raises(TimeoutError, match="discovering"):
        RobotDescription.from_node(node, timeout_sec=0.1)

    assert client.requests == []
    assert node.destroyed


@pytest.mark.parametrize("stage", ["discovery", "call", "future"])
def test_from_node_catches_transport_failures_and_destroys_client(stage):
    class FailingClient(_FakeParameterClient):
        def wait_for_service(self, timeout_sec=None):
            if stage == "discovery":
                raise RuntimeError("discovery failed")
            return True

        def call_async(self, request):
            if stage == "call":
                raise RuntimeError("call failed")
            future = Future()
            if stage == "future":
                future.set_exception(RuntimeError("response failed"))
            else:
                future.set_result(
                    type(
                        "Response",
                        (),
                        {"values": [_string_value(URDF), _string_value(SRDF)]},
                    )()
                )
            return future

    node = _FakeNode(FailingClient([]))
    with pytest.raises(RuntimeError, match="failed"):
        RobotDescription.from_node(node)
    assert node.destroyed
