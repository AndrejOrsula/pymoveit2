import math
from unittest.mock import patch

import pytest
from action_msgs.msg import GoalStatus
from conftest import GRIPPER_JOINTS, FakeActionClient, complete
from control_msgs.action import GripperCommand as GripperCommandAction
from moveit_msgs.action import MoveGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from pymoveit2 import GripperCommand, GripperInterface, MoveIt2Gripper, MoveIt2State


def make_gripper(rclpy_node, **kwargs):
    kwargs.setdefault("open_gripper_joint_positions", [0.04, 0.04])
    kwargs.setdefault("closed_gripper_joint_positions", [0.0, 0.0])
    return MoveIt2Gripper(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        gripper_group_name="hand",
        callback_group=ReentrantCallbackGroup(),
        **kwargs,
    )


def make_gripper_command(rclpy_node, **kwargs):
    kwargs.setdefault("open_gripper_joint_positions", [0.04, 0.04])
    kwargs.setdefault("closed_gripper_joint_positions", [0.0, 0.0])
    return GripperCommand(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        **kwargs,
    )


def make_interface(rclpy_node, **kwargs):
    kwargs.setdefault("open_gripper_joint_positions", [0.04, 0.04])
    kwargs.setdefault("closed_gripper_joint_positions", [0.0, 0.0])
    kwargs.setdefault("discovery_timeout_sec", 0.1)
    return GripperInterface(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        gripper_group_name="hand",
        **kwargs,
    )


def _joint_state(names, positions) -> JointState:
    message = JointState()
    message.name = list(names)
    message.position = list(positions)
    return message


def _gripper_result(reached_goal: bool = True) -> GripperCommandAction.Result:
    result = GripperCommandAction.Result()
    result.reached_goal = reached_goal
    return result


def test_gripper_skip_planning_constructs(rclpy_node):
    gripper = make_gripper(rclpy_node, skip_planning=True)
    assert gripper is not None
    gripper.destroy()


def test_gripper_skip_planning_open_close_do_not_raise(rclpy_node):
    gripper = make_gripper(rclpy_node, skip_planning=True)

    assert gripper.open(skip_if_noop=False) is False
    assert gripper.close(skip_if_noop=False) is False
    gripper.destroy()


def test_gripper_skip_planning_direct_path_respects_admission(rclpy_node):
    gripper = make_gripper(
        rclpy_node, skip_planning=True, ignore_new_calls_while_executing=True
    )
    client = FakeActionClient("execute_trajectory")
    gripper._execute_trajectory_action_client = client
    assert gripper.open() is True
    assert gripper.close() is False
    handle = client.accept()
    assert gripper.close() is False
    assert len(client.sent_goals) == 1
    complete(handle, GoalStatus.STATUS_SUCCEEDED)
    assert gripper.wait_until_executed(timeout_sec=0.1) is True
    assert gripper.close() is True
    assert (
        client.sent_goals[1].trajectory.joint_trajectory.points[0].positions[0] == 0.0
    )
    gripper.destroy()


def test_gripper_skip_planning_templates_are_copied_per_submission(rclpy_node):
    gripper = make_gripper(rclpy_node, skip_planning=True)
    client = FakeActionClient("execute_trajectory")
    gripper._execute_trajectory_action_client = client
    assert gripper.open() is True
    first = client.sent_goals[0]
    first_handle = client.accept()
    complete(first_handle, GoalStatus.STATUS_SUCCEEDED)
    assert gripper.open() is True
    second = client.sent_goals[1]
    assert first is not second
    first.trajectory.joint_trajectory.points[0].positions[0] = -1.0
    assert second.trajectory.joint_trajectory.points[0].positions[0] == 0.04
    gripper.destroy()


def test_gripper_length_mismatch_raises_value_error(rclpy_node):
    with pytest.raises(ValueError):
        MoveIt2Gripper(
            node=rclpy_node,
            gripper_joint_names=GRIPPER_JOINTS,
            open_gripper_joint_positions=[0.04],
            closed_gripper_joint_positions=[0.0, 0.0],
        )


@pytest.mark.parametrize(
    "kwargs",
    [
        {"gripper_joint_names": []},
        {"open_gripper_joint_positions": [float("nan"), 0.04]},
        {"closed_gripper_joint_positions": [0.0, float("inf")]},
    ],
)
def test_gripper_configuration_must_be_finite_and_nonempty(rclpy_node, kwargs):
    arguments = {
        "node": rclpy_node,
        "gripper_joint_names": GRIPPER_JOINTS,
        "open_gripper_joint_positions": [0.04, 0.04],
        "closed_gripper_joint_positions": [0.0, 0.0],
    }
    arguments.update(kwargs)
    with pytest.raises(ValueError):
        MoveIt2Gripper(**arguments)


def test_gripper_unsupported_methods_raise_clear_error(rclpy_node):
    gripper = make_gripper(rclpy_node)
    for name in ("move_to_pose", "set_pose_goal", "compute_fk", "compute_ik"):
        with pytest.raises(NotImplementedError, match="MoveIt2Gripper"):
            getattr(gripper, name)()
    with pytest.raises(NotImplementedError):
        gripper.plan_async(position=(0.1, 0.1, 0.1), start_joint_state=[0.0, 0.0])
    gripper.destroy()


def test_gripper_is_open_tracks_joint_state_reordering(rclpy_node):
    gripper = make_gripper(rclpy_node)
    assert gripper.is_open is False
    assert gripper.is_closed is False
    names = ["panda_joint1"] + GRIPPER_JOINTS
    gripper._MoveIt2__joint_state_callback(_joint_state(names, [0.0, 0.04, 0.04]))
    assert gripper.is_open is True
    reordered = GRIPPER_JOINTS[::-1] + ["panda_joint1"]
    gripper._MoveIt2__joint_state_callback(_joint_state(reordered, [0.0, 0.0, 1.0]))
    assert gripper.is_closed is True
    gripper._MoveIt2__joint_state_callback(_joint_state(reordered, [0.04, 0.04, 1.0]))
    assert gripper.is_open is True
    gripper.destroy()


def test_gripper_intermediate_state_is_unknown(rclpy_node):
    planned = make_gripper(rclpy_node)
    planned._MoveIt2__joint_state_callback(_joint_state(GRIPPER_JOINTS, [0.02, 0.02]))
    assert planned.is_open is False
    assert planned.is_closed is False
    planned.destroy()

    raw = make_gripper_command(rclpy_node)
    raw._GripperCommand__joint_state_callback(
        _joint_state(GRIPPER_JOINTS, [0.02, 0.02])
    )
    assert raw.is_open is False
    assert raw.is_closed is False
    raw.destroy()


def test_gripper_command_no_server_does_not_hang(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    assert gc.open() is False

    assert gc.wait_until_executed(timeout_sec=1.0) is False
    gc.destroy()


def test_gripper_command_default_action_name(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    assert (
        gc.gripper_command_action_client._action_name
        == "gripper_action_controller/gripper_cmd"
    )
    gc.destroy()


def test_gripper_command_scalar_positions(rclpy_node):
    gc = GripperCommand(
        node=rclpy_node,
        gripper_joint_names=GRIPPER_JOINTS,
        open_gripper_joint_positions=0.04,
        closed_gripper_joint_positions=0.0,
    )
    assert gc.open_gripper_joint_positions == [0.04, 0.04]
    assert gc.closed_gripper_joint_positions == [0.0, 0.0]
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client
    gc.close()
    assert client.sent_goals[0].command.position == 0.0
    gc.destroy()
    with pytest.raises(ValueError):
        GripperCommand(
            node=rclpy_node,
            gripper_joint_names=GRIPPER_JOINTS,
            open_gripper_joint_positions=[0.04],
            closed_gripper_joint_positions=[0.0, 0.0],
        )


def test_gripper_command_rejects_nonuniform_scalar_mapping(rclpy_node):
    with pytest.raises(ValueError, match="scalar actuator"):
        make_gripper_command(
            rclpy_node,
            open_gripper_joint_positions=[0.04, 0.02],
            closed_gripper_joint_positions=[0.0, 0.01],
        )


def test_gripper_command_templates_are_copied_per_submission(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client

    assert gc.open() is True
    first = client.sent_goals[0]
    first_handle = client.accept()
    complete(first_handle, GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gc.open() is True
    second = client.sent_goals[1]
    assert first is not second
    first.command.position = -1.0
    assert second.command.position == 0.04
    gc.destroy()


def test_gripper_command_reached_goal_controls_success(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client
    assert gc.open() is True
    handle = client.accept()
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _gripper_result(False))
    assert gc.wait_until_executed(timeout_sec=0.1) is False
    assert gc.motion_succeeded is False
    assert gc.get_last_execution_error_code() is None
    gc.destroy()


def test_gripper_command_toggle_unknown_state_refuses(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client
    assert gc.is_open is False
    assert gc.is_closed is False
    assert gc.toggle() is False
    assert len(client.sent_goals) == 0
    gc.destroy()


def test_gripper_command_pending_requests_cannot_overlap(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client
    assert gc.open() is True
    assert gc.query_state() == MoveIt2State.REQUESTING
    assert gc.close() is False
    assert gc.move_to_position(0.02) is False
    assert len(client.sent_goals) == 1
    handle = client.accept()
    assert gc.query_state() == MoveIt2State.EXECUTING
    assert gc.close() is False
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gc.query_state() == MoveIt2State.IDLE
    assert gc.motion_succeeded is True
    assert gc.move_to_position(0.02) is True
    assert client.sent_goals[1].command.position == 0.02
    gc.destroy()


def test_gripper_command_lifecycle_outcomes(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    client = FakeActionClient("gripper_cmd")
    gc._GripperCommand__gripper_command_action_client = client

    gc.open()
    client.reject()
    assert gc.query_state() == MoveIt2State.IDLE
    assert gc.wait_until_executed(timeout_sec=0.1) is False

    gc.open()
    handle = client.accept()
    assert gc.get_execution_future() is handle.result_future
    complete(handle, GoalStatus.STATUS_ABORTED, _gripper_result(False))
    assert gc.wait_until_executed(timeout_sec=0.1) is False
    assert gc.motion_succeeded is False

    gc.open()
    handle = client.accept()
    assert gc.cancel_execution() is True
    assert handle.cancel_requests == 1
    handle.result_future.set_exception(RuntimeError("boom"))
    assert gc.query_state() == MoveIt2State.IDLE

    gc.open()
    stale = client.accept()
    assert gc.reset_closed() is False
    gc.force_reset_executing_state()
    assert gc.reset_closed() is True
    fresh = client.accept()
    complete(stale, GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gc.query_state() == MoveIt2State.EXECUTING
    complete(fresh, GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gc.wait_until_executed(timeout_sec=0.1) is True
    gc.destroy()


def test_gripper_command_is_open_tracks_joint_state_reordering(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    assert gc.is_open is False
    assert gc.is_closed is False
    names = ["panda_joint1"] + GRIPPER_JOINTS
    gc._GripperCommand__joint_state_callback(_joint_state(names, [0.0, 0.04, 0.04]))
    assert gc.is_open is True
    reordered = GRIPPER_JOINTS[::-1] + ["panda_joint1"]
    gc._GripperCommand__joint_state_callback(_joint_state(reordered, [0.0, 0.0, 1.0]))
    assert gc.is_closed is True
    gc.destroy()


def test_gripper_command_accepts_unavailable_optional_measurements(rclpy_node):
    gc = make_gripper_command(rclpy_node)
    message = _joint_state(GRIPPER_JOINTS, [0.04, 0.04])
    message.velocity = [math.nan, math.nan]
    message.effort = [math.nan, math.nan]
    gc._GripperCommand__joint_state_callback(message)

    assert gc.new_joint_state_available is True
    assert gc.is_open is True
    assert list(gc.joint_state.velocity) == []
    assert list(gc.joint_state.effort) == []
    gc.destroy()


def test_planned_gripper_accepts_unavailable_optional_measurements(rclpy_node):
    gripper = make_gripper(rclpy_node)
    try:
        message = _joint_state(GRIPPER_JOINTS, [0.04, 0.04])
        message.effort = [math.nan, math.nan]
        gripper._MoveIt2__joint_state_callback(message)
        assert gripper.wait_for_joint_state(timeout_sec=0.0)
        assert gripper.is_open
        assert list(gripper.joint_state.effort) == []
    finally:
        gripper.destroy()


@pytest.mark.parametrize(
    "positions, effort",
    [([0.04, 0.04], [0.0]), ([math.inf, 0.04], [0.0, 0.0])],
)
def test_gripper_command_rejects_malformed_observations(rclpy_node, positions, effort):
    gc = make_gripper_command(rclpy_node)
    gc.reset_new_joint_state_checker()
    message = _joint_state(GRIPPER_JOINTS, positions)
    message.effort = effort
    gc._GripperCommand__joint_state_callback(message)
    assert gc.new_joint_state_available is False
    assert gc.joint_state is None
    gc.destroy()


def test_gripper_interface_without_servers_is_safe(rclpy_node):
    gi = make_interface(rclpy_node)
    assert gi.backend is None
    assert gi.interface is None
    assert gi.open() is False
    assert gi.wait_until_executed(timeout_sec=0.1) is False
    assert gi.query_state() == MoveIt2State.IDLE
    assert gi.is_open is False
    assert gi.is_closed is False
    assert gi.joint_state is None
    assert gi.motion_succeeded is False
    assert gi.cancel_execution() is False
    assert gi.get_execution_future() is None
    gi.destroy()


def test_gripper_interface_does_not_rediscover_after_construction(rclpy_node):
    probes = []

    class Probe(FakeActionClient):
        def __init__(self, **kwargs):
            probes.append(kwargs["action_name"])
            super().__init__(kwargs["action_name"], ready=False)

        def wait_for_server(self, timeout_sec=None):
            return False

    with patch("pymoveit2.gripper_interface.ActionClient", Probe):
        gi = make_interface(rclpy_node)
        assert gi.open() is False
        assert gi.close() is False
    assert probes == [
        "gripper_action_controller/gripper_cmd",
        "execute_trajectory",
    ]
    gi.destroy()


def test_gripper_interface_discovers_move_group_action_for_selected_mode(
    rclpy_node,
):
    probed = []
    probed_types = []

    class Probe(FakeActionClient):
        def __init__(self, **kwargs):
            probed.append(kwargs["action_name"])
            probed_types.append(kwargs["action_type"])
            super().__init__(kwargs["action_name"], ready=True)

        def wait_for_server(self, timeout_sec=None):
            return self._action_name == "move_action"

    with patch("pymoveit2.gripper_interface.ActionClient", Probe):
        gi = make_interface(
            rclpy_node,
            use_move_group_action=True,
            discovery_timeout_sec=0.1,
        )
    assert probed == [
        "gripper_action_controller/gripper_cmd",
        "move_action",
    ]
    assert probed_types == [GripperCommandAction, MoveGroup]
    assert gi.interface is MoveIt2Gripper
    gi.destroy()


def test_gripper_interface_skip_planning_probes_execute_trajectory(
    rclpy_node,
):
    probed = []

    class Probe(FakeActionClient):
        def __init__(self, **kwargs):
            probed.append(kwargs["action_name"])
            super().__init__(kwargs["action_name"], ready=True)

        def wait_for_server(self, timeout_sec=None):
            return self._action_name == "execute_trajectory"

    with patch("pymoveit2.gripper_interface.ActionClient", Probe):
        gi = make_interface(
            rclpy_node,
            use_move_group_action=True,
            skip_planning=True,
            discovery_timeout_sec=0.1,
        )
    assert probed == [
        "gripper_action_controller/gripper_cmd",
        "execute_trajectory",
    ]
    assert gi.interface is MoveIt2Gripper
    gi.destroy()


def test_gripper_interface_nonuniform_auto_mapping_falls_back_to_planned(
    rclpy_node,
):
    probed = []

    class Probe(FakeActionClient):
        def __init__(self, **kwargs):
            probed.append(kwargs["action_name"])
            super().__init__(kwargs["action_name"], ready=True)

        def wait_for_server(self, timeout_sec=None):
            return True

    with patch("pymoveit2.gripper_interface.ActionClient", Probe):
        gi = make_interface(
            rclpy_node,
            open_gripper_joint_positions=[0.04, 0.02],
            closed_gripper_joint_positions=[0.0, 0.01],
            discovery_timeout_sec=0.1,
        )
    assert probed == [
        "gripper_action_controller/gripper_cmd",
        "execute_trajectory",
    ]
    assert gi.interface is MoveIt2Gripper
    gi.destroy()


def test_gripper_interface_rejects_nonuniform_forced_raw_backend(rclpy_node):
    with pytest.raises(ValueError, match="scalar actuator"):
        make_interface(
            rclpy_node,
            interface=GripperCommand,
            open_gripper_joint_positions=[0.04, 0.02],
            closed_gripper_joint_positions=[0.0, 0.01],
        )


def test_gripper_interface_reset_dispatch_jtc(rclpy_node):
    gi = make_interface(rclpy_node, interface=MoveIt2Gripper)
    assert isinstance(gi.backend, MoveIt2Gripper)
    assert gi._interface is MoveIt2Gripper
    assert gi.reset_open() is False
    assert gi.reset_closed() is False
    gi.destroy()


def test_gripper_interface_wait_until_executed_accepts_timeout(rclpy_node):
    gi = make_interface(rclpy_node, interface=GripperCommand)
    assert gi.wait_until_executed(timeout_sec=1.0) is False
    gi.destroy()


def test_gripper_interface_delegates_completely_to_gripper_command(rclpy_node):
    gi = make_interface(rclpy_node, interface=GripperCommand)
    backend = gi.backend
    assert isinstance(backend, GripperCommand)
    client = FakeActionClient("gripper_cmd")
    backend._GripperCommand__gripper_command_action_client = client
    assert gi.gripper_command_action_client is client

    assert gi.close() is True
    assert gi.query_state() == MoveIt2State.REQUESTING
    handle = client.accept()
    assert gi.query_state() == MoveIt2State.EXECUTING
    assert gi.get_execution_future() is handle.result_future
    assert gi.cancel_execution() is True
    assert handle.cancel_requests == 1
    complete(handle, GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gi.query_state() == MoveIt2State.IDLE
    assert gi.motion_succeeded is True
    assert gi.wait_until_executed(timeout_sec=0.1) is True

    backend._GripperCommand__joint_state_callback(
        _joint_state(GRIPPER_JOINTS, [0.0, 0.0])
    )
    assert gi.joint_state == backend.joint_state
    assert gi.is_closed is True
    assert gi.new_joint_state_available is True
    gi.reset_new_joint_state_checker()
    assert gi.new_joint_state_available is False
    gi.destroy()


def test_gripper_interface_delegates_completely_to_moveit2_gripper(rclpy_node):
    gi = make_interface(rclpy_node, interface=MoveIt2Gripper, skip_planning=True)
    backend = gi.backend
    client = FakeActionClient("execute_trajectory")
    backend._execute_trajectory_action_client = client
    assert gi.gripper_command_action_client is None
    assert gi.open() is True
    handle = client.accept()
    assert gi.query_state() == MoveIt2State.EXECUTING
    assert gi.get_execution_future() is handle.result_future
    complete(handle, GoalStatus.STATUS_ABORTED)
    assert gi.motion_succeeded is False
    assert gi.wait_until_executed(timeout_sec=0.1) is False
    gi.force_reset_executing_state()
    assert gi.query_state() == MoveIt2State.IDLE
    gi.destroy()


def test_gripper_interface_motion_succeeded_updated_by_gripper_command_path(rclpy_node):
    gi = make_interface(rclpy_node, interface=GripperCommand)
    client = FakeActionClient("gripper_cmd")
    gi.backend._GripperCommand__gripper_command_action_client = client
    assert gi.motion_succeeded is False
    gi.open()
    complete(client.accept(), GoalStatus.STATUS_SUCCEEDED, _gripper_result())
    assert gi.motion_succeeded is True
    gi.open()
    complete(client.accept(), GoalStatus.STATUS_ABORTED, _gripper_result(False))
    assert gi.motion_succeeded is False
    gi.destroy()


def test_gripper_interface_rejects_unknown_interface(rclpy_node):
    with pytest.raises(ValueError):
        make_interface(rclpy_node, interface=int)
