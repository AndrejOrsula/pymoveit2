import pytest
from conftest import PANDA_JOINTS, FakeServiceClient
from moveit_msgs.msg import (
    AttachedCollisionObject,
    Constraints,
    ContactInformation,
    JointConstraint,
    RobotState,
)
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

from pymoveit2.moveit2 import init_joint_state


def _install_fk(moveit2) -> FakeServiceClient:
    client = FakeServiceClient("compute_fk")
    moveit2._MoveIt2__compute_fk_client = client
    return client


def _install_ik(moveit2) -> FakeServiceClient:
    client = FakeServiceClient("compute_ik")
    moveit2._MoveIt2__compute_ik_client = client
    return client


def _install_state_validity(moveit2, ready: bool = True) -> FakeServiceClient:
    client = FakeServiceClient("check_state_validity", ready=ready)
    moveit2._MoveIt2__check_state_validity_client = client
    return client


def test_fk_requests_do_not_leak_between_calls(moveit2):
    client = _install_fk(moveit2)
    moveit2.compute_fk_async(joint_state=[0.1] * 7, fk_link_names=["panda_link8"])
    moveit2.compute_fk_async()
    first, second = client.requests
    assert first is not second
    assert list(first.fk_link_names) == ["panda_link8"]
    assert list(first.robot_state.joint_state.position) == [0.1] * 7

    assert list(second.fk_link_names) == ["panda_hand"]
    assert list(second.robot_state.joint_state.position) == []


def test_ik_requests_do_not_leak_between_calls(moveit2):
    client = _install_ik(moveit2)
    constraints = Constraints()
    constraints.joint_constraints.append(JointConstraint(joint_name="panda_joint1"))
    moveit2.compute_ik_async(
        position=(0.3, 0.0, 0.5),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        ik_link_name="panda_link8",
        start_joint_state=[0.2] * 7,
        constraints=constraints,
    )
    moveit2.compute_ik_async(position=(0.4, 0.1, 0.6), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    first, second = client.requests
    assert first.ik_request.ik_link_name == "panda_link8"
    assert len(first.ik_request.constraints.joint_constraints) == 1
    assert list(first.ik_request.robot_state.joint_state.position) == [0.2] * 7
    assert second.ik_request.ik_link_name == ""
    assert len(second.ik_request.constraints.joint_constraints) == 0
    assert list(second.ik_request.robot_state.joint_state.position) == []
    assert second.ik_request.pose_stamped.pose.position.x == 0.4
    assert second.ik_request.group_name == "panda_arm"
    assert second.ik_request.avoid_collisions is True


def test_ik_result_handles_exceptional_future(moveit2):
    client = _install_ik(moveit2)
    future = moveit2.compute_ik_async(
        position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0)
    )
    client.futures[-1].set_exception(RuntimeError("transport"))
    assert moveit2.get_compute_ik_result(future) is None


def test_plan_request_is_snapshotted_and_constraints_cleared(moveit2):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    future = moveit2.plan_async(joint_positions=[0.1] * 7, start_joint_state=[0.0] * 7)
    assert future is planner.futures[0]
    request = planner.requests[0].motion_plan_request
    assert len(request.goal_constraints[0].joint_constraints) == 7

    moveit2.set_joint_goal([0.9] * 7)
    moveit2.max_velocity = 0.3
    assert request.goal_constraints[0].joint_constraints[0].position == 0.1
    assert request.max_velocity_scaling_factor == 0.0

    moveit2.clear_goal_constraints()
    moveit2.plan_async(joint_positions=[0.2] * 7, start_joint_state=[0.0] * 7)
    second = planner.requests[1].motion_plan_request
    assert len(second.goal_constraints[0].joint_constraints) == 7
    assert second.goal_constraints[0].joint_constraints[0].position == 0.2
    assert second.max_velocity_scaling_factor == 0.3


def test_plan_without_joint_state_times_out_cleanly(moveit2):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    assert (
        moveit2.plan_async(joint_positions=[0.1] * 7, joint_state_timeout_sec=0.05)
        is None
    )
    assert planner.requests == []


def test_cartesian_plan_requires_pose_goal(moveit2):
    moveit2._plan_cartesian_path_service = FakeServiceClient("compute_cartesian_path")
    with pytest.raises(ValueError, match="pose goal"):
        moveit2.plan_async(
            joint_positions=[0.1] * 7, cartesian=True, start_joint_state=[0.0] * 7
        )


def test_cartesian_plan_propagates_target_link_and_settings(moveit2):
    client = FakeServiceClient("compute_cartesian_path")
    moveit2._plan_cartesian_path_service = client
    moveit2.cartesian_avoid_collisions = False
    moveit2.cartesian_jump_threshold = 2.5
    moveit2.plan_async(
        position=(0.3, 0.0, 0.5),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        target_link="panda_link8",
        frame_id="world",
        cartesian=True,
        cartesian_max_step=0.01,
        start_joint_state=[0.0] * 7,
    )
    request = client.requests[0]
    assert request.link_name == "panda_link8"
    assert request.header.frame_id == "world"
    assert request.group_name == "panda_arm"
    assert request.max_step == 0.01
    assert request.avoid_collisions is False
    assert request.jump_threshold == 2.5
    assert len(request.waypoints) == 1
    assert request.waypoints[0].position.x == 0.3

    moveit2.plan_async(
        position=(0.3, 0.0, 0.5),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        cartesian=True,
        start_joint_state=[0.0] * 7,
    )
    assert client.requests[1].link_name == "panda_hand"
    assert client.requests[1].header.frame_id == "panda_link0"


def test_plan_start_state_from_list_and_joint_state(moveit2):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    moveit2.plan_async(joint_positions=[0.1] * 7, start_joint_state=[0.5] * 7)
    start = planner.requests[0].motion_plan_request.start_state.joint_state
    assert list(start.name) == PANDA_JOINTS
    assert list(start.position) == [0.5] * 7
    joint_state = init_joint_state(PANDA_JOINTS, [0.7] * 7)
    moveit2.plan_async(joint_positions=[0.1] * 7, start_joint_state=joint_state)
    start = planner.requests[1].motion_plan_request.start_state.joint_state
    assert list(start.position) == [0.7] * 7


def test_workspace_frame_defaults_to_base_link(moveit2):
    planner = FakeServiceClient("plan_kinematic_path")
    moveit2._plan_kinematic_path_service = planner
    moveit2.set_workspace_parameters((-2.0,) * 3, (2.0,) * 3, frame_id="world")
    assert moveit2.workspace_frame_id == "world"
    moveit2.set_workspace_parameters((-1.0,) * 3, (1.0,) * 3)
    assert moveit2.workspace_frame_id == "panda_link0"
    moveit2.plan_async(joint_positions=[0.1] * 7, start_joint_state=[0.0] * 7)
    ws = planner.requests[0].motion_plan_request.workspace_parameters
    assert ws.header.frame_id == "panda_link0"
    assert ws.min_corner.x == -1.0 and ws.max_corner.z == 1.0
    with pytest.raises(ValueError):
        moveit2.set_workspace_parameters((-1.0, -1.0), (1.0, 1.0, 1.0))


def test_joint_state_callback_updates_state(moveit2):
    message = JointState()
    message.name = PANDA_JOINTS + ["extra"]
    message.position = [0.1] * 8
    moveit2._MoveIt2__joint_state_callback(message)
    assert moveit2.joint_state == message
    assert moveit2.new_joint_state_available is True
    moveit2.reset_new_joint_state_checker()
    assert moveit2.new_joint_state_available is False
    assert moveit2.wait_for_joint_state(timeout_sec=0.01) is True
    partial = JointState()
    partial.name = ["panda_joint1"]
    moveit2._MoveIt2__joint_state_callback(partial)
    assert moveit2.joint_state == message


def test_plan_async_accepts_cartesian_max_step(moveit2, monkeypatch):
    captured = {}

    def fake_plan_cartesian_path(max_step, frame_id=None, **kwargs):
        captured["max_step"] = max_step
        return None

    monkeypatch.setattr(moveit2, "_plan_cartesian_path", fake_plan_cartesian_path)
    moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    moveit2.plan_async(
        cartesian=True, cartesian_max_step=0.01, start_joint_state=[0.0] * 7
    )
    assert captured["max_step"] == 0.01


def test_check_state_validity_requests_do_not_leak_between_calls(moveit2):
    client = _install_state_validity(moveit2)
    constraints = Constraints()
    constraints.joint_constraints.append(JointConstraint(joint_name="panda_joint1"))
    moveit2.check_state_validity_async(
        robot_state=[0.2] * 7,
        group_name="custom_group",
        constraints=constraints,
    )
    moveit2.check_state_validity_async(robot_state=[0.1] * 7)
    first, second = client.requests
    assert first is not second
    assert first.group_name == "custom_group"
    assert len(first.constraints.joint_constraints) == 1
    assert list(first.robot_state.joint_state.position) == [0.2] * 7

    assert second.group_name == "panda_arm"
    assert len(second.constraints.joint_constraints) == 0
    assert list(second.robot_state.joint_state.position) == [0.1] * 7


def test_check_state_validity_preserves_full_robot_state(moveit2):
    client = _install_state_validity(moveit2)
    state = RobotState()
    state.is_diff = True
    state.joint_state.name = ["custom_joint"]
    state.joint_state.position = [1.23]
    obj = AttachedCollisionObject()
    obj.link_name = "panda_hand"
    obj.object.id = "attached_box"
    state.attached_collision_objects.append(obj)

    moveit2.check_state_validity_async(robot_state=state)
    req = client.requests[-1]
    assert req.robot_state.is_diff is True
    assert list(req.robot_state.joint_state.name) == ["custom_joint"]
    assert list(req.robot_state.joint_state.position) == [1.23]
    assert len(req.robot_state.attached_collision_objects) == 1
    assert req.robot_state.attached_collision_objects[0].object.id == "attached_box"
    assert req.robot_state is not state


def test_check_state_validity_accepts_joint_state_object(moveit2):
    client = _install_state_validity(moveit2)
    js = JointState()
    js.name = PANDA_JOINTS
    js.position = [0.5] * 7
    moveit2.check_state_validity_async(robot_state=js)
    req = client.requests[-1]
    assert req.robot_state.is_diff is False
    assert list(req.robot_state.joint_state.position) == [0.5] * 7
    assert req.robot_state.joint_state.name == PANDA_JOINTS


def test_check_state_validity_accepts_joint_positions_list(moveit2):
    client = _install_state_validity(moveit2)
    moveit2.check_state_validity_async(robot_state=[0.3] * 7)
    req = client.requests[-1]
    assert req.robot_state.is_diff is False
    assert list(req.robot_state.joint_state.position) == [0.3] * 7
    assert req.robot_state.joint_state.name == PANDA_JOINTS


def test_check_state_validity_uses_current_joint_state_when_available(moveit2):
    client = _install_state_validity(moveit2)
    message = JointState()
    message.name = PANDA_JOINTS
    message.position = [0.4] * 7
    moveit2._MoveIt2__joint_state_callback(message)

    moveit2.check_state_validity_async()
    req = client.requests[-1]
    assert list(req.robot_state.joint_state.position) == [0.4] * 7


def test_check_state_validity_missing_current_joint_state(moveit2):
    client = _install_state_validity(moveit2)
    assert moveit2.joint_state is None
    assert moveit2.check_state_validity_async(robot_state=None) is None
    assert moveit2.check_state_validity(robot_state=None) is None
    assert len(client.requests) == 0


def test_check_state_validity_unavailable_service(moveit2):
    _install_state_validity(moveit2, ready=False)
    future = moveit2.check_state_validity_async(
        robot_state=[0.0] * 7, wait_for_server_timeout_sec=0.01
    )
    assert future is None
    res = moveit2.check_state_validity(
        robot_state=[0.0] * 7, wait_for_server_timeout_sec=0.01, timeout_sec=0.01
    )
    assert res is None


def test_check_state_validity_handles_exceptional_future(moveit2):
    client = _install_state_validity(moveit2)
    future = moveit2.check_state_validity_async(robot_state=[0.0] * 7)
    client.futures[-1].set_exception(RuntimeError("transport failure"))
    assert moveit2.get_check_state_validity_result(future) is None


def test_check_state_validity_valid_and_invalid_responses_with_contacts(moveit2):
    client = _install_state_validity(moveit2)

    # Valid response
    future_valid = moveit2.check_state_validity_async(robot_state=[0.0] * 7)
    resp_valid = GetStateValidity.Response()
    resp_valid.valid = True
    client.futures[-1].set_result(resp_valid)
    result = moveit2.get_check_state_validity_result(future_valid)
    assert result is not None
    assert result.valid is True
    assert len(result.contacts) == 0

    # Invalid response with contacts
    future_invalid = moveit2.check_state_validity_async(robot_state=[0.0] * 7)
    resp_invalid = GetStateValidity.Response()
    resp_invalid.valid = False
    contact = ContactInformation()
    contact.contact_body_1 = "panda_link5"
    contact.contact_body_2 = "obstacle"
    resp_invalid.contacts.append(contact)
    client.futures[-1].set_result(resp_invalid)
    result_inv = moveit2.get_check_state_validity_result(future_invalid)
    assert result_inv is not None
    assert result_inv.valid is False
    assert len(result_inv.contacts) == 1
    assert result_inv.contacts[0].contact_body_1 == "panda_link5"
    assert result_inv.contacts[0].contact_body_2 == "obstacle"


def test_check_state_validity_timeout_cleans_up_request(moveit2):
    client = _install_state_validity(moveit2)
    result = moveit2.check_state_validity(robot_state=[0.0] * 7, timeout_sec=0.01)
    assert result is None
    assert len(client.removed_requests) == 1


def test_check_state_validity_closed_state(moveit2):
    _install_state_validity(moveit2)
    moveit2.destroy()
    assert moveit2.check_state_validity_async(robot_state=[0.0] * 7) is None
    assert moveit2.check_state_validity(robot_state=[0.0] * 7) is None
