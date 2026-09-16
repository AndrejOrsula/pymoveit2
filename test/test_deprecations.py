import inspect

import pytest


@pytest.mark.parametrize(
    "class_name", ["MoveIt2", "MoveIt2Gripper", "GripperInterface"]
)
def test_follow_joint_trajectory_action_name_defaults_to_none(class_name):
    import pymoveit2

    cls = getattr(pymoveit2, class_name)
    parameter = inspect.signature(cls.__init__).parameters[
        "follow_joint_trajectory_action_name"
    ]
    assert parameter.default is None


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


def test_plan_async_max_step_deprecated_but_forwarded(moveit2, monkeypatch):
    captured = {}

    def fake_plan_cartesian_path(max_step, frame_id=None, **kwargs):
        captured["max_step"] = max_step
        return None

    monkeypatch.setattr(moveit2, "_plan_cartesian_path", fake_plan_cartesian_path)
    moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    moveit2.plan_async(cartesian=True, max_step=0.02, start_joint_state=[0.0] * 7)
    assert captured["max_step"] == 0.02


def test_plan_async_max_step_deprecation_warning_emission(moveit2, monkeypatch):
    warnings_logged = []
    captured = {}

    class FakeLogger:
        def warning(self, message, *args, **kwargs):
            warnings_logged.append(message)

        def __getattr__(self, name):
            return lambda *args, **kwargs: None

    def fake_plan_cartesian_path(max_step, frame_id=None, **kwargs):
        captured["max_step"] = max_step
        return None

    fake_logger = FakeLogger()
    monkeypatch.setattr(moveit2._node, "get_logger", lambda: fake_logger)
    monkeypatch.setattr(moveit2, "_plan_cartesian_path", fake_plan_cartesian_path)

    moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    moveit2.plan_async(cartesian=True, max_step=0.02, start_joint_state=[0.0] * 7)
    assert any("`max_step` is deprecated" in message for message in warnings_logged)
    assert captured["max_step"] == 0.02

    warnings_logged.clear()
    moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    moveit2.plan_async(
        cartesian=True, cartesian_max_step=0.01, start_joint_state=[0.0] * 7
    )
    assert not any("deprecated" in message for message in warnings_logged)
    assert captured["max_step"] == 0.01

    warnings_logged.clear()
    moveit2.set_pose_goal(position=(0.3, 0.0, 0.5), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    moveit2.plan_async(
        cartesian=True,
        max_step=0.02,
        cartesian_max_step=0.01,
        start_joint_state=[0.0] * 7,
    )
    assert any("ignored" in message for message in warnings_logged)
    assert captured["max_step"] == 0.01
