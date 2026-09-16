import importlib

import pytest

ROBOT_MODULES = ["crane_x7", "kinova", "lbr", "panda", "phantomx_pincher", "ur"]
EXPECTED_ARM_DOF = {"panda": 7, "ur": 6, "lbr": 7}


@pytest.mark.parametrize("name", ROBOT_MODULES)
def test_joint_names_unique_and_nonempty(name):
    robot = importlib.import_module(f"pymoveit2.robots.{name}")
    joint_names = robot.joint_names()
    assert len(joint_names) > 0
    assert len(joint_names) == len(set(joint_names))
    if name in EXPECTED_ARM_DOF:
        assert len(joint_names) == EXPECTED_ARM_DOF[name]


@pytest.mark.parametrize("name", ROBOT_MODULES)
def test_gripper_preset_consistency(name):
    robot = importlib.import_module(f"pymoveit2.robots.{name}")
    if not hasattr(robot, "gripper_joint_names"):
        return
    gripper_joints = robot.gripper_joint_names()
    assert isinstance(gripper_joints, list)
    assert len(gripper_joints) > 0
    if hasattr(robot, "OPEN_GRIPPER_JOINT_POSITIONS"):
        assert len(gripper_joints) == len(robot.OPEN_GRIPPER_JOINT_POSITIONS)
    if hasattr(robot, "CLOSED_GRIPPER_JOINT_POSITIONS"):
        assert len(gripper_joints) == len(robot.CLOSED_GRIPPER_JOINT_POSITIONS)


def test_lbr_joint_names_match_kuka_convention():
    from pymoveit2.robots import lbr

    assert lbr.joint_names(prefix="") == ["A1", "A2", "A3", "A4", "A5", "A6", "A7"]


def test_kinova_prefix_parsing_supports_multi_digit_variants():
    from pymoveit2.robots import kinova

    assert kinova.joint_names("j2n6s300_") == [
        f"j2n6s300_joint_{i}" for i in range(1, 7)
    ]
    assert kinova.gripper_joint_names("j2s7s300_") == [
        f"j2s7s300_joint_finger_{i}" for i in range(1, 4)
    ]

    assert len(kinova.joint_names(kinova.get_prefix(arm_dof=12, hand_dof=10))) == 12
    assert len(kinova.gripper_joint_names(kinova.get_prefix(hand_dof=10))) == 10
    with pytest.raises(ValueError):
        kinova.joint_names("not_a_prefix")
    assert kinova.parse_prefix("m1n4s200_") == {
        "version_prefix": "m1",
        "spherical": False,
        "arm_dof": 4,
        "assistive": False,
        "hand_dof": 2,
    }
