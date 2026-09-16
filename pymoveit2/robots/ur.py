"""Preset kinematic/config values for Universal Robots e-Series 6-DOF arms (matches ur_moveit_config, e.g. UR3e/UR5e/UR10e)."""

from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"

prefix: str = ""


def joint_names(prefix: str = prefix) -> List[str]:
    return [
        prefix + "shoulder_pan_joint",
        prefix + "shoulder_lift_joint",
        prefix + "elbow_joint",
        prefix + "wrist_1_joint",
        prefix + "wrist_2_joint",
        prefix + "wrist_3_joint",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "tool0"
