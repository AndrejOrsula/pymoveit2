from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"
MOVE_GROUP_GRIPPER: str = "gripper"

prefix: str = ""

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


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


def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return []
