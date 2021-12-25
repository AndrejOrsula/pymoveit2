from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
        prefix + "joint7",
    ]


def base_link_name(prefix: str = "panda_") -> str:
    return prefix + "link0"


def end_effector_name(prefix: str = "panda_") -> str:
    return prefix + "hand_tcp"


def gripper_joint_names(prefix: str = "panda_") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]
