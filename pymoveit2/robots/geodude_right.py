from typing import List

MOVE_GROUP_ARM: str = "right_arm"
MOVE_GROUP_GRIPPER: str = "right_hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0, 0.0, 0.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [1.04, 1.04, 1.04, 0.0]


def joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "j1",
        prefix + "j2",
        prefix + "j3",
        prefix + "j4",
        prefix + "j5",
        prefix + "j6",
        prefix + "j7",
    ]


def base_link_name(prefix: str = "right_") -> str:
    return prefix + "wam1"


def end_effector_name(prefix: str = "right_") -> str:
    return prefix + "hand_tcp"


def gripper_joint_names(prefix: str = "right_") -> List[str]:
    return [
        prefix + "j01",
        prefix + "j11",
        prefix + "j21",
        prefix + "j00",
    ]
