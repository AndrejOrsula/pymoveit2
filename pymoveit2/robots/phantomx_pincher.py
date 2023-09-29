from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.0158, 0.0158]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.001, 0.001]


def joint_names(prefix: str = "phantomx_pincher_") -> List[str]:
    return [
        prefix + "arm_shoulder_pan_joint",
        prefix + "arm_shoulder_lift_joint",
        prefix + "arm_elbow_flex_joint",
        prefix + "arm_wrist_flex_joint",
    ]


def base_link_name(prefix: str = "phantomx_pincher_") -> str:
    return prefix + "arm_base_link"


def end_effector_name(prefix: str = "phantomx_pincher_") -> str:
    return prefix + "end_effector"


def gripper_joint_names(prefix: str = "phantomx_pincher_") -> List[str]:
    return [
        prefix + "gripper_finger1_joint",
        prefix + "gripper_finger2_joint",
    ]
