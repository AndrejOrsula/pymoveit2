from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.7, 0.7]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

robot_prefix = "crane_x7_"


def joint_names(prefix: str = robot_prefix) -> List[str]:
    return [
        prefix + "shoulder_fixed_part_pan_joint",
        prefix + "shoulder_revolute_part_tilt_joint",
        prefix + "upper_arm_revolute_part_twist_joint",
        prefix + "upper_arm_revolute_part_rotate_joint",
        prefix + "lower_arm_fixed_part_joint",
        prefix + "lower_arm_revolute_part_joint",
        prefix + "wrist_joint",
    ]


def base_link_name(prefix: str = robot_prefix) -> str:
    return "base_link"


def end_effector_name(prefix: str = robot_prefix) -> str:
    return prefix + "gripper_base_link"


def gripper_joint_names(prefix: str = robot_prefix) -> List[str]:
    return [
        prefix + "gripper_finger_a_joint",
        prefix + "gripper_finger_b_joint",
    ]
