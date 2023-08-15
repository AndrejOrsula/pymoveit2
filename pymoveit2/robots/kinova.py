from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

JACO_PREFIX: str = "j2"
MICO_PREFIX: str = "m1"

def get_prefix(version_prefix: str = JACO_PREFIX, arm_dof: int = 6, hand_dof: int = 2) -> str:
    return version_prefix + "n" + str(arm_dof) + "s" + str(hand_dof) + "00_"

def joint_names(prefix: str = get_prefix()) -> List[str]:
    res = []
    arm_dof = int(prefix[3])
    for i in range(arm_dof):
        res.append(prefix + "joint_" + str(i+1))
    return res

def base_link_name(prefix: str = get_prefix()) -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = get_prefix()) -> str:
    return prefix + "end_effector"


def gripper_joint_names(prefix: str = get_prefix()) -> List[str]:
    res = []
    hand_dof = int(prefix[5])
    for i in range(hand_dof):
        res.append(prefix + "joint_finger_" + str(i+1))
    return res
