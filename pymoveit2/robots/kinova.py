"""Preset kinematic/config values for Kinova Gen2 (JACO/MICO) arms, with a configurable prefix for DOF/wrist/finger variants (matches kinova-ros naming)."""

import re
from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

JACO_PREFIX: str = "j2"
MICO_PREFIX: str = "m1"

# `<version><s|n><arm_dof><a|s><hand_dof>00_`, e.g. `j2n6s300_` or `j2s7s300_`
_PREFIX_PATTERN = re.compile(
    r"^(?P<version>[a-z]\d+)(?P<spherical>[sn])(?P<arm_dof>\d+)(?P<assistive>[as])(?P<hand_dof>\d+)00_$"
)


def get_prefix(
    version_prefix: str = JACO_PREFIX,
    arm_dof: int = 6,
    hand_dof: int = 2,
    spherical: bool = False,
    assistive: bool = False,
) -> str:
    return (
        version_prefix
        + ("s" if spherical else "n")
        + str(arm_dof)
        + ("a" if assistive else "s")
        + str(hand_dof)
        + "00_"
    )


def parse_prefix(prefix: str) -> dict:
    match = _PREFIX_PATTERN.match(prefix)
    if match is None:
        raise ValueError(
            f"'{prefix}' is not a kinova-ros prefix of the form "
            "'<version><s|n><arm_dof><a|s><hand_dof>00_' (e.g. 'j2n6s300_')"
        )
    return {
        "version_prefix": match.group("version"),
        "spherical": match.group("spherical") == "s",
        "arm_dof": int(match.group("arm_dof")),
        "assistive": match.group("assistive") == "a",
        "hand_dof": int(match.group("hand_dof")),
    }


def joint_names(prefix: str = get_prefix()) -> List[str]:
    arm_dof = parse_prefix(prefix)["arm_dof"]
    return [prefix + "joint_" + str(i + 1) for i in range(arm_dof)]


def base_link_name(prefix: str = get_prefix()) -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = get_prefix()) -> str:
    return prefix + "end_effector"


def gripper_joint_names(prefix: str = get_prefix()) -> List[str]:
    hand_dof = parse_prefix(prefix)["hand_dof"]
    return [prefix + "joint_finger_" + str(i + 1) for i in range(hand_dof)]
