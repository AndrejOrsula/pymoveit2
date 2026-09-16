"""Preset kinematic/config values for the KUKA LBR iiwa 7-joint arm (matches lbr_fri_ros2_stack)."""

from typing import List

MOVE_GROUP_ARM: str = "arm"


def joint_names(prefix: str = "") -> List[str]:
    return [prefix + f"A{i}" for i in range(1, 8)]


def base_link_name() -> str:
    return "link_0"


def end_effector_name() -> str:
    return "link_ee"
