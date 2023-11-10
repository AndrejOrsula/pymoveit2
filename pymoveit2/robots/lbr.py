from typing import List

# Documentation:
# https://github.com/lbr-stack/lbr_fri_ros2_stack/

MOVE_GROUP_ARM: str = "arm"


def joint_names() -> List[str]:
    return [f"A{i}" for i in range(8)]


def base_link_name() -> str:
    return "link_0"


def end_effector_name() -> str:
    return "link_ee"
