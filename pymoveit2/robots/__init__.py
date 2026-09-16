"""Static presets for robots without a usable URDF or SRDF. Prefer to use `RobotDescription` which reads the URDF and SRDF of a running `move_group`."""

from . import crane_x7, kinova, lbr, panda, phantomx_pincher, ur

__all__ = ["crane_x7", "kinova", "lbr", "panda", "phantomx_pincher", "ur"]
