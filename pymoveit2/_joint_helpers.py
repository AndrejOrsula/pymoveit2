"""Helpers shared by joint-space and gripper interfaces."""

import copy
from typing import Any, List, Optional, Sequence

from pymoveit2._validation import (
    finite_float,
    finite_vector,
    joint_names,
    validate_joint_state,
)

validate_joint_names = joint_names


def normalize_joint_positions(values: Any, count: int, name: str) -> List[float]:
    if count <= 0:
        raise ValueError("The configured joint count must be positive!")
    try:
        scalar = finite_float(values, name)
    except ValueError:
        return finite_vector(values, name=name, length=count)
    return [scalar] * count


def joint_state_indices(
    state_or_names: Any,
    required_names: Sequence[str],
) -> List[int]:
    names = (
        list(state_or_names.name)
        if hasattr(state_or_names, "name")
        else list(state_or_names)
    )
    if len(names) != len(set(names)):
        raise ValueError("JointState names must not contain duplicates!")
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("JointState names must be nonempty strings!")
    try:
        return [names.index(name) for name in required_names]
    except ValueError as exc:
        raise ValueError("JointState does not contain all configured joints!") from exc


def classify_gripper_state(
    state: Any,
    indices: Optional[Sequence[int]],
    open_positions: Sequence[float],
    tolerances: Sequence[float],
    closed_positions: Optional[Sequence[float]] = None,
) -> Optional[bool]:
    if state is None or indices is None:
        return None
    try:
        positions = list(state.position)
        if len(indices) != len(open_positions) or len(indices) != len(tolerances):
            return None
        if closed_positions is not None and len(closed_positions) != len(indices):
            return None

        def matches(targets: Sequence[float]) -> bool:
            if len(targets) != len(indices):
                return False
            for local_index, state_index in enumerate(indices):
                if state_index < 0 or state_index >= len(positions):
                    return False
                position = finite_float(positions[state_index], "joint_state.position")
                target = finite_float(targets[local_index], "gripper target")
                tolerance = finite_float(tolerances[local_index], "gripper tolerance")
                if abs(position - target) > tolerance:
                    return False
            return True

        if matches(open_positions):
            return True
        if closed_positions is not None and matches(closed_positions):
            return False
    except (AttributeError, TypeError, ValueError, IndexError):
        return None
    return None


def is_uniform(values: Sequence[float]) -> bool:
    return bool(values) and all(value == values[0] for value in values[1:])


def copy_joint_state(state: Any) -> Any:
    return copy.deepcopy(state) if state is not None else None


__all__ = [
    "classify_gripper_state",
    "copy_joint_state",
    "finite_float",
    "finite_vector",
    "is_uniform",
    "joint_state_indices",
    "normalize_joint_positions",
    "validate_joint_names",
    "validate_joint_state",
]
