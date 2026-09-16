"""Helpers for input validation."""

import copy
import math
from collections.abc import Iterable
from numbers import Real
from typing import Any, List, Optional


def finite_float(value: Any, name: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"`{name}` must be a finite number.")
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        suffix = "" if minimum is None else f" greater than or equal to {minimum}"
        raise ValueError(f"`{name}` must be finite{suffix}.")
    return result


def finite_vector(values: Any, name: str, length: Optional[int] = None) -> List[float]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Iterable):
        raise ValueError(f"`{name}` must be a sequence of finite numbers.")
    result = [finite_float(value, name) for value in values]
    if length is not None and len(result) != length:
        raise ValueError(f"`{name}` must contain exactly {length} values.")
    return result


def joint_names(values: Any, name: str = "joint_names") -> List[str]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Iterable):
        raise ValueError(f"`{name}` must be a nonempty sequence of joint names.")
    result = list(values)
    if not result or any(not isinstance(value, str) or not value for value in result):
        raise ValueError(f"`{name}` must contain nonempty strings.")
    if len(set(result)) != len(result):
        raise ValueError(f"`{name}` must not contain duplicates.")
    return result


def validate_joint_state(
    state: Any, required_names: Optional[Iterable[str]] = None
) -> None:
    try:
        names = joint_names(state.name, "JointState.name")
        finite_vector(state.position, "JointState.position", len(names))
        for field in ("velocity", "effort"):
            values = finite_vector(getattr(state, field), f"JointState.{field}")
            if values and len(values) != len(names):
                raise ValueError(
                    f"JointState.{field} must be empty or match name length."
                )
        if required_names is not None and not set(required_names).issubset(names):
            raise ValueError("JointState must contain every configured joint.")
    except AttributeError as err:
        raise ValueError(
            "Expected a JointState with names and position arrays."
        ) from err


def normalize_joint_state_observation(
    state: Any, required_names: Optional[Iterable[str]] = None
) -> Any:
    observation = copy.deepcopy(state)
    try:
        names = joint_names(observation.name, "JointState.name")
        for field in ("velocity", "effort"):
            values = list(getattr(observation, field))
            if values and len(values) != len(names):
                raise ValueError(
                    f"JointState.{field} must be empty or match name length."
                )
            unknown = False
            for value in values:
                if (
                    isinstance(value, Real)
                    and not isinstance(value, bool)
                    and math.isnan(value)
                ):
                    unknown = True
                else:
                    finite_float(value, f"JointState.{field}")
            if unknown:
                setattr(observation, field, [])
    except (AttributeError, TypeError) as err:
        raise ValueError(
            "Expected a JointState observation with numeric arrays."
        ) from err
    validate_joint_state(observation, required_names)
    return observation
