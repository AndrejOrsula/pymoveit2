from pymoveit2._joint_helpers import classify_gripper_state, normalize_joint_positions
from pymoveit2._validation import finite_float, finite_vector, joint_names

normalized_number: float = finite_float(1, "value")
normalized_values: list[float] = finite_vector((1, 2), "values")
normalized_names: list[str] = joint_names(("joint_a",), "names")
normalized_positions: list[float] = normalize_joint_positions((1, 2), 2, "positions")
known_gripper_state: bool | None = classify_gripper_state(
    state=None,
    indices=None,
    open_positions=(0.0,),
    tolerances=(0.01,),
)
