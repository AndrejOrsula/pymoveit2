import ast
from pathlib import Path

import pytest

PACKAGE = Path(__file__).parent.parent / "pymoveit2"


def test_no_assert_statements_in_production_code():
    offenders = []
    for path in sorted(PACKAGE.rglob("*.py")):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Assert):
                offenders.append(f"{path.name}:{node.lineno}")
    assert not offenders, "assert statements are stripped by `python -O`: " + ", ".join(
        offenders
    )


def test_no_wildcard_imports_in_production_code():
    for path in sorted(PACKAGE.rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                assert not any(alias.name == "*" for alias in node.names), path.name


def test_public_callables_have_return_annotations():
    missing = []
    for path in sorted(PACKAGE.rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
                if node.returns is None:
                    missing.append(f"{path.name}:{node.name}")
    assert not missing, missing


def test_collision_box_size_validation(moveit2):
    with pytest.raises(ValueError):
        moveit2.add_collision_box(
            id="box", size=(1.0, 1.0), position=(0, 0, 0), quat_xyzw=(0, 0, 0, 1)
        )


def test_pose_input_validation(moveit2):
    with pytest.raises(ValueError):
        moveit2.set_pose_goal(position=(0.0, 0.0), quat_xyzw=(0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        moveit2.set_pose_goal(position=(0.0, 0.0, 0.0), quat_xyzw=(0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        moveit2.set_pose_goal()
    with pytest.raises(ValueError):
        moveit2.set_pose_goal(pose="not a pose")
    with pytest.raises(ValueError):
        moveit2.set_orientation_goal(quat_xyzw=(0, 0, 0, 1), tolerance=(0.1, 0.1))
    with pytest.raises(ValueError):
        moveit2.set_joint_goal([0.0] * 8)
    with pytest.raises(ValueError):
        moveit2.add_collision_primitive(id="x", primitive_type=1, dimensions=(1,))


def test_set_pose_goal_accepts_pose_types(moveit2):
    from geometry_msgs.msg import Pose, PoseStamped

    moveit2.set_pose_goal(pose=Pose())
    moveit2.set_pose_goal(pose=PoseStamped())
    moveit2.set_pose_goal(
        position=(0.1, 0.2, 0.3), quat_xyzw=(0, 0, 0, 1), frame_id="world"
    )
    constraints = moveit2._MoveIt2__move_action_goal.request.goal_constraints[-1]
    assert len(constraints.position_constraints) == 3
    assert constraints.position_constraints[0].header.frame_id == "panda_link0"
    assert constraints.position_constraints[2].header.frame_id == "world"
    assert constraints.position_constraints[2].link_name == "panda_hand"
    moveit2.clear_goal_constraints()
    assert not moveit2._MoveIt2__move_action_goal.request.goal_constraints[
        -1
    ].position_constraints
