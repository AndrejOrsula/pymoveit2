import re
from pathlib import Path

REPO = Path(__file__).parent.parent
EXAMPLES = sorted(p.name for p in (REPO / "examples").glob("ex_*.py"))


def _cmake_examples():
    text = (REPO / "CMakeLists.txt").read_text()
    return sorted(set(re.findall(r"\$\{EXAMPLES_DIR\}/(ex_\w+\.py)", text)))


def test_cmake_installs_every_example():
    assert _cmake_examples() == EXAMPLES


def test_readme_lists_every_example():
    readme = (REPO / "README.md").read_text()
    for example in EXAMPLES:
        assert f"ros2 run pymoveit2 {example}" in readme, example


def test_example_docstrings_reference_existing_executables():
    installed = set(_cmake_examples())
    for path in (REPO / "examples").glob("ex_*.py"):
        for name in re.findall(r"ros2 run pymoveit2 (\S+)", path.read_text()):
            assert (
                name in installed
            ), f"{path.name} references unknown executable {name}"


def _uses_session(text):
    return "connect(" in text


def test_examples_discover_the_robot_configuration():
    for path in (REPO / "examples").glob("ex_*.py"):
        text = path.read_text()
        assert "from pymoveit2.robots import" not in text, path.name
        if _uses_session(text):
            assert "declare_robot_parameters(" not in text, path.name
            continue
        assert "declare_robot_parameters(node" in text, path.name
        if "MoveIt2Servo" in text:
            assert "frame_id=True" in text, path.name
            assert "robot.frame_id()" in text, path.name
        elif "GripperInterface" in text:
            assert "gripper=True" in text, path.name
            assert "robot.gripper_kwargs()" in text, path.name
        else:
            assert "robot.moveit2_kwargs()" in text, path.name


def test_examples_declare_no_robot_specific_defaults():
    for path in (REPO / "examples").glob("ex_*.py"):
        text = path.read_text()
        assert "RRTConnectkConfigDefault" not in text, path.name
        for name, default in re.findall(
            r'declare_parameter\(\s*"(\w*joint_positions)",\s*([^\n]+)', text
        ):
            assert "Parameter.Type" in default, f"{path.name}: {name}"


ROBOT_NAMES = re.compile(
    r"\b(panda|franka|emika|kinova|jaco|lbr|kuka|iiwa|crane_x7|phantomx"
    r"|ur3e?|ur5e?|ur10e?|ur16e?|xarm|fanuc)\b",
    re.IGNORECASE,
)


def test_only_the_presets_name_a_robot():
    sources = list((REPO / "examples").glob("ex_*.py"))
    sources += [p for p in (REPO / "pymoveit2").glob("*.py")]
    for path in sources:
        found = ROBOT_NAMES.findall(path.read_text())
        assert not found, f"{path.name} names {sorted(set(found))}"


def test_examples_start_the_executor_before_discovery():
    for path in (REPO / "examples").glob("ex_*.py"):
        text = path.read_text()
        if _uses_session(text):
            continue
        spin = text.index("executor_thread.start()")
        resolve = text.index("RobotConfiguration(node")
        assert spin < resolve, path.name


def test_example_assets_installed():
    assert (REPO / "examples" / "assets" / "suzanne.stl").exists()
    assert (
        "install(DIRECTORY ${EXAMPLES_DIR}/assets"
        in (REPO / "CMakeLists.txt").read_text()
    )
