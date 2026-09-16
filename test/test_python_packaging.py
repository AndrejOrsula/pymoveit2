import re
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PYPROJECT = (REPO / "pyproject.toml").read_text()
MANIFEST = (REPO / "MANIFEST.in").read_text()


ROS_ONLY_DEPENDENCIES = (
    "rclpy",
    "action_msgs",
    "control_msgs",
    "geometry_msgs",
    "moveit_msgs",
    "rcl_interfaces",
    "sensor_msgs",
    "shape_msgs",
    "std_msgs",
    "std_srvs",
    "trajectory_msgs",
)


def _table(name: str) -> str:
    match = re.search(
        rf"^\[{re.escape(name)}\]$(.*?)(?=^\[|\Z)", PYPROJECT, re.MULTILINE | re.DOTALL
    )
    assert match, f"missing [{name}] in pyproject.toml"
    return match.group(1)


def _package_xml_version() -> str:
    root = ET.parse(REPO / "package.xml").getroot()
    version = root.findtext("version")
    assert version, "package.xml declares no version"
    return version.strip()


def test_pypi_version_matches_the_ros_package():
    match = re.search(r'^version = "([^"]+)"$', _table("project"), re.MULTILINE)
    assert match, "pyproject.toml declares no project version"
    assert match.group(1) == _package_xml_version()


def test_changelog_documents_the_declared_version():
    changelog = (REPO / "CHANGELOG.rst").read_text()
    version = _package_xml_version()

    assert f"\n{version} (" in changelog or "\nForthcoming\n" in changelog


def test_every_importable_subpackage_is_published():
    declared = set(re.findall(r'"([\w.]+)"', _table("tool.setuptools")))
    found = {
        ".".join(path.parent.relative_to(REPO).parts)
        for path in (REPO / "pymoveit2").rglob("__init__.py")
    }
    assert found, "no Python packages found under pymoveit2/"
    assert found <= declared, f"unpublished subpackages: {sorted(found - declared)}"


def test_no_ros_only_runtime_dependencies_are_declared():
    project = _table("project")
    match = re.search(r"^dependencies = \[(.*?)\]", project, re.MULTILINE | re.DOTALL)
    assert match, "pyproject.toml declares no dependencies list"
    declared = set(re.findall(r'"([^"]+)"', match.group(1)))
    assert not declared & set(ROS_ONLY_DEPENDENCIES), sorted(declared)
    assert "numpy" in declared


def test_source_distribution_carries_the_ros_build_files():
    for entry in ("package.xml", "CMakeLists.txt", "CHANGELOG.rst", "LICENSE"):
        assert f"include {entry}" in MANIFEST, entry
    assert "recursive-include examples *.py" in MANIFEST


def test_readme_is_the_published_long_description():
    project = _table("project")
    assert 'readme = "README.md"' in project
    assert 'license = "BSD-3-Clause"' in project
