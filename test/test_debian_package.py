import ast
import os
import re
import stat
import subprocess
from pathlib import Path
from xml.etree import ElementTree

import pytest

REPO = Path(__file__).resolve().parents[1]
CHECK = REPO / ".ci" / "scripts" / "check-debian-package.bash"
BUILD = REPO / ".ci" / "scripts" / "build-debian-package.bash"
EXECUTABLES = (
    "ex_allow_collisions.py",
    "ex_clear_planning_scene.py",
    "ex_collision_mesh.py",
    "ex_collision_primitive.py",
    "ex_doctor.py",
    "ex_fk.py",
    "ex_gripper.py",
    "ex_ik.py",
    "ex_joint_goal.py",
    "ex_orientation_path_constraint.py",
    "ex_pose_goal.py",
    "ex_servo.py",
)


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _prepare_fake_runtime(
    tmp_path: Path,
    *,
    fk_status: int,
    diagnostic: str,
    timeout: bool = False,
    deb_version: str = "4.3.0-2026.09.08",
):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    prefix = tmp_path / "install"
    executable_dir = prefix / "lib" / "pymoveit2"
    (executable_dir / "assets").mkdir(parents=True)
    (executable_dir / "assets" / "suzanne.stl").write_text("solid stub\n")
    for executable in EXECUTABLES:
        (executable_dir / executable).write_text("#!/usr/bin/env python3\n")

    _write_executable(
        fake_bin / "apt-get",
        """#!/usr/bin/env bash
set -eu
if [[ "$*" == *--version* ]]; then
    echo 'apt 2.0.0 (fake)'
fi
""",
    )
    _write_executable(
        fake_bin / "apt-cache",
        """#!/usr/bin/env bash
set -eu
echo 'Candidate: 4.3.0-fake'
""",
    )
    _write_executable(
        fake_bin / "dpkg-deb",
        """#!/usr/bin/env bash
set -eu
case "$*" in
    *' Package') echo 'ros-jazzy-pymoveit2' ;;
    *' Version') echo "$FAKE_DEB_VERSION" ;;
    *) exit 2 ;;
esac
""",
    )
    _write_executable(
        fake_bin / "dpkg-query",
        """#!/usr/bin/env bash
set -eu
if [[ "$*" == *--version* ]]; then
    echo 'dpkg-query (fake)'
else
    echo 'install ok installed'
fi
""",
    )
    _write_executable(
        fake_bin / "python3",
        """#!/usr/bin/env bash
set -eu
if [[ "$1" == '--version' ]]; then
    echo 'Python 3.10.0 (fake)'
    elif [[ "$1" == '-m' && "$2" == 'py_compile' ]]; then
    exit 0
elif [[ "$1" == '-' ]]; then
    printf '%s\n' "$2" >>"$PYMOVEIT2_PYTHON_PROBE_LOG"
    cat >/dev/null
    exit 0
elif [[ "$1" == '-c' && "$2" == *'trimesh'* ]]; then
    exit 1
elif [[ "$1" == '-c' ]]; then
    echo '/opt/ros/jazzy/lib/python3.10/site-packages/pymoveit2/__init__.py'
fi
""",
    )
    _write_executable(
        fake_bin / "rosdep",
        """#!/usr/bin/env bash
set -eu
echo 'rosdep 0.26.0 (fake)'
""",
    )
    executable_output = " ".join(f'"pymoveit2 {name}"' for name in EXECUTABLES)
    _write_executable(
        fake_bin / "ros2",
        f"""#!/usr/bin/env bash
set -eu
case "$*" in
    '--version') echo 'ros2 0.33.0 (fake)' ;;
    'pkg executables pymoveit2')
        printf '%s\\n' {executable_output}
        ;;
    'pkg prefix pymoveit2') echo "$FAKE_PREFIX" ;;
    'run pymoveit2 ex_fk.py')
        echo "$FAKE_FK_DIAGNOSTIC"
        exit "$FAKE_FK_STATUS"
        ;;
    *) exit 2 ;;
esac
""",
    )
    _write_executable(
        fake_bin / "timeout",
        """#!/usr/bin/env bash
set -eu
while [[ "$1" == -* ]]; do shift; done
if [[ "$FAKE_TIMEOUT_MODE" == timeout ]]; then
    exit 124
fi
shift
exec "$@"
""",
    )

    artifact_dir = tmp_path / "apt_repo"
    artifact_dir.mkdir()
    (artifact_dir / "ros-jazzy-pymoveit2_4.3.0_amd64.deb").write_bytes(b"fake deb")
    setup = tmp_path / "setup.bash"
    setup.write_text("# fake ROS setup\n")
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "FAKE_PREFIX": str(prefix),
            "FAKE_FK_STATUS": str(fk_status),
            "FAKE_FK_DIAGNOSTIC": diagnostic,
            "FAKE_TIMEOUT_MODE": "timeout" if timeout else "",
            "FAKE_DEB_VERSION": deb_version,
            "PYMOVEIT2_ROS_SETUP_FILE": str(setup),
            "EXPECTED_PACKAGE_VERSION_PREFIX": "4.3.0",
            "PYMOVEIT2_EXPECTED_FK_DIAGNOSTIC": "Failed.",
            "PYMOVEIT2_PYTHON_PROBE_LOG": str(tmp_path / "python-probes.log"),
        }
    )
    return artifact_dir, environment


def _run_check(
    tmp_path: Path,
    *,
    fk_status: int,
    diagnostic: str = "Failed.",
    timeout: bool = False,
    deb_version: str = "4.3.0-2026.09.08",
):
    artifact_dir, environment = _prepare_fake_runtime(
        tmp_path,
        fk_status=fk_status,
        diagnostic=diagnostic,
        timeout=timeout,
        deb_version=deb_version,
    )
    return (
        subprocess.run(
            [str(CHECK), "jazzy", str(artifact_dir)],
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        ),
        artifact_dir,
    )


def test_check_accepts_only_the_expected_no_server_failure(tmp_path):
    result, artifact_dir = _run_check(tmp_path, fk_status=1)

    assert result.returncode == 0, result.stderr + result.stdout
    provenance = (artifact_dir / "provenance.txt").read_text()
    assert "artifact_smoke=passed\n" in provenance
    assert "package_version=4.3.0-2026.09.08\n" in provenance
    probes = (tmp_path / "python-probes.log").read_text().splitlines()
    assert len(probes) == len(EXECUTABLES)
    assert {Path(probe).name for probe in probes} == set(EXECUTABLES)


@pytest.mark.parametrize("status", (0, 124, 126, 127, 137, 139, 143))
def test_check_rejects_success_timeout_setup_and_signal_statuses(tmp_path, status):
    result, _ = _run_check(tmp_path, fk_status=status)

    assert result.returncode != 0
    assert "must exit exactly 1" in result.stderr


def test_check_rejects_a_timeout(tmp_path):
    result, _ = _run_check(tmp_path, fk_status=1, timeout=True)

    assert result.returncode != 0
    assert "must exit exactly 1" in result.stderr
    assert "got 124" in result.stderr


def test_check_rejects_status_one_without_the_expected_message(tmp_path):
    result, _ = _run_check(tmp_path, fk_status=1, diagnostic="setup failed")

    assert result.returncode != 0
    assert "lacked the expected message" in result.stderr


def test_check_rejects_a_missing_package_path(tmp_path):
    artifact_dir, environment = _prepare_fake_runtime(
        tmp_path, fk_status=1, diagnostic="Failed."
    )
    (artifact_dir / "ros-jazzy-pymoveit2_4.3.0_amd64.deb").unlink()

    result = subprocess.run(
        [str(CHECK), "jazzy", str(artifact_dir)],
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "exactly one pymoveit2 Debian package" in result.stderr


def test_check_rejects_a_version_without_a_release_boundary(tmp_path):
    result, _ = _run_check(tmp_path, fk_status=1, deb_version="4.3.01")

    assert result.returncode != 0
    assert "does not match 4.3.0" in result.stderr


def test_debian_builder_rejects_package_directory_as_artifact_directory(tmp_path):
    package_dir = tmp_path / "repo"
    package_dir.mkdir()
    (package_dir / "package.xml").write_text(
        '<package format="3"><name>pymoveit2</name><version>4.3.0</version></package>\n'
    )

    result = subprocess.run(
        [str(BUILD), "jazzy", "noble", "amd64", str(package_dir), str(package_dir)],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "artifact directory must be outside" in result.stderr


def test_debian_builder_rejects_artifact_descendant_of_package_directory(tmp_path):
    package_dir = tmp_path / "repo"
    package_dir.mkdir()
    (package_dir / "package.xml").write_text(
        '<package format="3"><name>pymoveit2</name><version>4.3.0</version></package>\n'
    )

    result = subprocess.run(
        [
            str(BUILD),
            "jazzy",
            "noble",
            "amd64",
            str(package_dir),
            str(package_dir / "artifacts"),
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "artifact directory must be outside" in result.stderr


def test_debian_builder_rejects_dangling_debian_metadata_link(tmp_path):
    package_dir = tmp_path / "repo"
    package_dir.mkdir()
    (package_dir / "package.xml").write_text(
        '<package format="3"><name>pymoveit2</name><version>4.3.0</version></package>\n'
    )
    (package_dir / "debian").symlink_to("missing-debian-metadata")

    result = subprocess.run(
        [
            str(BUILD),
            "jazzy",
            "noble",
            "amd64",
            str(package_dir),
            str(tmp_path / "apt"),
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "refusing to overwrite existing Debian metadata" in result.stderr


def test_the_checker_expects_exactly_the_installed_examples():
    listed = re.search(
        r"^expected_executables=\(\n(.*?)^\)$",
        CHECK.read_text(),
        re.MULTILINE | re.DOTALL,
    )
    assert listed is not None, "expected_executables array not found"
    from_checker = sorted(listed.group(1).split())
    from_examples = sorted(path.name for path in (REPO / "examples").glob("ex_*.py"))

    assert from_checker == from_examples
    assert sorted(EXECUTABLES) == from_examples


def _strings_raised_from_except_blocks(source: str) -> list:
    found = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.ExceptHandler):
            continue
        for inner in ast.walk(node):
            if isinstance(inner, ast.Constant) and isinstance(inner.value, str):
                found.append(inner.value)
            elif isinstance(inner, ast.JoinedStr):
                found.extend(
                    part.value
                    for part in inner.values
                    if isinstance(part, ast.Constant) and isinstance(part.value, str)
                )
    return found


def test_the_default_fk_diagnostic_is_what_a_failed_setup_prints():
    default = re.search(
        r'expected_diagnostic="\$\{PYMOVEIT2_EXPECTED_FK_DIAGNOSTIC:-([^}]+)\}"',
        CHECK.read_text(),
    )
    assert default is not None, "default FK diagnostic not found"

    emitted = _strings_raised_from_except_blocks(
        (REPO / "examples" / "ex_fk.py").read_text()
    )
    assert any(
        text.startswith(default.group(1)) for text in emitted
    ), f"{default.group(1)!r} is not printed by any except block of ex_fk.py: {emitted}"


def test_the_default_version_prefix_matches_the_manifest():
    default = re.search(
        r'expected_version_prefix="\$\{EXPECTED_PACKAGE_VERSION_PREFIX:-([^}]+)\}"',
        CHECK.read_text(),
    )
    assert default is not None, "default version prefix not found"
    manifest = ElementTree.parse(REPO / "package.xml").getroot().findtext("version")
    assert default.group(1) == (manifest or "").strip()
