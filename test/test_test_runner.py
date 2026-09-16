import atexit
import json
import os
import shutil
import signal
import stat
import subprocess
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).parents[1]
RUNNER = REPO / "test" / "scripts" / "run-tests.bash"
_FAKE_DOCKER_DIRS = []


@atexit.register
def _remove_fake_docker_dirs():
    for directory in _FAKE_DOCKER_DIRS:
        shutil.rmtree(directory, ignore_errors=True)


def _fake_docker(tmp_path):
    del tmp_path
    fake_dir = Path(tempfile.mkdtemp(prefix="pymoveit2-test-docker-", dir="/var/tmp"))
    _FAKE_DOCKER_DIRS.append(fake_dir)
    docker = fake_dir / "docker"
    docker.write_text("""#!/usr/bin/env python3
import json
import os
import signal
import sys
import time

args = sys.argv[1:]
log = os.environ["FAKE_DOCKER_LOG"]
with open(log, "a", encoding="utf-8") as stream:
    stream.write(json.dumps(args) + "\\n")

if args[:2] == ["image", "inspect"]:
    raise SystemExit(0)
if args[:2] == ["network", "create"]:
    print("network-owned-id")
    raise SystemExit(0)
if args[:2] == ["network", "rm"]:
    raise SystemExit(0)
if args and args[0] == "create":
    print("container-owned-id")
    raise SystemExit(0)
if args and args[0] == "start":
    if os.environ.get("FAKE_DOCKER_MODE") == "interrupt":
        with open(log + ".started", "w", encoding="utf-8") as stream:
            stream.write("1\\n")
        signal.pause()
    raise SystemExit(0)
if args and args[0] == "rm":
    raise SystemExit(0)
if args and args[0] == "pull":
    raise SystemExit(0)
raise SystemExit(0)
""")
    docker.chmod(docker.stat().st_mode | stat.S_IEXEC)
    return docker


def _run_runner(tmp_path, distro="jazzy", *, mode="success"):
    docker = _fake_docker(tmp_path)
    log = tmp_path / "docker.log"
    env = os.environ.copy()
    env["PATH"] = f"{docker.parent}{os.pathsep}{env['PATH']}"
    env["FAKE_DOCKER_LOG"] = str(log)
    env["FAKE_DOCKER_MODE"] = mode
    env["PYMOVEIT2_STAGE_TIMEOUT_SEC"] = "1"
    env["PYMOVEIT2_PULL_TIMEOUT_SEC"] = "1"
    env["PYMOVEIT2_DOCKER_TIMEOUT_SEC"] = "5"
    return (
        subprocess.run(
            [str(RUNNER), distro],
            cwd=REPO,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        ),
        log,
    )


def _calls(log):
    if not log.exists():
        return []
    return [json.loads(line) for line in log.read_text().splitlines()]


def test_unsupported_distribution_still_exits_two_without_docker(tmp_path):
    result, log = _run_runner(tmp_path, "no-such-distro")

    assert result.returncode == 2
    assert "Unsupported ROS 2 distribution" in result.stdout
    assert _calls(log) == []


def test_runner_declares_bounded_diagnostic_stages_and_cleanup():
    source = RUNNER.read_text()

    for stage in ("dns", "apt", "rosdep", "build", "test"):
        assert stage in source
    assert "timeout" in source
    assert "apt-get update" in source
    assert "rosdep update --rosdistro" in source
    assert "colcon build" in source
    assert "colcon test" in source
    assert "trap" in source
    assert "docker rm -f" in source
    assert "docker network rm" in source


def test_runner_uses_private_network_and_read_only_checkout(tmp_path):
    result, log = _run_runner(tmp_path)

    assert result.returncode == 0, result.stdout
    calls = _calls(log)
    create = next(call for call in calls if call and call[0] == "create")
    assert "--network" in create
    assert create[create.index("--network") + 1] == "network-owned-id"
    mounts = [
        value
        for index, value in enumerate(create)
        if index and create[index - 1] == "-v"
    ]
    assert any(value.endswith(":/repo:ro") for value in mounts)
    assert "/tmp:rw,exec,nosuid,nodev" in create
    assert "host" not in create
    assert "none" not in create
    assert "-e" in create
    assert "ROS_LOCALHOST_ONLY=1" in create
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in create
    assert any(call[:2] == ["network", "create"] for call in calls)
    assert ["rm", "-f", "container-owned-id"] in calls
    assert ["network", "rm", "network-owned-id"] in calls


def test_interrupt_returns_signal_status_and_removes_owned_resources(tmp_path):
    docker = _fake_docker(tmp_path)
    log = tmp_path / "docker.log"
    env = os.environ.copy()
    env["PATH"] = f"{docker.parent}{os.pathsep}{env['PATH']}"
    env["FAKE_DOCKER_LOG"] = str(log)
    env["FAKE_DOCKER_MODE"] = "interrupt"
    env["PYMOVEIT2_STAGE_TIMEOUT_SEC"] = "1"
    env["PYMOVEIT2_PULL_TIMEOUT_SEC"] = "1"
    env["PYMOVEIT2_DOCKER_TIMEOUT_SEC"] = "30"

    process = subprocess.Popen(
        [str(RUNNER), "jazzy"],
        cwd=REPO,
        env=env,
        start_new_session=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    started = Path(str(log) + ".started")
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not started.exists():
        time.sleep(0.02)
    assert started.exists(), (
        process.stdout.read() if process.poll() is not None else "runner did not start"
    )

    os.killpg(process.pid, signal.SIGINT)
    output, _ = process.communicate(timeout=10)

    assert process.returncode == 130, output
    calls = _calls(log)
    assert ["rm", "-f", "container-owned-id"] in calls
    assert ["network", "rm", "network-owned-id"] in calls
