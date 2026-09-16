import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT_NAME = ROOT.name
RUN = ROOT / ".docker" / "run.bash"
JOIN = ROOT / ".docker" / "join.bash"
BUILD = ROOT / ".docker" / "build.bash"
DEV_MOUNT = f"/root/ws/src/{PROJECT_NAME}"


def _write_executable(path: Path, contents: str) -> None:
    path.write_text(contents)
    path.chmod(0o755)


class DockerHelpersTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory(prefix="pymoveit2-docker-test-")
        self.tmp = Path(self.tempdir.name)
        self.fakebin = self.tmp / "bin"
        self.fakebin.mkdir()
        self.x11_dir = self.tmp / "x11"
        self.x11_dir.mkdir()
        self.docker_log = self.tmp / "docker.jsonl"
        self.xauth_log = self.tmp / "xauth.jsonl"
        self.xauth_merge_log = self.tmp / "xauth-merge.log"
        _write_executable(
            self.fakebin / "docker",
            """#!/usr/bin/env python3
import json
import os
import signal
import sys
import time

args = sys.argv[1:]
with open(os.environ["DOCKER_LOG"], "a", encoding="utf-8") as stream:
    stream.write(json.dumps(args) + "\\n")
if args[:2] == ["image", "inspect"]:
    raise SystemExit(int(os.environ.get("DOCKER_INSPECT_STATUS", "1")))
if args[:2] == ["container", "list"]:
    sys.stdout.write(os.environ.get("DOCKER_CONTAINERS", ""))
    raise SystemExit(0)
if args[:1] == ["run"]:
    if os.environ.get("DOCKER_SIGNAL_PARENT") == "TERM":
        os.kill(os.getppid(), signal.SIGTERM)
        time.sleep(0.2)
        raise SystemExit(143)
    raise SystemExit(int(os.environ.get("DOCKER_RUN_STATUS", "0")))
raise SystemExit(0)
""",
        )
        _write_executable(
            self.fakebin / "xauth",
            """#!/usr/bin/env python3
import json
import os
import sys

args = sys.argv[1:]
with open(os.environ["XAUTH_LOG"], "a", encoding="utf-8") as stream:
    stream.write(json.dumps(args) + "\\n")
auth_file = args[args.index("-f") + 1]
operation = args[args.index("-f") + 2]
if operation == "nlist":
    if os.environ.get("XAUTH_FAIL") == "1":
        raise SystemExit(1)
    sys.stdout.write("0100 0000 0000 0002 3939 0000 0000\\n")
elif operation == "nmerge":
    payload = sys.stdin.buffer.read()
    with open(os.environ["XAUTH_MERGE_LOG"], "wb") as stream:
        stream.write(payload)
    with open(auth_file, "wb") as stream:
        stream.write(payload or b"fixture-auth\\n")
""",
        )
        _write_executable(
            self.fakebin / "xhost",
            """#!/usr/bin/env python3
import os
with open(os.environ["XHOST_LOG"], "a", encoding="utf-8") as stream:
    stream.write("called\\n")
""",
        )

    def tearDown(self) -> None:
        self.tempdir.cleanup()

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["PATH"] = f"{self.fakebin}{os.pathsep}{env['PATH']}"
        env["DOCKER_LOG"] = str(self.docker_log)
        env["XAUTH_LOG"] = str(self.xauth_log)
        env["XAUTH_MERGE_LOG"] = str(self.xauth_merge_log)
        env["XHOST_LOG"] = str(self.tmp / "xhost.log")
        env["PYMOVEIT2_X11_SOCKET_DIR"] = str(self.x11_dir)
        env["HOME"] = str(self.tmp / "home")
        env.pop("DISPLAY", None)
        env.pop("XAUTHORITY", None)
        return env

    def _run(
        self, script: Path, *args: str, **overrides: str
    ) -> subprocess.CompletedProcess:
        env = self._env()
        env.update(overrides)
        return subprocess.run(
            [str(script), *args],
            cwd=ROOT,
            env=env,
            text=True,
            capture_output=True,
            check=False,
        )

    def _calls(self) -> list[list[str]]:
        if not self.docker_log.exists():
            return []
        return [json.loads(line) for line in self.docker_log.read_text().splitlines()]

    def _wrapper(self, run: list[str]) -> str:
        index = run.index("bash")
        assert run[index + 1] == "-c", run
        return run[index + 2]

    def test_default_run_is_headless_private_and_forwards_spaces(self) -> None:
        result = self._run(
            RUN,
            "-v",
            "/host path:/container path:ro",
            "-e",
            "KEY=value with spaces",
            "bash",
            "-lc",
            "printf 'value with spaces'",
            DISPLAY=":99",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self._calls()
        run = calls[-1]
        self.assertEqual(run[0], "run")
        self.assertEqual(run[run.index("--network") + 1], "bridge")
        self.assertEqual(run[run.index("--ipc") + 1], "private")
        self.assertTrue("ROS_LOCALHOST_ONLY=1" in run)
        self.assertTrue("ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in run)
        self.assertNotIn("host", run)
        self.assertNotIn("DISPLAY=:99", run)
        self.assertTrue("/host path:/container path:ro" in run)
        self.assertTrue("KEY=value with spaces" in run)
        self.assertFalse((self.tmp / "xhost.log").exists())

        self.assertIn(f"{ROOT}:{DEV_MOUNT}", run)
        self.assertIn(f"PYMOVEIT2_DEV_MOUNT={DEV_MOUNT}", run)
        self.assertEqual(run[run.index("--name") + 1], PROJECT_NAME)

        self.assertNotIn("--tty", run)

    def test_explicit_host_and_gui_options_use_scoped_read_only_auth(self) -> None:
        xauth = self.tmp / "source auth"
        xauth.write_bytes(b"fixture-auth")
        result = self._run(
            RUN,
            "--gui",
            "--network-host",
            "--ipc-host",
            "-v",
            "/host path:/container path:ro",
            "-e",
            "KEY=value with spaces",
            "demo",
            "bash",
            "-lc",
            "printf ok",
            DISPLAY=":99",
            XAUTHORITY=str(xauth),
            DOCKER_INSPECT_STATUS="0",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertEqual(run[run.index("--network") + 1], "host")
        self.assertEqual(run[run.index("--ipc") + 1], "host")
        self.assertNotIn("ROS_LOCALHOST_ONLY=1", run)
        self.assertTrue("DISPLAY=:99" in run)
        self.assertTrue("XAUTHORITY=/tmp/pymoveit2.xauth" in run)
        mount_index = run.index("--volume")
        mounts = run[mount_index + 1 :]
        self.assertTrue(f"{self.x11_dir}:/tmp/.X11-unix:ro" in mounts)
        auth_mount = next(
            value for value in mounts if ":/tmp/pymoveit2.xauth:ro" in value
        )
        self.assertFalse(Path(auth_mount.split(":", 1)[0]).exists())
        self.assertFalse((self.tmp / "xhost.log").exists())
        xauth_calls = [
            json.loads(line) for line in self.xauth_log.read_text().splitlines()
        ]
        self.assertEqual(sorted(call[2] for call in xauth_calls), ["nlist", "nmerge"])
        self.assertTrue(self.xauth_merge_log.read_bytes().startswith(b"ffff "))

    def test_conflicting_network_options_fail_before_docker(self) -> None:
        result = self._run(RUN, "--network-host", "--network", "private-net", "bash")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self._calls(), [])

    def test_gui_error_and_docker_error_clean_temporary_auth(self) -> None:
        missing = self.tmp / "missing auth"
        result = self._run(
            RUN,
            "--gui",
            "bash",
            DISPLAY=":99",
            XAUTHORITY=str(missing),
            TMPDIR=str(self.tmp),
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self._calls(), [])
        self.assertFalse((self.tmp / "xhost.log").exists())
        self.assertEqual(list(self.tmp.glob("pymoveit2-xauth.*")), [])

        xauth = self.tmp / "source auth"
        xauth.write_bytes(b"fixture-auth")
        result = self._run(
            RUN,
            "--gui",
            "bash",
            DISPLAY=":99",
            XAUTHORITY=str(xauth),
            TMPDIR=str(self.tmp),
            XAUTH_FAIL="1",
        )
        self.assertEqual(result.returncode, 2)
        self.assertEqual(list(self.tmp.glob("pymoveit2-xauth.*")), [])

        result = self._run(
            RUN,
            "--gui",
            "bash",
            DISPLAY=":99",
            XAUTHORITY=str(xauth),
            TMPDIR=str(self.tmp),
            DOCKER_RUN_STATUS="37",
        )
        self.assertEqual(result.returncode, 37)
        self.assertEqual(list(self.tmp.glob("pymoveit2-xauth.*")), [])

    def test_signal_path_cleans_temporary_auth(self) -> None:
        xauth = self.tmp / "source auth"
        xauth.write_bytes(b"fixture-auth")
        result = self._run(
            RUN,
            "--gui",
            "bash",
            DISPLAY=":99",
            XAUTHORITY=str(xauth),
            TMPDIR=str(self.tmp),
            DOCKER_SIGNAL_PARENT="TERM",
        )
        self.assertEqual(result.returncode, 143, result.stderr)
        self.assertEqual(list(self.tmp.glob("pymoveit2-xauth.*")), [])

    def test_dev_volume_wraps_command_for_path_and_ownership(self) -> None:
        result = self._run(RUN, "bash", "-lc", "printf ok")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertIn(f"{ROOT}:{DEV_MOUNT}", run)
        self.assertIn(f"PYMOVEIT2_HOST_UID={os.getuid()}", run)
        self.assertIn(f"PYMOVEIT2_HOST_GID={os.getgid()}", run)
        wrapper = self._wrapper(run)
        self.assertIn('export PYTHONPATH="${PYMOVEIT2_DEV_MOUNT}', wrapper)
        self.assertIn("chown -R --from=0:0", wrapper)
        self.assertIn("trap 'pymoveit2_restore_ownership' EXIT", wrapper)

        self.assertEqual(run[-3:], ["bash", "-lc", "printf ok"])

    def test_run_without_a_command_opens_a_shell_through_the_wrapper(self) -> None:
        result = self._run(RUN)
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertEqual(run[-4:-2], ["bash", "-c"])
        self.assertEqual(run[-2], self._wrapper(run))
        self.assertEqual(run[-1], f"{PROJECT_NAME}-dev")

        self.assertIn('if [ "${#}" -eq 0 ]; then', run[-2])

    def test_missing_image_is_reported_before_docker_attempts_a_pull(self) -> None:
        result = self._run(RUN, "bash")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("is not present locally", result.stderr)
        self.assertIn("build.bash", result.stderr)

        result = self._run(RUN, "bash", DOCKER_INSPECT_STATUS="0")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("is not present locally", result.stderr)

    def test_no_dev_volume_runs_the_image_sources_unwrapped(self) -> None:
        result = self._run(RUN, "--no-dev-volume", "bash", "-lc", "printf ok")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertNotIn(f"{ROOT}:{DEV_MOUNT}", run)
        self.assertNotIn(f"PYMOVEIT2_DEV_MOUNT={DEV_MOUNT}", run)
        self.assertEqual(run[-3:], ["bash", "-lc", "printf ok"])

    def test_dev_wrapper_can_be_disabled_while_keeping_the_mount(self) -> None:
        result = self._run(RUN, "bash", WITH_DEV_WRAPPER="false")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertIn(f"{ROOT}:{DEV_MOUNT}", run)
        self.assertNotIn(f"PYMOVEIT2_DEV_MOUNT={DEV_MOUNT}", run)
        self.assertEqual(run[-1], "bash")

    def test_invalid_boolean_flag_fails_before_docker(self) -> None:
        result = self._run(RUN, "bash", WITH_DEV_VOLUME="maybe")
        self.assertEqual(result.returncode, 2)
        self.assertIn("WITH_DEV_VOLUME", result.stderr)
        self.assertEqual(self._calls(), [])

    def test_concurrent_runs_get_distinct_container_names(self) -> None:
        result = self._run(
            RUN,
            "bash",
            DOCKER_CONTAINERS=f"{PROJECT_NAME}\n{PROJECT_NAME}-1\n",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self._calls()[-1]
        self.assertEqual(run[run.index("--name") + 1], f"{PROJECT_NAME}-2")

    def test_join_attaches_to_the_single_running_container(self) -> None:
        result = self._run(JOIN, DOCKER_CONTAINERS=f"{PROJECT_NAME}\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self._calls()[-1]
        self.assertEqual(call[0], "exec")
        self.assertEqual(call[-2:], [PROJECT_NAME, "bash"])

    def test_join_selects_a_container_by_id_and_forwards_the_command(self) -> None:
        result = self._run(
            JOIN,
            "2",
            "ros2",
            "topic list",
            DOCKER_CONTAINERS=f"{PROJECT_NAME}\n{PROJECT_NAME}-2\n",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self._calls()[-1]
        self.assertEqual(call[-3:], [f"{PROJECT_NAME}-2", "ros2", "topic list"])

    def test_join_requires_an_id_when_several_containers_run(self) -> None:
        result = self._run(
            JOIN, DOCKER_CONTAINERS=f"{PROJECT_NAME}\n{PROJECT_NAME}-1\n"
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn(f"{PROJECT_NAME}-1", result.stderr)
        self.assertEqual([call[0] for call in self._calls()], ["container"])

    def test_join_without_a_running_container_reports_how_to_start_one(self) -> None:
        result = self._run(JOIN)
        self.assertEqual(result.returncode, 1)
        self.assertIn(".docker/run.bash", result.stderr)

    def test_build_forwards_context_and_source_identity(self) -> None:
        result = self._run(
            BUILD,
            "test tag",
            "--build-arg",
            "ROS_DISTRO=humble",
            "--build-arg=WITH_DEMO=false",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self._calls()[-1]
        self.assertEqual(call[0], "build")
        self.assertEqual(
            call[call.index("--tag") + 1], f"andrejorsula/{PROJECT_NAME}:test tag"
        )
        self.assertEqual(call[-1], str(ROOT))
        source_index = call.index("--build-arg")
        self.assertTrue(call[source_index + 1].startswith("SOURCE_ID="))
        self.assertTrue("ROS_DISTRO=humble" in call)
        self.assertTrue("--build-arg=WITH_DEMO=false" in call)


if __name__ == "__main__":
    unittest.main()
