import re
from pathlib import Path

REPO = Path(__file__).parent.parent
MAX_ITEMS = 5
MARKDOWN = ("README.md", "CONTRIBUTING.md")
SCRIPT_DIRS = (".docker", ".ci/scripts", "test/scripts", ".git_hooks")


def test_scripts_lead_with_what_they_do_and_how_to_run_them():
    scripts = []
    for directory in SCRIPT_DIRS:
        found = sorted((REPO / directory).glob("*.bash"))
        assert found, f"no script found in {directory}"
        scripts.extend(found)
    for path in scripts:
        lines = path.read_text().splitlines()
        assert lines[0].startswith("#!"), path.name
        assert re.match(r"^# \S.*\.$", lines[1]), f"{path.name}: {lines[1]}"
        assert lines[2].startswith("# Usage: "), f"{path.name}: {lines[2]}"


def test_example_docstrings_lead_with_runnable_commands():
    for path in sorted((REPO / "examples").glob("ex_*.py")):
        body = path.read_text().split('"""')[1].strip().splitlines()
        assert not body[0].startswith("-"), path.name
        commands = [line for line in body if line.startswith("- ros2 run")]
        assert 1 <= len(commands) <= MAX_ITEMS, f"{path.name}: {len(commands)}"
        assert body[1 : 1 + len(commands)] == commands, path.name


def test_documents_keep_lists_short():
    for name in MARKDOWN:
        run = 0
        in_code = False
        for number, line in enumerate((REPO / name).read_text().splitlines(), 1):
            if line.startswith("```"):
                in_code = not in_code
            elif in_code or not line.strip():
                continue
            elif re.match(r"^\s*([-*]|\d+\.)\s", line):
                run += 1
                assert run <= MAX_ITEMS, f"{name}:{number} list of {run} items"
            elif not line.startswith(" "):
                run = 0
