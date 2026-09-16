import re
from pathlib import Path


def test_no_rclpy_logger_warn():
    package_root = Path(__file__).parent.parent / "pymoveit2"
    offenders = []
    for path in sorted(package_root.rglob("*.py")):
        for lineno, line in enumerate(path.read_text().splitlines(), start=1):
            if re.search(r"(?<!warnings)\.warn\(", line):
                offenders.append(f"{path.name}:{lineno}: {line.strip()}")
    assert not offenders, "Deprecated Logger.warn calls found:\n" + "\n".join(offenders)
