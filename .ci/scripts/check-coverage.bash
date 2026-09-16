#!/usr/bin/env bash
# Usage: [PYTHON_BIN=python3] [COVERAGE_BRANCH_FAIL_UNDER=68] .ci/scripts/check-coverage.bash [pytest arguments ...]
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
REPORT_DIR="${COVERAGE_REPORT_DIR:-${TMPDIR:-/tmp}/pymoveit2-coverage}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "Python interpreter '${PYTHON_BIN}' is unavailable." >&2
    exit 2
fi
if ! "${PYTHON_BIN}" -c "import coverage" >/dev/null 2>&1; then
    echo "coverage is unavailable in '${PYTHON_BIN}'; install .ci/requirements-dev.txt in a throwaway environment." >&2
    exit 2
fi

cd "${REPO_ROOT}"
mkdir -p "${REPORT_DIR}"
export COVERAGE_FILE="${COVERAGE_FILE:-${REPORT_DIR}/.coverage}"

"${PYTHON_BIN}" --version
"${PYTHON_BIN}" -m coverage --version
"${PYTHON_BIN}" -m coverage erase

test_status=0
"${PYTHON_BIN}" -m coverage run --branch --source=pymoveit2 \
    -m pytest test --ignore=test/integration -q \
    --junitxml="${REPORT_DIR}/junit.xml" "$@" || test_status=$?

report_status=0
"${PYTHON_BIN}" -m coverage report -m \
    | tee "${REPORT_DIR}/coverage-report.txt" || report_status=$?

"${PYTHON_BIN}" -m coverage xml \
    -o "${REPORT_DIR}/coverage.xml"
"${PYTHON_BIN}" -m coverage json --pretty-print \
    -o "${REPORT_DIR}/coverage.json"

branch_status=0
if [[ -n "${COVERAGE_BRANCH_FAIL_UNDER:-}" ]]; then
    "${PYTHON_BIN}" - "${REPORT_DIR}/coverage.json" \
        "${COVERAGE_BRANCH_FAIL_UNDER}" \
        > "${REPORT_DIR}/coverage-branch.txt" 2>&1 <<'PY' || branch_status=$?
import json
import math
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: coverage.json branch-floor", file=sys.stderr)
        return 2

    try:
        floor = float(sys.argv[2])
    except ValueError:
        print("branch floor must be a number", file=sys.stderr)
        return 2
    if not math.isfinite(floor) or floor < 0.0 or floor > 100.0:
        print("branch floor must be finite and between 0 and 100", file=sys.stderr)
        return 2

    with open(sys.argv[1], encoding="utf-8") as report_file:
        totals = json.load(report_file)["totals"]
    covered = int(totals["covered_branches"])
    denominator = int(totals["num_branches"])
    if denominator <= 0 or covered < 0 or covered > denominator:
        print("coverage JSON has invalid branch totals", file=sys.stderr)
        return 2

    percent = 100.0 * covered / denominator
    print(
        f"Branch coverage: {covered}/{denominator} = {percent:.2f}% "
        f"(floor {floor:g}%)"
    )
    if percent < floor:
        print("Branch coverage floor failed.", file=sys.stderr)
        return 1
    return 0


raise SystemExit(main())
PY
else
    printf '%s\n' \
        "No branch coverage floor configured; this run records the measured baseline." \
        > "${REPORT_DIR}/coverage-branch.txt"
fi

cat "${REPORT_DIR}/coverage-branch.txt"

"${PYTHON_BIN}" -m coverage html \
    -d "${REPORT_DIR}/html"
"${PYTHON_BIN}" -m coverage debug config \
    > "${REPORT_DIR}/coverage-config.txt"

cat > "${REPORT_DIR}/coverage-scope.txt" <<EOF
Measured source: pymoveit2
Test command: python -m pytest test --ignore=test/integration -q
Branch measurement: enabled
Path omissions: none
Report exclusions: if TYPE_CHECKING:, pragma: no cover
Configured branch-only floor: ${COVERAGE_BRANCH_FAIL_UNDER:-unset (baseline measurement only)}
Branch gate report: coverage-branch.txt
Machine-readable report: coverage.json and coverage.xml
Test-result report: junit.xml
Human-readable report: coverage-report.txt and html/index.html
EOF

if (( test_status != 0 )); then
    exit "${test_status}"
fi
if (( report_status != 0 )); then
    exit "${report_status}"
fi
exit "${branch_status}"
