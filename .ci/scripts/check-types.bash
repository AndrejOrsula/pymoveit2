#!/usr/bin/env bash
# Usage: [PYTHON_BIN=python3] .ci/scripts/check-types.bash [mypy arguments ...]
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "Python interpreter '${PYTHON_BIN}' is unavailable." >&2
    exit 2
fi
if ! "${PYTHON_BIN}" -c "import mypy" >/dev/null 2>&1; then
    echo "mypy is unavailable in '${PYTHON_BIN}'; install .ci/requirements-dev.txt in a throwaway environment." >&2
    exit 2
fi

cd "${REPO_ROOT}"
"${PYTHON_BIN}" --version
"${PYTHON_BIN}" -m mypy --version
exec "${PYTHON_BIN}" -m mypy --config-file "${REPO_ROOT}/pyproject.toml" "$@"
