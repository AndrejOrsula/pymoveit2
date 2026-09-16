#!/usr/bin/env bash
# Usage: .docker/join.bash [ID] [CMD...]
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_NAME="$(basename "${PROJECT_DIR}")"

DOCKER_TTY="${DOCKER_TTY:-auto}"

usage() {
    cat <<EOF
Usage: ${0} [ID] [CMD...]

Runs CMD, or a shell if you give none, inside a container that .docker/run.bash started. ID picks one when several are running: 0 is '${PROJECT_NAME}' and N is '${PROJECT_NAME}-N'. Leave it out while only one container runs.
EOF
}

die() {
    local status="${1}"
    shift
    printf 'error: %s\n' "$*" >&2
    exit "${status}"
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
    usage >&1
    exit 0
fi

ID=""
if [[ "${#}" -gt 0 && "${1}" =~ ^[0-9]+$ ]]; then
    ID="${1}"
    shift
fi

CMD=("${@}")
if [[ "${#CMD[@]}" -eq 0 ]]; then
    CMD=(bash)
fi

TTY_OPTS=()
case "${DOCKER_TTY,,}" in
    auto)
        if [[ -t 0 && -t 1 ]]; then
            TTY_OPTS=(--tty)
        fi
        ;;
    true) TTY_OPTS=(--tty) ;;
    false) ;;
    *) die 2 "DOCKER_TTY accepts only auto, true or false (got '${DOCKER_TTY}')" ;;
esac

mapfile -t RUNNING < <(
    docker container list --format '{{.Names}}' 2>/dev/null |
    grep -E "^${PROJECT_NAME}(-[0-9]+)?$" | sort --version-sort || true
)

if [[ "${#RUNNING[@]}" -eq 0 ]]; then
    die 1 "no running '${PROJECT_NAME}' container; start one with .docker/run.bash"
fi

if [[ -n "${ID}" ]]; then
    CONTAINER_NAME="${PROJECT_NAME}"
    if [[ "${ID}" -gt 0 ]]; then
        CONTAINER_NAME="${PROJECT_NAME}-${ID}"
    fi
    FOUND=0
    for name in "${RUNNING[@]}"; do
        if [[ "${name}" == "${CONTAINER_NAME}" ]]; then
            FOUND=1
            break
        fi
    done
    if [[ "${FOUND}" -eq 0 ]]; then
        printf "error: container '%s' is not running\n" "${CONTAINER_NAME}" >&2
        printf 'running containers:\n' >&2
        printf '  %s\n' "${RUNNING[@]}" >&2
        exit 1
    fi
elif [[ "${#RUNNING[@]}" -eq 1 ]]; then
    CONTAINER_NAME="${RUNNING[0]}"
else
    printf 'error: several containers are running; pass the ID to select one\n' >&2
    printf '  %s\n' "${RUNNING[@]}" >&2
    usage >&2
    exit 2
fi

DOCKER_EXEC_CMD=(
    docker exec
    --interactive
    "${TTY_OPTS[@]}"
    "${CONTAINER_NAME}"
    "${CMD[@]}"
)

printf '\033[1;90m[TRACE] docker exec container=%q command_args=%d\033[0m\n' \
    "${CONTAINER_NAME}" "${#CMD[@]}"
exec "${DOCKER_EXEC_CMD[@]}"
