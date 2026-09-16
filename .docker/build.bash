#!/usr/bin/env bash
# Usage: .docker/build.bash [TAG] [BUILD_ARGS...]
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
IMAGE_NAME="andrejorsula/$(basename "${PROJECT_DIR}")"

usage() {
    cat <<EOF
Usage: ${0} [TAG] [BUILD_ARGS...]

The first argument that is not an option becomes the image tag. The rest go to docker build unchanged, for example:
  ${0} humble --build-arg ROS_DISTRO=humble --build-arg WITH_DEMO=false
EOF
}

if [[ "${1-}" == "-h" || "${1-}" == "--help" ]]; then
    usage >&1
    exit 0
fi

TAG="${IMAGE_NAME}"
BUILD_ARGS=()
if [[ "${#}" -gt 0 ]]; then
    if [[ "${1}" != "-"* ]]; then
        TAG="${IMAGE_NAME}:${1}"
        BUILD_ARGS=("${@:2}")
    else
        BUILD_ARGS=("${@}")
    fi
fi

SOURCE_ID="${PYMOVEIT2_SOURCE_ID:-}"
SOURCE_ID_SET=0
for ((index = 0; index < ${#BUILD_ARGS[@]}; index++)); do
    case "${BUILD_ARGS[${index}]}" in
        --build-arg=SOURCE_ID=*)
            SOURCE_ID_SET=1
            ;;
        --build-arg)
            if [[ "${index}" -lt $(( ${#BUILD_ARGS[@]} - 1 )) &&
                "${BUILD_ARGS[$((index + 1))]}" == SOURCE_ID=* ]]; then
                SOURCE_ID_SET=1
            fi
            ;;
    esac
done
if [[ "${SOURCE_ID_SET}" -eq 0 && -z "${SOURCE_ID}" ]]; then
    if HEAD="$(git -C "${PROJECT_DIR}" rev-parse --verify HEAD 2>/dev/null)"; then
        if [[ -n "$(git -C "${PROJECT_DIR}" status --porcelain --untracked-files=all -- .)" ]]; then
            SOURCE_ID="${HEAD}-dirty"
        else
            SOURCE_ID="${HEAD}"
        fi
    else
        SOURCE_ID="working-tree"
    fi
fi

SOURCE_ARG=()
if [[ "${SOURCE_ID_SET}" -eq 0 ]]; then
    SOURCE_ARG=(--build-arg "SOURCE_ID=${SOURCE_ID}")
fi

DOCKER_BUILD_CMD=(
    docker build
    --tag "${TAG}"
    "${SOURCE_ARG[@]}"
    "${BUILD_ARGS[@]}"
    "${PROJECT_DIR}"
)

printf '\033[1;90m[TRACE] docker build tag=%q context=%q build_args=%d\033[0m\n' \
    "${TAG}" "${PROJECT_DIR}" "${#BUILD_ARGS[@]}"
exec "${DOCKER_BUILD_CMD[@]}"
