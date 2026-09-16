#!/usr/bin/env bash
# Usage: .docker/run.bash [OPTIONS] [TAG] [CMD...]
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
PROJECT_NAME="$(basename "${PROJECT_DIR}")"
IMAGE_NAME="andrejorsula/${PROJECT_NAME}"

WS_SRC_DIR="${PYMOVEIT2_WS_SRC_DIR:-/root/ws/src}"
DEV_MOUNT="${WS_SRC_DIR}/${PROJECT_NAME}"
WITH_DEV_VOLUME="${WITH_DEV_VOLUME:-true}"
WITH_DEV_WRAPPER="${WITH_DEV_WRAPPER:-true}"
DOCKER_TTY="${DOCKER_TTY:-auto}"

usage() {
    cat <<EOF
Usage: ${0} [OPTIONS] [TAG] [CMD...]

Options:
  -v, --volume VALUE    Pass a volume through to docker run.
  -e, --env VALUE       Pass an environment variable through to docker run.
      --dev-volume      Mount the checkout over the image sources (default).
      --no-dev-volume   Use the sources built into the image instead.
  -h, --help            Show this help.

Access to the host, all off by default:
      --gui             Share DISPLAY and the X11 socket, read-only.
      --network-host    Use the host network, so ROS sees host traffic.
      --ipc-host        Use the host IPC namespace, for shared-memory transport.
      --network NAME    Use a named Docker network instead.

Put -- before a command that starts with '-'.
EOF
}

die() {
    local status="${1}"
    shift
    printf 'error: %s\n' "$*" >&2
    exit "${status}"
}

require_value() {
    local option="${1}"
    if [[ "${#}" -lt 2 || -z "${2}" ]]; then
        usage >&2
        die 2 "${option} requires a value"
    fi
}

require_boolean() {
    local name="${1}" value="${2}"
    case "${value,,}" in
        true|false) ;;
        *) die 2 "${name} accepts only true or false (got '${value}')" ;;
    esac
}

CUSTOM_VOLUMES=()
CUSTOM_ENVS=()
GUI=0
HOST_NETWORK=0
HOST_IPC=0
NETWORK_MODE="bridge"
NETWORK_OPTION_SET=0

select_network() {
    local requested="${1}"
    [[ -n "${requested}" ]] || die 2 "--network requires a value"
    if [[ "${NETWORK_OPTION_SET}" -eq 1 && "${NETWORK_MODE}" != "${requested}" ]]; then
        die 2 "conflicting network options: ${NETWORK_MODE} and ${requested}"
    fi
    NETWORK_MODE="${requested}"
    NETWORK_OPTION_SET=1
    if [[ "${requested}" == "host" ]]; then
        HOST_NETWORK=1
    fi
}

while [[ "${#}" -gt 0 ]]; do
    case "${1}" in
        -v|--volume)
            require_value "${1}" "${2-}"
            CUSTOM_VOLUMES+=(--volume "${2}")
            shift 2
            ;;
        -v?*)
            CUSTOM_VOLUMES+=(--volume "${1#-v}")
            shift
            ;;
        --volume=*)
            CUSTOM_VOLUMES+=(--volume "${1#--volume=}")
            shift
            ;;
        -e|--env)
            require_value "${1}" "${2-}"
            CUSTOM_ENVS+=(--env "${2}")
            shift 2
            ;;
        -e?*)
            CUSTOM_ENVS+=(--env "${1#-e}")
            shift
            ;;
        --env=*)
            CUSTOM_ENVS+=(--env "${1#--env=}")
            shift
            ;;
        --dev-volume)
            WITH_DEV_VOLUME=true
            shift
            ;;
        --no-dev-volume)
            WITH_DEV_VOLUME=false
            shift
            ;;
        --gui)
            GUI=1
            shift
            ;;
        --network-host|--host-network)
            select_network host
            shift
            ;;
        --ipc-host|--host-ipc)
            HOST_IPC=1
            shift
            ;;
        --network)
            require_value "${1}" "${2-}"
            select_network "${2}"
            shift 2
            ;;
        --network=*)
            select_network "${1#--network=}"
            shift
            ;;
        --ipc)
            require_value "${1}" "${2-}"
            case "${2}" in
                host)
                    HOST_IPC=1
                    ;;
                private)
                    HOST_IPC=0
                    ;;
                *)
                    die 2 "--ipc accepts only host or private"
                    ;;
            esac
            shift 2
            ;;
        --ipc=*)
            case "${1#--ipc=}" in
                host)
                    HOST_IPC=1
                    ;;
                private)
                    HOST_IPC=0
                    ;;
                *)
                    die 2 "--ipc accepts only host or private"
                    ;;
            esac
            shift
            ;;
        -h|--help)
            usage >&1
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            usage >&2
            die 2 "unknown option: ${1}"
            ;;
        *)
            break
            ;;
    esac
done

require_boolean WITH_DEV_VOLUME "${WITH_DEV_VOLUME}"
require_boolean WITH_DEV_WRAPPER "${WITH_DEV_WRAPPER}"

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

XAUTH_TMP=""
cleanup() {
    local status="${?}"
    if [[ -n "${XAUTH_TMP}" ]]; then
        rm -f -- "${XAUTH_TMP}" || true
        XAUTH_TMP=""
    fi
    return "${status}"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 131' QUIT
trap 'exit 143' TERM

GUI_OPTS=()
if [[ "${GUI}" -eq 1 ]]; then
    X11_SOCKET_DIR="${PYMOVEIT2_X11_SOCKET_DIR:-/tmp/.X11-unix}"
    [[ -n "${DISPLAY:-}" ]] || die 2 "--gui requires DISPLAY"
    [[ -d "${X11_SOCKET_DIR}" && -r "${X11_SOCKET_DIR}" ]] || \
        die 2 "--gui requires a readable X11 socket directory: ${X11_SOCKET_DIR}"

    if [[ -n "${XAUTHORITY:-}" ]]; then
        XAUTH_SOURCE="${XAUTHORITY}"
    elif [[ -n "${HOME:-}" ]]; then
        XAUTH_SOURCE="${HOME}/.Xauthority"
    else
        die 2 "--gui requires XAUTHORITY or HOME"
    fi
    [[ -r "${XAUTH_SOURCE}" ]] || die 2 "--gui requires readable Xauthority: ${XAUTH_SOURCE}"

    XAUTH_TMP="$(mktemp "${TMPDIR:-/tmp}/pymoveit2-xauth.XXXXXX")" || \
        die 2 "could not create a temporary Xauthority file"
    chmod 600 "${XAUTH_TMP}"
    if ! xauth -f "${XAUTH_SOURCE}" nlist "${DISPLAY}" | \
        sed -E 's/^0100[[:space:]]+/ffff /' | \
        xauth -f "${XAUTH_TMP}" nmerge - >/dev/null 2>&1; then
        die 2 "could not copy Xauthority for DISPLAY ${DISPLAY}"
    fi
    [[ -s "${XAUTH_TMP}" ]] || die 2 "no Xauthority entry for DISPLAY ${DISPLAY}"

    GUI_OPTS=(
        --env "DISPLAY=${DISPLAY}"
        --env XAUTHORITY=/tmp/pymoveit2.xauth
        --volume "${X11_SOCKET_DIR}:/tmp/.X11-unix:ro"
        --volume "${XAUTH_TMP}:/tmp/pymoveit2.xauth:ro"
    )
fi

CMD=()
TAG="${IMAGE_NAME}"
if [[ "${#}" -gt 0 ]]; then
    if docker image inspect "${IMAGE_NAME}:${1}" &>/dev/null; then
        TAG="${IMAGE_NAME}:${1}"
        CMD=("${@:2}")
    else
        CMD=("${@}")
    fi
fi

if ! docker image inspect "${TAG}" &>/dev/null; then
    printf 'note: %s is not present locally; Docker will try to pull it. Build it with %s/build.bash\n' \
        "${TAG}" "${SCRIPT_DIR}" >&2
fi

CMD_ARG_COUNT="${#CMD[@]}"

NETWORK_OPTS=(--network "${NETWORK_MODE}")
IPC_OPTS=(--ipc private)
if [[ "${HOST_NETWORK}" -eq 1 ]]; then
    NETWORK_OPTS=(--network host)
fi
if [[ "${HOST_IPC}" -eq 1 ]]; then
    IPC_OPTS=(--ipc host)
fi

ROS_ENVS=()
if [[ "${HOST_NETWORK}" -eq 0 && "${NETWORK_MODE}" == "bridge" ]]; then
    ROS_ENVS=(
        --env ROS_LOCALHOST_ONLY=1
        --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    )
elif [[ "${HOST_NETWORK}" -eq 0 && "${NETWORK_MODE}" != "none" ]]; then
    ROS_ENVS=(
        --env ROS_LOCALHOST_ONLY=0
        --env ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    )
fi

read -r -d '' DEV_WRAPPER <<'WRAPPER' || true
if [ -n "${PYMOVEIT2_DEV_MOUNT:-}" ]; then
    export PYTHONPATH="${PYMOVEIT2_DEV_MOUNT}${PYTHONPATH:+:${PYTHONPATH}}"
fi
pymoveit2_restore_ownership() {
    [ -n "${PYMOVEIT2_HOST_UID:-}" ] || return 0
    [ -n "${PYMOVEIT2_HOST_GID:-}" ] || return 0
    [ -n "${PYMOVEIT2_DEV_MOUNT:-}" ] && [ -e "${PYMOVEIT2_DEV_MOUNT}" ] || return 0
    chown -R --from=0:0 \
        "${PYMOVEIT2_HOST_UID}:${PYMOVEIT2_HOST_GID}" \
        "${PYMOVEIT2_DEV_MOUNT}" 2>/dev/null || true
}
trap 'pymoveit2_restore_ownership' EXIT
if [ "${#}" -eq 0 ]; then
    bash
    pymoveit2_status="${?}"
else
    "${@}" &
    pymoveit2_child="${!}"
    trap ':' INT
    trap 'kill -TERM "${pymoveit2_child}" 2>/dev/null || true' TERM
    while :; do
        wait "${pymoveit2_child}"
        pymoveit2_status="${?}"
        kill -0 "${pymoveit2_child}" 2>/dev/null || break
    done
fi
pymoveit2_restore_ownership
trap - EXIT
exit "${pymoveit2_status}"
WRAPPER

DEV_OPTS=()
if [[ "${WITH_DEV_VOLUME,,}" == "true" ]]; then
    DEV_OPTS=(--volume "${PROJECT_DIR}:${DEV_MOUNT}")
    if [[ "${WITH_DEV_WRAPPER,,}" == "true" ]]; then
        DEV_OPTS+=(
            --env "PYMOVEIT2_DEV_MOUNT=${DEV_MOUNT}"
            --env "PYMOVEIT2_HOST_UID=$(id -u)"
            --env "PYMOVEIT2_HOST_GID=$(id -g)"
        )
        CMD=(bash -c "${DEV_WRAPPER}" "${PROJECT_NAME}-dev" "${CMD[@]}")
    fi
fi

CONTAINER_NAME="${PROJECT_NAME}"
EXISTING_NAMES="$(docker container list --all --format '{{.Names}}' 2>/dev/null || true)"
if printf '%s\n' "${EXISTING_NAMES}" | grep -Fqx -- "${CONTAINER_NAME}"; then
    NAME_INDEX=1
    while printf '%s\n' "${EXISTING_NAMES}" | \
        grep -Fqx -- "${PROJECT_NAME}-${NAME_INDEX}"; do
        NAME_INDEX=$((NAME_INDEX + 1))
    done
    CONTAINER_NAME="${PROJECT_NAME}-${NAME_INDEX}"
fi

DOCKER_RUN_CMD=(
    docker run
    --rm
    --interactive
    "${TTY_OPTS[@]}"
    --name "${CONTAINER_NAME}"
    "${DEV_OPTS[@]}"
    "${NETWORK_OPTS[@]}"
    "${IPC_OPTS[@]}"
    "${ROS_ENVS[@]}"
    "${GUI_OPTS[@]}"
    "${CUSTOM_VOLUMES[@]}"
    "${CUSTOM_ENVS[@]}"
    "${TAG}"
    "${CMD[@]}"
)

printf '\033[1;90m[TRACE] docker run image=%q container=%q network=%q ipc=%q dev_volume=%q volumes=%d env=%d command_args=%d\033[0m\n' \
    "${TAG}" "${CONTAINER_NAME}" \
    "${NETWORK_OPTS[-1]}" "${IPC_OPTS[-1]}" "${WITH_DEV_VOLUME,,}" \
    "$(( ${#CUSTOM_VOLUMES[@]} / 2 ))" "$(( ${#CUSTOM_ENVS[@]} / 2 ))" \
    "${CMD_ARG_COUNT}"

if "${DOCKER_RUN_CMD[@]}"; then
    DOCKER_STATUS=0
else
    DOCKER_STATUS="${?}"
fi
exit "${DOCKER_STATUS}"
