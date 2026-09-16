#!/usr/bin/env bash
# Usage: test/scripts/run-tests.bash [distro]    (default: lyrical; one of humble|jazzy|lyrical|rolling)

set -u -o pipefail

DISTRO="${1:-lyrical}"
case "${DISTRO}" in
    humble | jazzy | lyrical | rolling) ;;
    *)
        printf "Unsupported ROS 2 distribution '%s' (expected humble|jazzy|lyrical|rolling)\n" "${DISTRO}" >&2
        exit 2
        ;;
esac

REPO_ROOT="$(git rev-parse --show-toplevel)"
STAGE_TIMEOUT_SEC="${PYMOVEIT2_STAGE_TIMEOUT_SEC:-300}"
PULL_TIMEOUT_SEC="${PYMOVEIT2_PULL_TIMEOUT_SEC:-300}"
DOCKER_TIMEOUT_SEC="${PYMOVEIT2_DOCKER_TIMEOUT_SEC:-$((STAGE_TIMEOUT_SEC * 6 + 30))}"
CLEANUP_TIMEOUT_SEC="${PYMOVEIT2_CLEANUP_TIMEOUT_SEC:-15}"

validate_timeout() {
    local name="$1"
    local value="$2"
    case "${value}" in
        '' | *[!0-9]*)
            printf "%s must be a positive integer, got '%s'\n" "${name}" "${value}" >&2
            exit 2
            ;;
    esac
    if [ "${value}" -lt 1 ]; then
        printf "%s must be a positive integer, got '%s'\n" "${name}" "${value}" >&2
        exit 2
    fi
}

validate_timeout PYMOVEIT2_STAGE_TIMEOUT_SEC "${STAGE_TIMEOUT_SEC}"
validate_timeout PYMOVEIT2_PULL_TIMEOUT_SEC "${PULL_TIMEOUT_SEC}"
validate_timeout PYMOVEIT2_DOCKER_TIMEOUT_SEC "${DOCKER_TIMEOUT_SEC}"
validate_timeout PYMOVEIT2_CLEANUP_TIMEOUT_SEC "${CLEANUP_TIMEOUT_SEC}"

IMAGE="${PYMOVEIT2_IMAGE:-ros:${DISTRO}-ros-base}"
NETWORK_NAME="pymoveit2-test-${DISTRO}-$$-${RANDOM}"
CONTAINER_NAME="pymoveit2-test-${DISTRO}-$$-${RANDOM}"
NETWORK_ID=""
CONTAINER_ID=""
LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/pymoveit2-test.XXXXXX")"

log() {
    printf '[run-tests] %s\n' "$*" >&2
}

cleanup() {
    local status="$?"
    trap - EXIT INT TERM

    if [ -n "${CONTAINER_ID}" ]; then
        log "cleanup container ${CONTAINER_ID}"
        if ! timeout --foreground "${CLEANUP_TIMEOUT_SEC}s" docker rm -f "${CONTAINER_ID}"; then
            log "cleanup container failed for owned id ${CONTAINER_ID}"
        fi
    fi
    if [ -n "${NETWORK_ID}" ]; then
        log "cleanup network ${NETWORK_ID}"
        if ! timeout --foreground "${CLEANUP_TIMEOUT_SEC}s" docker network rm "${NETWORK_ID}"; then
            log "cleanup network failed for owned id ${NETWORK_ID}"
        fi
    fi
    rm -rf "${LOG_DIR}"
    exit "${status}"
}

on_signal() {
    local status="$1"
    log "received signal ${status}; stopping owned resources"
    exit "${status}"
}

trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

run_host_stage() {
    local stage="$1"
    local timeout_sec="$2"
    shift
    shift
    local stage_log="${LOG_DIR}/${stage}.log"
    local status

    log "[stage:${stage}] START: $*"
    set +e
    timeout --foreground --kill-after=5s "${timeout_sec}s" "$@" 2>&1 | tee "${stage_log}"
    status="${PIPESTATUS[0]}"
    set -u
    if [ "${status}" -ne 0 ]; then
        log "[stage:${stage}] FAIL exit=${status}"
        log "[stage:${stage}] log=${stage_log}"
        return "${status}"
    fi
    log "[stage:${stage}] PASS"
    return 0
}

create_owned_network() {
    local network_log="${LOG_DIR}/network-create.log"
    local status

    log "[stage:network] START: docker network create ${NETWORK_NAME}"
    set +e
    timeout --foreground --kill-after=5s "${DOCKER_TIMEOUT_SEC}s" \
        docker network create --driver bridge --attachable \
        --label com.pymoveit2.runner=owned "${NETWORK_NAME}" >"${network_log}" 2>&1
    status="$?"
    set -u
    cat "${network_log}"
    if [ "${status}" -ne 0 ]; then
        log "[stage:network] FAIL exit=${status}"
        log "[stage:network] log=${network_log}"
        return "${status}"
    fi
    NETWORK_ID="$(tail -n 1 "${network_log}" | tr -d '\r')"
    if [ -z "${NETWORK_ID}" ]; then
        log "[stage:network] FAIL: docker returned no owned network id"
        return 1
    fi
    log "[stage:network] PASS id=${NETWORK_ID}"
    return 0
}


DNS_OPTS=()
for dns_server in ${PYMOVEIT2_DOCKER_DNS:-}; do
    DNS_OPTS+=(--dns "${dns_server//,/}")
done

create_owned_container() {
    local container_log="${LOG_DIR}/container-create.log"
    local status
    local inner_script

    inner_script="$(cat <<'INNER_SCRIPT'
set -o pipefail

STAGE_TIMEOUT_SEC="${PYMOVEIT2_STAGE_TIMEOUT_SEC}"
STAGE_LOG_DIR="/tmp/pymoveit2-stage-logs"
mkdir -p "${STAGE_LOG_DIR}"

run_stage() {
    local stage="$1"
    shift
    local stage_log="${STAGE_LOG_DIR}/${stage}.log"
    local status

    printf '[stage:%s] START: %s\n' "${stage}" "$*"
    set +e
    timeout --foreground --kill-after=5s "${STAGE_TIMEOUT_SEC}s" "$@" 2>&1 | tee "${stage_log}"
    status="${PIPESTATUS[0]}"
    set -o pipefail
    if [ "${status}" -ne 0 ]; then
        printf '[stage:%s] FAIL exit=%s\n' "${stage}" "${status}"
        printf '[stage:%s] log=%s\n' "${stage}" "${stage_log}"
        tail -n 80 "${stage_log}" || true
        return "${status}"
    fi
    printf '[stage:%s] PASS\n' "${stage}"
    return 0
}

run_stage checkout bash -c '
    set -e -o pipefail
    mkdir -p /ws/src/pymoveit2
    cp -r /repo/. /ws/src/pymoveit2/
    test -f /ws/src/pymoveit2/package.xml
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_stage dns bash -c '
    set -e -o pipefail
    . /etc/os-release
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"
    printf "ROS_DISTRO=%s\\nOS_ID=%s\\nOS_CODENAME=%s\\n" \
        "${PYMOVEIT2_DISTRO}" "${ID:-unknown}" "${codename}"
    test "${codename}" != unknown
    command -v getent
    getent hosts packages.ros.org
    getent hosts raw.githubusercontent.com
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_stage apt bash -c '
    set -e -o pipefail
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    . /etc/os-release
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-unknown}}"
    package="ros-${PYMOVEIT2_DISTRO}-moveit-msgs"
    candidate="$(apt-cache policy "${package}" | awk "/Candidate:/ { print \$2; exit }")"
    printf "ROS_DISTRO=%s OS_CODENAME=%s PACKAGE=%s CANDIDATE=%s\\n" \
        "${PYMOVEIT2_DISTRO}" "${codename}" "${package}" "${candidate:-none}"
    if [ -z "${candidate}" ] || [ "${candidate}" = "(none)" ]; then
        printf "missing ROS apt metadata for %s\\n" "${package}" >&2
        apt-cache policy "${package}" || true
        exit 1
    fi
    apt-get install -y --no-install-recommends python3-pytest
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_stage rosdep bash -c '
    set -e -o pipefail
    source "/opt/ros/${PYMOVEIT2_DISTRO}/setup.bash"
    rosdep update --rosdistro "${PYMOVEIT2_DISTRO}"
    rosdep install -y --from-paths /ws/src --ignore-src --rosdistro "${PYMOVEIT2_DISTRO}"
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_stage build bash -c '
    set -e -o pipefail
    source "/opt/ros/${PYMOVEIT2_DISTRO}/setup.bash"
    cd /ws
    colcon build --merge-install --symlink-install --event-handlers console_direct+
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_stage test bash -c '
    set -e -o pipefail
    source "/opt/ros/${PYMOVEIT2_DISTRO}/setup.bash"
    source /ws/install/local_setup.bash
    cd /ws
    python3 -c "import pymoveit2; print(\"SOURCE=\" + pymoveit2.__file__)"
    if colcon test --merge-install --event-handlers console_direct+ --return-code-on-test-failure; then
        test_status=0
    else
        test_status="$?"
    fi
    if colcon test-result --verbose; then
        result_status=0
    else
        result_status="$?"
    fi
    if [ "${test_status}" -ne 0 ]; then
        exit "${test_status}"
    fi
    exit "${result_status}"
'
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"
INNER_SCRIPT
)"

    log "[stage:container] START: docker create ${CONTAINER_NAME}"
    set +e
    timeout --foreground --kill-after=5s "${DOCKER_TIMEOUT_SEC}s" \
        docker create --name "${CONTAINER_NAME}" \
        --network "${NETWORK_ID}" \
        "${DNS_OPTS[@]}" \
        --tmpfs /tmp:rw,exec,nosuid,nodev --tmpfs /run:rw,exec,nosuid,nodev \
        -e "ROS_LOCALHOST_ONLY=1" \
        -e "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" \
        -e "PYMOVEIT2_DISTRO=${DISTRO}" \
        -e "PYMOVEIT2_STAGE_TIMEOUT_SEC=${STAGE_TIMEOUT_SEC}" \
        -v "${REPO_ROOT}:/repo:ro" \
        -w /repo --entrypoint bash "${IMAGE}" -c "${inner_script}" >"${container_log}" 2>&1
    status="$?"
    set -u
    cat "${container_log}"
    if [ "${status}" -ne 0 ]; then
        log "[stage:container] FAIL exit=${status}"
        log "[stage:container] log=${container_log}"
        return "${status}"
    fi
    CONTAINER_ID="$(tail -n 1 "${container_log}" | tr -d '\r')"
    if [ -z "${CONTAINER_ID}" ]; then
        log "[stage:container] FAIL: docker returned no owned container id"
        return 1
    fi
    log "[stage:container] PASS id=${CONTAINER_ID}"
    return 0
}

if timeout --foreground "${PULL_TIMEOUT_SEC}s" docker image inspect "${IMAGE}" >/dev/null 2>"${LOG_DIR}/image-inspect.log"; then
    log "[stage:image] PASS cached ${IMAGE}"
else
    inspect_status="$?"
    log "[stage:image] image inspect exit=${inspect_status}; pulling ${IMAGE}"
    run_host_stage image-pull "${PULL_TIMEOUT_SEC}" docker pull "${IMAGE}"
    status="$?"
    [ "${status}" -eq 0 ] || exit "${status}"
fi

create_owned_network
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

create_owned_container
status="$?"
[ "${status}" -eq 0 ] || exit "${status}"

run_host_stage container "${DOCKER_TIMEOUT_SEC}" docker start -a "${CONTAINER_ID}"
status="$?"
exit "${status}"
