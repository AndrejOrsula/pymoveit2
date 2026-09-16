#!/usr/bin/env bash
# Usage: test/scripts/run-integration.bash [humble|jazzy] [-- pytest arguments ...]

set -Eeuo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    echo "Usage: ${0} [humble|jazzy] [-- pytest arguments ...]" >&2
}

DISTRO="${ROS_DISTRO:-}"
if [[ "${1:-}" == "humble" || "${1:-}" == "jazzy" ]]; then
    DISTRO="$1"
    shift
fi
if [[ -z "${DISTRO}" ]]; then
    echo "ROS_DISTRO is unset; source the requested ROS 2 distribution first" >&2
    usage
    exit 2
fi
if [[ "${DISTRO}" != "humble" && "${DISTRO}" != "jazzy" ]]; then
    echo "Unsupported integration distribution '${DISTRO}' (expected humble or jazzy)" >&2
    exit 2
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is unavailable; source /opt/ros/${DISTRO}/setup.bash first" >&2
    exit 127
fi


export ROS_DOMAIN_ID="${PYMOVEIT2_INTEGRATION_DOMAIN_ID:-$((($$ % 200) + 1))}"
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

if [[ -n "${PYMOVEIT2_INTEGRATION_LOG_DIR:-}" ]]; then
    LOG_DIR="${PYMOVEIT2_INTEGRATION_LOG_DIR}"
else
    LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/pymoveit2-integration.XXXXXX")"
fi
mkdir -p "${LOG_DIR}"

DEMO_PID=""
SERVO_PID=""
SERVO_ONLY="${PYMOVEIT2_SERVO_ONLY:-0}"
REQUIRE_SERVO="${PYMOVEIT2_REQUIRE_SERVO:-0}"

if [[ "${REQUIRE_SERVO}" != "0" && "${REQUIRE_SERVO}" != "1" ]]; then
    echo "PYMOVEIT2_REQUIRE_SERVO must be 0 or 1" >&2
    exit 2
fi

start_owned() {
    local logfile="$1"
    shift
    if command -v setsid >/dev/null 2>&1; then
        setsid "$@" >"${logfile}" 2>&1 &
    else
        "$@" >"${logfile}" 2>&1 &
    fi
    echo "$!"
}

stop_owned() {
    local pid="$1"
    [[ -z "${pid}" ]] && return 0
    local process_group=0
    if kill -0 -- "-${pid}" 2>/dev/null; then
        process_group=1
        kill -TERM -- "-${pid}" 2>/dev/null || true
    elif kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
    fi
    for _ in $(seq 1 20); do
        if [[ "${process_group}" -eq 1 ]]; then
            kill -0 -- "-${pid}" 2>/dev/null || break
        else
            kill -0 "${pid}" 2>/dev/null || break
        fi
        sleep 0.1
    done
    if [[ "${process_group}" -eq 1 ]]; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
    else
        kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_owned "${SERVO_PID}"
    stop_owned "${DEMO_PID}"
    echo "Integration logs: ${LOG_DIR}" >&2
    exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

wait_for_servo() {
    local requested="${PYMOVEIT2_SERVO_INTERFACE:-}"
    local ready=0
    for _ in $(seq 1 120); do
        services="$(ros2 service list 2>/dev/null || true)"
        modern=false
        legacy=false
        if [[ "${services}" == *pause_servo* &&
              "${services}" == *switch_command_type* ]]; then
            modern=true
        fi
        if [[ "${services}" == *start_servo* &&
              "${services}" == *stop_servo* ]]; then
            legacy=true
        fi
        if [[ ( "${requested}" == "modern" && "${modern}" == "true" ) ||
              ( "${requested}" == "legacy" && "${legacy}" == "true" ) ||
              ( -z "${requested}" && ( "${modern}" == "true" || "${legacy}" == "true" ) ) ]] &&
           timeout 5 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
            ready=1
            break
        fi
        sleep 1
    done
    if [[ "${ready}" -ne 1 ]]; then
        echo "MoveIt Servo did not become ready" >&2
        ros2 node list >&2 || true
        ros2 service list >&2 || true
        tail -n 120 "${LOG_DIR}/servo.log" >&2 || true
        exit 1
    fi
}

if [[ "${SERVO_ONLY}" == "1" ]]; then
    if [[ -z "${PYMOVEIT2_SERVO_LAUNCH:-}" ]]; then
        echo "PYMOVEIT2_SERVO_LAUNCH is required with PYMOVEIT2_SERVO_ONLY=1" >&2
        exit 2
    fi
    SERVO_PID="$(start_owned "${LOG_DIR}/servo.log" bash -c "${PYMOVEIT2_SERVO_LAUNCH}")"
    wait_for_servo
else
    if [[ "${REQUIRE_SERVO}" == "1" && -z "${PYMOVEIT2_SERVO_LAUNCH:-}" ]]; then
        echo "PYMOVEIT2_SERVO_LAUNCH is required when PYMOVEIT2_REQUIRE_SERVO=1" >&2
        exit 2
    fi
    DEMO_PID="$(start_owned "${LOG_DIR}/panda-demo.log" \
        ros2 launch moveit_resources_panda_moveit_config demo.launch.py use_rviz:=false)"
fi

if [[ "${SERVO_ONLY}" != "1" ]]; then
    ready=0
    for _ in $(seq 1 120); do
        actions="$(ros2 action list 2>/dev/null || true)"
        services="$(ros2 service list 2>/dev/null || true)"
        controllers="$(ros2 control list_controllers 2>/dev/null || true)"
        if [[ "${actions}" == *move_action* &&
              "${services}" == *compute_fk* &&
              "${services}" == *get_planning_scene* &&
              "${controllers}" == *panda_arm_controller*active* ]] &&
           timeout 5 ros2 topic echo --once /joint_states >/dev/null 2>&1; then
            ready=1
            break
        fi
        sleep 1
    done

    if [[ "${ready}" -ne 1 ]]; then
        echo "Panda demo did not become ready" >&2
        echo "--- ros2 node list ---" >&2
        ros2 node list >&2 || true
        echo "--- ros2 action list ---" >&2
        ros2 action list >&2 || true
        echo "--- ros2 service list ---" >&2
        ros2 service list >&2 || true
        echo "--- ros2 control list_controllers ---" >&2
        ros2 control list_controllers >&2 || true
        echo "--- panda-demo.log (tail) ---" >&2
        tail -n 120 "${LOG_DIR}/panda-demo.log" >&2 || true
        exit 1
    fi

    if [[ -n "${PYMOVEIT2_SERVO_LAUNCH:-}" ]]; then
        SERVO_PID="$(start_owned "${LOG_DIR}/servo.log" bash -c "${PYMOVEIT2_SERVO_LAUNCH}")"
        wait_for_servo
    fi
fi

if [[ "${1:-}" == "--" ]]; then
    shift
fi
python3 -c 'import pymoveit2; print("PYMOVEIT2_IMPORT=" + pymoveit2.__file__)'
PYMOVEIT2_INTEGRATION=1 python3 -m pytest "${REPO_ROOT}/test/integration" -v -p no:cacheprovider "$@"
