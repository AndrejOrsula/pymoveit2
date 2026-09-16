#!/usr/bin/env bash
# Usage: .ci/scripts/check-debian-package.bash ROS_DISTRO ARTIFACT_DIR

set -Eeuo pipefail

fail() {
    echo "::error::$*" >&2
    exit 1
}

usage() {
    echo "Usage: $0 ROS_DISTRO ARTIFACT_DIR" >&2
    exit 2
}

if (( $# != 2 )); then
    usage
fi

ros_distro="$1"
artifact_dir="$2"
[[ "${ros_distro}" =~ ^[a-z0-9]+$ ]] || fail "invalid ROS distribution: ${ros_distro}"
[[ -d "${artifact_dir}" ]] || fail "package artifact directory does not exist: ${artifact_dir}"

ros_setup_file="${PYMOVEIT2_ROS_SETUP_FILE:-/opt/ros/${ros_distro}/setup.bash}"
[[ -r "${ros_setup_file}" ]] || fail "ROS setup file is missing: ${ros_setup_file}"
set +u
# shellcheck disable=SC1090
source "${ros_setup_file}"
set -u

expected_package="ros-${ros_distro}-pymoveit2"
expected_version_prefix="${EXPECTED_PACKAGE_VERSION_PREFIX:-5.0.1}"
expected_diagnostic="${PYMOVEIT2_EXPECTED_FK_DIAGNOSTIC:-FK example failed:}"
fk_timeout="${PYMOVEIT2_FK_TIMEOUT:-30s}"

expected_executables=(
    ex_allow_collisions.py
    ex_clear_planning_scene.py
    ex_collision_mesh.py
    ex_collision_primitive.py
    ex_doctor.py
    ex_fk.py
    ex_gripper.py
    ex_ik.py
    ex_joint_goal.py
    ex_orientation_path_constraint.py
    ex_pose_goal.py
    ex_servo.py
    ex_session.py
)

mapfile -d '' deb_files < <(
    find "${artifact_dir}" -maxdepth 1 -type f \
        -name 'ros-*-pymoveit2_*.deb' -print0
)
(( ${#deb_files[@]} == 1 )) || fail \
    "expected exactly one pymoveit2 Debian package in ${artifact_dir}"
deb_file="${deb_files[0]}"

deb_package="$(dpkg-deb --field "${deb_file}" Package 2>/dev/null)" || \
    fail "cannot read Debian package metadata: ${deb_file}"
deb_version="$(dpkg-deb --field "${deb_file}" Version 2>/dev/null)" || \
    fail "cannot read Debian package version: ${deb_file}"
[[ "${deb_package}" == "${expected_package}" ]] || fail \
    "package name ${deb_package} does not match ${expected_package}"
case "${deb_version}" in
    "${expected_version_prefix}"|"${expected_version_prefix}"-*) ;;
    *)
        fail "package version ${deb_version} does not match ${expected_version_prefix} or ${expected_version_prefix}-*"
        ;;
esac

testing_source_file=""
if [[ "${ROS_TESTING:-false}" == true ]]; then
    ros_keyring="/usr/share/keyrings/ros2-archive-keyring.gpg"
    [[ -r "${ros_keyring}" ]] || fail "ROS 2 archive keyring is missing: ${ros_keyring}"
    if ! grep -Rqs --include='*.list' --include='*.sources' \
        'packages.ros.org/ros2-testing/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d; then
        testing_source_file="$(mktemp /etc/apt/sources.list.d/pymoveit2-ros-testing.XXXXXX.list)"
        printf 'deb [signed-by=%s] http://packages.ros.org/ros2-testing/ubuntu %s main\n' \
            "${ros_keyring}" "${DEB_DISTRO:-unknown}" >"${testing_source_file}"
        cleanup_testing_source() {
            rm -f "${testing_source_file}"
        }
        trap cleanup_testing_source EXIT
    fi
fi

apt-get update -qq || fail "apt metadata update failed"
apt-get install -y -qq "${deb_file}" || fail "Debian package installation failed"
installed_status="$(
    dpkg-query -W -f='${Status}' "${expected_package}" 2>/dev/null || true
)"
[[ "${installed_status}" == "install ok installed" ]] || fail \
    "${expected_package} is not installed: ${installed_status:-no status}"

python3 -c 'import pymoveit2; print(pymoveit2.__file__)' || \
    fail "installed pymoveit2 import failed"

executables_output="$(ros2 pkg executables pymoveit2 2>/dev/null)" || \
    fail "ros2 could not enumerate pymoveit2 executables"
mapfile -t installed_executables < <(
    awk '$1 == "pymoveit2" && NF == 2 { print $2 }' <<<"${executables_output}"
)
(( ${#installed_executables[@]} == ${#expected_executables[@]} )) || fail \
    "expected ${#expected_executables[@]} executables, found ${#installed_executables[@]}"
if ! diff -u \
    <(printf '%s\n' "${expected_executables[@]}" | sort) \
    <(printf '%s\n' "${installed_executables[@]}" | sort); then
    fail "installed executable names do not match the package contract"
fi

install_prefix="$(ros2 pkg prefix pymoveit2 2>/dev/null)" || \
    fail "ros2 could not resolve the pymoveit2 install prefix"
[[ -n "${install_prefix}" ]] || fail "pymoveit2 install prefix is empty"
asset_path="${install_prefix}/lib/pymoveit2/assets/suzanne.stl"
[[ -f "${asset_path}" ]] || fail "installed mesh asset is missing: ${asset_path}"
for executable in "${installed_executables[@]}"; do
    executable_path="${install_prefix}/lib/pymoveit2/${executable}"
    [[ -f "${executable_path}" ]] || fail "installed executable is missing: ${executable_path}"
    python3 -m py_compile "${executable_path}" || fail \
        "installed executable does not compile: ${executable}"
    python3 - "${executable_path}" <<'PY' || fail \
        "installed executable dependency imports failed: ${executable}"
import runpy
import sys

runpy.run_path(sys.argv[1], run_name="pymoveit2_installed_import_probe")
PY
done

command -v timeout >/dev/null 2>&1 || fail "timeout command is unavailable"
negative_log="$(mktemp)"
negative_status=0
if timeout --kill-after=5s "${fk_timeout}" ros2 run pymoveit2 ex_fk.py \
    >"${negative_log}" 2>&1; then
    negative_status=0
else
    negative_status=$?
fi
cat "${negative_log}"
if (( negative_status != 1 )); then
    rm -f "${negative_log}"
    fail "the no-server FK run must exit exactly 1; got ${negative_status}"
fi
if ! grep -Fq -- "${expected_diagnostic}" "${negative_log}"; then
    rm -f "${negative_log}"
    fail "the no-server FK run lacked the expected message: ${expected_diagnostic}"
fi
if grep -q '^Traceback (most recent call last):' "${negative_log}"; then
    rm -f "${negative_log}"
    fail "the no-server FK run printed a traceback instead of a diagnostic"
fi
rm -f "${negative_log}"

provenance_file="${PYMOVEIT2_PROVENANCE_FILE:-${artifact_dir}/provenance.txt}"
mkdir -p "$(dirname "${provenance_file}")"
provenance_tmp="${provenance_file}.tmp"

record_first_line() {
    local name="$1"
    shift
    local output
    if output="$("$@" 2>&1)"; then
        printf '%s=%s\n' "${name}" "${output%%$'\n'*}"
    else
        printf '%s=unavailable\n' "${name}"
    fi
}

{
    printf 'artifact_smoke=passed\n'
    printf 'source_revision=%s\n' "${SOURCE_REVISION:-unknown}"
    printf 'ros_distro=%s\n' "${ros_distro}"
    printf 'deb_distro=%s\n' "${DEB_DISTRO:-unknown}"
    printf 'ros_testing=%s\n' "${ROS_TESTING:-false}"
    printf 'ros_apt_source_sha=%s\n' "${ROS_APT_SOURCE_SHA:-unknown}"
    if [[ "${ROS_TESTING:-false}" == true ]]; then
        printf 'ros_repository=http://packages.ros.org/ros2-testing/ubuntu %s main\n' "${DEB_DISTRO:-unknown}"
    else
        printf 'ros_repository=http://packages.ros.org/ros2/ubuntu %s main\n' "${DEB_DISTRO:-unknown}"
    fi
    printf 'base_image=%s\n' "${BASE_IMAGE:-unknown}"
    printf 'base_image_digest=%s\n' "${BASE_IMAGE_DIGEST:-unknown}"
    printf 'builder_action_sha=%s\n' "${BUILDER_ACTION_SHA:-unknown}"
    printf 'package_file=%s\n' "$(basename "${deb_file}")"
    printf 'package_sha256=%s\n' "$(sha256sum "${deb_file}" | awk '{print $1}')"
    printf 'package=%s\n' "${deb_package}"
    printf 'package_version=%s\n' "${deb_version}"
    printf 'expected_version_prefix=%s\n' "${expected_version_prefix}"
    record_first_line python_version python3 --version
    record_first_line ros2_version ros2 --version
    record_first_line rosdep_version rosdep --version
    record_first_line apt_version apt-get --version
    if optional_trimesh="$(python3 -c \
        'import importlib.metadata as m; print(m.version("trimesh"))' 2>/dev/null)"; then
        printf 'optional_trimesh=%s\n' "${optional_trimesh%%$'\n'*}"
    else
        printf 'optional_trimesh=unavailable\n'
    fi
    if [[ "${ros_distro}" == humble ]]; then
        printf 'optional_trimesh_bound=trimesh<4\n'
    else
        printf 'optional_trimesh_bound=trimesh\n'
    fi
    printf 'apt_resolved_versions<<EOF\n'
    if command -v apt-cache >/dev/null 2>&1; then
        apt-cache policy "${expected_package}" \
            "ros-${ros_distro}-ros-base" 2>&1 || true
    else
        printf 'apt-cache=unavailable\n'
    fi
    printf 'EOF\n'
    printf 'installed_packages<<EOF\n'
    if command -v dpkg-query >/dev/null 2>&1; then
        dpkg-query -W -f='${binary:Package}=${Version}\n' 2>&1 || true
    else
        printf 'dpkg-query=unavailable\n'
    fi
    printf 'EOF\n'
    printf 'dpkg_status=%s\n' "${installed_status}"
} >"${provenance_tmp}"
mv "${provenance_tmp}" "${provenance_file}"

echo "Debian package checks passed: ${deb_file}"
