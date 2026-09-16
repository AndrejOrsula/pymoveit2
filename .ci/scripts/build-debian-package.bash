#!/usr/bin/env bash
# Usage: .ci/scripts/build-debian-package.bash ROS_DISTRO DEB_DISTRO DEB_ARCH [PACKAGE_DIR] [ARTIFACT_DIR]

set -Eeuo pipefail

usage() {
    echo "Usage: $0 ROS_DISTRO DEB_DISTRO DEB_ARCH [PACKAGE_DIR] [ARTIFACT_DIR]" >&2
    exit 2
}

(( $# >= 3 && $# <= 5 )) || usage

ros_distro="${1}"
deb_distro="${2}"
deb_arch="${3}"
package_dir="${4:-repo}"
artifact_dir="${5:-apt_repo}"

[[ "${ros_distro}" =~ ^[a-z0-9]+$ ]] || {
    echo "invalid ROS distribution: ${ros_distro}" >&2
    exit 1
}
[[ "${deb_distro}" =~ ^[a-z0-9]+$ ]] || {
    echo "invalid Debian distribution: ${deb_distro}" >&2
    exit 1
}
[[ "${deb_arch}" =~ ^[a-z0-9_]+$ ]] || {
    echo "invalid Debian architecture: ${deb_arch}" >&2
    exit 1
}
package_dir="$(cd -- "${package_dir}" 2>/dev/null && pwd -P)" || {
    echo "package directory is missing: ${package_dir}" >&2
    exit 1
}
artifact_dir="$(mkdir -p -- "${artifact_dir}" && cd -- "${artifact_dir}" && pwd -P)"
if [[ "${artifact_dir}" == "/" || "${artifact_dir}" == "${package_dir}" ||
      "${artifact_dir}" == "${package_dir}/"* ]]; then
    echo "artifact directory must be outside the package directory and filesystem root" >&2
    exit 1
fi
[[ -f "${package_dir}/package.xml" ]] || {
    echo "package manifest is missing: ${package_dir}/package.xml" >&2
    exit 1
}
[[ ! -e "${package_dir}/debian" && ! -L "${package_dir}/debian" ]] || {
    echo "refusing to overwrite existing Debian metadata: ${package_dir}/debian" >&2
    exit 1
}

readarray -t manifest_values < <(
    python3 - "${package_dir}/package.xml" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
for name in ("name", "version"):
    value = root.findtext(name)
    if not value:
        raise SystemExit(f"package.xml has no {name}")
    print(value.strip())
PY
)
(( ${#manifest_values[@]} == 2 )) || {
    echo "package.xml manifest values are incomplete" >&2
    exit 1
}
package_name="${manifest_values[0]}"
package_version="${manifest_values[1]}"
[[ "${package_name}" == pymoveit2 ]] || {
    echo "unexpected package name: ${package_name}" >&2
    exit 1
}
[[ "${package_version}" =~ ^[0-9]+(\.[0-9]+){2}$ ]] || {
    echo "package.xml release version is not a stable semver: ${package_version}" >&2
    exit 1
}

if ubuntu-distro-info --all | grep -Fxq -- "${deb_distro}"; then
    distribution=ubuntu
elif debian-distro-info --all | grep -Fxq -- "${deb_distro}"; then
    echo "Debian targets are not supported by the ROS 2 Ubuntu package repository: ${deb_distro}" >&2
    exit 1
else
    echo "unknown Debian distribution: ${deb_distro}" >&2
    exit 1
fi

find "${artifact_dir}" -maxdepth 1 -type f \
    \( -name 'ros-*-pymoveit2_*.deb' -o -name 'ros-*-pymoveit2_*.dsc' \
    -o -name 'ros-*-pymoveit2_*.changes' -o -name 'ros-*-pymoveit2_*.buildinfo' \) \
    -delete

extra_repository_args=()
if [[ "${ros_distro}" != debian ]]; then
    ros_apt_source_commit=38f7b0941bd184cc7bb4c77f3a5a5c15bbe647ca
    keyring="${artifact_dir}/ros2-archive-keyring.gpg"
    curl -fsSL \
        "https://github.com/ros-infrastructure/ros-apt-source/raw/${ros_apt_source_commit}/ros-apt-source/keys/ros2-archive-keyring.gpg" \
        -o "${keyring}"
    ros_repository="http://packages.ros.org/ros2/ubuntu"
    if [[ "${ROS_TESTING:-false}" == true ]]; then
        ros_repository="http://packages.ros.org/ros2-testing/ubuntu"
    fi
    extra_repository_args+=(
        "--extra-repository=deb ${ros_repository} ${deb_distro} main"
        "--extra-repository-key=${keyring}"
    )
fi

printf '%s:\n  %s:\n  - %s\n' \
    "${package_name}" "${distribution}" "ros-${ros_distro}-${package_name}" \
    >"${artifact_dir}/local.yaml"
printf 'yaml file://%s/local.yaml %s\n' "$(cd "${artifact_dir}" && pwd)" "${ros_distro}" \
    >"${artifact_dir}/1-local.list"
: >"${artifact_dir}/2-remote.list"

ROS_HOME="${artifact_dir}/ros" \
ROSDEP_SOURCE_PATH="${artifact_dir}:/etc/ros/rosdep/sources.list.d/" \
    rosdep update --include-eol-distros

cleanup_debian_metadata() {
    if [[ -d "${package_dir}/debian" ]]; then
        rm -f "${package_dir}/debian/rules" \
            "${package_dir}/debian/compat" \
            "${package_dir}/debian/changelog" \
            "${package_dir}/debian/control" \
            "${package_dir}/debian/copyright" \
            "${package_dir}/debian/source/format" \
            "${package_dir}/debian/source/options"
        if [[ -d "${package_dir}/debian/source" ]]; then
            find "${package_dir}/debian/source" -type f -delete
            rmdir "${package_dir}/debian/source" 2>/dev/null || true
        fi
        rmdir "${package_dir}/debian" 2>/dev/null || true
    fi
}
trap cleanup_debian_metadata EXIT

build_timestamp="$(date +%Y.%m.%d.%H.%M)"
(
    cd "${package_dir}"
    ROS_HOME="${artifact_dir}/ros" BLOOM_SKIP_ROSDEP_UPDATE=1 \
        bloom-generate rosdebian \
        --os-name="${distribution}" \
        --os-version="${deb_distro}" \
        --ros-distro="${ros_distro}"
    sed -i "1 s@([^)]*)@(${package_version}-${build_timestamp})@" debian/changelog
    echo 11 >debian/compat
    DEB_BUILD_OPTIONS=nocheck sbuild \
        --chroot-mode=unshare \
        --no-clean-source \
        --no-run-lintian \
        --no-source \
        --dpkg-source-opts="-Zgzip -z1 --format=1.0 -sn" \
        --build-dir="${artifact_dir}" \
        --extra-package="${artifact_dir}" \
        --arch="${deb_arch}" \
        "${extra_repository_args[@]}"
)

if [[ -d "${artifact_dir}/ros" ]]; then
    find "${artifact_dir}/ros" -depth -mindepth 1 -delete
    rmdir "${artifact_dir}/ros"
fi
(cd "${artifact_dir}" && apt-ftparchive packages . >Packages && apt-ftparchive release . >Release)

echo "Built ${package_name} ${package_version}-* for ${ros_distro}/${deb_distro}/${deb_arch}"
