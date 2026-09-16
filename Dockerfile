ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG SOURCE_ID=unknown
LABEL org.opencontainers.image.revision="${SOURCE_ID}"
ENV PYMOVEIT2_SOURCE_ID=${SOURCE_ID}

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Define the workspace
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_INSTALL_DIR=${WS_DIR}/install
WORKDIR ${WS_DIR}

### Install MoveIt 2 and a demo robot to run the examples against
ARG WITH_DEMO=true
RUN if [[ "${WITH_DEMO}" = true ]]; then \
    apt-get update && \
    apt-get install -yq --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-rviz2 \
    xauth && \
    rm -rf /var/lib/apt/lists/* ; \
    fi

### Install the optional `trimesh` dependency of mesh collision objects
ARG WITH_TRIMESH=true
RUN if [[ "${WITH_TRIMESH}" = true ]]; then \
    apt-get update && \
    apt-get install -yq --no-install-recommends python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    # Humble ships numpy 1.21, which trimesh >= 4 no longer imports with
    if [[ "${ROS_DISTRO}" = humble ]]; then TRIMESH="trimesh<4"; else TRIMESH="trimesh"; fi && \
    { pip install --no-cache-dir --break-system-packages "${TRIMESH}" || \
    pip install --no-cache-dir "${TRIMESH}" ; } ; \
    fi

### Install dependencies of this package and build it
COPY . ${WS_SRC_DIR}/pymoveit2/
RUN apt-get update && \
    rosdep update --rosdistro "${ROS_DISTRO}" && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths "${WS_SRC_DIR}" && \
    rm -rf /var/lib/apt/lists/* && \
    chmod +x "${WS_SRC_DIR}/pymoveit2"/examples/ex_*.py && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf "${WS_DIR}/log"

### Source the workspace in the entrypoint and in interactive shells
RUN for entrypoint in /ros_entrypoint.sh /entrypoint.sh; do \
    if [ -f "${entrypoint}" ]; then \
    sed -i '$i source "${WS_INSTALL_DIR}/local_setup.bash" --' "${entrypoint}" ; \
    fi ; \
    done && \
    echo 'source "${WS_INSTALL_DIR}/local_setup.bash"' >> ~/.bashrc
