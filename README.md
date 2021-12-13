# pymoveit2

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services.

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work (not tested).

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Rolling](https://docs.ros.org/en/rolling/Installation.html)
  - [Foxy](https://docs.ros.org/en/galactic/Installation.html) and [Galactic](https://docs.ros.org/en/galactic/Installation.html) might work too (not tested).
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary)
  - Install/build a version based on the selected ROS 2 release
- [Python 3](https://www.python.org/downloads) (tested with `3.8`)

### Building

Clone this repository, install dependencies with [rosdep](https://github.com/ros-infrastructure/rosdep), and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/pymoveit2.git
# Install dependencies
rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace overlay.

```bash
source ${PYMOVEIT2_WS_DIR}/install/local_setup.bash
```

This enables importing of `pymoveit2` module from external workspaces.
