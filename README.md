# pymoveit2

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services.

| <img width="100%" src="https://user-images.githubusercontent.com/22929099/147369355-5f1b33ef-2e18-4042-9ea3-cd85b1a78fa0.gif" alt="Animation of ex_joint_goal.py"/> | <img width="100%" src="https://user-images.githubusercontent.com/22929099/147369356-b8ad2f4c-1996-47ac-9bfb-7fccd243fd56.gif" alt="Animation of ex_pose_goal.py"/> | <img width="100%" src="https://user-images.githubusercontent.com/22929099/147369354-640831e2-4661-4f3d-8fc2-3e97d7766e1a.gif" alt="Animation of ex_gripper.py"/> | <img width="100%" src="https://user-images.githubusercontent.com/22929099/147374152-50128188-ab73-4d55-a537-b641325ce9c6.gif" alt="Animation of ex_servo.py"/> |
| :-----------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|                                                                             Joint Goal                                                                              |                                                                             Pose Goal                                                                              |                                                                          Gripper Action                                                                          |                                                                         MoveIt 2 Servo                                                                         |

## Instructions

### Requirements

- **OS:** Ubuntu 20.04 (Focal)
  - Other distributions might work (not tested).

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Rolling](https://docs.ros.org/en/rolling/Installation.html)
  - [Foxy](https://docs.ros.org/en/galactic/Installation.html) and [Galactic](https://docs.ros.org/en/galactic/Installation.html) might work too (not tested)
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

## Examples

To demostrate `pymoveit2` usage, [examples](./examples) directory contains scripts that demonstrate the basic functionality.

Prior to running the examples, configure an environment for control of a robot with MoveIt 2, e.g. one of the following launch scripts from [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2).

```bash
# RViz (fake) ROS 2 control
ros2 launch panda_moveit_config ex_fake_control.launch.py
# Ignition Gazebo (simulated) ROS 2 control
ros2 launch panda_moveit_config ex_ign_control.launch.py
```

After that, the individual scripts can be run.

```bash
# Move to joint configuration
ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
# Move to Cartesian pose
ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
# Repeatadly toggle the gripper (or use "open"/"close" actions)
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"
# Example of using MoveIt 2 Servo to move the end-effector in a circular motion
ros2 run pymoveit2 ex_servo.py
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── examples/              # [dir] Examples demonstrating the use of `pymoveit2`
├── pymoveit2/             # [dir] ROS 2 launch scripts
    ├── robots/            # [dir] Presets for robots (data that can be extracted from URDF/SRDF)
    ├── gripper_command.py # Interface for Gripper that is controlled by GripperCommand
    ├── moveit2_gripper.py # Interface for MoveIt 2 Gripper that is controlled by JointTrajectoryController
    ├── moveit2_servo.py   # Interface for MoveIt 2 Servo that enables real-time control in Cartesian Space
    └── moveit2.py         # Interface for MoveIt 2 that enables planning and execution of trajectories
├── CMakeLists.txt         # Colcon-enabled CMake recipe
└── package.xml            # ROS 2 package metadata
```
