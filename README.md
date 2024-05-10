# pymoveit2

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services.

> Note: The official Python library for MoveIt 2 `moveit_py` is now available. Check the announcement [here](https://picknik.ai/moveit/ros/python/google/2023/04/28/GSOC-MoveIt-2-Python-Bindings.html)!

<div align="center" class="tg-wrap">
<table>
<tbody>
  <tr>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369355-5f1b33ef-2e18-4042-9ea3-cd85b1a78fa0.gif" alt="Animation of ex_joint_goal.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369356-b8ad2f4c-1996-47ac-9bfb-7fccd243fd56.gif" alt="Animation of ex_pose_goal.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369354-640831e2-4661-4f3d-8fc2-3e97d7766e1a.gif" alt="Animation of ex_gripper.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147374152-50128188-ab73-4d55-a537-b641325ce9c6.gif" alt="Animation of ex_servo.py"/></td>
  </tr>
  <tr>
    <td width="25%"><div align="center">Joint Goal</div></td>
    <td width="25%"><div align="center">Pose Goal</div></td>
    <td width="25%"><div align="center">Gripper Action</div></td>
    <td width="25%"><div align="center">MoveIt 2 Servo</div></td>
  </tr>
</tbody>
</table>
</div>

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html), [Humble](https://docs.ros.org/en/humble/Installation.html) or [Iron](https://docs.ros.org/en/iron/Installation.html)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary) corresponding to the selected ROS 2 distribution

All additional dependencies are installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

### Building

Clone this repository, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/pymoveit2.git
# Install dependencies
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables importing of `pymoveit2` module from external workspaces.

## Examples

To demonstrate `pymoveit2` usage, [examples](./examples) directory contains scripts that demonstrate the basic functionality. Additional examples can be found under [ign_moveit2_examples](https://github.com/AndrejOrsula/ign_moveit2_examples) repository.

Prior to running the examples, configure an environment for control of a robot with MoveIt 2. For instance, one of the following launch scripts from [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2) repository can be used.

```bash
# RViz (fake) ROS 2 control
ros2 launch panda_moveit_config ex_fake_control.launch.py
# Gazebo (simulated) ROS 2 control
ros2 launch panda_moveit_config ex_ign_control.launch.py
```

After that, the individual scripts can be run.

```bash
# Move to joint configuration
ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
# Move to Cartesian pose (motion in either joint or Cartesian space)
ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
# Repeatadly toggle the gripper (or use "open"/"close" actions)
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"
# Example of using MoveIt 2 Servo to move the end-effector in a circular motion
ros2 run pymoveit2 ex_servo.py
# Example of adding a collision object with primitive geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
# Example of adding a collision object with mesh geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_mesh.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"
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
