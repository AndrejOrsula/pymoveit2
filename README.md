# pymoveit2

[![ci](https://github.com/AndrejOrsula/pymoveit2/actions/workflows/ci.yml/badge.svg)](https://github.com/AndrejOrsula/pymoveit2/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/AndrejOrsula/pymoveit2/graph/badge.svg)](https://codecov.io/gh/AndrejOrsula/pymoveit2)

> Move a robot from Python by communicating with MoveIt 2 over ROS 2 actions and services

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
    <td width="25%"><div align="center">Joint Configuration</div></td>
    <td width="25%"><div align="center">Cartesian Pose</div></td>
    <td width="25%"><div align="center">Gripper Action</div></td>
    <td width="25%"><div align="center">Real-Time Servoing</div></td>
  </tr>
</tbody>
</table>
</div>

## Installation

| **`humble`** | **`jazzy`** | **`lyrical`** | **`rolling`** |
| :----------: | :---------: | :-----------: | :-----------: |
| ✅ | ✅ | ✅ | ✅ |

<!--
### Binary package (apt)

```bash
sudo apt install ros-$ROS_DISTRO-pymoveit2
```
-->

### Python package (PyPI)

```bash
pip install pymoveit2
```

### Source build (colcon)

```bash
cd $COLCON_WS
git clone https://github.com/AndrejOrsula/pymoveit2.git src/pymoveit2
rosdep install -y -r -i --rosdistro $ROS_DISTRO --from-paths src/pymoveit2
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
source install/local_setup.bash
```

### Docker image

```bash
cd $WS
git clone https://github.com/AndrejOrsula/pymoveit2.git pymoveit2
pymoveit2/.docker/run.bash $ROS_DISTRO --network-host --ipc-host --gui
```

## Quick start (demo with Franka Emika Panda)

1. Configure MoveIt 2 for your robot (install for widely available robots, or build one on your own):

   ```bash
   sudo apt install ros-$ROS_DISTRO-moveit-resources-panda-moveit-config ros-$ROS_DISTRO-controller-manager
   ```

1. Launch the MoveIt 2 setup for your robot:

   ```bash
   ros2 launch moveit_resources_panda_moveit_config demo.launch.py
   ```

1. Move the robot via `pymoveit2` examples:

   ```bash
   ros2 run pymoveit2 ex_joint_goal.py
   ```

Nothing moved? Run `ros2 run pymoveit2 ex_doctor.py` to investigate the problem.

## Examples

### Kinematics

```bash
# Forward (joint positions -> end effector pose)
ros2 run pymoveit2 ex_fk.py
# Inverse (end effector pose -> joint positions)
ros2 run pymoveit2 ex_ik.py --ros-args -p position:="[0.3, 0.0, 0.3]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
```

### Motion planning and execution

```bash
# Move to a joint configuration (default to SRDF group state)
ros2 run pymoveit2 ex_joint_goal.py
# Move to a pose (motion in joint space or Cartesian space)
ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.3, 0.0, 0.3]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
# Move while maintaining a fixed orientation of the end effector
ros2 run pymoveit2 ex_orientation_path_constraint.py --ros-args -p use_orientation_constraint:=True
# Move via real-time servoing (MoveIt 2 Servo)
ros2 run pymoveit2 ex_servo.py
# Actuate the gripper (action: {toggle, open, close})
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"
```

### Planning scene

```bash
# Add a primitive shape to the planning scene (shape: {box, sphere, cone, cylinder})
ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
# Add a triangular mesh to the planning scene (action: {add, remove}) [Note: Requires `trimesh` Python package]
ros2 run pymoveit2 ex_collision_mesh.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"
# Allow or forbid collisions with a planning scene object (allow: {true, false})
ros2 run pymoveit2 ex_allow_collisions.py --ros-args -p id:="sphere" -p allow:=true
# Remove all objects from the planning scene
ros2 run pymoveit2 ex_clear_planning_scene.py
```

## Python API

### RobotSession

The simplest way to use `pymoveit2` is via **`connect()`** that provides a context manager for your robot:

```python
from pymoveit2 import connect

with connect() as robot:
    # Move to a pose while checking for failures
    if not robot.move_to_pose([0.4, 0.0, 0.4], [1.0, 0.0, 0.0, 0.0]):
        print(robot.last_failure())
        exit(1)

    # Actuate the gripper
    robot.gripper.close()

    # Move to SRDF group state
    robot.move_to_configuration()
```

The `robot` object represents a `RobotSession` that manages:

1. ROS 2 node: `robot.node`
1. ROS 2 executor: `robot.executor`
1. Robot description (automatically discovered): `robot.description`
1. MoveIt 2 interface for the motion planning: `robot.arm`
1. (optional) MoveIt 2 Gripper interface: `robot.gripper`
1. (optional) MoveIt 2 Servo interface: `robot.servo`

### RobotDescription + MoveIt2 + GripperInterface + MoveIt2Servo

Alternatively, you can build and customize the interfaces while managing the ROS 2 node and executor yourself:

```python
from pymoveit2 import RobotDescription, MoveIt2, GripperInterface, MoveIt2Servo

node = ...  # your rclpy node

description = RobotDescription.from_node(node) # note: in the unlikely case of failure, `pymoveit2.robots` provides static robot presets
moveit2 = MoveIt2(node=node, **description.moveit2_kwargs())
gripper = GripperInterface(node=node, **description.moveit2_gripper_kwargs()) if description.gripper_group_name else None
servo = MoveIt2Servo(node=node, frame_id=str(description.moveit2_kwargs()["base_link_name"]))
```
