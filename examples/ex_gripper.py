#!/usr/bin/env python3
"""
Example of interacting with the gripper.
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="open"
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="close"
"""

import rclpy
from rclpy.node import Node
from pymoveit2 import GripperInterface
from pymoveit2.robots import kinova as robot

class KinovaGripper(Node):
    def __init__(self):
        super().__init__("ex_gripper")

        # Declare parameter for gripper action
        self.declare_parameter("action", "toggle")

        # Create gripper interface with Kinova Jaco2 settings
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            gripper_command_action_name="/j2n6s300_gripper/gripper_command",
        )

    def run(self):
        """Main function to execute gripper action based on user input."""
        action = self.get_parameter("action").get_parameter_value().string_value
        self.get_logger().info(f'Performing gripper action "{action}"')

        if action == "open":
            self.gripper_interface.open()
            self.gripper_interface.wait_until_executed()
        elif action == "close":
            self.gripper_interface.close()
            self.gripper_interface.wait_until_executed()
        else:
            self.get_logger().error(f"Unknown action: {action}")

        rclpy.shutdown()

def main():
    rclpy.init()
    gripper_node = KinovaGripper()
    gripper_node.run()

if __name__ == "__main__":
    main()