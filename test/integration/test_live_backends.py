import os
import threading
import time

import pytest
import rclpy
from control_msgs.action import GripperCommand as GripperCommandAction
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

from pymoveit2 import GripperCommand, MoveIt2State, RobotDescription

pytestmark = pytest.mark.skipif(
    os.environ.get("PYMOVEIT2_INTEGRATION") != "1",
    reason="integration environment not available",
)

READY_TIMEOUT_SEC = 10.0


MODEL_URDF = """
<robot name="wp13_tinybot">
  <link name="base"/>
  <link name="link1"/>
  <link name="tip"/>
  <link name="clamp_link"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="tip"/>
  </joint>
  <joint name="clamp_joint" type="prismatic">
    <parent link="tip"/><child link="clamp_link"/>
  </joint>
</robot>
"""

MODEL_SRDF = """
<robot name="wp13_tinybot">
  <group name="tiny_arm">
    <chain base_link="base" tip_link="tip"/>
  </group>
  <group name="clamp">
    <joint name="clamp_joint"/>
  </group>
  <group_state group="clamp" name="open">
    <joint name="clamp_joint" value="0.04"/>
  </group_state>
  <group_state group="clamp" name="closed">
    <joint name="clamp_joint" value="0.0"/>
  </group_state>
  <end_effector group="clamp" name="clamp" parent_group="tiny_arm"
                parent_link="tip"/>
</robot>
"""


def _wait_until(predicate, timeout_sec: float = READY_TIMEOUT_SEC) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


class _SimulatedRawGripper:
    def __init__(self, node, action_name: str):
        self._node = node
        self._lock = threading.Lock()
        self.position = 0.04
        self.commands = []
        self.results = []
        self._state_pub = node.create_publisher(
            JointState,
            "joint_states",
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        self._state_timer = node.create_timer(0.05, self._publish_state)
        self._action_server = ActionServer(
            node,
            GripperCommandAction,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_request,
            cancel_callback=self._cancel_request,
            callback_group=ReentrantCallbackGroup(),
        )

    def destroy(self) -> None:
        self._action_server.destroy()
        self._node.destroy_timer(self._state_timer)
        self._node.destroy_publisher(self._state_pub)

    def _publish_state(self) -> None:
        with self._lock:
            position = self.position
        self._state_pub.publish(
            JointState(name=["wp13_clamp_joint"], position=[position])
        )

    def _goal_request(self, request):
        with self._lock:
            self.commands.append(float(request.command.position))
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_request(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        target = float(goal_handle.request.command.position)
        result = GripperCommandAction.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.reached_goal = False
            result.position = target
            result.effort = 0.0
            result.stalled = False
            return result

        with self._lock:
            self.position = target
            self.results.append((target, True))
        goal_handle.succeed()
        result.reached_goal = True
        result.position = target
        result.effort = 0.0
        result.stalled = False
        return result


class _ModelParameterServer:
    def __init__(self, node, service_name: str):
        self._node = node
        self._service = node.create_service(
            GetParameters, service_name, self._get_parameters
        )

    def destroy(self) -> None:
        self._node.destroy_service(self._service)

    @staticmethod
    def _get_parameters(request, response):
        values = []
        for name in request.names:
            if name == "robot_description":
                value = MODEL_URDF
            elif name == "robot_description_semantic":
                value = MODEL_SRDF
            else:
                value = ""
            values.append(
                ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=value,
                )
            )
        response.values = values
        return response


@pytest.fixture(scope="module")
def live_backend_graph():
    rclpy.init()
    namespace = f"/pymoveit2_wp13_{os.getpid()}"
    server_node = rclpy.create_node("wp13_backend_server", namespace=namespace)
    client_node = rclpy.create_node("wp13_backend_client", namespace=namespace)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    action_name = f"{namespace}/gripper_cmd"
    model_service_name = f"{namespace}/model/get_parameters"
    action_server = _SimulatedRawGripper(server_node, action_name)
    parameter_server = _ModelParameterServer(server_node, model_service_name)
    gripper = GripperCommand(
        node=client_node,
        gripper_joint_names=["wp13_clamp_joint"],
        open_gripper_joint_positions=[0.04],
        closed_gripper_joint_positions=[0.0],
        callback_group=ReentrantCallbackGroup(),
        gripper_command_action_name=action_name,
    )

    try:
        assert gripper.gripper_command_action_client.wait_for_server(READY_TIMEOUT_SEC)
        assert _wait_until(lambda: gripper.joint_state is not None)
        yield {
            "client_node": client_node,
            "gripper": gripper,
            "action_server": action_server,
            "model_service_name": model_service_name,
        }
    finally:
        gripper.destroy()
        parameter_server.destroy()
        action_server.destroy()
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        executor.remove_node(server_node)
        executor.remove_node(client_node)
        server_node.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def test_raw_gripper_command_uses_real_action_transport(live_backend_graph):
    gripper = live_backend_graph["gripper"]
    action_server = live_backend_graph["action_server"]

    assert gripper.open() is True
    assert gripper.wait_until_executed(timeout_sec=READY_TIMEOUT_SEC) is True
    assert _wait_until(lambda: gripper.is_open is True)
    assert action_server.commands[-1] == pytest.approx(0.04)
    assert action_server.results[-1][0] == pytest.approx(0.04)
    assert action_server.results[-1][1] is True
    assert gripper.get_last_execution_error_code() is None

    assert gripper.close() is True
    assert gripper.wait_until_executed(timeout_sec=READY_TIMEOUT_SEC) is True
    assert _wait_until(lambda: gripper.is_closed is True)
    assert action_server.commands[-1] == pytest.approx(0.0)
    assert gripper.get_last_execution_error_code() is None
    assert gripper.query_state() == MoveIt2State.IDLE


def test_non_panda_runtime_description_comes_from_ros_parameters(live_backend_graph):
    description = RobotDescription.from_node(
        live_backend_graph["client_node"],
        remote_node_name=live_backend_graph["model_service_name"].removesuffix(
            "/get_parameters"
        ),
        timeout_sec=READY_TIMEOUT_SEC,
        callback_group=ReentrantCallbackGroup(),
    )

    assert description.name == "wp13_tinybot"
    assert description.arm_group_name == "tiny_arm"
    assert description.gripper_group_name == "clamp"
    assert description.moveit2_kwargs() == {
        "joint_names": ["joint1", "joint2"],
        "base_link_name": "base",
        "end_effector_name": "tip",
        "group_name": "tiny_arm",
    }
    assert description.moveit2_gripper_kwargs() == {
        "gripper_joint_names": ["clamp_joint"],
        "open_gripper_joint_positions": [0.04],
        "closed_gripper_joint_positions": [0.0],
        "gripper_group_name": "clamp",
    }
