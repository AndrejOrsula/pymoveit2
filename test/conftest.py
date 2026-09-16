from types import SimpleNamespace
from typing import Any, List, Optional

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

PANDA_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
GRIPPER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


@pytest.fixture(scope="session")
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node("pymoveit2_test_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture()
def moveit2(rclpy_node):
    from pymoveit2 import MoveIt2

    interface = MoveIt2(
        node=rclpy_node,
        joint_names=PANDA_JOINTS,
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        callback_group=ReentrantCallbackGroup(),
    )
    yield interface
    interface.destroy()


class FakeGoalHandle:
    def __init__(self, accepted: bool = True):
        self.accepted = accepted
        self.result_future: Future = Future()
        self.cancel_future: Future = Future()
        self.cancel_requests = 0

    def get_result_async(self) -> Future:
        return self.result_future

    def cancel_goal_async(self) -> Future:
        self.cancel_requests += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self, name: str = "fake_action", ready: bool = True):
        self._action_name = name
        self.ready = ready
        self.sent_goals: List[Any] = []
        self.response_futures: List[Future] = []
        self.destroyed = False

    def server_is_ready(self) -> bool:
        return self.ready

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        return self.ready

    def send_goal_async(self, goal: Any, feedback_callback: Any = None) -> Future:
        self.sent_goals.append(goal)
        future: Future = Future()
        self.response_futures.append(future)
        return future

    def destroy(self) -> None:
        self.destroyed = True

    def accept(self, index: int = -1) -> FakeGoalHandle:
        handle = FakeGoalHandle(accepted=True)
        self.response_futures[index].set_result(handle)
        return handle

    def reject(self, index: int = -1) -> FakeGoalHandle:
        handle = FakeGoalHandle(accepted=False)
        self.response_futures[index].set_result(handle)
        return handle


class FakeServiceClient:
    def __init__(self, name: str = "fake_service", ready: bool = True):
        self.srv_name = name
        self.ready = ready
        self.requests: List[Any] = []
        self.futures: List[Future] = []
        self.removed_requests: List[Future] = []

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        return self.ready

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request: Any) -> Future:
        self.requests.append(request)
        future: Future = Future()
        self.futures.append(future)
        return future

    def call(self, request: Any) -> Any:
        self.requests.append(request)
        return self.sync_response

    def remove_pending_request(self, future: Future) -> None:
        self.removed_requests.append(future)


def action_result(status: int = GoalStatus.STATUS_SUCCEEDED, result: Any = None):
    return SimpleNamespace(status=status, result=result)


def complete(
    handle: FakeGoalHandle, status: int = GoalStatus.STATUS_SUCCEEDED, result=None
):
    handle.result_future.set_result(action_result(status, result))


@pytest.fixture()
def fake_execute_client(moveit2):
    client = FakeActionClient("execute_trajectory")
    moveit2._execute_trajectory_action_client = client
    return client
