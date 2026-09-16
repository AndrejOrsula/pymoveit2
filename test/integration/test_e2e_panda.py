import copy
import os
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

pytestmark = pytest.mark.skipif(
    os.environ.get("PYMOVEIT2_INTEGRATION") != "1",
    reason="integration environment not available",
)

HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
READY_TIMEOUT_SEC = 60.0


def _defer_cleanup(live, cleanup):
    live[2].append(cleanup)


@pytest.fixture(scope="module")
def live():
    from pymoveit2 import MoveIt2

    rclpy.init()
    node = rclpy.create_node("pymoveit2_e2e")
    moveit2 = MoveIt2(
        node=node,
        joint_names=[f"panda_joint{i}" for i in range(1, 8)],
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        callback_group=ReentrantCallbackGroup(),
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    assert moveit2.wait_for_joint_state(timeout_sec=READY_TIMEOUT_SEC)
    assert moveit2._plan_kinematic_path_service.wait_for_service(READY_TIMEOUT_SEC)
    assert moveit2._execute_trajectory_action_client.wait_for_server(READY_TIMEOUT_SEC)

    cleanup_callbacks = [moveit2.destroy]
    yield node, moveit2, cleanup_callbacks
    executor.shutdown(timeout_sec=5.0)
    thread.join(timeout=5.0)
    executor.remove_node(node)
    for cleanup in reversed(cleanup_callbacks):
        cleanup()
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture()
def moveit2_live(live):
    return live[1]


@pytest.fixture()
def admission_moveit2(live):
    from pymoveit2 import MoveIt2

    node = live[0]
    moveit2 = MoveIt2(
        node=node,
        joint_names=[f"panda_joint{i}" for i in range(1, 8)],
        base_link_name="panda_link0",
        end_effector_name="panda_hand",
        group_name="panda_arm",
        use_move_group_action=False,
        ignore_new_calls_while_executing=True,
        callback_group=ReentrantCallbackGroup(),
    )
    try:
        assert moveit2.wait_for_joint_state(timeout_sec=READY_TIMEOUT_SEC)
        assert moveit2._execute_trajectory_action_client.wait_for_server(
            READY_TIMEOUT_SEC
        )
        yield moveit2
    finally:
        if moveit2.query_state().name != "IDLE":
            moveit2.cancel_execution()
            moveit2.wait_until_executed(timeout_sec=10.0)
        _defer_cleanup(live, moveit2.destroy)


def _stretch_trajectory(trajectory):
    stretched = copy.deepcopy(trajectory)
    for index, point in enumerate(stretched.points):
        current = point.time_from_start.sec + (
            point.time_from_start.nanosec / 1_000_000_000.0
        )
        total = max(current * 10.0, float(index))
        point.time_from_start.sec = int(total)
        nanosec = int(round((total - point.time_from_start.sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            point.time_from_start.sec += 1
            nanosec -= 1_000_000_000
        point.time_from_start.nanosec = nanosec
    return stretched


def _wait_for_scene_ids(moveit2, required_ids, absent_ids=()):
    required_ids = set(required_ids)
    absent_ids = set(absent_ids)
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if moveit2.update_planning_scene(timeout_sec=2.0):
            scene = moveit2.planning_scene
            if scene is None:
                continue
            objects = {obj.id for obj in scene.world.collision_objects}
            if required_ids.issubset(objects) and not absent_ids.intersection(objects):
                return True
        time.sleep(0.1)
    return False


def test_joint_goal_executes(moveit2_live):
    assert moveit2_live.move_to_configuration(HOME) is True
    assert moveit2_live.wait_until_executed(timeout_sec=30.0) is True
    from pymoveit2 import MoveIt2State

    assert moveit2_live.query_state() == MoveIt2State.IDLE


def test_overlap_admission_refuses_second_live_goal(admission_moveit2):
    from pymoveit2 import MoveIt2State

    trajectory = admission_moveit2.plan(
        joint_positions=HOME,
        timeout_sec=20.0,
    )
    assert trajectory is not None
    trajectory = _stretch_trajectory(trajectory)

    assert admission_moveit2.execute(trajectory) is True
    first_operation = admission_moveit2.current_operation
    assert first_operation is not None
    assert admission_moveit2.execute(copy.deepcopy(trajectory)) is False
    assert admission_moveit2.current_operation is first_operation
    assert admission_moveit2.wait_until_executed(timeout_sec=45.0) is True
    assert admission_moveit2.query_state() == MoveIt2State.IDLE
    assert admission_moveit2.motion_succeeded is True


def test_pose_goal_executes(moveit2_live):
    assert (
        moveit2_live.move_to_pose(
            position=[0.4, 0.0, 0.4], quat_xyzw=[1.0, 0.0, 0.0, 0.0]
        )
        is True
    )
    assert moveit2_live.wait_until_executed(timeout_sec=30.0) is True


def test_fk(moveit2_live):
    pose = moveit2_live.compute_fk(timeout_sec=10.0)
    assert pose is not None
    poses = moveit2_live.compute_fk(
        joint_state=HOME, fk_link_names=["panda_link8", "panda_hand"], timeout_sec=10.0
    )
    assert isinstance(poses, list) and len(poses) == 2


def test_ik(moveit2_live):
    solution = moveit2_live.compute_ik(
        position=(0.4, 0.0, 0.4), quat_xyzw=(1.0, 0.0, 0.0, 0.0), timeout_sec=10.0
    )
    assert solution is not None

    solution = moveit2_live.compute_ik(
        position=(0.4, 0.0, 0.4),
        quat_xyzw=(1.0, 0.0, 0.0, 0.0),
        ik_link_name="panda_hand",
        timeout_sec=10.0,
    )
    assert solution is not None


def test_cartesian_plan(moveit2_live):
    trajectory = moveit2_live.plan(
        position=[0.3, 0.1, 0.5],
        quat_xyzw=[1.0, 0.0, 0.0, 0.0],
        cartesian=True,
        timeout_sec=10.0,
    )
    assert trajectory is not None


def test_cancel_execution_stops_goal(moveit2_live):
    from pymoveit2 import MoveIt2State

    moveit2_live.move_to_configuration(HOME)
    moveit2_live.wait_until_executed(timeout_sec=30.0)
    moveit2_live.max_velocity = 0.1
    moveit2_live.max_acceleration = 0.1
    try:
        assert (
            moveit2_live.move_to_configuration([0.8, -0.3, 0.3, -2.0, 0.2, 1.8, 1.2])
            is True
        )
        deadline = time.monotonic() + 10.0
        while (
            moveit2_live.query_state() != MoveIt2State.EXECUTING
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert moveit2_live.query_state() == MoveIt2State.EXECUTING
        future = moveit2_live.get_execution_future()
        assert future is not None

        time.sleep(0.5)
        cancelled_at = time.monotonic()
        assert moveit2_live.cancel_execution() is True
        assert moveit2_live.wait_until_executed(timeout_sec=15.0) is False

        assert time.monotonic() - cancelled_at < 2.0
        assert future.done()
        assert moveit2_live.query_state() == MoveIt2State.IDLE
        assert moveit2_live.get_last_execution_error_code().val == (
            MoveItErrorCodes.PREEMPTED
        )
    finally:
        moveit2_live.max_velocity = 0.0
        moveit2_live.max_acceleration = 0.0
        moveit2_live.move_to_configuration(HOME)
        moveit2_live.wait_until_executed(timeout_sec=30.0)


def test_planning_scene_transaction(moveit2_live):
    assert moveit2_live.update_planning_scene(timeout_sec=5.0) is True
    moveit2_live.add_collision_sphere(
        id="e2e_sphere", radius=0.05, position=(0.6, 0.3, 0.5)
    )
    deadline = time.monotonic() + 10.0
    present = False
    while time.monotonic() < deadline and not present:
        moveit2_live.update_planning_scene(timeout_sec=2.0)
        present = any(
            obj.id == "e2e_sphere"
            for obj in moveit2_live.planning_scene.world.collision_objects
        )
        time.sleep(0.1)
    assert present

    future = moveit2_live.allow_collisions("e2e_sphere", True, timeout_sec=5.0)
    assert future is not None
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(timeout=10.0)
    assert moveit2_live.process_allow_collision_future(future) is True
    acm = moveit2_live.planning_scene.allowed_collision_matrix
    assert "e2e_sphere" in acm.entry_names

    future = moveit2_live.clear_all_collision_objects(timeout_sec=5.0)
    assert future is not None
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(timeout=10.0)
    assert moveit2_live.process_clear_all_collision_objects_future(future) is True
    assert moveit2_live.planning_scene.world.collision_objects == []


def test_clear_preserves_external_world_object(moveit2_live, live):
    node = live[0]
    target_id = f"e2e_clear_target_{os.getpid()}"
    external_id = f"e2e_external_{os.getpid()}"
    moveit2_live.add_collision_sphere(
        id=target_id, radius=0.03, position=(0.55, 0.25, 0.5)
    )
    assert _wait_for_scene_ids(moveit2_live, [target_id], [external_id])
    scene_before_clear = moveit2_live.planning_scene
    assert scene_before_clear is not None
    octomap_before_clear = copy.deepcopy(scene_before_clear.world.octomap)

    external_client = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    assert external_client.wait_for_service(READY_TIMEOUT_SEC)
    original_apply_client = moveit2_live._apply_planning_scene_service
    injected = False

    class _ExternalApplyBeforeLibraryApply:
        def __init__(self, delegate):
            self._delegate = delegate

        def __getattr__(self, name):
            return getattr(self._delegate, name)

        def call_async(self, request):
            nonlocal injected
            if not injected:
                injected = True
                external_scene = PlanningScene(is_diff=True)
                external_scene.robot_state.is_diff = True
                external_scene.world.collision_objects = [
                    CollisionObject(
                        id=external_id,
                        operation=CollisionObject.ADD,
                        header=Header(frame_id="panda_link0"),
                        primitives=[
                            SolidPrimitive(
                                type=SolidPrimitive.SPHERE, dimensions=[0.03]
                            )
                        ],
                        primitive_poses=[Pose(orientation=Quaternion(w=1.0))],
                    )
                ]
                external_request = ApplyPlanningScene.Request()
                external_request.scene = external_scene
                external_future = external_client.call_async(external_request)
                done = threading.Event()
                external_future.add_done_callback(lambda _: done.set())
                assert done.wait(timeout=READY_TIMEOUT_SEC)
                assert external_future.result().success is True
            return self._delegate.call_async(request)

    moveit2_live._apply_planning_scene_service = _ExternalApplyBeforeLibraryApply(
        original_apply_client
    )
    try:
        future = moveit2_live.clear_all_collision_objects(timeout_sec=READY_TIMEOUT_SEC)
        assert future is not None
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout=READY_TIMEOUT_SEC)
        assert moveit2_live.process_clear_all_collision_objects_future(future) is True
        assert injected is True
        assert _wait_for_scene_ids(moveit2_live, [external_id], [target_id])
        scene_after_clear = moveit2_live.planning_scene
        assert scene_after_clear is not None
        assert scene_after_clear.world.octomap == octomap_before_clear
    finally:
        moveit2_live._apply_planning_scene_service = original_apply_client
        moveit2_live.remove_collision_object(external_id)
        moveit2_live.remove_collision_object(target_id)
        _wait_for_scene_ids(moveit2_live, [], [external_id, target_id])
        _defer_cleanup(live, lambda: node.destroy_client(external_client))


def test_gripper_planned_backend(live):
    from pymoveit2 import GripperInterface, MoveIt2Gripper

    node = live[0]
    gripper = GripperInterface(
        node=node,
        gripper_joint_names=["panda_finger_joint1", "panda_finger_joint2"],
        open_gripper_joint_positions=[0.04, 0.04],
        closed_gripper_joint_positions=[0.0, 0.0],
        gripper_group_name="hand",
        callback_group=ReentrantCallbackGroup(),
        discovery_timeout_sec=5.0,
    )
    try:
        assert gripper.interface is MoveIt2Gripper
        assert gripper.backend.wait_for_joint_state(timeout_sec=10.0)
        assert gripper.close() is True
        assert gripper.wait_until_executed(timeout_sec=20.0) is True
        assert gripper.is_closed is True
        assert gripper.open() is True
        assert gripper.wait_until_executed(timeout_sec=20.0) is True
        assert gripper.is_open is True
    finally:
        _defer_cleanup(live, gripper.destroy)


def test_robot_description_matches_the_running_move_group(live):
    from pymoveit2 import MoveIt2, RobotDescription

    node = live[0]
    description = RobotDescription.from_node(
        node, timeout_sec=30.0, callback_group=ReentrantCallbackGroup()
    )
    assert description.arm_group_name == "panda_arm"
    assert description.gripper_group_name == "hand"
    kwargs = description.moveit2_kwargs()
    assert kwargs["joint_names"] == [f"panda_joint{i}" for i in range(1, 8)]
    assert kwargs["base_link_name"] == "panda_link0"
    assert description.moveit2_gripper_kwargs()["gripper_joint_names"] == [
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]

    derived = MoveIt2(node=node, callback_group=ReentrantCallbackGroup(), **kwargs)
    try:
        assert derived.plan(joint_positions=HOME, timeout_sec=20.0) is not None
    finally:
        _defer_cleanup(live, derived.destroy)


def test_invalid_cartesian_input_raises(moveit2_live):
    with pytest.raises(ValueError):
        moveit2_live.plan(joint_positions=HOME, cartesian=True, timeout_sec=5.0)
