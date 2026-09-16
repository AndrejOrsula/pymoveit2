import copy
import math
import threading

import pytest
from conftest import FakeServiceClient
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.task import Future

from pymoveit2._planning_scene import (
    ALLOWED_COLLISION_MATRIX,
    ROBOT_STATE_ATTACHED_OBJECTS,
    WORLD_OBJECT_NAMES,
    SceneClient,
)


def _scene() -> PlanningScene:
    scene = PlanningScene()
    scene.world.collision_objects = [
        CollisionObject(id="box"),
        CollisionObject(id="keep"),
    ]
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(
            object=CollisionObject(id="tool", operation=CollisionObject.ADD)
        )
    ]
    scene.allowed_collision_matrix.entry_names = ["box", "keep"]
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[False, False]),
        AllowedCollisionEntry(enabled=[False, False]),
    ]
    scene.world.octomap.octomap.data = [1, 2, 3]
    return scene


def _install_get_response(client: FakeServiceClient, scene: PlanningScene):
    original = client.call_async

    def call_async(request):
        future = original(request)
        future.set_result(GetPlanningScene.Response(scene=copy.deepcopy(scene)))
        return future

    client.call_async = call_async


def _make(rclpy_node, *, get=None, apply=None):
    get = get or FakeServiceClient("get_planning_scene")
    apply = apply or FakeServiceClient("apply_planning_scene")
    return (
        SceneClient(
            node=rclpy_node,
            get_planning_scene_service=get,
            apply_planning_scene_service=apply,
        ),
        get,
        apply,
    )


class _FailingSecondClientNode:
    def __init__(self):
        self.created = []
        self.destroyed = []

    def get_logger(self):
        return None

    def create_client(self, **_kwargs):
        if self.created:
            raise RuntimeError("apply client creation failed")
        client = FakeServiceClient("get_planning_scene")
        self.created.append(client)
        return client

    def destroy_client(self, client):
        self.destroyed.append(client)


def test_owned_client_creation_unwinds_when_apply_creation_fails():
    node = _FailingSecondClientNode()

    with pytest.raises(RuntimeError, match="apply client creation failed"):
        SceneClient(node=node)

    assert len(node.created) == 1
    assert node.destroyed == node.created


def test_allow_uses_narrow_acm_diff_and_auto_settles(rclpy_node):
    client, get, apply = _make(rclpy_node)
    try:
        _install_get_response(get, _scene())
        future = client.allow_collisions("new", True)

        assert future is apply.futures[0]
        assert get.requests[0].components.components == ALLOWED_COLLISION_MATRIX
        sent = apply.requests[0].scene
        assert sent.is_diff is True
        assert sent.robot_state.is_diff is True
        assert sent.world.collision_objects == []
        assert sent.robot_state.attached_collision_objects == []
        assert list(sent.allowed_collision_matrix.entry_names) == [
            "box",
            "keep",
            "new",
        ]

        apply.futures[0].set_result(ApplyPlanningScene.Response(success=True))
        assert client.process_allow_collision_future(future) is True
        assert client.process_allow_collision_future(future) is True
        assert "new" in client.planning_scene.allowed_collision_matrix.entry_names
    finally:
        client.destroy()


def test_clear_sends_observed_removes_and_preserves_octomap(rclpy_node):
    client, get, apply = _make(rclpy_node)
    try:
        scene = _scene()
        _install_get_response(get, scene)
        assert client.update_planning_scene() is True

        def narrow_call(request):
            future = Future()
            get.requests.append(request)
            get.futures.append(future)
            partial = PlanningScene()
            partial.world.collision_objects = copy.deepcopy(
                scene.world.collision_objects
            )
            partial.robot_state.attached_collision_objects = copy.deepcopy(
                scene.robot_state.attached_collision_objects
            )
            future.set_result(GetPlanningScene.Response(scene=partial))
            return future

        get.call_async = narrow_call
        future = client.clear_all_collision_objects()
        assert future is apply.futures[0]
        assert (
            get.requests[1].components.components
            == WORLD_OBJECT_NAMES | ROBOT_STATE_ATTACHED_OBJECTS
        )
        sent = apply.requests[0].scene
        assert [obj.id for obj in sent.world.collision_objects] == ["box", "keep"]
        assert all(
            obj.operation == CollisionObject.REMOVE
            for obj in sent.world.collision_objects
        )
        assert [
            obj.object.id for obj in sent.robot_state.attached_collision_objects
        ] == ["tool"]
        assert sent.robot_state.is_diff is True

        apply.futures[0].set_result(ApplyPlanningScene.Response(success=True))
        assert client.process_clear_all_collision_objects_future(future) is True
        cached = client.planning_scene
        assert cached.world.collision_objects == []
        assert list(cached.world.octomap.octomap.data) == [1, 2, 3]
        assert client.scene_cache_dirty is True
    finally:
        client.destroy()


def test_names_only_read_marks_geometry_dirty_when_apply_discovery_fails(rclpy_node):
    client, get, apply = _make(rclpy_node)
    try:
        scene = _scene()
        _install_get_response(get, scene)
        assert client.update_planning_scene() is True

        def narrow_call(request):
            future = Future()
            get.requests.append(request)
            get.futures.append(future)
            partial = PlanningScene()
            partial.world.collision_objects = [
                CollisionObject(id=obj.id) for obj in scene.world.collision_objects
            ]
            partial.robot_state.attached_collision_objects = copy.deepcopy(
                scene.robot_state.attached_collision_objects
            )
            future.set_result(GetPlanningScene.Response(scene=partial))
            return future

        get.call_async = narrow_call
        apply.ready = False
        assert client.clear_all_collision_objects(timeout_sec=0.05) is None
        assert client.scene_cache_dirty is True
    finally:
        client.destroy()


def test_overlap_is_refused_without_fetch_or_send(rclpy_node):
    client, get, apply = _make(rclpy_node)
    first_get = Future()
    started = threading.Event()

    def call_async(request):
        get.requests.append(request)
        get.futures.append(first_get)
        started.set()
        return first_get

    get.call_async = call_async
    first = []

    def run_first():
        first.append(client.allow_collisions("a", True, timeout_sec=None))

    thread = threading.Thread(target=run_first)
    thread.start()
    assert started.wait(timeout=1.0)
    assert client.allow_collisions("b", True) is None
    assert len(get.requests) == 1
    assert not apply.requests

    first_get.set_result(GetPlanningScene.Response(scene=_scene()))
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert first == [apply.futures[0]]
    apply.futures[0].set_result(ApplyPlanningScene.Response(success=True))
    assert client.process_apply_planning_scene_future(first[0]) is True
    client.destroy()


def test_cancelled_or_exception_apply_quarantines_later_writes(rclpy_node):
    client, get, apply = _make(rclpy_node)
    try:
        _install_get_response(get, _scene())
        future = client.allow_collisions("a", True)
        client.cancel_apply_planning_scene_future(future)
        assert client.mutation_quarantined is True
        assert client.allow_collisions("b", True) is None

        apply.futures[0].set_result(ApplyPlanningScene.Response(success=True))
        assert client.process_apply_planning_scene_future(future) is True
        assert client.mutation_quarantined is False
    finally:
        client.destroy()


def test_destroy_makes_late_callback_inert(rclpy_node):
    client, get, apply = _make(rclpy_node)
    _install_get_response(get, _scene())
    future = client.allow_collisions("a", True)
    before = client.planning_scene
    client.destroy()
    apply.futures[0].set_result(ApplyPlanningScene.Response(success=True))
    assert client.planning_scene == before
    assert client.process_apply_planning_scene_future(future) is False


def test_read_timeout_removes_owned_pending_request(rclpy_node):
    client, get, _ = _make(rclpy_node)
    try:
        assert client.update_planning_scene(timeout_sec=0.01) is False
        assert len(get.removed_requests) == 1
        assert get.removed_requests[0] is get.futures[0]
    finally:
        client.destroy()


def test_invalid_timeout_does_not_strand_mutation_admission(rclpy_node):
    client, get, apply = _make(rclpy_node)
    try:
        _install_get_response(get, _scene())
        try:
            client.allow_collisions("bad", True, timeout_sec=math.nan)
        except ValueError:
            pass
        else:
            raise AssertionError("non-finite timeout must raise ValueError")
        assert client.allow_collisions("good", True) is apply.futures[0]
    finally:
        client.destroy()


def test_destroy_wakes_an_unbounded_scene_read(rclpy_node):
    client, get, _ = _make(rclpy_node)
    started = threading.Event()
    original = get.call_async

    def call_async(request):
        future = original(request)
        started.set()
        return future

    get.call_async = call_async
    result = []
    thread = threading.Thread(
        target=lambda: result.append(client.update_planning_scene(timeout_sec=None))
    )
    thread.start()
    assert started.wait(timeout=1.0)
    client.destroy()
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert result == [False]
    assert get.removed_requests == [get.futures[0]]
