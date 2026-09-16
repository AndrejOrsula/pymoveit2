import time

from conftest import FakeServiceClient
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.task import Future


def _scene_with_objects(*ids: str) -> PlanningScene:
    scene = PlanningScene()
    for object_id in ids:
        scene.world.collision_objects.append(CollisionObject(id=object_id))
        scene.allowed_collision_matrix.entry_names.append(object_id)
    for _ in ids:
        scene.allowed_collision_matrix.entry_values.append(
            AllowedCollisionEntry(enabled=[False] * len(ids))
        )
    return scene


def _install_scene_services(moveit2, scene: PlanningScene):
    get_client = FakeServiceClient("get_planning_scene")
    apply_client = FakeServiceClient("apply_planning_scene")
    moveit2._get_planning_scene_service = get_client
    moveit2._apply_planning_scene_service = apply_client

    original_call_async = get_client.call_async

    def call_async(request):
        future = original_call_async(request)
        future.set_result(GetPlanningScene.Response(scene=scene))
        return future

    get_client.call_async = call_async
    return get_client, apply_client


def test_update_planning_scene_times_out_cleanly(moveit2):
    moveit2._get_planning_scene_service = FakeServiceClient("get_planning_scene")
    start = time.monotonic()
    assert moveit2.update_planning_scene(timeout_sec=0.2) is False
    elapsed = time.monotonic() - start
    assert 0.15 <= elapsed < 2.0


def test_update_planning_scene_none_result_returns_false(moveit2):
    client = FakeServiceClient("get_planning_scene")
    moveit2._get_planning_scene_service = client
    original = client.call_async

    def call_async(request):
        future: Future = original(request)
        future.set_result(None)
        return future

    client.call_async = call_async
    assert moveit2.update_planning_scene(timeout_sec=0.5) is False


def test_planning_scene_property_returns_copy(moveit2):
    _install_scene_services(moveit2, _scene_with_objects("box"))
    assert moveit2.update_planning_scene() is True
    copy_one = moveit2.planning_scene
    copy_one.world.collision_objects.clear()
    assert len(moveit2.planning_scene.world.collision_objects) == 1


def test_allow_collisions_commits_only_after_success(moveit2):
    _, apply_client = _install_scene_services(moveit2, _scene_with_objects("box"))
    future = moveit2.allow_collisions("sphere", True)
    assert future is apply_client.futures[0]

    sent = apply_client.requests[0].scene
    assert "sphere" in sent.allowed_collision_matrix.entry_names
    assert "sphere" not in moveit2.planning_scene.allowed_collision_matrix.entry_names

    assert moveit2.process_allow_collision_future(future) is False

    apply_client.futures[0].set_result(ApplyPlanningScene.Response(success=False))
    assert moveit2.process_allow_collision_future(future) is False
    assert "sphere" not in moveit2.planning_scene.allowed_collision_matrix.entry_names

    future = moveit2.allow_collisions("sphere", True)
    apply_client.futures[1].set_result(ApplyPlanningScene.Response(success=True))
    assert moveit2.process_allow_collision_future(future) is True
    acm = moveit2.planning_scene.allowed_collision_matrix
    assert "sphere" in acm.entry_names
    j = acm.entry_names.index("sphere")
    assert all(entry.enabled[j] for i, entry in enumerate(acm.entry_values) if i != j)
    assert list(acm.entry_values[j].enabled) == [True, True]


def test_allow_collisions_existing_entry(moveit2):
    _, apply_client = _install_scene_services(moveit2, _scene_with_objects("a", "b"))
    future = moveit2.allow_collisions("a", True)
    sent = apply_client.requests[0].scene.allowed_collision_matrix
    assert list(sent.entry_names) == ["a", "b"]
    assert list(sent.entry_values[0].enabled) == [True, True]
    assert list(sent.entry_values[1].enabled) == [True, False]
    apply_client.futures[0].set_result(ApplyPlanningScene.Response(success=True))
    assert moveit2.process_allow_collision_future(future) is True


def test_overlapping_scene_mutation_is_refused_until_ack(moveit2):
    _, apply_client = _install_scene_services(moveit2, _scene_with_objects("box"))
    clear_future = moveit2.clear_all_collision_objects()
    assert clear_future is apply_client.futures[0]
    assert moveit2.allow_collisions("sphere", False) is None
    assert len(apply_client.requests) == 1

    apply_client.futures[0].set_result(ApplyPlanningScene.Response(success=True))
    assert moveit2.process_clear_all_collision_objects_future(clear_future) is True

    allow_future = moveit2.allow_collisions("sphere", False)
    assert allow_future is apply_client.futures[1]
    apply_client.futures[1].set_result(ApplyPlanningScene.Response(success=True))
    assert moveit2.process_allow_collision_future(allow_future) is True
    scene = moveit2.planning_scene
    assert scene.world.collision_objects == []
    assert "sphere" in scene.allowed_collision_matrix.entry_names


def test_clear_all_collision_objects_and_cancel(moveit2):
    _, apply_client = _install_scene_services(moveit2, _scene_with_objects("box"))
    future = moveit2.clear_all_collision_objects()
    sent = apply_client.requests[0].scene
    assert [obj.id for obj in sent.world.collision_objects] == ["box"]
    assert all(
        obj.operation == CollisionObject.REMOVE for obj in sent.world.collision_objects
    )
    assert len(moveit2.planning_scene.world.collision_objects) == 1
    moveit2.cancel_clear_all_collision_objects_future(future)
    assert apply_client.removed_requests == [future]
    assert moveit2.planning_scene_mutation_quarantined is True
    assert moveit2.allow_collisions("later", True) is None

    apply_client.futures[0].set_result(ApplyPlanningScene.Response(success=True))
    assert moveit2.process_clear_all_collision_objects_future(future) is True
    assert moveit2.planning_scene_mutation_quarantined is False
    assert moveit2.planning_scene.world.collision_objects == []


def test_apply_failure_paths(moveit2):
    _, apply_client = _install_scene_services(moveit2, _scene_with_objects("box"))
    future = moveit2.allow_collisions("box", True)
    apply_client.futures[0].set_exception(RuntimeError("transport"))
    assert moveit2.process_allow_collision_future(future) is False
    apply_client.ready = False
    assert moveit2.allow_collisions("box", True) is None
    assert moveit2.clear_all_collision_objects() is None
