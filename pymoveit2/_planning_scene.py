"""Helpers for planning-scene interface."""

from __future__ import annotations

import copy
import math
import threading
import time
import weakref
from dataclasses import dataclass
from typing import Any, Callable, FrozenSet, Optional, Tuple

from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future

DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC = 3.0


def _reliable_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _component(name: str, fallback: int) -> int:
    return int(getattr(PlanningSceneComponents, name, fallback))


SCENE_SETTINGS = _component("SCENE_SETTINGS", 1)
ROBOT_STATE = _component("ROBOT_STATE", 2)
ROBOT_STATE_ATTACHED_OBJECTS = _component("ROBOT_STATE_ATTACHED_OBJECTS", 4)
WORLD_OBJECT_NAMES = _component("WORLD_OBJECT_NAMES", 8)
WORLD_OBJECT_GEOMETRY = _component("WORLD_OBJECT_GEOMETRY", 16)
OCTOMAP = _component("OCTOMAP", 32)
TRANSFORMS = _component("TRANSFORMS", 64)
ALLOWED_COLLISION_MATRIX = _component("ALLOWED_COLLISION_MATRIX", 128)
OBJECT_COLORS = _component("OBJECT_COLORS", 512)
LINK_PADDING_AND_SCALE = int(
    getattr(
        PlanningSceneComponents,
        "LINK_PADDING_AND_SCALING",
        getattr(PlanningSceneComponents, "LINK_PADDING_AND_SCALE", 256),
    )
)
OCTOMAP_GEOMETRY = _component("OCTOMAP_GEOMETRY", 0)

_KNOWN_COMPONENTS = (
    SCENE_SETTINGS
    | ROBOT_STATE
    | ROBOT_STATE_ATTACHED_OBJECTS
    | WORLD_OBJECT_NAMES
    | WORLD_OBJECT_GEOMETRY
    | OCTOMAP
    | TRANSFORMS
    | ALLOWED_COLLISION_MATRIX
    | OBJECT_COLORS
    | LINK_PADDING_AND_SCALE
    | OCTOMAP_GEOMETRY
)
_ALL_CONSTANT = getattr(PlanningSceneComponents, "ALL", None)
ALL_COMPONENTS = (
    int(_ALL_CONSTANT) if _ALL_CONSTANT not in (None, 0, -1) else int(_KNOWN_COMPONENTS)
)


class _Deadline:
    def __init__(self, timeout_sec: Optional[float]):
        if timeout_sec is not None:
            timeout_sec = float(timeout_sec)
            if not math.isfinite(timeout_sec):
                raise ValueError("timeout_sec must be finite or None")
        self._deadline = (
            None if timeout_sec is None else time.monotonic() + max(0.0, timeout_sec)
        )

    def remaining(self, cap: Optional[float] = None) -> Optional[float]:
        if self._deadline is None:
            return cap
        remaining = max(0.0, self._deadline - time.monotonic())
        return remaining if cap is None else min(cap, remaining)

    def expired(self) -> bool:
        return self._deadline is not None and time.monotonic() >= self._deadline


@dataclass
class _Mutation:
    token: object
    kind: str
    observed_world_ids: FrozenSet[str] = frozenset()
    observed_attached_ids: FrozenSet[str] = frozenset()
    acm: Optional[Any] = None
    settled: bool = False
    outcome: bool = False
    unknown: bool = False
    settling: bool = False
    cancel_requested: bool = False


class SceneClient:
    """
    Planning-scene client interface.
    """

    def __init__(
        self,
        node: Node,
        callback_group: Optional[CallbackGroup] = None,
        get_planning_scene_service: Optional[Any] = None,
        apply_planning_scene_service: Optional[Any] = None,
        owns_clients: Optional[bool] = None,
        get_client: Optional[Callable[[], Any]] = None,
        apply_client: Optional[Callable[[], Any]] = None,
    ) -> None:
        self._node = node
        self._callback_group = callback_group
        self._logger = getattr(node, "get_logger", lambda: None)()
        self._get_client_getter = get_client
        self._apply_client_getter = apply_client

        self._owns_get_client = (
            get_planning_scene_service is None and get_client is None
        )
        self._owns_apply_client = (
            apply_planning_scene_service is None and apply_client is None
        )
        if owns_clients is not None:
            self._owns_get_client = bool(owns_clients)
            self._owns_apply_client = bool(owns_clients)

        created_get_client = False
        if get_planning_scene_service is None and get_client is None:
            get_planning_scene_service = node.create_client(
                srv_type=GetPlanningScene,
                srv_name="get_planning_scene",
                qos_profile=_reliable_qos(1),
                callback_group=callback_group,
            )
            created_get_client = True
        try:
            if apply_planning_scene_service is None and apply_client is None:
                apply_planning_scene_service = node.create_client(
                    srv_type=ApplyPlanningScene,
                    srv_name="apply_planning_scene",
                    qos_profile=_reliable_qos(1),
                    callback_group=callback_group,
                )
        except Exception:
            if created_get_client:
                self._destroy_client(get_planning_scene_service)
            raise

        self._get_planning_scene_service = get_planning_scene_service
        self._apply_planning_scene_service = apply_planning_scene_service

        self._state_lock = threading.Lock()
        self._cache_lock = threading.Lock()
        self._closed = False
        self._mutation_active = False
        self._mutation_token: Optional[object] = None
        self._quarantined = False

        self._planning_scene: Optional[PlanningScene] = None
        self._known_components = 0
        self._dirty_components = 0
        self._cache_epoch = 0
        self._read_epoch = 0
        self._latest_read_epoch = 0
        self._read_waiters: set[threading.Event] = set()

        self._pending: weakref.WeakKeyDictionary[Any, _Mutation] = (
            weakref.WeakKeyDictionary()
        )

    def destroy(self) -> None:
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
            self._mutation_active = False
            self._mutation_token = None
            self._pending.clear()
            for waiter in self._read_waiters:
                waiter.set()
            self._read_waiters.clear()

        if self._owns_get_client:
            self._destroy_client(self._get_planning_scene_service)
        if self._owns_apply_client:
            self._destroy_client(self._apply_planning_scene_service)

    def __enter__(self) -> "SceneClient":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.destroy()

    @property
    def planning_scene(self) -> Optional[PlanningScene]:
        with self._cache_lock:
            return copy.deepcopy(self._planning_scene)

    @property
    def scene_cache_dirty(self) -> bool:
        with self._cache_lock:
            return bool(self._dirty_components)

    @property
    def mutation_quarantined(self) -> bool:
        with self._state_lock:
            return self._quarantined

    def update_planning_scene(self, timeout_sec: Optional[float] = 1.0) -> bool:
        deadline = _Deadline(timeout_sec)
        result = self._read_scene(ALL_COMPONENTS, deadline)
        return result is not None

    def _get_client(self) -> Any:
        if self._get_client_getter is not None:
            return (
                self._get_client_getter()
                if callable(self._get_client_getter)
                else self._get_client_getter
            )
        return self._get_planning_scene_service

    def _apply_client(self) -> Any:
        if self._apply_client_getter is not None:
            return (
                self._apply_client_getter()
                if callable(self._apply_client_getter)
                else self._apply_client_getter
            )
        return self._apply_planning_scene_service

    def _read_scene(
        self, component_mask: int, deadline: _Deadline
    ) -> Optional[PlanningScene]:
        with self._state_lock:
            if self._closed:
                return None
            self._read_epoch += 1
            read_epoch = self._read_epoch
            self._latest_read_epoch = read_epoch
            with self._cache_lock:
                cache_epoch = self._cache_epoch

        try:
            get_client = self._get_client()
        except Exception as err:
            self._warn(f"Could not resolve the planning-scene client: {err}")
            return None
        if not self._wait_for_service(
            get_client, deadline.remaining(DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC)
        ):
            return None
        if deadline.expired():
            return None

        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(components=int(component_mask))
        try:
            with self._state_lock:
                if self._closed or deadline.expired():
                    return None
                future = get_client.call_async(request)
        except Exception as err:
            self._warn(f"Could not request the planning scene: {err}")
            return None

        if not self._wait_for_future(future, deadline.remaining(), get_client):
            self._warn("Timed out while waiting for the planning scene future.")
            return None
        if deadline.expired():
            self._warn("Timed out while waiting for the planning scene future.")
            return None

        try:
            response = future.result()
        except Exception as err:
            self._warn(f"Planning scene request failed: {err}")
            return None
        scene = getattr(response, "scene", None)
        if scene is None:
            self._warn("Planning scene request returned no scene.")
            return None

        with self._state_lock:
            if self._closed:
                return None
            if read_epoch != self._latest_read_epoch:
                return None
            with self._cache_lock:
                if self._cache_epoch != cache_epoch:
                    return None
                self._merge_read_locked(scene, int(component_mask))
        return scene

    def _merge_read_locked(self, scene: PlanningScene, component_mask: int) -> None:
        if self._planning_scene is None:
            self._planning_scene = copy.deepcopy(scene)
            if component_mask != ALL_COMPONENTS:
                self._dirty_components |= ALL_COMPONENTS & ~int(component_mask)
        elif component_mask == ALL_COMPONENTS:
            self._planning_scene = copy.deepcopy(scene)
        else:
            self._copy_components_locked(scene, component_mask)
            if component_mask & WORLD_OBJECT_NAMES and not (
                component_mask & WORLD_OBJECT_GEOMETRY
            ):
                self._dirty_components |= WORLD_OBJECT_GEOMETRY
        self._known_components |= int(component_mask)
        self._dirty_components &= ~int(component_mask)
        self._cache_epoch += 1

    def _admit_mutation(self, kind: str) -> Optional[object]:
        with self._state_lock:
            if self._closed:
                return None
            if self._quarantined:
                self._warn(
                    f"Refusing {kind}: an earlier planning-scene apply has unknown outcome."
                )
                return None
            if self._mutation_active:
                self._warn(f"Refusing {kind}: another planning-scene mutation is busy.")
                return None
            token = object()
            self._mutation_active = True
            self._mutation_token = token
            return token

    def _release_mutation(self, token: object, *, quarantine: bool = False) -> None:
        with self._state_lock:
            if self._mutation_token is not token:
                return
            self._mutation_active = False
            self._mutation_token = None
            if quarantine:
                self._quarantined = True

    def allow_collisions(
        self, id: str, allow: bool, timeout_sec: Optional[float] = 1.0
    ) -> Optional[Future]:
        deadline = _Deadline(timeout_sec)
        token = self._admit_mutation("allow_collisions")
        if token is None:
            return None
        try:
            fetched = self._read_scene(ALLOWED_COLLISION_MATRIX, deadline)
            if fetched is None:
                self._release_mutation(token)
                return None
            acm = copy.deepcopy(fetched.allowed_collision_matrix)
            self._set_collision_permission(acm, id, bool(allow))
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.allowed_collision_matrix = acm
            return self._apply(
                token,
                "allow_collisions",
                scene,
                deadline,
                acm=acm,
            )
        except Exception:
            self._release_mutation(token)
            raise

    @staticmethod
    def _set_collision_permission(acm: Any, object_id: str, allow: bool) -> None:
        names = list(acm.entry_names)
        rows = [list(entry.enabled) for entry in acm.entry_values]
        count = len(names)
        while len(rows) < count:
            rows.append([False] * count)
        rows = [(row + [False] * count)[:count] for row in rows[:count]]

        try:
            column = names.index(object_id)
        except ValueError:
            column = count
            names.append(object_id)
            for row in rows:
                row.append(bool(allow))
            rows.append([bool(allow)] * (count + 1))
            count += 1

        for row_index in range(count):
            if row_index != column:
                rows[row_index][column] = bool(allow)
        rows[column] = [bool(allow)] * count

        acm.entry_names = names
        acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in rows]

    def clear_all_collision_objects(
        self, timeout_sec: Optional[float] = 1.0
    ) -> Optional[Future]:
        deadline = _Deadline(timeout_sec)
        token = self._admit_mutation("clear_all_collision_objects")
        if token is None:
            return None
        mask = WORLD_OBJECT_NAMES | ROBOT_STATE_ATTACHED_OBJECTS
        try:
            fetched = self._read_scene(mask, deadline)
            if fetched is None:
                self._release_mutation(token)
                return None

            world_ids = frozenset(
                str(obj.id) for obj in fetched.world.collision_objects if obj.id
            )
            attached_ids = frozenset(
                str(obj.object.id)
                for obj in fetched.robot_state.attached_collision_objects
                if obj.object.id
            )
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            scene.world.collision_objects = [
                CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
                for object_id in sorted(world_ids)
            ]
            scene.robot_state.attached_collision_objects = [
                AttachedCollisionObject(
                    object=CollisionObject(
                        id=object_id, operation=CollisionObject.REMOVE
                    )
                )
                for object_id in sorted(attached_ids)
            ]
            return self._apply(
                token,
                "clear_all_collision_objects",
                scene,
                deadline,
                observed_world_ids=world_ids,
                observed_attached_ids=attached_ids,
            )
        except Exception:
            self._release_mutation(token)
            raise

    def _apply(
        self,
        token: object,
        kind: str,
        scene: PlanningScene,
        deadline: _Deadline,
        *,
        observed_world_ids: FrozenSet[str] = frozenset(),
        observed_attached_ids: FrozenSet[str] = frozenset(),
        acm: Optional[Any] = None,
    ) -> Optional[Future]:
        try:
            apply_client = self._apply_client()
        except Exception as err:
            self._warn(f"Could not resolve the apply planning-scene client: {err}")
            self._release_mutation(token)
            return None
        if not self._wait_for_service(
            apply_client, deadline.remaining(DEFAULT_WAIT_FOR_SERVER_TIMEOUT_SEC)
        ):
            self._release_mutation(token)
            return None
        if deadline.expired():
            self._release_mutation(token)
            return None

        request = ApplyPlanningScene.Request()
        request.scene = scene
        try:
            with self._state_lock:
                if self._closed or self._mutation_token is not token:
                    return None
                if deadline.expired():
                    self._mutation_active = False
                    self._mutation_token = None
                    return None
                future = apply_client.call_async(request)
        except Exception as err:
            self._warn(f"Could not apply the planning scene: {err}")
            with self._state_lock:
                if self._mutation_token is token:
                    self._mutation_active = False
                    self._mutation_token = None
                    if not self._closed:
                        self._quarantined = True
            return None

        record = _Mutation(
            token=token,
            kind=kind,
            observed_world_ids=observed_world_ids,
            observed_attached_ids=observed_attached_ids,
            acm=copy.deepcopy(acm),
        )
        with self._state_lock:
            if self._closed:
                return future
            try:
                self._pending[future] = record
            except TypeError:
                self._warn("Apply future cannot be weakly associated; quarantining.")
                self._mutation_active = False
                self._mutation_token = None
                self._quarantined = True
                return future

        try:
            future.add_done_callback(
                lambda done_future: self._on_apply_done(done_future, record)
            )
        except Exception as err:
            self._warn(f"Could not observe planning-scene apply future: {err}")
            self._mark_unknown(record)
        return future

    def _on_apply_done(self, future: Any, record: _Mutation) -> None:
        with self._state_lock:
            if record.settled or record.settling:
                return
            record.settling = True
            closed = self._closed

        if closed:
            with self._state_lock:
                record.settled = True
                record.settling = False
                self._discard_mutation_payload(record)
            return

        outcome, unknown = self._apply_outcome(future, record)

        with self._state_lock:
            if self._closed:
                record.settled = True
                record.settling = False
                self._discard_mutation_payload(record)
                return
            if outcome and not unknown:
                with self._cache_lock:
                    self._commit_mutation_locked(record)
            record.outcome = bool(outcome and not unknown)
            record.unknown = unknown
            record.settled = True
            record.settling = False
            if unknown:
                self._quarantined = True
            elif record.cancel_requested:
                self._quarantined = False
            if self._mutation_token is record.token:
                self._mutation_active = False
                self._mutation_token = None
            self._discard_mutation_payload(record)

    @staticmethod
    def _apply_outcome(future: Any, record: _Mutation) -> Tuple[bool, bool]:
        try:
            if bool(getattr(future, "cancelled", lambda: False)()):
                return False, True
            response = future.result()
        except Exception:
            return False, True
        if response is None or not hasattr(response, "success"):
            return False, True
        return bool(response.success), False

    def _commit_mutation_locked(self, record: _Mutation) -> None:
        if record.kind == "allow_collisions":
            if record.acm is None:
                return
            if self._planning_scene is None:
                self._planning_scene = PlanningScene()
            self._planning_scene.allowed_collision_matrix = copy.deepcopy(record.acm)
            self._known_components |= ALLOWED_COLLISION_MATRIX
            self._dirty_components &= ~ALLOWED_COLLISION_MATRIX
            self._cache_epoch += 1
            return

        if self._planning_scene is None:
            self._planning_scene = PlanningScene()
        world = self._planning_scene.world.collision_objects
        self._planning_scene.world.collision_objects = [
            obj for obj in world if obj.id not in record.observed_world_ids
        ]
        attached = self._planning_scene.robot_state.attached_collision_objects
        self._planning_scene.robot_state.attached_collision_objects = [
            obj for obj in attached if obj.object.id not in record.observed_attached_ids
        ]
        required = WORLD_OBJECT_NAMES | ROBOT_STATE_ATTACHED_OBJECTS
        self._dirty_components |= required
        self._known_components |= required
        self._cache_epoch += 1

    def _mark_unknown(self, record: _Mutation) -> None:
        with self._state_lock:
            if record.settled:
                return
            record.settled = True
            record.settling = False
            record.unknown = True
            record.outcome = False
            self._quarantined = True
            if self._mutation_token is record.token:
                self._mutation_active = False
                self._mutation_token = None
            self._discard_mutation_payload(record)

    @staticmethod
    def _discard_mutation_payload(record: _Mutation) -> None:
        record.observed_world_ids = frozenset()
        record.observed_attached_ids = frozenset()
        record.acm = None

    def process_apply_planning_scene_future(self, future: Any) -> bool:
        with self._state_lock:
            if self._closed:
                return False
            try:
                record = self._pending.get(future)
            except (TypeError, KeyError):
                return False
        if record is None or not bool(getattr(future, "done", lambda: False)()):
            return False
        self._on_apply_done(future, record)
        with self._state_lock:
            return bool(record.settled and record.outcome and not record.unknown)

    def process_allow_collision_future(self, future: Any) -> bool:
        return self.process_apply_planning_scene_future(future)

    def process_clear_all_collision_objects_future(self, future: Any) -> bool:
        return self.process_apply_planning_scene_future(future)

    def cancel_apply_planning_scene_future(self, future: Any) -> None:
        with self._state_lock:
            try:
                record = self._pending.get(future)
            except (TypeError, KeyError):
                return
            if record is None or record.settled:
                return
            record.cancel_requested = True
            self._quarantined = True
            if self._mutation_token is record.token:
                self._mutation_active = False
                self._mutation_token = None
        try:
            self._apply_client().remove_pending_request(future)
        except Exception as err:
            self._warn(f"Could not remove pending planning-scene request: {err}")

    def cancel_clear_all_collision_objects_future(self, future: Any) -> None:
        self.cancel_apply_planning_scene_future(future)

    def cancel_allow_collision_future(self, future: Any) -> None:
        self.cancel_apply_planning_scene_future(future)

    def _copy_components_locked(
        self, scene: PlanningScene, component_mask: int
    ) -> None:
        if component_mask & (WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY):
            self._planning_scene.world.collision_objects = copy.deepcopy(
                scene.world.collision_objects
            )
        if component_mask & OCTOMAP:
            self._planning_scene.world.octomap = copy.deepcopy(scene.world.octomap)
        if component_mask & ROBOT_STATE_ATTACHED_OBJECTS:
            self._planning_scene.robot_state.attached_collision_objects = copy.deepcopy(
                scene.robot_state.attached_collision_objects
            )
        if component_mask & ALLOWED_COLLISION_MATRIX:
            self._planning_scene.allowed_collision_matrix = copy.deepcopy(
                scene.allowed_collision_matrix
            )

    def _wait_for_future(
        self, future: Any, timeout_sec: Optional[float], client: Any
    ) -> bool:
        if bool(getattr(future, "done", lambda: False)()):
            return True
        done = threading.Event()
        with self._state_lock:
            if self._closed:
                close_before_wait = True
            else:
                close_before_wait = False
                self._read_waiters.add(done)
        if close_before_wait:
            self._remove_pending_request(client, future)
            return False
        try:
            future.add_done_callback(lambda _: done.set())
        except Exception:
            with self._state_lock:
                self._read_waiters.discard(done)
            self._remove_pending_request(client, future)
            return False
        try:
            done.wait(timeout=timeout_sec)
        finally:
            with self._state_lock:
                self._read_waiters.discard(done)
        if bool(getattr(future, "done", lambda: False)()):
            return True
        self._remove_pending_request(client, future)
        return False

    def _wait_for_service(self, client: Any, timeout_sec: Optional[float]) -> bool:
        if client is None:
            return False
        deadline = (
            None
            if timeout_sec is None
            else time.monotonic() + max(0.0, float(timeout_sec))
        )
        try:
            while not self._is_closed():
                if deadline is None:
                    wait_slice = 0.05
                else:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        wait_slice = 0.0
                    else:
                        wait_slice = min(0.05, remaining)
                try:
                    ready = client.wait_for_service(timeout_sec=wait_slice)
                except TypeError:
                    ready = client.wait_for_service(wait_slice)
                if ready:
                    return True
                if deadline is not None and wait_slice == 0.0:
                    return False
            return False
        except Exception:
            return False

    def _is_closed(self) -> bool:
        with self._state_lock:
            return self._closed

    @staticmethod
    def _remove_pending_request(client: Any, future: Any) -> None:
        try:
            client.remove_pending_request(future)
        except Exception:
            pass

    def _destroy_client(self, client: Any) -> None:
        try:
            self._node.destroy_client(client)
        except Exception:
            destroy = getattr(client, "destroy", None)
            if destroy is not None:
                try:
                    destroy()
                except Exception:
                    pass

    def _warn(self, message: str) -> None:
        warning = getattr(self._logger, "warning", None)
        if warning is not None:
            try:
                warning(message)
            except Exception:
                pass


__all__ = [
    "ALL_COMPONENTS",
    "ALLOWED_COLLISION_MATRIX",
    "ROBOT_STATE_ATTACHED_OBJECTS",
    "WORLD_OBJECT_NAMES",
    "SceneClient",
]
