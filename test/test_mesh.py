from pathlib import Path

import numpy as np
import pytest

try:
    import trimesh
except Exception as err:
    trimesh = None
    _TRIMESH_ERROR = str(err)
else:
    _TRIMESH_ERROR = ""

pytestmark = pytest.mark.skipif(
    trimesh is None, reason=f"trimesh unavailable: {_TRIMESH_ERROR}"
)

ASSET = Path(__file__).parent.parent / "examples" / "assets" / "suzanne.stl"


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture()
def mesh_publisher(moveit2):
    publisher = RecordingPublisher()
    moveit2._MoveIt2__collision_object_publisher = publisher
    return publisher


def test_add_collision_mesh_from_file(moveit2, mesh_publisher):
    moveit2.add_collision_mesh(
        filepath=str(ASSET),
        id="suzanne",
        position=(0.5, 0.0, 0.5),
        quat_xyzw=(0, 0, 0, 1),
    )
    message = mesh_publisher.messages[0]
    assert message.id == "suzanne"
    assert message.header.frame_id == "panda_link0"
    assert len(message.meshes[0].triangles) == len(trimesh.load(str(ASSET)).faces)


def test_add_collision_mesh_accepts_scene_and_scales(moveit2, mesh_publisher):
    box = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    scene = trimesh.Scene([box, trimesh.creation.box(extents=(0.5, 0.5, 0.5))])
    moveit2.add_collision_mesh(
        filepath=None,
        id="scene",
        mesh=scene,
        position=(0.0, 0.0, 0.0),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        scale=2,
    )
    message = mesh_publisher.messages[0]
    assert len(message.meshes[0].triangles) == 2 * len(box.faces)
    xs = [v.x for v in message.meshes[0].vertices]
    assert pytest.approx(max(xs)) == 1.0

    assert pytest.approx(box.extents.max()) == 1.0


def test_add_collision_mesh_anisotropic_scale(moveit2, mesh_publisher):
    box = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
    moveit2.add_collision_mesh(
        filepath=None,
        id="box",
        mesh=box,
        position=(0.0, 0.0, 0.0),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        scale=(1.0, 2.0, 3.0),
    )
    vertices = np.array(
        [[v.x, v.y, v.z] for v in mesh_publisher.messages[0].meshes[0].vertices]
    )
    assert np.allclose(vertices.max(axis=0), [0.5, 1.0, 1.5])
    assert np.allclose(box.vertices.max(axis=0), [0.5, 0.5, 0.5])


def test_add_collision_mesh_validation(moveit2, mesh_publisher):
    with pytest.raises(ValueError):
        moveit2.add_collision_mesh(
            filepath=None, id="x", position=(0, 0, 0), quat_xyzw=(0, 0, 0, 1)
        )
    with pytest.raises(ValueError):
        moveit2.add_collision_mesh(
            filepath=str(ASSET),
            id="x",
            mesh=trimesh.creation.box(),
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
        )
    with pytest.raises(ValueError):
        moveit2.add_collision_mesh(
            filepath=None,
            id="x",
            mesh="not a mesh",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
        )
    with pytest.raises(ValueError):
        moveit2.add_collision_mesh(
            filepath=str(ASSET),
            id="x",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
            scale=(1.0, 2.0),
        )
    assert mesh_publisher.messages == []


@pytest.mark.parametrize("limits", [{"max_vertices": 1}, {"max_faces": 1}])
def test_mesh_count_limits_precede_ros_conversion(moveit2, mesh_publisher, limits):
    with pytest.raises(ValueError, match="exceeds"):
        moveit2.add_collision_mesh(
            filepath=None,
            mesh=trimesh.creation.box(),
            id="bounded",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
            **limits,
        )
    assert not mesh_publisher.messages


def test_mesh_file_limit_precedes_decoder(moveit2, mesh_publisher, monkeypatch):
    def must_not_load(*args, **kwargs):
        pytest.fail("file limit must be checked before decoder allocation")

    monkeypatch.setattr(trimesh, "load", must_not_load)
    with pytest.raises(ValueError, match="max_file_bytes"):
        moveit2.add_collision_mesh(
            filepath=str(ASSET),
            id="bounded",
            max_file_bytes=1,
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
        )
    assert not mesh_publisher.messages


@pytest.mark.parametrize(
    "scale", [float("nan"), (1.0, float("inf"), 1.0), (1.0, 0.0, 1.0), None]
)
def test_mesh_scale_validation(moveit2, mesh_publisher, scale):
    with pytest.raises(ValueError):
        moveit2.add_collision_mesh(
            filepath=None,
            mesh=trimesh.creation.box(),
            id="invalid",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
            scale=scale,
        )
    assert not mesh_publisher.messages


def test_mesh_nonfinite_vertices_and_bad_indices(moveit2, mesh_publisher):
    mesh = trimesh.Trimesh(
        vertices=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, float("nan"), 0.0]],
        faces=[[0, 1, 2]],
        process=False,
    )
    with pytest.raises(ValueError, match="finite"):
        moveit2.add_collision_mesh(
            filepath=None,
            mesh=mesh,
            id="invalid",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
        )
    mesh = trimesh.Trimesh(
        vertices=[[0.0, 0.0, 0.0]] * 3, faces=[[0, 1, 4]], process=False
    )
    with pytest.raises(ValueError, match="indices"):
        moveit2.add_collision_mesh(
            filepath=None,
            mesh=mesh,
            id="invalid",
            position=(0, 0, 0),
            quat_xyzw=(0, 0, 0, 1),
        )
    assert not mesh_publisher.messages


def test_mesh_reflection_matches_trimesh_winding(moveit2, mesh_publisher):
    mesh = trimesh.creation.box()
    expected = mesh.copy()
    expected.apply_transform(np.diag([-1.0, 1.0, 1.0, 1.0]))
    moveit2.add_collision_mesh(
        filepath=None,
        mesh=mesh,
        id="reflected",
        scale=(-1.0, 1.0, 1.0),
        position=(0, 0, 0),
        quat_xyzw=(0, 0, 0, 1),
    )
    actual = mesh_publisher.messages[0].meshes[0]
    assert np.array_equal(
        [list(t.vertex_indices) for t in actual.triangles], expected.faces
    )
    assert np.allclose([[v.x, v.y, v.z] for v in actual.vertices], expected.vertices)
