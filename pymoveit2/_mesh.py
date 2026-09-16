"""Helpers for mesh conversions."""

import os
from typing import Any, Optional

import numpy as np
from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle

from pymoveit2._validation import finite_vector


def mesh_message(
    trimesh: Any,
    filepath: Optional[str],
    mesh: Any,
    scale: Any,
    max_file_bytes: Optional[int],
    max_vertices: Optional[int],
    max_faces: Optional[int],
) -> Mesh:
    for name, limit in (
        ("max_file_bytes", max_file_bytes),
        ("max_vertices", max_vertices),
        ("max_faces", max_faces),
    ):
        if limit is not None and (
            isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0
        ):
            raise ValueError(f"`{name}` must be a positive integer or None.")
    if isinstance(scale, (int, float)):
        scale = (scale, scale, scale)
    scale = finite_vector(scale, "scale", 3)
    if any(value == 0.0 for value in scale):
        raise ValueError("Mesh scale must be nonzero on every axis.")
    if filepath is not None:
        if max_file_bytes is not None and os.path.getsize(filepath) > max_file_bytes:
            raise ValueError("Mesh file exceeds max_file_bytes.")
        mesh = trimesh.load(filepath)
    mesh = _as_single_trimesh(trimesh, mesh)
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices):
        raise ValueError("Mesh must contain three-dimensional vertices.")
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
        raise ValueError("Mesh must contain triangle faces.")
    if max_vertices is not None and len(vertices) > max_vertices:
        raise ValueError("Mesh exceeds max_vertices.")
    if max_faces is not None and len(faces) > max_faces:
        raise ValueError("Mesh exceeds max_faces.")
    if not np.isfinite(vertices).all():
        raise ValueError("Mesh vertices must be finite.")
    if (
        not np.issubdtype(faces.dtype, np.integer)
        or np.any(faces < 0)
        or np.any(faces >= len(vertices))
    ):
        raise ValueError("Mesh faces contain invalid vertex indices.")
    if np.prod(np.asarray(scale)) < 0:
        faces = faces[:, ::-1]
    with np.errstate(over="ignore", invalid="ignore"):
        vertices = vertices * np.asarray(scale)
    if not np.isfinite(vertices).all():
        raise ValueError("Scaled mesh vertices must be finite.")

    return Mesh(
        triangles=[
            MeshTriangle(vertex_indices=[int(v) for v in face]) for face in faces
        ],
        vertices=[
            Point(x=float(vert[0]), y=float(vert[1]), z=float(vert[2]))
            for vert in vertices
        ],
    )


def _as_single_trimesh(trimesh: Any, mesh: Any) -> Any:
    if isinstance(mesh, trimesh.Trimesh):
        return mesh
    if isinstance(mesh, trimesh.Scene):
        geometries = mesh.dump()
        if isinstance(geometries, trimesh.Trimesh):
            return geometries
        meshes = [g for g in geometries if isinstance(g, trimesh.Trimesh)]
        if not meshes:
            raise ValueError("The mesh scene does not contain any triangle mesh!")
        return trimesh.util.concatenate(meshes)
    if isinstance(mesh, (list, tuple)):
        meshes = [g for g in mesh if isinstance(g, trimesh.Trimesh)]
        if not meshes:
            raise ValueError("The mesh file does not contain any triangle mesh!")
        return trimesh.util.concatenate(meshes)
    raise ValueError(
        f"Unsupported mesh type '{type(mesh).__name__}'; expected a triangle mesh."
    )
