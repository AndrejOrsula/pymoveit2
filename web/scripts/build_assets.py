"""Build the robot and Suzanne assets of the web demo from pinned upstream meshes.

Usage:
  python3 web/scripts/build_assets.py --licenses-only  # fetch pinned license files
  uv run --no-project --python 3.12 --with trimesh==4.8.3 --with pycollada==0.9.3 \
    --with fast-simplification==0.2.0 --with scipy==1.18.1 --with networkx==3.7 \
    --with lxml==6.1.3 --with numpy==2.5.4 python -I web/scripts/build_assets.py

It sparse-clones franka_description at FRANKA_COMMIT and open_manipulator-RMinS
at RMINS_COMMIT into a temporary directory. For the Franka Emika Robot (Panda) it
merges each visual mesh by color; for the OpenMANIPULATOR-X it scales the STL
meshes from millimeters and applies the URDF visual origins, leaving out the
course's mounting platform. It decimates each mesh and writes one GLB per link
to web/static/assets/<robot>/. The GLBs keep the link frames of the URDF
(meters) and carry no normals; the viewer computes creased normals. It also
fits collision spheres to each collision mesh, and to
examples/assets/suzanne.stl, and writes them to web/src/core/geometry.js.
Repeated runs write identical files.
"""

import json
import pathlib
import subprocess
import sys
import tempfile
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import fast_simplification
    import numpy as np
    import trimesh
    from scipy.cluster.vq import kmeans2

FRANKA_REPOSITORY = "https://github.com/frankarobotics/franka_description.git"
FRANKA_COMMIT = "7aeeddc449edf8d62b594f9e36a81da53e7796f9"
MESH_DIR = "meshes/robots/fer"
HAND_DIR = "meshes/robot_ee/franka_hand_white"
KEEP_RATIO = 0.25
RMINS_REPOSITORY = "https://github.com/snt-spacer/open_manipulator-RMinS.git"
RMINS_COMMIT = "5a2e3de9a7ebc019bad3a3849396545638dd4551"
OMX_DIR = "open_manipulator_description/meshes/open_manipulator_x"
OMX_KEEP_RATIO = 0.5
# Some meshes hold many sub-millimeter triangles that block decimation, so their
# vertices snap to this grid (meters) first.
OMX_GRID = 0.0003
# Mesh and URDF visual origin (z, meters) per link; the meshes are in millimeters.
OMX_LINKS = {
    "link1": ("link1", 0.0),
    "link2": ("link2", 0.019),
    "link3": ("link3", 0.0),
    "link4": ("link4", 0.0),
    "link5": ("link5", 0.0),
    "gripper_left_link": ("gripper_left_palm", 0.0),
    "gripper_right_link": ("gripper_right_palm", 0.0),
}
# Spheres per arm link; like the Panda's, the fingers carry none in the demo.
OMX_SPHERES = {"link1": 8, "link2": 6, "link3": 10, "link4": 10, "link5": 8}
LINKS = ["link0", "link1", "link2", "link3", "link4", "link5", "link6", "link7"]
# Spheres per collision mesh; the fingers have no collision mesh upstream.
SPHERES = {
    "link0": 10,
    "link1": 12,
    "link2": 12,
    "link3": 12,
    "link4": 12,
    "link5": 14,
    "link6": 10,
    "link7": 8,
    "hand": 10,
}
SUZANNE_SPHERES = 10
SAMPLES = 4000

WEB = pathlib.Path(__file__).resolve().parent.parent
REPO = WEB.parent
OUT_MESHES = WEB / "static" / "assets"
OUT_GEOMETRY = WEB / "src" / "core" / "geometry.js"
OUT_LICENSES = WEB / "static" / "licenses"
LICENSE_SOURCES = (
    ("franka_description", FRANKA_REPOSITORY, FRANKA_COMMIT, ("LICENSE", "NOTICE")),
    ("open_manipulator-RMinS", RMINS_REPOSITORY, RMINS_COMMIT, ("LICENSE",)),
)


def color_name(rgba):
    rgb = tuple(int(round(float(x))) for x in rgba[:3])
    if min(rgb) >= 200:
        return "body"
    if max(rgb) <= 80:
        return "dark"
    return "accent_%02x%02x%02x" % rgb


def material_color(geometry):
    material = getattr(geometry.visual, "material", None)
    for name in ("diffuse", "baseColorFactor", "main_color"):
        value = getattr(material, name, None)
        if value is not None:
            return value
    return (255, 255, 255, 255)


def decimate(mesh, keep):
    if len(mesh.faces) <= 400:
        return mesh
    vertices, faces = fast_simplification.simplify(
        mesh.vertices.astype(np.float32),
        mesh.faces.astype(np.int32),
        target_reduction=1.0 - keep,
    )
    return trimesh.Trimesh(vertices, faces, process=True)


def to_glb(parts_by_color, keep=KEEP_RATIO):
    scene = trimesh.Scene()
    for name in sorted(parts_by_color):
        mesh = trimesh.util.concatenate(parts_by_color[name])
        mesh.merge_vertices()
        mesh = decimate(mesh, keep)
        mesh.vertices = np.round(mesh.vertices, 5)
        stripped = trimesh.Trimesh(mesh.vertices, mesh.faces, process=False)
        scene.add_geometry(stripped, geom_name=name, node_name=name)
    return scene.export(file_type="glb", include_normals=False)


def convert_visual(path):
    scene = trimesh.load(path, force="scene")
    parts = {}
    for node in sorted(scene.graph.nodes_geometry):
        transform, geometry_name = scene.graph[node]
        geometry = scene.geometry[geometry_name]
        mesh = trimesh.Trimesh(geometry.vertices, geometry.faces, process=False)
        mesh.apply_transform(transform)
        parts.setdefault(color_name(material_color(geometry)), []).append(mesh)
    return to_glb(parts)


def fit_spheres(mesh, count, seed):
    """Spheres around k-means centers of the solid (its convex hull), each covering
    the surface points nearest to it, so together they cover the whole surface."""
    surface, _ = trimesh.sample.sample_surface_even(mesh, SAMPLES, seed=seed)
    rng = np.random.default_rng(seed)
    low, high = mesh.bounds
    candidates = rng.uniform(low, high, size=(SAMPLES * 4, 3))
    hull = mesh.convex_hull
    offsets = np.einsum("ij,ij->i", hull.face_normals, hull.triangles[:, 0])
    inside = np.all(candidates @ hull.face_normals.T - offsets <= 0.0, axis=1)
    solid = candidates[inside]
    if len(solid) < count * 10:
        solid = surface
    centers, _ = kmeans2(solid, count, seed=rng, minit="++")
    nearest = np.argmin(
        np.linalg.norm(surface[:, None, :] - centers[None, :, :], axis=2), axis=1
    )
    spheres = []
    for index in range(count):
        members = surface[nearest == index]
        if len(members) == 0:
            continue
        radius = float(np.max(np.linalg.norm(members - centers[index], axis=1)))
        spheres.append([*np.round(centers[index], 4).tolist(), round(radius, 4)])
    return sorted(spheres)


def clone(destination, repository, commit, paths):
    subprocess.run(
        ["git", "init", "-q", str(destination)],
        check=True,
    )
    for args in (
        ["remote", "add", "origin", repository],
        ["config", "core.sparseCheckout", "true"],
    ):
        subprocess.run(["git", "-C", str(destination), *args], check=True)
    sparse = destination / ".git" / "info" / "sparse-checkout"
    sparse.write_text("".join(f"/{path}\n" for path in paths))
    subprocess.run(
        ["git", "-C", str(destination), "fetch", "-q", "--depth", "1", "origin"]
        + [commit],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(destination), "checkout", "-q", "FETCH_HEAD"], check=True
    )


def build_licenses(work):
    """Copy the exact license and notice files from the pinned asset sources."""
    work = pathlib.Path(work)
    work.mkdir(parents=True, exist_ok=True)
    for name, repository, commit, files in LICENSE_SOURCES:
        source = work / name
        clone(source, repository, commit, list(files))
        target = OUT_LICENSES / name
        target.mkdir(parents=True, exist_ok=True)
        for filename in files:
            data = (source / filename).read_bytes()
            (target / filename).write_bytes(data)
            print(f"licenses/{name}/{filename}: {len(data)} bytes", file=sys.stderr)
    provenance = [
        "Generated by web/scripts/build_assets.py; upstream files copied verbatim.",
        f"https://github.com/frankarobotics/franka_description/tree/{FRANKA_COMMIT}",
        f"https://github.com/snt-spacer/open_manipulator-RMinS/tree/{RMINS_COMMIT}",
        "",
    ]
    (OUT_LICENSES / "SOURCES.txt").write_text("\n".join(provenance))


def omx_link(source, name, z):
    mesh = trimesh.load(source / OMX_DIR / f"{name}.stl", force="mesh")
    mesh.apply_scale(0.001)
    mesh.apply_translation([0.0, 0.0, z])
    return mesh


def snap(mesh):
    snapped = trimesh.Trimesh(np.round(mesh.vertices / OMX_GRID) * OMX_GRID, mesh.faces)
    snapped.update_faces(snapped.nondegenerate_faces())
    snapped.remove_unreferenced_vertices()
    return snapped


def build_omx(work):
    """GLBs and link spheres of the OpenMANIPULATOR-X arm."""
    out = OUT_MESHES / "open_manipulator_x"
    out.mkdir(parents=True, exist_ok=True)
    source = pathlib.Path(work) / "open_manipulator-RMinS"
    clone(source, RMINS_REPOSITORY, RMINS_COMMIT, [f"{OMX_DIR}/", "LICENSE"])
    meshes = {}
    for name, z in OMX_LINKS.values():
        meshes[name] = omx_link(source, name, z)
    for name, mesh in meshes.items():
        data = to_glb({"dark": [snap(mesh)]}, OMX_KEEP_RATIO)
        (out / f"{name}.glb").write_bytes(data)
        print(f"open_manipulator_x/{name}.glb: {len(data)} bytes", file=sys.stderr)
    return {
        link: fit_spheres(meshes[OMX_LINKS[link][0]], count, 100 + seed)
        for seed, (link, count) in enumerate(sorted(OMX_SPHERES.items()))
    }


def main():
    if sys.argv[1:] == ["--licenses-only"]:
        with tempfile.TemporaryDirectory() as work:
            build_licenses(work)
        return
    if sys.argv[1:]:
        raise SystemExit("usage: build_assets.py [--licenses-only]")

    import fast_simplification
    import numpy as np
    import trimesh
    from scipy.cluster.vq import kmeans2

    globals().update(
        fast_simplification=fast_simplification,
        np=np,
        trimesh=trimesh,
        kmeans2=kmeans2,
    )
    panda_out = OUT_MESHES / "panda"
    panda_out.mkdir(parents=True, exist_ok=True)
    collision = {}
    with tempfile.TemporaryDirectory() as work:
        build_licenses(pathlib.Path(work) / "licenses")
        omx_spheres = build_omx(work)
        source = pathlib.Path(work) / "franka_description"
        clone(
            source,
            FRANKA_REPOSITORY,
            FRANKA_COMMIT,
            [f"{MESH_DIR}/", f"{HAND_DIR}/", "LICENSE", "NOTICE"],
        )
        visual = source / MESH_DIR / "visual"
        for name in [*LINKS, "hand", "finger"]:
            data = convert_visual(visual / f"{name}.dae")
            (panda_out / f"{name}.glb").write_bytes(data)
            print(f"{name}.glb: {len(data)} bytes", file=sys.stderr)
        for seed, (name, count) in enumerate(sorted(SPHERES.items())):
            mesh = trimesh.load(source / MESH_DIR / "collision" / f"{name}.stl")
            collision[name] = fit_spheres(mesh, count, seed)

    suzanne = trimesh.load(REPO / "examples" / "assets" / "suzanne.stl")
    suzanne_glb = trimesh.Scene()
    suzanne_glb.add_geometry(
        trimesh.Trimesh(np.round(suzanne.vertices, 5), suzanne.faces, process=False),
        geom_name="suzanne",
        node_name="suzanne",
    )
    data = suzanne_glb.export(file_type="glb", include_normals=False)
    (OUT_MESHES / "suzanne.glb").write_bytes(data)
    suzanne_spheres = fit_spheres(suzanne, SUZANNE_SPHERES, 99)

    def table(name, links, entry=json.dumps):
        return [
            f"export const {name} = {{",
            *[
                f"  {link}: [\n"
                + "".join(f"    {entry(item)},\n" for item in items)
                + "  ],"
                for link, items in sorted(links.items())
            ],
            "};",
            "",
        ]

    lines = [
        "// Generated by web/scripts/build_assets.py; do not edit.",
        "// Collision spheres [x, y, z, radius] in link frames, fitted to the collision",
        f"// meshes of franka_description {FRANKA_COMMIT[:12]} (Apache-2.0), of",
        f"// open_manipulator-RMinS {RMINS_COMMIT[:12]} (Apache-2.0) and to",
        "// examples/assets/suzanne.stl (meters).",
        "",
        *table("LINK_SPHERES", collision),
        "export const SUZANNE_SPHERES = " + json.dumps(suzanne_spheres) + ";",
        "",
        *table("OMX_LINK_SPHERES", omx_spheres),
    ]
    OUT_GEOMETRY.parent.mkdir(parents=True, exist_ok=True)
    OUT_GEOMETRY.write_text("\n".join(lines))


if __name__ == "__main__":
    main()
