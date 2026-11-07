"""Conservative collision hulls for the RevoTron arms and central body.

Each hull encloses one URDF link's collision surfaces. Cavities are filled;
these are collision proxies, not claims about the exact manufactured surface.
"""

import numpy as np
import trimesh

from .body_envelope import CentralBodyEnvelope
from .hand import origin


ARM_LINKS = tuple(
    name
    for side in ("L", "R")
    for name in [f"Base_{side}"] + [f"Link{i}_{side}" for i in range(1, 8)]
)
ROBOT_BODY_LINKS = CentralBodyEnvelope.LINKS + ARM_LINKS


def load_robot_hulls(robot):
    """Keep every hull in its link frame so kinematic bodies follow FK."""
    hulls = {}
    for name in ROBOT_BODY_LINKS:
        link = robot.xml.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError(f"Missing robot collision link: {name}")
        vertices = []
        for collision in link.findall("collision"):
            spec = collision.find("geometry/mesh")
            if spec is None:
                raise ValueError(f"Expected robot collision mesh: {name}")
            mesh = trimesh.load_mesh(robot.path.parent / spec.get("filename"))
            mesh.apply_scale(np.fromstring(spec.get("scale", "1 1 1"), sep=" "))
            mesh.apply_transform(origin(collision.find("origin")))
            vertices.extend(mesh.vertices)
        if not vertices:
            raise ValueError(f"Missing robot collision surface: {name}")
        hulls[name] = trimesh.convex.convex_hull(np.asarray(vertices))
    return hulls


def hull_planes(hull):
    """Unit outward halfspaces, with coplanar triangle duplicates removed."""
    planes = np.c_[
        hull.face_normals,
        -np.einsum("ij,ij->i", hull.face_normals, hull.triangles[:, 0]),
    ]
    _, indices = np.unique(np.round(planes, 9), axis=0, return_index=True)
    return planes[np.sort(indices)]


def edges_enter_hull(points, planes, inset=1e-7):
    """Clip closed-loop centerline edges against inward-offset halfspaces.

    This detects edges crossing a convex solid even if both endpoints are
    outside. It does not measure rope-radius clearance or swept collisions.
    A positive inset excludes boundary tangencies and numerical surface noise.
    """
    points, planes = np.asarray(points), np.asarray(planes)
    if (
        points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
        or planes.ndim != 2 or planes.shape[1] != 4 or not len(planes)
        or not np.isfinite(points).all() or not np.isfinite(planes).all()
        or not np.isfinite(inset) or inset < 0
        or not np.allclose(np.linalg.norm(planes[:, :3], axis=1), 1)
    ):
        raise ValueError("Invalid hull clipping geometry")
    distance = points @ planes[:, :3].T + planes[:, 3] + inset
    slope = np.roll(distance, -1, axis=0) - distance
    parallel = np.abs(slope) < 1e-14
    crossing = -distance / np.where(parallel, 1, slope)
    lower = np.maximum(0, np.where(slope < -1e-14, crossing, -np.inf).max(axis=1))
    upper = np.minimum(1, np.where(slope > 1e-14, crossing, np.inf).min(axis=1))
    excluded = np.any(parallel & (distance > 0), axis=1)
    return (~excluded) & (lower < upper)


class RobotHullAudit:
    """Check complete rope centerlines against all arm and body hulls."""

    def __init__(self, robot):
        self.hulls = {
            name: (hull.bounds, hull_planes(hull))
            for name, hull in load_robot_hulls(robot).items()
        }

    def measure(self, transforms, points, inset=1e-7):
        hits = {}
        for name, (bounds, planes) in self.hulls.items():
            tf = transforms[name]
            local = (points - tf[:3, 3]) @ tf[:3, :3]
            # Most frames are far from most links; prune only a whole loop
            # that lies beyond one separating AABB face.
            if np.any(local.max(axis=0) < bounds[0]) or np.any(local.min(axis=0) > bounds[1]):
                continue
            edges = np.flatnonzero(edges_enter_hull(local, planes, inset))
            if len(edges):
                hits[name] = edges.tolist()
        return hits
