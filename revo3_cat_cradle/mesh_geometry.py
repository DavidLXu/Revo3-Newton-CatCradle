"""Exact triangle-surface diagnostics for optional URDF knuckle collisions."""

import numpy as np
import trimesh
from .hand import origin
from .bimanual_checks import gaps


def signed_surface_clearance(surface, points):
    """Positive outside, negative inside a closed triangle surface; meters."""
    if not surface.is_watertight:
        raise ValueError("Winding audit requires a watertight collision surface")
    _, distances, _ = trimesh.proximity.closest_point_naive(surface, points)
    result = []
    for point, distance in zip(points, distances):
        a, b, c = (surface.triangles - point).transpose(1, 0, 2)
        an, bn, cn = (np.linalg.norm(x, axis=1) for x in (a, b, c))
        dot = lambda x, y: np.einsum("ij,ij->i", x, y)
        numerator = dot(a, np.cross(b, c))
        denominator = an * bn * cn + dot(a, b) * cn + dot(b, c) * an + dot(c, a) * bn
        winding = np.sum(2 * np.arctan2(numerator, denominator)) / (4 * np.pi)
        result.append(-distance if abs(winding) > 0.5 else distance)
    return np.asarray(result)


def load_knuckle_meshes(robot):
    result = {}
    for side in ("left", "right"):
        for finger in ("index", "middle", "ring", "little"):
            name = f"{side}_{finger}_MPR_link"
            surfaces = []
            for collision in robot.xml.find(f"link[@name='{name}']").findall(
                "collision"
            ):
                mesh = collision.find("geometry/mesh")
                surface = trimesh.load_mesh(robot.path.parent / mesh.get("filename"))
                surface.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
                surface.apply_transform(origin(collision.find("origin")))
                surfaces.append(surface)
            result[name] = surfaces
    return result


def support_gap(transforms, points, side, finger, meshes, radius=0.001):
    """Minimum gap to capsules plus enabled MPR surfaces, without a hull proxy.

    AABB distance bounds prune only triangles that cannot improve the existing
    minimum. Interior candidates are never pruned based on unsigned distance.
    """
    best = float(gaps(transforms, points, side, finger).min())
    name = f"{side}_{finger}_MPR_link"
    if name not in meshes:
        return best
    tf = transforms[name]
    local = (points - tf[:3, 3]) @ tf[:3, :3]
    for surface in meshes[name]:
        offset = np.maximum(
            np.maximum(surface.bounds[0] - local, local - surface.bounds[1]), 0
        )
        bound = np.linalg.norm(offset, axis=1)
        candidates = local[(bound == 0) | (bound - radius < best)]
        if len(candidates):
            best = min(
                best,
                float(signed_surface_clearance(surface, candidates).min() - radius),
            )
    return best


def support_material_distances(transforms, points, side, finger, meshes, radius=0.001):
    """Per-material gaps for support tracking, including enabled MPR meshes."""
    result = gaps(transforms, points, side, finger)
    name = f"{side}_{finger}_MPR_link"
    if name not in meshes:
        return result
    tf = transforms[name]
    local = (points - tf[:3, 3]) @ tf[:3, :3]
    for surface in meshes[name]:
        offset = np.maximum(
            np.maximum(surface.bounds[0] - local, local - surface.bounds[1]), 0
        )
        bound = np.linalg.norm(offset, axis=1)
        ids = np.flatnonzero((bound == 0) | (bound - radius < result))
        if len(ids):
            result[ids] = np.minimum(
                result[ids], signed_surface_clearance(surface, local[ids]) - radius
            )
    return result


def support_contact_indices(
    transforms, points, side, finger, meshes, radius=0.001, threshold=0.0015
):
    """Exact contact membership without measuring every distant mesh gap.

    The AABB lower bound can exclude outside points from this threshold query.
    Interior candidates always remain, and a capsule contact needs no mesh query.
    """
    contacts = gaps(transforms, points, side, finger) < threshold
    name = f"{side}_{finger}_MPR_link"
    if name not in meshes:
        return np.flatnonzero(contacts)
    tf = transforms[name]
    local = (points - tf[:3, 3]) @ tf[:3, :3]
    for surface in meshes[name]:
        offset = np.maximum(
            np.maximum(surface.bounds[0] - local, local - surface.bounds[1]), 0
        )
        bound = np.linalg.norm(offset, axis=1)
        ids = np.flatnonzero(~contacts & ((bound == 0) | (bound - radius < threshold)))
        if len(ids):
            contacts[ids] = (
                signed_surface_clearance(surface, local[ids]) - radius < threshold
            )
    return np.flatnonzero(contacts)
