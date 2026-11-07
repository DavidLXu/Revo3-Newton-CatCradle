"""Conservative opposing finger/palm checks for kinematic hand trajectories."""

import numpy as np
import trimesh

from .contact_geometry import finger_capsule
from .hand import origin


def capsule_hull_clearance(surface, a, b, radius, *, max_step=0.0005):
    """Lower-bound capsule clearance using the signed distance along its axis.

    Signed distance to a convex hull is 1-Lipschitz. Subtracting half the actual
    sample spacing bounds a missed minimum between samples. A negative bound
    alone is inconclusive; a negative sampled gap confirms intersection.
    """
    if max_step <= 0 or not np.isfinite(max_step) or radius < 0:
        raise ValueError("Invalid clearance sampling parameters")
    a, b = np.asarray(a), np.asarray(b)
    length = float(np.linalg.norm(b - a))
    intervals = max(1, int(np.ceil(length / max_step)))
    points = np.linspace(a, b, intervals + 1)
    _, distances, _ = trimesh.proximity.closest_point_naive(surface, points)
    planes = np.einsum("ij,ij->i", surface.face_normals, surface.triangles_center)
    inside = np.all(points @ surface.face_normals.T - planes <= 1e-10, axis=1)
    distances[inside] *= -1
    sampled = float(distances.min() - radius)
    return sampled - length / intervals / 2, sampled


class OpposingPalmClearance:
    """Use the same convex palm surfaces as BimanualSimulation's hull mode.

    This covers opposite-hand capsules against palms, plus a bounding-sphere
    separation check for the two palms. Knuckles, arms and swept motion are
    outside its scope.
    """

    def __init__(self, robot, transforms, *, geometry="hull"):
        if geometry not in ("hull", "box"):
            raise ValueError(f"Unsupported palm collision geometry: {geometry}")
        self.hulls = {}
        self.bounds = {}
        for side in ("left", "right"):
            vertices = []
            base_inverse = np.linalg.inv(transforms[side + "_hand_base_link"])
            for suffix in (
                ("plam_link", "plam_rubber_link") if geometry == "hull" else ()
            ):
                name = side + "_" + suffix
                for collision in robot.xml.find(f"link[@name='{name}']").findall(
                    "collision"
                ):
                    mesh = collision.find("geometry/mesh")
                    if mesh is None:
                        continue
                    surface = trimesh.load_mesh(
                        robot.path.parent / mesh.get("filename")
                    )
                    surface.apply_transform(origin(collision.find("origin")))
                    surface.apply_transform(base_inverse @ transforms[name])
                    vertices.extend(surface.vertices)
            if geometry == "hull":
                surface = trimesh.convex.convex_hull(np.asarray(vertices))
            else:
                surface = trimesh.creation.box(extents=(0.026, 0.078, 0.078))
                surface.apply_translation((0, 0, 0.058))
            self.hulls[side] = surface
            center = surface.bounds.mean(axis=0)
            radius = float(np.linalg.norm(surface.vertices - center, axis=1).max())
            self.bounds[side] = center, radius

    def measure(self, transforms, *, quick=False):
        worst, sampled, label = float("inf"), float("inf"), None
        centers = {}
        for side in ("left", "right"):
            base = transforms[side + "_hand_base_link"]
            centers[side] = base[:3, :3] @ self.bounds[side][0] + base[:3, 3]
            other = "right" if side == "left" else "left"
            inv = np.linalg.inv(transforms[other + "_hand_base_link"])
            surface = self.hulls[other]
            for finger in ("thumb", "index", "middle", "ring", "little"):
                for segment in ("MCP", "PIP", "DIP"):
                    name = f"{side}_{finger}_{segment}_link"
                    center, axis, radius, half = finger_capsule(finger, segment, side)
                    tf = inv @ transforms[name]
                    ends = np.array([center - half * axis, center + half * axis])
                    ends = ends @ tf[:3, :3].T + tf[:3, 3]
                    # AABB distance is already a lower bound; prune only when
                    # it cannot improve the currently measured global minimum.
                    delta = np.maximum(
                        np.maximum(
                            surface.bounds[0] - ends.max(axis=0),
                            ends.min(axis=0) - surface.bounds[1],
                        ),
                        0,
                    )
                    if np.linalg.norm(delta) - radius > worst:
                        continue
                    planes = np.einsum(
                        "ij,ij->i", surface.face_normals, surface.triangles_center
                    )
                    # Any separating hull face bounds the entire capsule,
                    # without axis sampling or a nearest-triangle query.
                    plane_bound = float(
                        (ends @ surface.face_normals.T - planes).min(axis=0).max()
                        - radius
                    )
                    if quick and plane_bound > 0.0005:
                        bound, gap = plane_bound, None
                    else:
                        bound, gap = capsule_hull_clearance(surface, *ends, radius)
                    if bound < worst:
                        worst, sampled, label = (
                            bound,
                            gap,
                            name + " / " + other + " palm",
                        )
        palm_bound = float(
            np.linalg.norm(centers["left"] - centers["right"])
            - self.bounds["left"][1]
            - self.bounds["right"][1]
        )
        return dict(
            finger_palm_lower_bound_m=worst,
            sampled_finger_palm_gap_m=sampled,
            closest_pair=label,
            palm_palm_sphere_lower_bound_m=palm_bound,
        )
