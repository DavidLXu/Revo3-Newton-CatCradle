"""Conservative rope clearance against the robot's central-body envelopes.

The supplied torso collision meshes are not all watertight. Bounding boxes can
certify separation without assigning an unreliable inside/outside sign to those
meshes. Overlap is inconclusive and rejects a route; it is not a penetration
measurement. Arms and hand self-collisions are outside this check's scope.
"""

from itertools import product

import numpy as np
import trimesh

from .hand import origin


def edge_box_face_margins(points, bounds, radius):
    """A positive margin certifies each closed-loop edge's separation from a box.

    This checks complete edges, including ones with both endpoints outside the
    box. An outside diagonal edge may lack a separating box face, so a negative
    result alone cannot prove a collision with either the box or its mesh.
    """
    points, bounds = np.asarray(points), np.asarray(bounds)
    if (
        points.ndim != 2 or points.shape[1] != 3 or len(points) < 2
        or bounds.shape != (2, 3) or np.any(bounds[1] < bounds[0])
        or not np.isfinite(points).all() or not np.isfinite(bounds).all()
        or not np.isfinite(radius) or radius < 0
    ):
        raise ValueError("Invalid rope edge or body envelope geometry")
    ends = np.roll(points, -1, axis=0)
    low, high = np.minimum(points, ends), np.maximum(points, ends)
    face_margins = np.maximum(bounds[0] - high, low - bounds[1])
    return face_margins.max(axis=1) - radius


class CentralBodyEnvelope:
    LINKS = ("base_link", "waist_Link", "neck_Link", "head_Link", "head_oak_link")

    def __init__(self, robot):
        self.boxes = []
        for name in self.LINKS:
            link = robot.xml.find(f"link[@name='{name}']")
            if link is None:
                raise ValueError(f"Missing central body link: {name}")
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Unsupported body collision geometry: {name}")
                surface = trimesh.load_mesh(robot.path.parent / mesh.get("filename"))
                surface.apply_scale(np.fromstring(mesh.get("scale", "1 1 1"), sep=" "))
                surface.apply_transform(origin(collision.find("origin")))
                lo, hi = surface.bounds
                corners = np.asarray(list(product(*zip(lo, hi))))
                self.boxes.append((name, corners))
        if not self.boxes:
            raise ValueError("No central-body collision geometry")

    def measure(self, transforms, points, radius=0.001):
        worst = float("inf")
        overlap = np.zeros(len(points), dtype=bool)
        label, edge = None, None
        for name, corners in self.boxes:
            tf = transforms[name]
            world = corners @ tf[:3, :3].T + tf[:3, 3]
            bounds = np.array([world.min(axis=0), world.max(axis=0)])
            margins = edge_box_face_margins(points, bounds, radius)
            overlap |= margins < 0
            k = int(margins.argmin())
            if margins[k] < worst:
                worst, label, edge = float(margins[k]), name, k
        return dict(
            minimum_face_margin_m=worst,
            closest_envelope=label,
            closest_edge=edge,
            unresolved_edge_count=int(overlap.sum()),
        )
