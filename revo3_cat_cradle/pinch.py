"""Sampled thumb/index pinch geometry; not a grip-force or topology certificate.

Both capsule surfaces must approach the same material point from opposing
directions. Point and edge-sample modes use the same geometric thresholds;
knuckle-mesh contacts are outside these diagnostics.
"""

import numpy as np

from .contact_geometry import finger_capsule


def capsule_surface_samples(transforms, points, side, finger, rope_radius):
    """Return nearest capsule gap and outward world normal for each point."""
    best = np.full(len(points), np.inf)
    normals = np.zeros((len(points), 3))
    for segment in ("MCP", "PIP", "DIP"):
        center, axis, radius, half = finger_capsule(finger, segment, side)
        tf = transforms[f"{side}_{finger}_{segment}_link"]
        local = (points - tf[:3, 3]) @ tf[:3, :3]
        axial = np.clip((local - center) @ axis, -half, half)
        delta = local - center - axial[:, None] * axis
        distance = np.linalg.norm(delta, axis=1)
        gap = distance - radius - rope_radius
        update = gap < best
        best[update] = gap[update]
        normals[update] = (
            delta[update] / np.maximum(distance[update, None], 1e-15)
        ) @ tf[:3, :3].T
    return best, normals


def near_pinch_samples(transforms, rope, side, material_indices, rope_radius=.001):
    """Find opposing contact-compatible samples on the specified strand only.

    The 0.1 mm gap tolerance admits solver contact error. More than 0.3 mm of
    penetration is rejected, as is a normal dot product above -0.8. Persistent
    candidates still require a dynamic hold, support tracking and strain checks.
    """
    rope = np.asarray(rope, dtype=float)
    ids = np.asarray(material_indices)
    if (side not in ("left", "right") or rope.ndim != 2 or rope.shape[1] != 3
            or not np.isfinite(rope).all() or ids.ndim != 1
            or not np.issubdtype(ids.dtype, np.integer)
            or not len(ids) or np.any(ids < 0) or np.any(ids >= len(rope))
            or not np.isfinite(rope_radius) or rope_radius <= 0):
        raise ValueError("Expected finite rope points, valid material indices and radius")
    thumb, tn = capsule_surface_samples(transforms, rope[ids], side, "thumb", rope_radius)
    index, inn = capsule_surface_samples(transforms, rope[ids], side, "index", rope_radius)
    dot = np.einsum("ij,ij->i", tn, inn)
    paired = ((np.maximum(thumb, index) <= .0001)
              & (np.minimum(thumb, index) >= -.0003) & (dot <= -.8))
    return [dict(particle=int(i), thumb_gap_m=float(t), index_gap_m=float(n),
                 normal_dot=float(d), paired=bool(p))
            for i, t, n, d, p in zip(ids, thumb, index, dot, paired)]


def near_pinch_edge_samples(transforms, rope, side, edge_indices, rope_radius=.001,
                            samples_per_edge=21):
    """Check both surfaces at the same fractions of selected closed-loop edges.

    Positive samples locate a shared cross-section between particles. Absence
    of samples is not an exact continuous-edge exclusion. Contact at separate
    fractions on the same edge does not constitute a paired sample.
    """
    rope = np.asarray(rope, dtype=float)
    edges = np.asarray(edge_indices)
    if (rope.ndim != 2 or rope.shape[1] != 3 or edges.ndim != 1
            or not np.issubdtype(edges.dtype, np.integer) or not len(edges)
            or np.any(edges < 0) or np.any(edges >= len(rope))
            or not isinstance(samples_per_edge, int) or samples_per_edge < 2):
        raise ValueError("Expected valid rope edges and at least two samples per edge")
    fractions = np.linspace(0, 1, samples_per_edge)
    points = (rope[edges, None] * (1 - fractions)[None, :, None]
              + rope[(edges + 1) % len(rope), None] * fractions[None, :, None])
    points = points.reshape(-1, 3)
    rows = near_pinch_samples(transforms, points, side, np.arange(len(points)), rope_radius)
    for row in rows:
        sample = row.pop("particle")
        row.update(edge=int(edges[sample // samples_per_edge]),
                   fraction=float(fractions[sample % samples_per_edge]))
    return rows
