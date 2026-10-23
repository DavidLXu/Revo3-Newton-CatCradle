"""Geometric diagnostics for rope supports, material pickup and finger clearance.

These discrete checks are fixture diagnostics, not general topology proofs.
"""

import numpy as np
from .contact_geometry import finger_capsule


def gaps(tf, points, side, finger):
    result = np.full(len(points), np.inf)
    for segment in ("MCP", "PIP", "DIP"):
        c, axis, radius, half = finger_capsule(finger, segment, side)
        t = tf[f"{side}_{finger}_{segment}_link"]
        local = (points - t[:3, 3]) @ t[:3, :3] - c
        local -= np.clip(local @ axis, -half, half)[:, None] * axis
        result = np.minimum(result, np.linalg.norm(local, axis=1) - radius - 0.001)
    return result


def segment_separation(points):
    """Minimum centerline distance between non-neighboring closed-loop edges."""
    n = len(points)
    i, j = np.triu_indices(n, 3)
    mask = np.minimum(j - i, n - j + i) > 2
    i, j = i[mask], j[mask]
    x = points[i]
    y = points[j]
    u = points[(i + 1) % n] - x
    v = points[(j + 1) % n] - y
    w = x - y
    dot = lambda a, b: np.einsum("ij,ij->i", a, b)
    a = dot(u, u)
    b = dot(u, v)
    c = dot(v, v)
    d = dot(u, w)
    e = dot(v, w)
    denom = a * c - b * b
    s = np.clip(
        np.divide(b * e - c * d, denom, out=np.zeros_like(denom), where=denom > 1e-16),
        0,
        1,
    )
    t = (b * s + e) / np.maximum(c, 1e-16)
    s = np.where(t < 0, np.clip(-d / np.maximum(a, 1e-16), 0, 1), s)
    s = np.where(t > 1, np.clip((b - d) / np.maximum(a, 1e-16), 0, 1), s)
    t = np.clip(t, 0, 1)
    return float(np.linalg.norm(x + s[:, None] * u - y - t[:, None] * v, axis=1).min())


def hand_clearance(transforms, self_side=None, all_pairs=False):
    capsules = {}
    for side in ("left", "right"):
        capsules[side] = []
        for finger in ("thumb", "index", "middle", "ring", "little"):
            for segment in ("MCP", "PIP", "DIP"):
                c, axis, radius, half = finger_capsule(
                    finger, segment, self_side or side
                )
                t = transforms[f"{self_side or side}_{finger}_{segment}_link"]
                ends = (
                    np.array([c - axis * half, c + axis * half]) @ t[:3, :3].T
                    + t[:3, 3]
                )
                capsules[side].append((finger + "_" + segment, ends, radius))
    hits = []
    for name1, x, ra in capsules["left"]:
        for name2, y, rb in capsules["right"]:
            if self_side and name1.split("_")[0] >= name2.split("_")[0]:
                continue
            u, v, w = x[1] - x[0], y[1] - y[0], x[0] - y[0]
            a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
            den = a * c - b * b
            s = np.clip((b * e - c * d) / den, 0, 1) if den > 1e-16 else 0.0
            t = (b * s + e) / max(c, 1e-16)
            if t < 0:
                s = np.clip(-d / max(a, 1e-16), 0, 1)
            if t > 1:
                s = np.clip((b - d) / max(a, 1e-16), 0, 1)
            t = np.clip(t, 0, 1)
            gap = np.linalg.norm(x[0] + s * u - y[0] - t * v) - ra - rb
            hits.append((float(gap), name1, name2))
    return sorted(hits) if all_pairs else min(hits)


def palmar_arc(robot, q, points, side):
    tf = robot.fk(q)
    contacts = [
        np.flatnonzero(gaps(tf, points, side, f) < 0.0015) for f in ("thumb", "little")
    ]
    if any(not len(ids) for ids in contacts):
        raise RuntimeError("Initial support contacts are missing")
    a, b = [int(np.median(ids)) for ids in contacts]
    n = len(points)
    arcs = [
        np.arange(a, b + n if b < a else b + 1) % n,
        np.arange(b, a + n if a < b else a + 1) % n,
    ]
    return min(
        arcs, key=lambda ids: np.linalg.norm(np.diff(points[ids], axis=0), axis=1).sum()
    )


def triangle_coordinates(point, triangle):
    a, b, c = triangle
    normal = np.cross(a - c, b - c)
    size = np.linalg.norm(normal)
    if size < 1e-10:
        return None
    normal /= size
    coordinates = np.linalg.lstsq(np.array([a - c, b - c]).T, point - c, rcond=None)[0]
    return float((point - c) @ normal), np.r_[coordinates, 1 - coordinates.sum()]
