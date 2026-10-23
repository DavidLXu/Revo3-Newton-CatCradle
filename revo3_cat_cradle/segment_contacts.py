"""Segment contact for the two-hand rope, integrated inside Newton XPBD.

The normal correction is distributed to four endpoints using the closest
points' barycentric weights. This is discrete contact, not continuous CCD.
"""

from types import FunctionType
import warp as wp
from newton.solvers import SolverXPBD

# The same verified bound is consumed by the trajectory validator.
MAX_SEGMENT_LENGTH = wp.constant(0.016)


@wp.kernel
def segment_contacts(
    grid: wp.uint64,
    particle_x: wp.array[wp.vec3],
    particle_v: wp.array[wp.vec3],
    particle_invmass: wp.array[float],
    particle_radius: wp.array[float],
    particle_flags: wp.array[wp.int32],
    k_mu: float,
    k_cohesion: float,
    max_radius: float,
    dt: float,
    relaxation: float,
    deltas: wp.array[wp.vec3],
):
    i = wp.hash_grid_point_id(grid, wp.tid())
    if i < 0:
        return
    count = particle_x.shape[0]
    i1 = (i + 1) % count
    x = particle_x[i]
    u = particle_x[i1] - x
    # Active length control can lengthen the initially 3 mm edges. Include
    # a full 16 mm neighbor edge; the exported trajectory verifies this bound.
    query = wp.hash_grid_query(
        grid,
        x + u * 0.5,
        wp.length(u) * 0.5 + MAX_SEGMENT_LENGTH + 2.0 * particle_radius[i],
    )
    j = int(0)
    while wp.hash_grid_query_next(query, j):
        gap = wp.min(wp.abs(j - i), count - wp.abs(j - i))
        if j > i and gap > 2:
            j1 = (j + 1) % count
            y = particle_x[j]
            v = particle_x[j1] - y
            w = x - y
            a = wp.dot(u, u)
            b = wp.dot(u, v)
            c = wp.dot(v, v)
            d = wp.dot(u, w)
            e = wp.dot(v, w)
            denominator = a * c - b * b
            s = float(0.0)
            if denominator > 1.0e-16:
                s = wp.clamp((b * e - c * d) / denominator, 0.0, 1.0)
            t = (b * s + e) / wp.max(c, 1.0e-16)
            if t < 0.0:
                t = 0.0
                s = wp.clamp(-d / wp.max(a, 1.0e-16), 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = wp.clamp((b - d) / wp.max(a, 1.0e-16), 0.0, 1.0)
            offset = x + s * u - y - t * v
            distance = wp.length(offset)
            radius = (
                (1.0 - s) * particle_radius[i]
                + s * particle_radius[i1]
                + (1.0 - t) * particle_radius[j]
                + t * particle_radius[j1]
            )
            wi = particle_invmass[i]
            wi1 = particle_invmass[i1]
            wj = particle_invmass[j]
            wj1 = particle_invmass[j1]
            weight = (
                (1.0 - s) * (1.0 - s) * wi
                + s * s * wi1
                + (1.0 - t) * (1.0 - t) * wj
                + t * t * wj1
            )
            if distance < radius and distance > 1.0e-9 and weight > 0.0:
                normal = offset / distance
                relative_v = (
                    (1.0 - s) * particle_v[i]
                    + s * particle_v[i1]
                    - (1.0 - t) * particle_v[j]
                    - t * particle_v[j1]
                )
                tangent = relative_v - normal * wp.dot(relative_v, normal)
                friction = wp.normalize(tangent) * wp.min(
                    k_mu * (radius - distance), wp.length(tangent) * dt
                )
                correction = (
                    (normal * (radius - distance) - friction) * relaxation / weight
                )
                wp.atomic_add(deltas, i, correction * ((1.0 - s) * wi))
                wp.atomic_add(deltas, i1, correction * (s * wi1))
                wp.atomic_add(deltas, j, -correction * ((1.0 - t) * wj))
                wp.atomic_add(deltas, j1, -correction * (t * wj1))


_globals = dict(SolverXPBD.step.__globals__)
assert "solve_particle_particle_contacts" in _globals
_globals["solve_particle_particle_contacts"] = segment_contacts


class SegmentRopeSolverXPBD(SolverXPBD):
    step = FunctionType(
        SolverXPBD.step.__code__,
        _globals,
        SolverXPBD.step.__name__,
        SolverXPBD.step.__defaults__,
        SolverXPBD.step.__closure__,
    )
    step.__kwdefaults__ = SolverXPBD.step.__kwdefaults__.copy()
