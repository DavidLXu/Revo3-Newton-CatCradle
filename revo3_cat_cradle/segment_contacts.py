"""Segment contact for the two-hand rope, integrated inside Newton XPBD.

The normal correction is distributed to four endpoints using the closest
points' barycentric weights. This is discrete contact, not continuous CCD.
"""

from types import FunctionType
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverXPBD

# The same verified bound is consumed by the trajectory validator.
MAX_SEGMENT_LENGTH = wp.constant(0.016)


@wp.kernel
def segment_capsule_contacts(
    points: wp.array[wp.vec3],
    velocities: wp.array[wp.vec3],
    invmass: wp.array[float],
    radii: wp.array[float],
    capsule_shapes: wp.array[int],
    shape_body: wp.array[int],
    shape_transform: wp.array[wp.transform],
    shape_scale: wp.array[wp.vec3],
    shape_mu: wp.array[float],
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    particle_mu: float,
    dt: float,
    relaxation: float,
    deltas: wp.array[wp.vec3],
):
    """Interior edge contact against kinematic capsules, with Coulomb friction.

    Endpoint minima remain the responsibility of Newton's particle contacts.
    This adds unilateral contact constraints, not attachments or targets.
    """
    i, ci = wp.tid()
    j = (i + 1) % points.shape[0]
    shape = capsule_shapes[ci]
    body = shape_body[shape]
    transform = wp.transform_multiply(body_q[body], shape_transform[shape])
    scale = shape_scale[shape]
    a = wp.transform_point(transform, wp.vec3(0.0, 0.0, -scale[1]))
    b = wp.transform_point(transform, wp.vec3(0.0, 0.0, scale[1]))
    x = points[i]
    u = points[j] - x
    v = b - a
    w = x - a
    aa = wp.dot(u, u)
    bb = wp.dot(u, v)
    cc = wp.dot(v, v)
    dd = wp.dot(u, w)
    ee = wp.dot(v, w)
    denominator = aa * cc - bb * bb
    s = float(0.0)
    if denominator > 1.0e-16:
        s = wp.clamp((bb * ee - cc * dd) / denominator, 0.0, 1.0)
    t = (bb * s + ee) / wp.max(cc, 1.0e-16)
    if t < 0.0:
        t = 0.0
        s = wp.clamp(-dd / wp.max(aa, 1.0e-16), 0.0, 1.0)
    elif t > 1.0:
        t = 1.0
        s = wp.clamp((bb - dd) / wp.max(aa, 1.0e-16), 0.0, 1.0)
    # Sphere (zero-height capsule) needs a point-to-segment query.
    if cc < 1.0e-16:
        s = wp.clamp(-dd / wp.max(aa, 1.0e-16), 0.0, 1.0)
        t = 0.0
    if s <= 1.0e-5 or s >= 1.0 - 1.0e-5:
        return
    closest = x + s * u
    axis_point = a + t * v
    offset = closest - axis_point
    distance = wp.length(offset)
    radius = scale[0] + (1.0 - s) * radii[i] + s * radii[j]
    weight = (1.0 - s) * (1.0 - s) * invmass[i] + s * s * invmass[j]
    if distance >= radius or distance <= 1.0e-9 or weight <= 0.0:
        return
    normal = offset / distance
    contact = axis_point + normal * scale[0]
    arm = contact - wp.transform_point(body_q[body], body_com[body])
    bv = wp.spatial_top(body_qd[body]) + wp.cross(wp.spatial_bottom(body_qd[body]), arm)
    relative = (1.0 - s) * velocities[i] + s * velocities[j] - bv
    tangent = relative - normal * wp.dot(relative, normal)
    penetration = radius - distance
    mu = 0.5 * (particle_mu + shape_mu[shape])
    friction = wp.normalize(tangent) * wp.min(mu * penetration, wp.length(tangent) * dt)
    correction = (normal * penetration - friction) * relaxation / weight
    wp.atomic_add(deltas, i, correction * ((1.0 - s) * invmass[i]))
    wp.atomic_add(deltas, j, correction * (s * invmass[j]))


@wp.func
def local_tube_neighbor(
    points: wp.array[wp.vec3], i: int, j: int, s: float, t: float, diameter: float
):
    """Exclude a continuous tube neighborhood, independent of particle spacing."""
    count = points.shape[0]
    ilength = wp.length(points[(i + 1) % count] - points[i])
    jlength = wp.length(points[(j + 1) % count] - points[j])
    forward = (1.0 - s) * ilength + t * jlength
    edge = (i + 1) % count
    while edge != j and forward <= diameter:
        forward += wp.length(points[(edge + 1) % count] - points[edge])
        edge = (edge + 1) % count
    if forward <= diameter:
        return True
    backward = s * ilength + (1.0 - t) * jlength
    edge = (j + 1) % count
    while edge != i and backward <= diameter:
        backward += wp.length(points[(edge + 1) % count] - points[edge])
        edge = (edge + 1) % count
    return backward <= diameter


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
                if local_tube_neighbor(particle_x, i, j, s, t, radius):
                    continue
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
    def __init__(self, model, *, segment_hand_contacts=False, **kwargs):
        super().__init__(model, **kwargs)
        types = model.shape_type.numpy()
        bodies = model.shape_body.numpy()
        candidates = np.flatnonzero(types == int(newton.GeoType.CAPSULE))
        if any(bodies[s] < 0 for s in candidates):
            raise ValueError("Segment capsule contacts require kinematic bodies")
        if any(
            not (model.body_flags.numpy()[bodies[s]] & int(newton.BodyFlags.KINEMATIC))
            for s in candidates
        ):
            raise ValueError("Segment capsule contacts require kinematic bodies")
        self.capsule_shapes = wp.array(candidates, dtype=int, device=model.device)
        self.segment_hand_contacts = bool(segment_hand_contacts)

    def _apply_particle_deltas(self, model, state_in, state_out, particle_deltas, dt):
        if self.segment_hand_contacts and len(self.capsule_shapes):
            current = (
                state_out
                if state_in.requires_grad or self._particle_delta_counter == 0
                else state_in
            )
            wp.launch(
                segment_capsule_contacts,
                dim=(model.particle_count, len(self.capsule_shapes)),
                inputs=[
                    current.particle_q,
                    current.particle_qd,
                    model.particle_inv_mass,
                    model.particle_radius,
                    self.capsule_shapes,
                    model.shape_body,
                    model.shape_transform,
                    model.shape_scale,
                    model.shape_material_mu,
                    state_in.body_q,
                    state_in.body_qd,
                    model.body_com,
                    model.soft_contact_mu,
                    dt,
                    self.soft_contact_relaxation,
                ],
                outputs=[particle_deltas],
                device=model.device,
            )
        return super()._apply_particle_deltas(
            model, state_in, state_out, particle_deltas, dt
        )

    step = FunctionType(
        SolverXPBD.step.__code__,
        _globals,
        SolverXPBD.step.__name__,
        SolverXPBD.step.__defaults__,
        SolverXPBD.step.__closure__,
    )
    step.__kwdefaults__ = SolverXPBD.step.__kwdefaults__.copy()
