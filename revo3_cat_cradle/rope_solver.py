# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0
"""Newton XPBD adapter: exclude a closed rope's topological contact neighbors.

The pinned Newton version has no particle-pair filter hook. Clone its step
function's globals for this subclass only, replacing one contact kernel. This
does not mutate Newton's module or change other SolverXPBD instances.
Contact equations follow Newton's Apache-2.0 XPBD particle-contact kernel.
"""
from types import FunctionType
import warp as wp
from newton.solvers import SolverXPBD


@wp.kernel
def rope_self_contacts(
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
    i=wp.hash_grid_point_id(grid,wp.tid())
    if i == -1:
        return
    x=particle_x[i]
    radius=particle_radius[i]
    w1=particle_invmass[i]
    query=wp.hash_grid_query(grid,x,radius+max_radius+k_cohesion)
    j=int(0)
    delta=wp.vec3(0.0)
    while wp.hash_grid_query_next(query,j):
        gap=wp.abs(j-i)
        gap=wp.min(gap,particle_x.shape[0]-gap)
        # Adjacent and two-hop particles represent overlapping portions of
        # one continuous strand, including across the closed-loop seam.
        if gap > 2:
            n=x-particle_x[j]
            distance=wp.length(n)
            error=distance-radius-particle_radius[j]
            denom=w1+particle_invmass[j]
            if error <= k_cohesion and denom > 0.0 and distance > 0.0:
                n=n/distance
                relative_v=particle_v[i]-particle_v[j]
                tangent_v=relative_v-n*wp.dot(n,relative_v)
                normal_delta=n*error
                tangent_delta=wp.normalize(tangent_v)*wp.max(k_mu*error,-wp.length(tangent_v)*dt)
                delta+=(tangent_delta-normal_delta)/denom
    wp.atomic_add(deltas,i,delta*w1*relaxation)


_step_globals=dict(SolverXPBD.step.__globals__)
assert 'solve_particle_particle_contacts' in _step_globals, 'Newton adapter needs review'
_step_globals['solve_particle_particle_contacts']=rope_self_contacts


class RopeSolverXPBD(SolverXPBD):
    """For one all-dynamic closed particle loop, indexed in strand order."""
    step=FunctionType(SolverXPBD.step.__code__,_step_globals,
                      SolverXPBD.step.__name__,SolverXPBD.step.__defaults__,
                      SolverXPBD.step.__closure__)
    step.__kwdefaults__=SolverXPBD.step.__kwdefaults__.copy()
