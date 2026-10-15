"""Newton XPBD deformable loop with kinematic Revo3 collision proxies."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
import warp as wp
import newton
from .rope_solver import RopeSolverXPBD

from .hand import Hand
from .motion import RecordedMotion, PickupMotion
from .rope import build_rope, site_positions, wrap_sites
from .contact_geometry import finger_capsule, clear_initial_intersections
from .tension import adjust_lengths, tensioned_lengths


@dataclass
class Config:
    pickup_correction: bool = True
    fps: int = 30
    substeps: int = 96
    iterations: int = 32
    contact_relaxation: float = .35
    friction: float = .65
    radius: float = .001
    spacing: float = .003
    stretch_stiffness: float = 800.
    stretch_damping: float = .01
    rope_linear_density: float = .004
    adaptive_length: bool = True
    adaptive_take_up_m: float = .01
    rope_length_m: float = .4417950868606567
    bend_stiffness: float = .002
    gravity: float = -9.81
    device: str = "cpu"


class Simulation:
    def __init__(self, config=None):
        self.config = config or Config()
        c = self.config
        if c.fps <= 0 or c.substeps <= 0 or c.iterations <= 0:
            raise ValueError("fps, substeps and iterations must be positive")
        if not np.isfinite(c.friction) or c.friction < 0:
            raise ValueError("friction must be finite and nonnegative")
        if not np.isfinite(c.rope_linear_density) or c.rope_linear_density <= 0:
            raise ValueError("rope_linear_density must be finite and positive")
        if not np.isfinite(c.stretch_stiffness) or c.stretch_stiffness <= 0:
            raise ValueError("stretch_stiffness must be finite and positive")
        if not np.isfinite(c.adaptive_take_up_m) or not 0 <= c.adaptive_take_up_m <= .05:
            raise ValueError("adaptive_take_up_m must be between 0 and 0.05 m")
        wp.init()
        self.hand = Hand()
        self.motion = RecordedMotion(self.hand)
        if c.pickup_correction:
            self.motion = PickupMotion(self.motion)
        self.t = 0.
        self.q, self.phase = self.motion.sample(0)
        self.transforms = self.hand.fk(self.q)
        # These sites only construct the initial loop; they are never tracked.
        initial_sites = site_positions(self.transforms, wrap_sites(self.hand.side,c.radius))
        span_lengths = np.linalg.norm(np.roll(initial_sites,-1,axis=0)-initial_sites,axis=1)
        free_spans = np.arange(2, 15, 3)
        # Budget the sampled half-circle lengths, not their shorter chords;
        # otherwise moving the wraps rearward creates artificial free-span sag.
        wrap_length = sum(np.pi*(finger_capsule(finger,"DIP",self.hand.side)[2]+c.radius+.00015)
                          for finger in ("thumb","index","middle","ring","little"))
        requested_length = c.rope_length_m
        if not np.isfinite(requested_length) or requested_length <= wrap_length:
            raise ValueError(f"rope length must exceed the initial wrap length ({wrap_length:.4f} m)")
        span_lengths[free_spans] *= (requested_length-wrap_length)/span_lengths[free_spans].sum()
        p, ids, _ = build_rope(self.transforms, c.spacing, self.hand.side, span_lengths, c.radius)
        p = clear_initial_intersections(p,self.transforms,self.hand.side,c.radius)
        self.initial_wrap_indices = ids
        self.anchor_indices = np.empty(0, dtype=np.int32)
        self.initial_positions = p.copy()
        self.rest_lengths = np.linalg.norm(np.roll(p, -1, axis=0)-p, axis=1)
        # Fixed mode keeps this material budget. Active mode initializes and
        # updates rest lengths from rope strain, with no hand-position target.
        self.rest_lengths *= requested_length/self.rest_lengths.sum()
        if c.adaptive_length:
            self.rest_lengths=tensioned_lengths(
                np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1),c.stretch_stiffness,
                take_up_m=c.adaptive_take_up_m).astype(np.float32)
        self.initial_rest_lengths=self.rest_lengths.copy()
        self.proxy_links = []
        b = newton.ModelBuilder(gravity=(0., 0., c.gravity))
        cfg = b.default_shape_cfg.copy()
        cfg.mu = c.friction
        # Proxies interact only with particles, never with each other.
        cfg.has_shape_collision = False
        for finger in ("thumb", "index", "middle", "ring", "little"):
            for segment in ("MCP", "PIP", "DIP"):
                name = f"{self.hand.side}_{finger}_{segment}_link"
                self.proxy_links.append(name)
                body = b.add_body(xform=self.as_warp(self.transforms[name]), is_kinematic=True, label=name)
                pos, axis, radius, half_height = finger_capsule(finger,segment,self.hand.side)
                rot = wp.quat_from_axis_angle(wp.vec3(1.,0.,0.), np.pi/2) if finger=="thumb" else wp.quat_identity()
                b.add_shape_capsule(body, xform=wp.transform(pos,rot), radius=radius,
                                    half_height=half_height, cfg=cfg)
        self.proxy_links.append(f"{self.hand.side}_hand_base_link")
        palm = b.add_body(xform=self.as_warp(self.transforms[self.proxy_links[-1]]), is_kinematic=True, label="palm")
        b.add_shape_box(palm, xform=wp.transform((0.,0.,.058), wp.quat_identity()),
                        hx=.013, hy=.039, hz=.039, cfg=cfg)
        b.add_ground_plane(cfg=cfg)
        material_edges=np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1)
        masses=c.rope_linear_density*(material_edges+np.roll(material_edges,1))/2
        for point,mass in zip(p,masses):
            b.add_particle(point, (0.,0.,0.), mass=float(mass), radius=c.radius)
        n = len(p)
        for i in range(n):
            b.add_spring(i, (i+1)%n, ke=c.stretch_stiffness, kd=c.stretch_damping, control=0.)
            b.spring_rest_length[-1] = float(self.rest_lengths[i])
        # Weak two-hop springs resist kinking; this is a discrete elastic string,
        # not a torsion-bearing Cosserat rod or tetrahedral solid.
        for i in range(n):
            b.add_spring(i, (i+2)%n, ke=c.bend_stiffness, kd=.00001, control=0.)
        self.model = b.finalize(device=c.device)
        self.spring_rest_buffer=self.model.spring_rest_length.numpy().copy()
        self.body_com_local = self.model.body_com.numpy().copy()
        self.model.soft_contact_mu = c.friction
        self.model.particle_adhesion = 0.
        self.model.particle_cohesion = 0.
        self.model.particle_mu = .25
        self.solver = RopeSolverXPBD(self.model, iterations=c.iterations,
                                 soft_body_relaxation=.65, soft_contact_relaxation=c.contact_relaxation)
        self.contact_capacity = max(1024, 4*n)
        self.pipeline = newton.CollisionPipeline(self.model, soft_contact_max=self.contact_capacity, rigid_contact_max=0)
        self.contacts = self.pipeline.contacts()
        self.state = self.model.state()
        self.next = self.model.state()
        self.control = self.model.control()
        self.contact_peak = 0

    @staticmethod
    def as_warp(t):
        return wp.transform(t[:3,3], Rotation.from_matrix(t[:3,:3]).as_quat())

    @staticmethod
    def body_velocity(tf, previous, local_com, dt):
        # Newton expects linear velocity at the COM, not at the link origin.
        com = tf[:3,:3] @ local_com + tf[:3,3]
        old_com = previous[:3,:3] @ local_com + previous[:3,3]
        angular = Rotation.from_matrix(tf[:3,:3] @ previous[:3,:3].T).as_rotvec()/dt
        return np.r_[(com-old_com)/dt, angular]

    def advance(self, end_time, move=True):
        c = self.config
        dt = 1/(c.fps*c.substeps)
        for sub in range(c.substeps):
            t = end_time - (c.substeps-sub-1)*dt if move else self.t
            self.q, self.phase = self.motion.sample(t)
            previous = self.transforms
            self.transforms = self.hand.fk(self.q)
            poses, velocities = [], []
            for name, local_com in zip(self.proxy_links, self.body_com_local):
                tf, prev = self.transforms[name], previous[name]
                poses.append(np.r_[tf[:3,3], Rotation.from_matrix(tf[:3,:3]).as_quat()])
                velocities.append(self.body_velocity(tf, prev, local_com, dt))
            self.state.body_q.assign(np.asarray(poses, dtype=np.float32))
            self.state.body_qd.assign(np.asarray(velocities, dtype=np.float32))
            self.state.clear_forces()
            if c.adaptive_length:
                self.rest_lengths=adjust_lengths(self.state.particle_q.numpy(),self.rest_lengths,
                    self.initial_rest_lengths,c.stretch_stiffness,dt,c.adaptive_take_up_m)
                self.spring_rest_buffer[:self.model.particle_count]=self.rest_lengths
                self.model.spring_rest_length.assign(self.spring_rest_buffer)
            self.pipeline.collide(self.state, self.contacts)
            count = int(self.contacts.soft_contact_count.numpy()[0])
            if count > self.contact_capacity:
                raise RuntimeError("Soft contact buffer overflow; increase contact_capacity")
            self.contact_peak = max(self.contact_peak, count)
            self.solver.step(self.state, self.next, self.control, self.contacts, dt)
            self.state, self.next = self.next, self.state
        self.t = end_time if move else self.t
        self.contact_peak = max(self.contact_peak, int(self.contacts.soft_contact_count.numpy()[0]))

    def snapshot(self):
        positions = self.state.particle_q.numpy()
        if not np.isfinite(positions).all():
            raise RuntimeError(f"Non-finite rope state at {self.t:.3f}s")
        return dict(time=round(self.t, 6), phase=self.phase, joints=self.q.tolist(),
            rope=positions.tolist(),
            rope_rest_length_m=float(self.rest_lengths.sum()),
            rest_lengths=self.rest_lengths.tolist(),
            transforms=[np.round(self.transforms[name].T.ravel(), 7).tolist() for name in self.hand.links])

    def metrics(self):
        p = self.state.particle_q.numpy()
        lengths = np.linalg.norm(np.roll(p,-1,axis=0)-p, axis=1)
        return dict(max_stretch_ratio=float(np.max(lengths/self.rest_lengths)),
            rope_rest_length_m=float(self.rest_lengths.sum()),
            mean_stretch_ratio=float(np.mean(lengths/self.rest_lengths)),
            initial_wrap_displacement_m=float(np.max(np.linalg.norm(
                p[self.initial_wrap_indices]-self.initial_positions[self.initial_wrap_indices],axis=1))),
            min_height_m=float(p[:,2].min()), max_coordinate_m=float(np.abs(p).max()))
