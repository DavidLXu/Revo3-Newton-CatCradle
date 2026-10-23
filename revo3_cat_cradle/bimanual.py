"""RevoTron kinematics and a dynamic rope shared by its two Revo3 hands."""

from dataclasses import asdict
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import warp as wp
import newton
from .hand import Hand, ROOT
from .sim import Simulation, Config
from .contact_geometry import finger_capsule, clear_initial_intersections
from .segment_contacts import SegmentRopeSolverXPBD, MAX_SEGMENT_LENGTH
from .tension import tensioned_lengths

ROBOT_URDF = (
    ROOT
    / "assets/revotron-description/revotron_system/urdf/revotron_bimanual_revo3.urdf"
)


class RevoTron(Hand):
    def __init__(self, path=ROBOT_URDF):
        super().__init__(path)
        self.root_pose = np.eye(4)
        self.root_pose[2, 3] = 0.85
        self.indices = {n: i for i, n in enumerate(self.names)}
        self.order = []
        known = {"world"}
        pending = self.joints.copy()
        while pending:
            ready = [j for j in pending if j["parent"] in known]
            if not ready:
                raise ValueError("Disconnected robot")
            for j in ready:
                self.order.append(j)
                known.add(j["child"])
                pending.remove(j)
        self.arm_indices = {
            side: [self.indices[f"Joint{i}_{letter}"] for i in range(1, 8)]
            for side, letter in [("left", "L"), ("right", "R")]
        }
        self.chains = {}
        for side in ("left", "right"):
            child = f"{side}_hand_base_link"
            chain = []
            while child != "world":
                j = next(j for j in self.joints if j["child"] == child)
                chain.append(j)
                child = j["parent"]
            self.chains[side] = chain[::-1]
        self._fk_q = None
        self._fk_transforms = None
        self._joint_rotations = {}

    def fk(self, q):
        if self._fk_q is not None and np.array_equal(q, self._fk_q):
            return self._fk_transforms
        transforms = {"world": self.root_pose}
        changed = {"world": self._fk_q is None}
        for j in self.order:
            index = self.indices.get(j["name"])
            dirty = changed[j["parent"]] or (
                index is not None
                and (self._fk_q is None or q[index] != self._fk_q[index])
            )
            changed[j["child"]] = dirty
            if not dirty:
                transforms[j["child"]] = self._fk_transforms[j["child"]]
                continue
            t = transforms[j["parent"]] @ j["origin"]
            if j["kind"] == "revolute":
                if self._fk_q is None or q[index] != self._fk_q[index]:
                    self._joint_rotations[j["name"]] = Rotation.from_rotvec(
                        j["axis"] * q[index]
                    ).as_matrix()
                d = self._joint_rotations[j["name"]]
                t[:3, :3] = t[:3, :3] @ d
            transforms[j["child"]] = t
        self._fk_q = np.array(q).copy()
        self._fk_transforms = transforms
        return transforms

    def wrist(self, q, side):
        t = self.root_pose.copy()
        for j in self.chains[side]:
            t = t @ j["origin"]
            if j["kind"] == "revolute":
                t[:3, :3] = (
                    t[:3, :3]
                    @ Rotation.from_rotvec(
                        j["axis"] * q[self.indices[j["name"]]]
                    ).as_matrix()
                )
        return t

    def ik(self, q, side, position, rotation):
        ids = self.arm_indices[side]
        initial = q[ids].copy()
        trial = q.copy()

        def residual(x):
            trial[ids] = x
            t = self.wrist(trial, side)
            return np.r_[
                (t[:3, 3] - position) * 5,
                Rotation.from_matrix(rotation @ t[:3, :3].T).as_rotvec(),
                0.003 * (x - initial),
            ]

        candidates = [
            initial,
            np.array([0, -0.7, 0, -1.4, 0, 0, 0]),
            np.array([1, -1, 0, -1.4, 0, 0, 0]),
        ]
        best = None
        for seed in candidates:
            result = least_squares(
                residual,
                np.clip(seed, self.limits[ids, 0] + 1e-7, self.limits[ids, 1] - 1e-7),
                bounds=(self.limits[ids, 0], self.limits[ids, 1]),
                max_nfev=160,
                ftol=1e-9,
                xtol=1e-9,
                gtol=1e-9,
            )
            if best is None or np.linalg.norm(result.fun[:6]) < np.linalg.norm(
                best.fun[:6]
            ):
                best = result
            if np.linalg.norm(best.fun[:6]) < 0.003:
                break
        q = q.copy()
        q[ids] = best.x
        t = self.wrist(q, side)
        error = np.linalg.norm(t[:3, 3] - position)
        if error > 0.002:
            raise ValueError(f"{side} wrist IK misses by {error:.4f} m")
        orientation_error = np.linalg.norm(
            Rotation.from_matrix(rotation @ t[:3, :3].T).as_rotvec()
        )
        if orientation_error > 0.01:
            raise ValueError(
                f"{side} wrist IK rotation misses by {orientation_error:.4f} rad"
            )
        return q


def resample_loop(points, spacing=0.003):
    p = np.asarray(points)
    p = np.r_[p, p[:1]]
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    count = max(8, int(np.ceil(arc[-1] / spacing)))
    samples = np.arange(count) * arc[-1] / count
    return np.column_stack([np.interp(samples, arc, p[:, i]) for i in range(3)]).astype(
        np.float32
    )


class BimanualSimulation(Simulation):
    def __init__(
        self,
        robot,
        motion,
        points,
        config=None,
        palm_geometry="box",
        length_control="local",
        rest_length_max_scale=2.0,
    ):
        self.palm_geometry = palm_geometry
        self.length_control = length_control
        self.rest_length_max_scale = float(rest_length_max_scale)
        if (
            not np.isfinite(self.rest_length_max_scale)
            or self.rest_length_max_scale < 1
        ):
            raise ValueError("rest_length_max_scale must be finite and at least one")
        self.config = config or Config(spacing=0.003, adaptive_take_up_m=0.005)
        c = self.config
        wp.init()
        self.hand = robot
        self.motion = motion
        self.t = 0.0
        self.q, self.phase = motion.sample(0)
        self.transforms = robot.fk(self.q)
        p = np.asarray(points, dtype=np.float32)
        for side in ("left", "right"):
            p = clear_initial_intersections(p, self.transforms, side, c.radius)
        p = resample_loop(p, c.spacing)
        for side in ("left", "right"):
            p = clear_initial_intersections(p, self.transforms, side, c.radius)
        self.initial_positions = p.copy()
        self.initial_wrap_indices = np.arange(len(p), dtype=np.int32)
        self.anchor_indices = np.empty(0, dtype=np.int32)
        edges = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
        self.rest_lengths = tensioned_lengths(
            edges, c.stretch_stiffness, take_up_m=c.adaptive_take_up_m
        ).astype(np.float32)
        self.initial_rest_lengths = self.rest_lengths.copy()
        b = newton.ModelBuilder(gravity=(0, 0, c.gravity))
        cfg = b.default_shape_cfg.copy()
        cfg.mu = c.friction
        cfg.has_shape_collision = False
        self.proxy_links = []
        for side in ("left", "right"):
            for finger in ("thumb", "index", "middle", "ring", "little"):
                for segment in ("MCP", "PIP", "DIP"):
                    name = f"{side}_{finger}_{segment}_link"
                    self.proxy_links.append(name)
                    body = b.add_body(
                        xform=self.as_warp(self.transforms[name]),
                        is_kinematic=True,
                        label=name,
                    )
                    center, axis, radius, half = finger_capsule(finger, segment, side)
                    rot = (
                        wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi / 2)
                        if finger == "thumb"
                        else wp.quat_identity()
                    )
                    b.add_shape_capsule(
                        body,
                        xform=wp.transform(center, rot),
                        radius=radius,
                        half_height=half,
                        cfg=cfg,
                    )
            name = f"{side}_hand_base_link"
            self.proxy_links.append(name)
            body = b.add_body(
                xform=self.as_warp(self.transforms[name]), is_kinematic=True, label=name
            )
            if palm_geometry == "hull":
                import trimesh
                from .hand import origin

                vertices = []
                base_inverse = np.linalg.inv(self.transforms[name])
                for suffix in ("plam_link", "plam_rubber_link"):
                    link_name = f"{side}_{suffix}"
                    link = robot.xml.find(f"link[@name='{link_name}']")
                    for collision in link.findall("collision"):
                        mesh = collision.find("geometry/mesh")
                        if mesh is None:
                            continue
                        surface = trimesh.load_mesh(
                            robot.path.parent / mesh.get("filename")
                        )
                        surface.apply_transform(origin(collision.find("origin")))
                        surface.apply_transform(
                            base_inverse @ self.transforms[link_name]
                        )
                        vertices.extend(surface.vertices)
                hull = trimesh.convex.convex_hull(np.asarray(vertices))
                b.add_shape_mesh(
                    body,
                    mesh=newton.Mesh(
                        hull.vertices.astype(np.float32),
                        hull.faces.astype(np.int32).ravel(),
                    ),
                    cfg=cfg,
                )
            elif palm_geometry == "box":
                b.add_shape_box(
                    body,
                    xform=wp.transform((0, 0, 0.058), wp.quat_identity()),
                    hx=0.013,
                    hy=0.039,
                    hz=0.039,
                    cfg=cfg,
                )
            else:
                raise ValueError(f"Unknown palm collision geometry: {palm_geometry}")
        b.add_ground_plane(cfg=cfg)
        masses = c.rope_linear_density * (edges + np.roll(edges, 1)) / 2
        for point, mass in zip(p, masses):
            b.add_particle(point, (0, 0, 0), mass=float(mass), radius=c.radius)
        n = len(p)
        for i in range(n):
            b.add_spring(
                i,
                (i + 1) % n,
                ke=c.stretch_stiffness,
                kd=c.stretch_damping,
                control=0.0,
            )
            b.spring_rest_length[-1] = float(self.rest_lengths[i])
        for i in range(n):
            b.add_spring(i, (i + 2) % n, ke=c.bend_stiffness, kd=0.00001, control=0.0)
        self.model = b.finalize(device=c.device)
        # A conservative broad-phase bound gives segment queries 10 mm grid
        # cells. Actual collision radii remain in model.particle_radius.
        self.model.particle_max_radius = max(self.model.particle_max_radius, 0.005)
        self.spring_rest_buffer = self.model.spring_rest_length.numpy().copy()
        self.body_com_local = self.model.body_com.numpy().copy()
        self.model.soft_contact_mu = c.friction
        self.model.particle_adhesion = 0.0
        self.model.particle_cohesion = 0.0
        self.model.particle_mu = 0.25
        self.solver = SegmentRopeSolverXPBD(
            self.model,
            iterations=c.iterations,
            soft_body_relaxation=0.65,
            soft_contact_relaxation=c.contact_relaxation,
        )
        self.contact_capacity = max(4096, 8 * n)
        self.pipeline = newton.CollisionPipeline(
            self.model, soft_contact_max=self.contact_capacity, rigid_contact_max=0
        )
        self.contacts = self.pipeline.contacts()
        self.state = self.model.state()
        self.next = self.model.state()
        self.control = self.model.control()
        self.contact_peak = 0

    def update_rest_lengths(self, dt):
        c = self.config
        p = self.state.particle_q.numpy()
        lengths = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
        if self.length_control == "global":
            initial_total = float(self.initial_rest_lengths.sum())
            target_total = float(
                tensioned_lengths(
                    lengths, c.stretch_stiffness, take_up_m=c.adaptive_take_up_m
                ).sum()
            )
            minimum = initial_total * 0.65
            target_total = np.clip(target_total, minimum, initial_total * 2)
            total = float(self.rest_lengths.sum())
            total += max(target_total - total, -0.30 * dt)
            self.rest_lengths = (
                self.initial_rest_lengths * (total / initial_total)
            ).astype(np.float32)
            self.spring_rest_buffer[: self.model.particle_count] = self.rest_lengths
            self.model.spring_rest_length.assign(self.spring_rest_buffer)
            return
        target = tensioned_lengths(
            lengths, c.stretch_stiffness, take_up_m=c.adaptive_take_up_m
        )
        target = np.clip(
            target,
            self.initial_rest_lengths * 0.25,
            self.initial_rest_lengths * self.rest_length_max_scale,
        )
        self.rest_lengths = (
            self.rest_lengths
            + np.maximum(
                target - self.rest_lengths,
                -0.30
                * dt
                * self.initial_rest_lengths
                / self.initial_rest_lengths.sum(),
            )
        ).astype(np.float32)
        self.spring_rest_buffer[: self.model.particle_count] = self.rest_lengths
        self.model.spring_rest_length.assign(self.spring_rest_buffer)


def export_bimanual(sim, frames, metrics, out, meshes=False):
    out.mkdir(parents=True, exist_ok=True)
    validation = {
        "deformation_bounds_passed": max(m["max_stretch_ratio"] for m in metrics) < 1.2,
        "max_stretch_ratio": max(m["max_stretch_ratio"] for m in metrics),
    }
    configuration = asdict(sim.config)
    configuration.pop("rope_length_m", None)
    payload = dict(
        config=configuration,
        links=sim.hand.links,
        joint_names=sim.hand.names,
        frames=frames,
        motion=sim.motion.metadata,
        anchor_indices=[],
        interaction="contact_friction_only",
        rope_self_contact="segment_segment",
        self_contact_search_bound_m=float(MAX_SEGMENT_LENGTH),
        length_control=sim.length_control,
        rest_length_max_scale=sim.rest_length_max_scale,
        collision_geometry=dict(fingers="capsules", palm=sim.palm_geometry),
        robot="revotron",
        validation=validation,
    )
    (out / "trajectory.json.tmp").write_text(json.dumps(payload, separators=(",", ":")))
    (out / "trajectory.json.tmp").replace(out / "trajectory.json")
    np.savez_compressed(
        out / "trajectory.pending.npz",
        time=[f["time"] for f in frames],
        joints=[f["joints"] for f in frames],
        rope=[f["rope"] for f in frames],
        rest_lengths=[f["rest_lengths"] for f in frames],
        particle_masses=sim.model.particle_mass.numpy(),
        edges=sim.model.spring_indices.numpy().reshape(-1, 2)[
            : sim.model.particle_count
        ],
        anchors=sim.anchor_indices,
    )
    (out / "trajectory.pending.npz").replace(out / "trajectory.npz")
    (out / "metrics.json.tmp").write_text(
        json.dumps(
            {"validation": validation, "frames": len(frames), "metrics": metrics},
            indent=2,
        )
    )
    (out / "metrics.json.tmp").replace(out / "metrics.json")
    sim.save_checkpoint(out / "checkpoint.pending.npz")
    (out / "checkpoint.pending.npz").replace(out / "checkpoint.npz")
    if meshes:
        (out / "meshes.json").write_text(
            json.dumps(sim.hand.visuals(max_faces=3000), separators=(",", ":"))
        )
