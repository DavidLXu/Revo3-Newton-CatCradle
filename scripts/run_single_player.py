"""RevoTron single-player string figures: cross-hand pickups from Position 1."""

import sys, json, argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.bimanual_checks import hand_clearance, self_clearance_margin, palmar_arc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_opening_a_stage1"
    )
    parser.add_argument("--stage", choices=["initial", "right"], default="right")
    parser.add_argument("--substeps", type=int, default=32)
    parser.add_argument("--iterations", type=int, default=24)
    parser.add_argument(
        "--stretch-stiffness",
        type=float,
        default=800.0,
        help="Per-edge stretch spring stiffness in N/m",
    )
    parser.add_argument(
        "--stretch-damping",
        type=float,
        default=0.01,
        help="Per-edge axial viscous damping in N s/m",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=0.003,
        help="Initial rope sampling interval in meters",
    )
    parser.add_argument("--friction", type=float, default=1.2)
    parser.add_argument(
        "--tension",
        type=float,
        default=0.08,
        help="Nominal active tension target in Newtons",
    )
    parser.add_argument(
        "--take-up",
        type=float,
        default=0.01,
        help="Additional loop-wide active take-up in meters",
    )
    parser.add_argument(
        "--half-span",
        type=float,
        default=0.15,
        help="Initial half wrist separation; the seated loop grows with this geometry",
    )
    parser.add_argument("--pickup-x", type=float, default=0.002)
    parser.add_argument("--pickup-depth", type=float, default=0.012)
    parser.add_argument("--check-clearance", action="store_true")
    parser.add_argument(
        "--idle-posture",
        type=Path,
        help="JSON 'changed' angles for idle middle/ring MPR/MCP joints",
    )
    parser.add_argument("--symmetric", action="store_true")
    parser.add_argument("--rest-length-max-scale", type=float, default=2.0)
    parser.add_argument("--approach", choices=["curled", "upright"], default="upright")
    parser.add_argument("--initial", type=Path)
    parser.add_argument(
        "--initial-extra-length",
        type=float,
        default=0,
        help="Additional initial rest length in meters; no rope point relocation",
    )
    parser.add_argument(
        "--initial-posture",
        type=Path,
        help="JSON 'changed' thumb joint angles for initial placement",
    )
    parser.add_argument("--settle-seconds", type=float, default=2)
    parser.add_argument(
        "--fixed-length-settle",
        action="store_true",
        help="Hold initial rest lengths during the two-second pre-roll, then enable active length control",
    )
    parser.add_argument("--resume", type=Path)
    parser.add_argument(
        "--length-control", choices=["local", "global"], default="local"
    )
    parser.add_argument("--palm", choices=["box", "hull"], default="hull")
    parser.add_argument("--knuckles", choices=["none", "mesh"], default="none")
    parser.add_argument("--depth", type=float, default=0.025)
    parser.add_argument("--lower", type=float,
                        help="Index approach height below the palmar strand; defaults to -0.045 upright or -0.035 curled")
    parser.add_argument("--lift", type=float, default=0.015)
    args = parser.parse_args()
    if args.lower is None:
        args.lower = -0.045 if args.approach == "upright" else -0.035
    if not np.isfinite(args.lower) or not -0.10 <= args.lower < 0:
        parser.error("Pickup approach height must be finite, negative and at least -0.10 m")
    if not np.isfinite(args.initial_extra_length) or args.initial_extra_length < 0:
        parser.error("Initial extra length must be finite and nonnegative")
    if not np.isfinite(args.settle_seconds) or args.settle_seconds < 1 / 30:
        parser.error("Settling must last at least one frame")
    if not np.isfinite(args.stretch_stiffness) or args.stretch_stiffness <= 0:
        parser.error("Stretch stiffness must be finite and positive")
    if not np.isfinite(args.stretch_damping) or args.stretch_damping < 0:
        parser.error("Stretch damping must be finite and nonnegative")
    if not np.isfinite(args.tension) or args.tension < 0:
        parser.error("Tension must be finite and nonnegative")
    if not np.isfinite(args.take_up) or not 0 <= args.take_up <= 0.05:
        parser.error("Take-up must be between 0 and 0.05 m")
    if not np.isfinite(args.spacing) or not 0 < args.spacing < 0.016:
        parser.error("Initial rope spacing must be finite, positive and below 0.016 m")
    out = args.output
    r = RevoTron()
    motion = Motion(
        r,
        upright=args.approach == "upright",
        symmetric=args.symmetric,
        half_span=args.half_span,
    )
    motion.metadata["initial_rope_extra_m"] = args.initial_extra_length
    if args.initial_posture:
        changes = json.loads(args.initial_posture.read_text())["changed"]
        allowed = {
            f"{s}_thumb_{j}_joint"
            for s in ("left", "right")
            for j in ("CMP", "CMR", "MCP", "PIP", "DIP")
        }
        for name, angle in changes.items():
            if name not in allowed or not np.isfinite(angle):
                parser.error(
                    "Initial posture may only change finite thumb joint angles"
                )
            i = r.indices[name]
            if not r.limits[i, 0] <= angle <= r.limits[i, 1]:
                parser.error("Initial posture violates joint limits")
            motion.targets[0][i] = angle
        motion.metadata["initial_thumb_posture"] = changes
    cfg = Config(
        stretch_stiffness=args.stretch_stiffness,
        stretch_damping=args.stretch_damping,
        spacing=args.spacing,
        substeps=args.substeps,
        iterations=args.iterations,
        adaptive_take_up_m=args.take_up,
        adaptive_tension_n=args.tension,
        friction=args.friction,
    )
    sim = BimanualSimulation(
        r,
        motion,
        initial_loop(r, motion),
        cfg,
        palm_geometry=args.palm,
        knuckle_geometry=args.knuckles,
        length_control=args.length_control,
        rest_length_max_scale=args.rest_length_max_scale,
    )
    motion.metadata["initial_rest_length_m"] = float(sim.initial_rest_lengths.sum())
    motion.metadata["initial_centerline_length_m"] = float(
        np.linalg.norm(
            np.roll(sim.initial_positions, -1, axis=0) - sim.initial_positions, axis=1
        ).sum()
    )
    if args.initial:
        if args.fixed_length_settle:
            parser.error(
                "Fixed-length settling cannot be combined with a restored initial state"
            )
        sim.load_checkpoint(args.initial)
    else:
        settling = []
        cfg.adaptive_length = not args.fixed_length_settle
        settle_frames = round(args.settle_seconds * cfg.fps)
        for _ in range(settle_frames):
            sim.advance(0, move=False)
            settling.append(
                dict(
                    positions=sim.state.particle_q.numpy().copy(),
                    velocities=sim.state.particle_qd.numpy().copy(),
                    rest_lengths=sim.rest_lengths.copy(),
                    peak_stretch=sim.metrics()["max_stretch_ratio"],
                )
            )
        cfg.adaptive_length = True
        motion.metadata["settling"] = dict(
            duration_s=settle_frames / cfg.fps,
            fixed_rest_lengths=args.fixed_length_settle,
        )
        out.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            out / "settling.npz",
            time=np.arange(1, settle_frames + 1) / cfg.fps,
            rope=[s["positions"] for s in settling],
            velocities=[s["velocities"] for s in settling],
            rest_lengths=[s["rest_lengths"] for s in settling],
        )
        (out / "settling_audit.json").write_text(
            json.dumps(
                dict(
                    fixed_rest_lengths=args.fixed_length_settle,
                    peak_stretch=max(s["peak_stretch"] for s in settling),
                    final_max_speed_m_s=float(
                        np.linalg.norm(settling[-1]["velocities"], axis=1).max()
                    ),
                    final_rest_length_m=float(sim.rest_lengths.sum()),
                ),
                indent=2,
            )
        )
    sim.save_checkpoint(out / "initial.npz")
    frames = [sim.snapshot()]
    metrics = [sim.metrics()]
    initial = sim.state.particle_q.numpy()

    def save():
        export_bimanual(sim, frames, metrics, out)
        (out / "joint_commands.json").write_text(
            json.dumps(
                dict(
                    times=motion.times,
                    targets=[q.tolist() for q in motion.targets],
                    phases=motion.phases,
                ),
                indent=2,
            )
        )

    save()
    # A coordinate corridor alone can select another span after a thumb loop
    # falls off. First identify the actual palmar material between its supports.
    source_arcs = {s: palmar_arc(r, sim.q, initial, s) for s in ("left", "right")}
    left_arc = source_arcs["left"]
    left_ids = left_arc[(initial[left_arc, 1] > 0.07)
                        & (initial[left_arc, 0] > 0.395)
                        & (initial[left_arc, 0] < 0.405)]
    if not len(left_ids):
        raise RuntimeError("Left palmar strand not found in the pickup corridor")
    target = initial[left_ids].mean(axis=0)
    (out / "plan.json").write_text(json.dumps(dict(
        left_palmar_indices=left_arc.tolist(),
        left_target_indices=left_ids.tolist(),
        left_target=target.tolist(),
    ), indent=2))
    print("Left target", target, "ids", left_ids, flush=True)
    if args.stage == "initial":
        return
    if args.approach == "upright":
        q = motion.targets[-1].copy()
        if args.idle_posture:
            changes = json.loads(args.idle_posture.read_text())["changed"]
            allowed = {
                f"{s}_{f}_{j}_joint"
                for s in ("left", "right")
                for f in ("middle", "ring")
                for j in ("MPR", "MCP")
            }
            for name, angle in changes.items():
                if name not in allowed or not np.isfinite(angle):
                    raise ValueError(
                        "Idle posture may only change finite middle/ring MPR/MCP angles"
                    )
                i = r.indices[name]
                if not r.limits[i, 0] <= angle <= r.limits[i, 1]:
                    raise ValueError("Idle posture violates joint limits")
                q[i] = angle
            motion.append(1, q, "Clear idle fingers for the first pickup")
        else:
            motion.append(1, q, "Hold Position 1")
        if args.symmetric:
            q = motion.finger(q, "left", "index", [0, 0, 0])
            for f, a in [("index", 0.35), ("middle", 0.4), ("ring", -0.4)]:
                q[r.indices[f"left_{f}_MPR_joint"]] = a
        q = motion.aim_center(
            q, "right", "index", target + [args.pickup_x, -0.20, args.lower]
        )
        motion.append(2.5, q, "Lower right index below left string")
        q = motion.aim_center(
            q, "right", "index", target + [args.pickup_x, args.pickup_depth, args.lower]
        )
        motion.append(4, q, "Pass beneath left palmar string")
        q = motion.aim_center(
            q, "right", "index", target + [args.pickup_x, args.pickup_depth, 0.012]
        )
        motion.append(5, q, "Pick up left palmar string")
        q[r.indices["right_index_MPR_joint"]] = 0
        q = motion.finger(q, "right", "index", [0.3, 0, 0])
        q = motion.finger(q, "right", "little", [0.4, 0, 0])
        q = motion.finger(q, "left", "little", [0.4, 0, 0])
        q = motion.aim_center(
            q, "right", "index", target + [args.pickup_x, -0.06, 0.045]
        )
        motion.append(6.2, q, "Raise and retain right index bight")
        q = motion.finger(q, "right", "index", [-0.08, -0.1, -0.2])
        q = r.ik(
            q,
            "right",
            np.array([0.4, -args.half_span, 1.13]),
            motion.rotations["right"],
        )
        motion.append(8, q, "Extend right hand with captured bight")
        motion.append(10, q, "Hold right index bight")
    else:
        q = motion.targets[-1].copy()
        motion.append(1, q, "Hold Position 1")
        q[r.indices["right_index_MPR_joint"]] = 0.4
        q[r.indices["right_middle_MPR_joint"]] = 0
        q[r.indices["right_ring_MPR_joint"]] = 0
        motion.append(1.5, q, "Abduct right index")
        q = motion.finger(q, "right", "index", [1.58, 0.15, 0.1])
        motion.append(2.5, q, "Prepare right index")
        q[r.indices["right_index_MPR_joint"]] = 0.0
        q = motion.aim_tip(q, "right", "index", target + [0.002, -0.06, args.lower])
        motion.append(3, q, "Approach left palm")
        q = motion.aim_tip(
            q, "right", "index", target + [0.002, args.depth, args.lower]
        )
        motion.append(4, q, "Pass below left palmar string")
        q = motion.finger(q, "right", "index", [1.4, -0.14, -0.3])
        q = motion.aim_tip(q, "right", "index", target + [0.002, args.depth, args.lift])
        motion.append(5, q, "Lift left palmar string")
        tf = r.fk(q)
        c, axis, rad, h = finger_capsule("index", "DIP", "right")
        center = (tf["right_index_DIP_link"] @ np.r_[c, 1])[:3]
        q[r.indices["right_index_MPR_joint"]] = 0
        q[r.indices["right_middle_MPR_joint"]] = 0
        q[r.indices["right_ring_MPR_joint"]] = 0
        q = motion.finger(q, "right", "index", [0.7, 0, 0])
        q = motion.finger(q, "right", "little", [0.4, 0, 0])
        q = motion.aim_center(q, "right", "index", center + [0, -0.025, 0])
        motion.append(5.8, q, "Raise index with captured loop")
        q = motion.finger(q, "right", "index", [-0.08, -0.1, -0.2])
        q = motion.finger(q, "right", "little", [-0.08, 0, 0])
        q = motion.aim_center(q, "right", "index", center + [0, -0.08, 0.015])
        motion.append(6.6, q, "Secure index loop")
        q = motion.aim_center(q, "right", "index", center + [0, -0.18, 0.015])
        motion.append(8, q, "Draw right index loop")
        motion.append(10, q, "Hold right index loop")
    if args.check_clearance:
        margins = []
        for qa, qb, phase in zip(motion.targets, motion.targets[1:], motion.phases[1:]):
            transforms = [r.fk(qa + u * (qb - qa)) for u in np.linspace(0, 1, 41)]
            margins.append(
                dict(
                    phase=phase,
                    gap_m=min(hand_clearance(tf)[0] for tf in transforms),
                    self_margin_m=min(
                        self_clearance_margin(tf, side)
                        for tf in transforms
                        for side in ("left", "right")
                    ),
                )
            )
        (out / "planned_clearance.json").write_text(json.dumps(margins, indent=2))
        if any(item["gap_m"] < -0.001 for item in margins):
            raise ValueError("Planned pickup intersects the opposing hand")
        if any(item["self_margin_m"] < 0 for item in margins):
            raise ValueError("Planned pickup fails same-hand finger clearance")
    if args.resume:
        sim.load_checkpoint(args.resume)
        count = round(sim.t * 30) + 1
        previous = json.loads((args.resume.parent / "trajectory.json").read_text())
        frames = previous["frames"][:count]
        metrics = json.loads((args.resume.parent / "metrics.json").read_text())[
            "metrics"
        ][:count]
    for i in range(round(sim.t * 30) + 1, 301):
        sim.advance(i / 30)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        if (
            not np.isfinite(sim.state.particle_q.numpy()).all()
            or metrics[-1]["max_coordinate_m"] > 3
        ):
            save()
            raise RuntimeError(f"Rope left the workspace at {sim.t:.3f} s")
        if i % 30 == 0:
            save()
            sim.save_checkpoint(out / f"state_{i:04d}.npz")
            print(sim.t, sim.phase, metrics[-1]["max_stretch_ratio"], flush=True)
    save()


if __name__ == "__main__":
    main()
