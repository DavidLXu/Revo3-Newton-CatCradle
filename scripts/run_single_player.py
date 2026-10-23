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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_opening_a_stage1"
    )
    parser.add_argument("--stage", choices=["initial", "right"], default="right")
    parser.add_argument("--substeps", type=int, default=32)
    parser.add_argument("--iterations", type=int, default=24)
    parser.add_argument("--friction", type=float, default=1.2)
    parser.add_argument("--symmetric", action="store_true")
    parser.add_argument("--rest-length-max-scale", type=float, default=2.0)
    parser.add_argument("--approach", choices=["curled", "upright"], default="upright")
    parser.add_argument("--initial", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument(
        "--length-control", choices=["local", "global"], default="local"
    )
    parser.add_argument("--palm", choices=["box", "hull"], default="hull")
    parser.add_argument("--depth", type=float, default=0.025)
    parser.add_argument("--lower", type=float, default=-0.035)
    parser.add_argument("--lift", type=float, default=0.015)
    args = parser.parse_args()
    out = args.output
    r = RevoTron()
    motion = Motion(r, upright=args.approach == "upright", symmetric=args.symmetric)
    cfg = Config(
        substeps=args.substeps,
        iterations=args.iterations,
        adaptive_take_up_m=0.01,
        friction=args.friction,
    )
    sim = BimanualSimulation(
        r,
        motion,
        initial_loop(r, motion),
        cfg,
        palm_geometry=args.palm,
        length_control=args.length_control,
        rest_length_max_scale=args.rest_length_max_scale,
    )
    if args.initial:
        sim.load_checkpoint(args.initial)
    else:
        for _ in range(60):
            sim.advance(0, move=False)
    sim.save_checkpoint(out / "initial.npz")
    frames = [sim.snapshot()]
    metrics = [sim.metrics()]
    initial = sim.state.particle_q.numpy()
    left_ids = np.flatnonzero(
        (initial[:, 1] > 0.07) & (initial[:, 0] > 0.395) & (initial[:, 0] < 0.405)
    )
    if not len(left_ids):
        raise RuntimeError("Left palmar strand not found")
    target = initial[left_ids].mean(axis=0)
    (out).mkdir(parents=True, exist_ok=True)
    (out / "plan.json").write_text(
        json.dumps(
            dict(left_palmar_indices=left_ids.tolist(), left_target=target.tolist()),
            indent=2,
        )
    )
    print("Left target", target, "ids", left_ids, flush=True)

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
    if args.stage == "initial":
        return
    if args.approach == "upright":
        q = motion.targets[-1].copy()
        motion.append(1, q, "Hold Position 1")
        if args.symmetric:
            q = motion.finger(q, "left", "index", [0, 0, 0])
            for f, a in [("index", 0.35), ("middle", 0.4), ("ring", -0.4)]:
                q[r.indices[f"left_{f}_MPR_joint"]] = a
        q = motion.aim_center(q, "right", "index", target + [0.002, -0.20, -0.045])
        motion.append(2.5, q, "Lower right index below left string")
        q = motion.aim_center(q, "right", "index", target + [0.002, 0.012, -0.045])
        motion.append(4, q, "Pass beneath left palmar string")
        q = motion.aim_center(q, "right", "index", target + [0.002, 0.012, 0.012])
        motion.append(5, q, "Pick up left palmar string")
        q[r.indices["right_index_MPR_joint"]] = 0
        q = motion.finger(q, "right", "index", [0.3, 0, 0])
        q = motion.finger(q, "right", "little", [0.4, 0, 0])
        q = motion.finger(q, "left", "little", [0.4, 0, 0])
        q = motion.aim_center(q, "right", "index", target + [0.002, -0.06, 0.045])
        motion.append(6.2, q, "Raise and retain right index bight")
        q = motion.finger(q, "right", "index", [-0.08, -0.1, -0.2])
        q = r.ik(q, "right", np.array([0.4, -0.15, 1.13]), motion.rotations["right"])
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
