"""Thread the left index through the right bight and pick up the right palmar strand."""

import sys, json, argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.bimanual_checks import palmar_arc
from revo3_cat_cradle.bimanual_checks import hand_clearance
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.sim import Config


def pickup_plan(robot, motion, q, target, little_curl):
    """Find a nearby point on the same strand with a clear finger sweep."""
    best = None
    for depth in (0.025, 0.022, 0.018):
        for offset in (0, 0.002, 0.004, 0.006, 0.008, 0.010):
            point = target + np.array([offset, 0, 0])
            trial = motion.finger(q, "left", "little", [little_curl, 0, 0])
            trial = motion.finger(trial, "left", "index", [1.58, 0.15, 0.1])
            approach = motion.aim_tip(trial, "left", "index", point + [0, 0.06, -0.035])
            under = motion.aim_tip(
                approach, "left", "index", point + [0, -depth, -0.035]
            )
            trial = motion.finger(under, "left", "index", [1.4, -0.14, -0.3])
            lifted = motion.aim_tip(trial, "left", "index", point + [0, -depth, 0.02])
            margin = min(
                hand_clearance(robot.fk((1 - u) * a + u * b))[0]
                for a, b in ((q, approach), (approach, under), (under, lifted))
                for u in np.linspace(0, 1, 17)
            )
            candidate = (margin, point, [approach, under, lifted], depth)
            if best is None or margin > best[0]:
                best = candidate
            if margin >= 0.001:
                return candidate
    if best[0] < -0.001:
        raise RuntimeError(f"No collision-free left pickup: {best[0] * 1000:.1f} mm")
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, default=ROOT / "output/revotron_opening_a_stage1"
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_opening_a"
    )
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--little-curl", type=float, default=0.9)
    parser.add_argument("--final-half-span", type=float, default=0.075)
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    payload = json.loads((args.source / "trajectory.json").read_text())
    command = json.loads((args.source / "joint_commands.json").read_text())
    checkpoint = args.checkpoint or args.source / "checkpoint.npz"
    saved = np.load(checkpoint)
    cfg = Config(**json.loads(str(saved["config"])))
    robot = RevoTron()
    motion = Motion(robot)
    motion.times = command["times"]
    motion.targets = [np.array(q) for q in command["targets"]]
    motion.phases = command["phases"]
    keep = [i for i, t in enumerate(motion.times) if t <= float(saved["time"]) + 1e-6]
    motion.times = [motion.times[i] for i in keep]
    motion.targets = [motion.targets[i] for i in keep]
    motion.phases = [motion.phases[i] for i in keep]
    motion.metadata = payload["motion"].copy()
    motion.metadata.update(source="Single-player Opening A", status="experimental")
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        cfg,
        palm_geometry="hull",
        length_control=payload.get("length_control", "local"),
        rest_length_max_scale=payload.get("rest_length_max_scale", 2.0),
    )
    sim.load_checkpoint(checkpoint)
    count = round(sim.t * 30) + 1
    frames = payload["frames"][:count]
    metrics = json.loads((args.source / "metrics.json").read_text())["metrics"][:count]
    p0 = np.array(frames[0]["rope"])
    q0 = np.array(frames[0]["joints"])
    right_ids = palmar_arc(robot, q0, p0, "right")
    plan = json.loads((args.source / "plan.json").read_text())
    plan["right_palmar_indices"] = right_ids.tolist()
    (out / "plan.json").write_text(json.dumps(plan, indent=2))

    def save():
        export_bimanual(sim, frames, metrics, out)
        (out / "joint_commands.json").write_text(
            json.dumps(
                dict(
                    times=motion.times,
                    targets=[q.tolist() for q in motion.targets],
                    phases=motion.phases,
                )
            )
        )

    def move(t, q, phase):
        motion.append(t, q, phase)
        print("Plan", t, phase, "hand gap", hand_clearance(robot.fk(q)), flush=True)
        start = round(sim.t * 30) + 1
        for i in range(start, round(t * 30) + 1):
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
                print(sim.t, phase, metrics[-1]["max_stretch_ratio"], flush=True)
        save()
        sim.save_checkpoint(out / f"state_{round(t * 30):04d}.npz")

    q = sim.q.copy()
    if sim.t < 11.5:
        for f, a in [("index", 0.35), ("middle", 0.4), ("ring", -0.4)]:
            q[robot.indices[f"right_{f}_MPR_joint"]] = a
        for f in ("index", "middle", "ring"):
            q[robot.indices[f"left_{f}_MPR_joint"]] = 0
        q = robot.ik(q, "left", np.array([0.44, 0.15, 1.13]), motion.rotations["left"])
        move(11.5, q, "Open a passage through the right index bight")
    if sim.t < 13:
        q = motion.finger(q, "left", "index", [1.5, 0.15, 0.1])
        move(13, q, "Thread left index through the right bight")
        p = sim.state.particle_q.numpy()
        candidate = right_ids[np.abs(p[right_ids, 0] - 0.4) < 0.012]
        if not len(candidate):
            raise RuntimeError("Right palmar strand is not in the pickup corridor")
        target = p[candidate].mean(0)
        plan.update(
            right_target=target.tolist(),
            right_target_indices=candidate.tolist(),
            threading_interval_s=[11.5, 13],
        )
        (out / "plan.json").write_text(json.dumps(plan, indent=2))
        print("Right target", target, flush=True)
    target = np.array(plan["right_target"])
    if sim.t < 16:
        margin, target, pickup_targets, depth = pickup_plan(
            robot, motion, q, target, args.little_curl
        )
        plan.update(
            right_target=target.tolist(),
            pickup_hand_clearance_m=float(margin),
            pickup_depth_m=depth,
        )
        (out / "plan.json").write_text(json.dumps(plan, indent=2))
        print("Pickup corridor", target, "clearance", margin, flush=True)
    if sim.t < 14:
        q = pickup_targets[0]
        move(14, q, "Approach right palmar string from inside the bight")
    if sim.t < 15:
        q = pickup_targets[1]
        move(15, q, "Pass under right palmar string")
    if sim.t < 16:
        q = pickup_targets[2]
        move(16, q, "Pick up right palmar string")
    if sim.t < 17.2:
        q = motion.finger(q, "left", "index", [0.3, 0, 0])
        q = motion.aim_center(q, "left", "index", target + [0, 0.07, 0.045])
        move(17.2, q, "Raise left index with captured strand")
    if sim.t < 19:
        q = motion.finger(q, "left", "little", [0.4, 0, 0])
        q = motion.finger(q, "left", "index", [-0.08, -0.1, -0.2])
        q = robot.ik(
            q,
            "left",
            np.array([0.4, args.final_half_span, 1.13]),
            motion.rotations["left"],
        )
        q = robot.ik(
            q,
            "right",
            np.array([0.4, -args.final_half_span, 1.13]),
            motion.rotations["right"],
        )
        move(19, q, "Extend both hands into Opening A")
    move(21, q, "Hold Opening A")


if __name__ == "__main__":
    main()
