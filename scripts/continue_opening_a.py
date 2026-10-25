"""Thread the left index through the right bight and pick up the right palmar strand."""

import sys, json, argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.bimanual_checks import palmar_arc
from revo3_cat_cradle.bimanual_checks import hand_clearance, self_clearance_margin
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH


def pickup_plan(robot, motion, q, target, little_curl):
    """Find a nearby point on the same strand with a clear finger sweep."""
    best = None
    # Root collisions can seat the strand lower, where a deep sweep strikes
    # the opposite middle/ring MCP. Try a shallow pickup without relaxing the
    # clearance gate; contact and bight retention still require a rollout.
    for depth in (0.025, 0.022, 0.018, 0.013, 0.008, 0.003):
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
    parser.add_argument(
        "--pickup-half-span",
        type=float,
        help="Optional wrist half-span while preparing the second cross-hand pickup",
    )
    parser.add_argument("--check-clearance", action="store_true")
    parser.add_argument(
        "--extension-friction",
        type=float,
        help="Change friction at the start of the final two-hand extension",
    )
    parser.add_argument(
        "--extension-rest-length-max-scale",
        type=float,
        help="Local rest-length payout cap during final extension, relative to initial segment lengths",
    )
    parser.add_argument(
        "--right-little-curl",
        type=float,
        help="Right little MCP angle during final extension",
    )
    parser.add_argument(
        "--right-wrist-lift",
        type=float,
        default=0,
        help="Additional right wrist height during final extension, meters",
    )
    parser.add_argument(
        "--right-little-pose",
        type=float,
        nargs=3,
        help="Right little MCP/PIP/DIP angles during final extension",
    )
    parser.add_argument(
        "--right-little-spread",
        type=float,
        help="Right little MPR angle during final extension",
    )
    parser.add_argument(
        "--left-index-pose",
        type=float,
        nargs=3,
        default=[-0.08, -0.1, -0.2],
        help="Left index MCP/PIP/DIP angles during final extension",
    )
    args = parser.parse_args()
    if args.extension_rest_length_max_scale is not None and (
        not np.isfinite(args.extension_rest_length_max_scale)
        or args.extension_rest_length_max_scale < 1
    ):
        parser.error("Extension rest-length cap must be finite and at least one")
    if not np.isfinite(args.right_wrist_lift) or abs(args.right_wrist_lift) > 0.10:
        parser.error("Right wrist lift must be finite and within 0.10 m")
    if (
        not np.isfinite(args.final_half_span)
        or not 0.075 <= args.final_half_span <= 0.30
    ):
        parser.error("Final half-span must be between 0.075 and 0.30 m")
    if args.pickup_half_span is not None and (
        not np.isfinite(args.pickup_half_span)
        or not 0.075 <= args.pickup_half_span <= 0.30
    ):
        parser.error("Pickup half-span must be between 0.075 and 0.30 m")
    if args.extension_friction is not None and (
        not np.isfinite(args.extension_friction) or args.extension_friction < 0
    ):
        parser.error("Extension friction must be finite and nonnegative")
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    payload = json.loads((args.source / "trajectory.json").read_text())
    command = json.loads((args.source / "joint_commands.json").read_text())
    checkpoint = args.checkpoint or args.source / "checkpoint.npz"
    saved = np.load(checkpoint)
    cfg = Config(**json.loads(str(saved["config"])))
    robot = RevoTron()
    ids = [robot.indices[f"left_index_{j}_joint"] for j in ("MCP", "PIP", "DIP")]
    pose = np.asarray(args.left_index_pose)
    if (
        not np.isfinite(pose).all()
        or np.any(pose < robot.limits[ids, 0])
        or np.any(pose > robot.limits[ids, 1])
    ):
        parser.error("Left index pose violates joint limits")
    if args.right_little_spread is not None:
        low, high = robot.limits[robot.indices["right_little_MPR_joint"]]
        if (
            not np.isfinite(args.right_little_spread)
            or not low <= args.right_little_spread <= high
        ):
            parser.error("Right little spread violates joint limits")
    if args.right_little_pose is not None:
        ids = [robot.indices[f"right_little_{j}_joint"] for j in ("MCP", "PIP", "DIP")]
        pose = np.asarray(args.right_little_pose)
        if (
            args.right_little_curl is not None
            or not np.isfinite(pose).all()
            or np.any(pose < robot.limits[ids, 0])
            or np.any(pose > robot.limits[ids, 1])
        ):
            parser.error(
                "Little pose must satisfy joint limits and cannot combine with --right-little-curl"
            )
    if args.right_little_curl is not None:
        limits = robot.limits[robot.indices["right_little_MCP_joint"]]
        if (
            not np.isfinite(args.right_little_curl)
            or not limits[0] <= args.right_little_curl <= limits[1]
        ):
            parser.error("Right little curl violates joint limits")
    motion = Motion(robot)
    motion.times = command["times"]
    motion.targets = [np.array(q) for q in command["targets"]]
    motion.phases = command["phases"]
    keep = [i for i, t in enumerate(motion.times) if t <= float(saved["time"]) + 1e-6]
    motion.times = [motion.times[i] for i in keep]
    motion.targets = [motion.targets[i] for i in keep]
    motion.phases = [motion.phases[i] for i in keep]
    motion.metadata = payload["motion"].copy()
    if "physics_events" in motion.metadata:
        motion.metadata["physics_events"] = [
            event
            for event in motion.metadata["physics_events"]
            if event["time"] < float(saved["time"]) - 1e-6
        ]
    motion.metadata.update(source="Single-player Opening A", status="experimental")
    motion.metadata["opening_half_span_m"] = args.final_half_span
    initial_half_span = motion.metadata.get("initial_half_span_m", 0.15)
    pickup_half_span = args.pickup_half_span or initial_half_span
    motion.metadata["pickup_half_span_m"] = pickup_half_span
    # Legacy checkpoints did not save the length controller. Recover its
    # historical value rather than applying the source trajectory's final cap
    # to an earlier checkpoint. New checkpoints restore their own controller.
    rest_cap = payload.get("rest_length_max_scale", 2.0)
    if "initial_rest_length_max_scale" in motion.metadata:
        rest_cap = motion.metadata["initial_rest_length_max_scale"]
        for event in motion.metadata.get("physics_events", []):
            if event["parameter"] == "rest_length_max_scale":
                rest_cap = event["value"]
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        cfg,
        palm_geometry="hull",
        knuckle_geometry=payload.get("collision_geometry", {}).get("knuckles", "none"),
        length_control=payload.get("length_control", "local"),
        rest_length_max_scale=rest_cap,
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
        if args.check_clearance:
            for u in np.linspace(0, 1, 41):
                transforms = robot.fk(sim.q + u * (q - sim.q))
                if hand_clearance(transforms)[0] < -0.001:
                    raise ValueError(
                        f"Opening A phase crosses the opposing hand: {phase}"
                    )
                if (
                    min(
                        self_clearance_margin(transforms, side)
                        for side in ("left", "right")
                    )
                    < 0
                ):
                    raise ValueError(
                        f"Opening A phase fails finger self-clearance: {phase}"
                    )
        motion.append(t, q, phase)
        print("Plan", t, phase, "hand gap", hand_clearance(robot.fk(q)), flush=True)
        start = round(sim.t * 30) + 1
        for i in range(start, round(t * 30) + 1):
            sim.advance(i / 30)
            frames.append(sim.snapshot())
            metrics.append(sim.metrics())
            points = np.asarray(frames[-1]["rope"])
            longest = float(
                np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1).max()
            )
            if longest >= MAX_SEGMENT_LENGTH:
                (out / "aborted.json").write_text(
                    json.dumps(
                        dict(
                            time_s=sim.t,
                            reason="Rope segment exceeds the collision search bound",
                            maximum_edge_m=longest,
                            search_bound_m=float(MAX_SEGMENT_LENGTH),
                            status="rejected",
                        ),
                        indent=2,
                    )
                )
                save()
                raise RuntimeError(
                    f"Rope segment exceeds collision search bound at {sim.t:.3f} s"
                )
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
        q = robot.ik(
            q,
            "left",
            np.array([0.44, pickup_half_span, 1.13]),
            motion.rotations["left"],
        )
        if args.pickup_half_span is not None:
            q = robot.ik(
                q,
                "right",
                np.array([0.4, -pickup_half_span, 1.13]),
                motion.rotations["right"],
            )
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
        if args.extension_rest_length_max_scale is not None:
            motion.metadata.setdefault(
                "initial_rest_length_max_scale", sim.rest_length_max_scale
            )
            motion.metadata.setdefault("physics_events", []).append(
                dict(
                    time=sim.t,
                    parameter="rest_length_max_scale",
                    previous_value=sim.rest_length_max_scale,
                    value=args.extension_rest_length_max_scale,
                )
            )
            sim.rest_length_max_scale = args.extension_rest_length_max_scale
        if args.extension_friction is not None:
            motion.metadata.setdefault("initial_friction", sim.config.friction)
            motion.metadata.setdefault("physics_events", []).append(
                dict(
                    time=sim.t,
                    parameter="friction",
                    previous_value=sim.config.friction,
                    value=args.extension_friction,
                )
            )
            sim.set_contact_friction(args.extension_friction)
        q = motion.finger(q, "left", "little", [0.4, 0, 0])
        if args.right_little_curl is not None:
            q[robot.indices["right_little_MCP_joint"]] = args.right_little_curl
        if args.right_little_pose is not None:
            q = motion.finger(q, "right", "little", args.right_little_pose)
        if args.right_little_spread is not None:
            q[robot.indices["right_little_MPR_joint"]] = args.right_little_spread
        q = motion.finger(q, "left", "index", args.left_index_pose)
        q = robot.ik(
            q,
            "left",
            np.array([0.4, args.final_half_span, 1.13]),
            motion.rotations["left"],
        )
        q = robot.ik(
            q,
            "right",
            np.array([0.4, -args.final_half_span, 1.13 + args.right_wrist_lift]),
            motion.rotations["right"],
        )
        move(19, q, "Extend both hands into Opening A")
    move(21, q, "Hold Opening A")


if __name__ == "__main__":
    main()
