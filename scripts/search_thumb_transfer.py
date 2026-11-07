"""Offline contact-aware search; every proposal is replayed in Newton from a checkpoint.

The saved winner is an experimental continuation, not an accepted string figure.
No particle positions, velocities, masses or hand attachments are optimized.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.bimanual_checks import gaps, hand_clearance
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.transfers import original_thumb_loop, support_arc, thumb_arc_sweep

CONTEXT = None


def initialize(source, reference, duration, score_mode="contact"):
    global CONTEXT
    source, reference = Path(source), Path(reference)
    payload = json.loads((source / "trajectory.json").read_text())
    commands = json.loads((source / "joint_commands.json").read_text())
    seed = np.load(source / "checkpoint.npz")
    original = np.load(reference / "checkpoint.npz")
    robot = RevoTron()
    motion = Motion(robot)
    motion.times = commands["times"].copy()
    motion.targets = [np.array(q) for q in commands["targets"]]
    motion.phases = commands["phases"].copy()
    motion.metadata = payload["motion"].copy()
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        Config(**json.loads(str(seed["config"]))),
        palm_geometry="hull",
        knuckle_geometry=payload.get("collision_geometry", {}).get("knuckles", "none"),
        robot_body_geometry=payload.get("collision_geometry", {}).get("robot_body", "none"),
        length_control=payload.get("length_control", "local"),
        rest_length_max_scale=payload.get("rest_length_max_scale", 2.0),
    )
    plan = {}
    for side, other in [("left", "right"), ("right", "left")]:
        q0, p0 = original["joints"], original["particle_q"]
        plan[side] = dict(
            far_indices=support_arc(
                robot, q0, p0, (side, "index"), (other, "little")
            ).tolist(),
            near_indices=support_arc(
                robot, q0, p0, (side, "index"), (other, "thumb")
            ).tolist(),
            old_loop_indices=original_thumb_loop(robot, q0, p0, side).tolist(),
            old_thumb_indices=np.flatnonzero(
                gaps(robot.fk(q0), p0, side, "thumb") < 0.0015
            ).tolist(),
        )
    CONTEXT = dict(
        source=source,
        payload=payload,
        commands=commands,
        robot=robot,
        motion=motion,
        sim=sim,
        plan=plan,
        duration=duration,
        score_mode=score_mode,
        q=seed["joints"].copy(),
        time=float(seed["time"]),
    )


def measure(sim, plan):
    p = sim.state.particle_q.numpy()
    tf = sim.transforms
    result = {}
    for side in ("left", "right"):
        for name in ("far_indices", "near_indices", "old_loop_indices"):
            result[f"{side}_{name}"] = float(
                gaps(tf, p[plan[side][name]], side, "thumb").min()
            )
    result["other_support_gap"] = max(
        float(gaps(tf, p, side, finger).min())
        for side in ("left", "right")
        for finger in ("index", "little")
    )
    result["stretch"] = sim.metrics()["max_stretch_ratio"]
    if CONTEXT["score_mode"] == "wrap":
        for side in ("left", "right"):
            for kind in ("far_indices", "near_indices", "old_loop_indices"):
                result[f"{side}_{kind}_sweep"] = max(
                    thumb_arc_sweep(tf, p[plan[side][kind]], side, segment)
                    for segment in ("PIP", "DIP")
                )
    return result


def rollout(proposal, keep=False):
    c = CONTEXT
    robot, motion, sim = c["robot"], c["motion"], c["sim"]
    q0 = c["q"]
    waypoints = [np.array(q) for q in proposal.get("waypoints", [proposal["joints"]])]
    # Reject paths through the other hand and newly introduced self-overlap.
    thumb_pairs = lambda tf: min(
        g
        for g, a, b in hand_clearance(tf, "right", all_pairs=True)
        if a.startswith("thumb_") or b.startswith("thumb_")
    )
    original_self = thumb_pairs(robot.fk(q0))
    previous = q0
    for q in waypoints:
        for u in np.linspace(0, 1, 9):
            tf = robot.fk((1 - u) * previous + u * q)
            if hand_clearance(tf)[0] < -0.0005 or thumb_pairs(tf) < min(
                -0.0005, original_self - 0.0005
            ):
                return dict(id=proposal["id"], score=None, rejected="hand collision")
        previous = q
    motion.times = c["commands"]["times"].copy()
    motion.targets = [np.array(v) for v in c["commands"]["targets"]]
    motion.phases = c["commands"]["phases"].copy()
    for step, q in enumerate(waypoints, start=1):
        motion.append(
            c["time"] + step * c["duration"], q, "Contact-aware thumb and wrist sweep"
        )
    motion.append(
        c["time"] + len(waypoints) * c["duration"] + 0.5,
        waypoints[-1],
        "Hold new thumb bights",
    )
    sim.load_checkpoint(c["source"] / "checkpoint.npz")
    states, frames, metrics = [], [], []
    first = round(sim.t * 30) + 1
    for frame in range(first, round(motion.times[-1] * 30) + 1):
        sim.advance(frame / 30)
        states.append(measure(sim, c["plan"]))
        if keep:
            frames.append(sim.snapshot())
            metrics.append(sim.metrics())
    last = states[-15:]
    worst = lambda name: max(s[name] for s in states)
    closest = lambda name: min(s[name] for s in last)
    # Units are mm. All retained loops and non-target supports matter, not just
    # fingertip proximity to the desired strand. Physics failures remain visible.
    far = max(s["right_far_indices"] for s in last) * 1000
    loss = sum(
        max(worst(name) * 1000 - 1.5, 0)
        for name in (
            "right_old_loop_indices",
            "left_far_indices",
            "left_old_loop_indices",
            "other_support_gap",
        )
    )
    wrong = sum(
        max(2 - closest(name) * 1000, 0)
        for name in ("right_near_indices", "left_near_indices")
    )
    stretch = worst("stretch")
    score = max(far, 0) + 8 * loss + 3 * wrong + 10 * max(stretch - 1.2, 0)
    if c["score_mode"] == "wrap":
        # Contact with the proximal side is not yet a captured bight. Encourage
        # an open-arc sweep around the thumb, while preserving the existing loops.
        target_sweep = min(s["right_far_indices_sweep"] for s in last)
        lost_sweep = sum(
            max(np.pi - min(s[name] for s in last), 0) / np.pi
            for name in (
                "left_far_indices_sweep",
                "left_old_loop_indices_sweep",
                "right_old_loop_indices_sweep",
            )
        )
        wrong_sweep = sum(
            max(max(s[name] for s in last) - np.pi, 0) / np.pi
            for name in ("left_near_indices_sweep", "right_near_indices_sweep")
        )
        score = (
            max(far, 0)
            + 8 * loss
            + 0.5 * wrong
            + 10 * max(stretch - 1.2, 0)
            + 20 * max(np.pi - target_sweep, 0) / np.pi
            + 25 * lost_sweep
            + 40 * wrong_sweep
        )
    result = dict(
        id=proposal["id"],
        score=float(score),
        target_gap_mm=float(far),
        support_loss_mm=float(loss),
        wrong_strand_penalty_mm=float(wrong),
        peak_stretch=float(stretch),
        final=states[-1],
    )
    if c["score_mode"] == "wrap":
        result.update(
            minimum_target_sweep_rad=float(target_sweep),
            minimum_retained_sweep_rad=float(
                min(
                    s[name]
                    for s in last
                    for name in (
                        "left_far_indices_sweep",
                        "left_old_loop_indices_sweep",
                        "right_old_loop_indices_sweep",
                    )
                )
            ),
            maximum_near_sweep_rad=float(
                max(
                    s[name]
                    for s in last
                    for name in ("left_near_indices_sweep", "right_near_indices_sweep")
                )
            ),
        )
    if keep:
        result.update(frames=frames, metrics=metrics)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--support-reference", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--samples", type=int, default=24)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument("--duration", type=float, default=1.0)
    p.add_argument(
        "--horizon",
        type=int,
        default=1,
        help="Number of sequential joint targets per physical rollout",
    )
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--score-mode", choices=["contact", "wrap"], default="contact")
    p.add_argument(
        "--candidate-file",
        type=Path,
        help="Replay one saved proposal instead of sampling; it remains experimental",
    )
    p.add_argument(
        "--center-proposal",
        type=Path,
        help="Refine around a saved proposal while replaying every candidate from --source",
    )
    p.add_argument(
        "--wrist-rotation-std",
        type=float,
        default=0.0,
        help="World rotation-vector proposal standard deviation in radians",
    )
    args = p.parse_args()
    if args.horizon < 1 or args.duration <= 0:
        p.error("horizon and duration must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    manifest["source_checkpoint_sha256"] = hashlib.sha256(
        (args.source / "checkpoint.npz").read_bytes()
    ).hexdigest()
    manifest["segment_contacts_sha256"] = hashlib.sha256(
        (ROOT / "revo3_cat_cradle/segment_contacts.py").read_bytes()
    ).hexdigest()
    (args.output / "search_config.json").write_text(json.dumps(manifest, indent=2))
    robot = RevoTron()
    q0 = np.load(args.source / "checkpoint.npz")["joints"]
    ids = [i for i, name in enumerate(robot.names) if name.startswith("right_thumb_")]
    center = q0.copy()
    if args.center_proposal:
        center = np.array(json.loads(args.center_proposal.read_text())["joints"])
        active = set(ids) | set(robot.arm_indices["right"])
        fixed = [i for i in range(len(q0)) if i not in active]
        if center.shape != q0.shape or not np.array_equal(center[fixed], q0[fixed]):
            raise ValueError(
                "Proposal center changes joints outside the right thumb and arm"
            )
    rng = np.random.default_rng(args.seed)
    std = np.array(
        [0.25] * 5
        + [0.01] * 3
        + ([args.wrist_rotation_std] * 3 if args.wrist_rotation_std else [])
    )
    bound = np.array(
        [0.45] * 5 + [0.018] * 3 + ([0.2] * 3 if args.wrist_rotation_std else [])
    )
    proposals = [dict(id=0, joints=q0.tolist(), control=[0.0] * len(std))]
    if args.horizon > 1:
        proposals[0]["waypoints"] = [q0.tolist()] * args.horizon
    for i in range(1, 1 if args.candidate_file else args.samples):
        q = center.copy()
        waypoints, controls = [], []
        for step in range(args.horizon):
            wrist = robot.wrist(q, "right")
            control = np.clip(
                rng.normal(size=len(std)) * std * args.scale, -bound, bound
            )
            q[ids] = np.clip(
                q[ids] + control[:5], robot.limits[ids, 0], robot.limits[ids, 1]
            )
            try:
                rotation = wrist[:3, :3]
                if args.wrist_rotation_std:
                    rotation = Rotation.from_rotvec(control[8:]).as_matrix() @ rotation
                q = robot.ik(q, "right", wrist[:3, 3] + control[5:8], rotation)
            except ValueError:
                break
            waypoints.append(q.tolist())
            controls.append(control.tolist())
        if len(waypoints) != args.horizon:
            continue
        proposal = dict(id=i, joints=q.tolist(), control=controls[-1])
        if args.horizon > 1:
            proposal.update(waypoints=waypoints, controls=controls)
        proposals.append(proposal)
    if args.candidate_file:
        proposals = [json.loads(args.candidate_file.read_text())]
    (args.output / "proposals.json").write_text(json.dumps(proposals, indent=2))
    results = []
    with ProcessPoolExecutor(
        max_workers=args.workers,
        initializer=initialize,
        initargs=(
            str(args.source),
            str(args.support_reference),
            args.duration,
            args.score_mode,
        ),
    ) as pool:
        for result in pool.map(rollout, proposals):
            results.append(result)
            print(json.dumps(result), flush=True)
            (args.output / "search_history.json").write_text(
                json.dumps(results, indent=2)
            )
    valid = [r for r in results if r["score"] is not None]
    if not valid:
        raise RuntimeError("No geometrically valid candidate")
    winner = min(valid, key=lambda r: r["score"])
    proposal = next(p for p in proposals if p["id"] == winner["id"])
    initialize(
        str(args.source), str(args.support_reference), args.duration, args.score_mode
    )
    replay = rollout(proposal, keep=True)
    if not np.isclose(replay["score"], winner["score"], atol=1e-5):
        raise RuntimeError("Selected replay differs from its search rollout")
    c = CONTEXT
    c["motion"].metadata.update(
        status="experimental",
        figure="Thumb-loop transfer",
        source="Contact-aware candidate search",
    )
    frames = c["payload"]["frames"] + replay.pop("frames")
    metrics = json.loads((args.source / "metrics.json").read_text())[
        "metrics"
    ] + replay.pop("metrics")
    export_bimanual(c["sim"], frames, metrics, args.output)
    c["sim"].save_checkpoint(args.output / "checkpoint.npz")
    (args.output / "transfer_plan.json").write_text(json.dumps(c["plan"], indent=2))
    (args.output / "joint_commands.json").write_text(
        json.dumps(
            dict(
                times=c["motion"].times,
                targets=[q.tolist() for q in c["motion"].targets],
                phases=c["motion"].phases,
            )
        )
    )
    (args.output / "winner.json").write_text(
        json.dumps(dict(proposal=proposal, evaluation=replay), indent=2)
    )
    print(
        "Selected",
        proposal["id"],
        "score",
        replay["score"],
        "(still requires full validation)",
        flush=True,
    )


if __name__ == "__main__":
    main()
