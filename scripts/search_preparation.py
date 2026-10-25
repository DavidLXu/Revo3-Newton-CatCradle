"""Search coordinated hand poses before thumb pickup, using full Newton rollouts.

The objective preserves the six original supports and evaluates static thumb
access only after the rope settles. A selected preparation is not a pickup.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import search_thumb_transfer as engine
from revo3_cat_cradle.bimanual import RevoTron, export_bimanual
from revo3_cat_cradle.bimanual_checks import gaps, hand_clearance
from revo3_cat_cradle.transfers import fingertip, support_arc


def initialize(source, reference, duration):
    engine.initialize(source, reference, duration)


def thumb_access(robot, q, rope, plan):
    """Rank reachable, clear approach/reach endpoints; not a dynamic certificate."""
    try:
        arc = support_arc(robot, q, rope, ("left", "index"), ("right", "little"))
        near = support_arc(robot, q, rope, ("left", "index"), ("right", "thumb"))
    except ValueError as exc:
        return dict(cost_mm=100.0, rejected=str(exc))
    arc = np.intersect1d(arc, plan["left"]["far_indices"])
    tf = robot.fk(q)
    arc = arc[
        (gaps(tf, rope[arc], "left", "index") > 0.003)
        & (gaps(tf, rope[arc], "right", "little") > 0.003)
    ]
    if not len(arc):
        return dict(cost_mm=100.0, rejected="No free far-strand section")
    # Keep points near this thumb, but sample the whole free section rather than
    # selecting a fixed world-Y coordinate that may be outside its workspace.
    arc = arc[
        np.linalg.norm(rope[arc] - fingertip(robot, q, "left", "thumb"), axis=1) < 0.12
    ]
    if not len(arc):
        return dict(cost_mm=100.0, rejected="Far strand outside thumb neighborhood")
    arc = arc[np.unique(np.linspace(0, len(arc) - 1, min(8, len(arc))).astype(int))]
    ids = [i for i, n in enumerate(robot.names) if n.startswith("left_thumb_")]
    candidates = []
    for k in arc:
        trial = q.copy()
        poses, errors, margins = [], [], []
        for offset in ([-0.012, 0, 0.005], [0.012, 0, 0.005]):
            target = rope[k] + offset
            original = trial[ids].copy()

            def residual(x):
                trial[ids] = x
                return np.r_[
                    (fingertip(robot, trial, "left", "thumb") - target) * 100,
                    0.01 * (x - original),
                ]

            fit = least_squares(
                residual,
                original,
                bounds=(robot.limits[ids, 0], robot.limits[ids, 1]),
                max_nfev=100,
            )
            trial[ids] = fit.x
            transforms = robot.fk(trial)
            errors.append(
                float(np.linalg.norm(fingertip(robot, trial, "left", "thumb") - target))
            )
            own = min(
                g
                for g, a, b in hand_clearance(transforms, "left", all_pairs=True)
                if a.startswith("thumb_") or b.startswith("thumb_")
            )
            margins.append(
                dict(
                    own=own,
                    other=hand_clearance(transforms)[0],
                    near=float(gaps(transforms, rope[near], "left", "thumb").min()),
                )
            )
            poses.append(trial.tolist())
        penalty = max(errors) * 1000
        for margin in margins:
            penalty += 4 * max(-0.0005 - min(margin["own"], margin["other"]), 0) * 1000
            penalty += 2 * max(0.0015 - margin["near"], 0) * 1000
        candidates.append(
            dict(
                cost_mm=float(penalty),
                target_index=int(k),
                target=rope[k].tolist(),
                errors_m=errors,
                clearances_m=margins,
                waypoints=poses,
                over_height_m=0.005,
            )
        )
    return min(candidates, key=lambda x: x["cost_mm"])


def rollout(proposal, keep=False):
    c = engine.CONTEXT
    robot, motion, sim = c["robot"], c["motion"], c["sim"]
    q0, q = c["q"], np.asarray(proposal["joints"])
    if (
        not np.isfinite(q).all()
        or np.any(q < robot.limits[:, 0])
        or np.any(q > robot.limits[:, 1])
    ):
        return dict(id=proposal["id"], score=None, rejected="joint limits")
    original_self = {s: hand_clearance(robot.fk(q0), s)[0] for s in ("left", "right")}
    for u in np.linspace(0, 1, 17):
        tf = robot.fk((1 - u) * q0 + u * q)
        if hand_clearance(tf)[0] < -0.0005 or any(
            hand_clearance(tf, s)[0] < min(-0.0005, original_self[s] - 0.0005)
            for s in ("left", "right")
        ):
            return dict(id=proposal["id"], score=None, rejected="hand collision")
    motion.times = c["commands"]["times"].copy()
    motion.targets = [np.array(v) for v in c["commands"]["targets"]]
    motion.phases = c["commands"]["phases"].copy()
    motion.append(
        c["time"] + c["duration"], q, "Coordinate wrist and index for thumb access"
    )
    motion.append(
        c["time"] + c["duration"] + 1, q, "Hold six supports before thumb pickup"
    )
    sim.load_checkpoint(c["source"] / "checkpoint.npz")
    worst = {
        f"{s}_{f}": -np.inf
        for s in ("left", "right")
        for f in ("thumb", "index", "little")
    }
    peak = 0.0
    frames, metrics = [], []
    for frame in range(round(sim.t * 30) + 1, round(motion.times[-1] * 30) + 1):
        sim.advance(frame / 30)
        rope = sim.state.particle_q.numpy()
        for side in ("left", "right"):
            for finger in ("thumb", "index", "little"):
                name = f"{side}_{finger}"
                worst[name] = max(
                    worst[name], float(gaps(sim.transforms, rope, side, finger).min())
                )
        metric = sim.metrics()
        peak = max(peak, metric["max_stretch_ratio"])
        if keep:
            frames.append(sim.snapshot())
            metrics.append(metric)
    loss = sum(max(g - 0.0015, 0) * 1000 for g in worst.values())
    access = (
        thumb_access(robot, q, rope, c["plan"])
        if loss < 0.1
        else dict(cost_mm=100.0, rejected="Original support lost during rollout")
    )
    score = access["cost_mm"] + 20 * loss + 20 * max(peak - 1.2, 0)
    result = dict(
        id=proposal["id"],
        score=float(score),
        access=access,
        support_loss_mm=float(loss),
        worst_support_gaps_m=worst,
        peak_stretch=float(peak),
    )
    if keep:
        result.update(frames=frames, metrics=metrics)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--support-reference", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--samples", type=int, default=40)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument("--seed", type=int, default=211)
    p.add_argument("--duration", type=float, default=2.0)
    p.add_argument(
        "--proposals",
        type=Path,
        help="Replay an explicit list of joint-target proposals",
    )
    a = p.parse_args()
    if min(a.samples, a.workers, a.duration) <= 0:
        p.error("samples, workers and duration must be positive")
    a.output.mkdir(parents=True, exist_ok=False)
    manifest = {k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()}
    manifest["source_checkpoint_sha256"] = hashlib.sha256(
        (a.source / "checkpoint.npz").read_bytes()
    ).hexdigest()
    manifest["segment_contacts_sha256"] = hashlib.sha256(
        (engine.ROOT / "revo3_cat_cradle/segment_contacts.py").read_bytes()
    ).hexdigest()
    (a.output / "search_config.json").write_text(json.dumps(manifest, indent=2))
    robot = RevoTron()
    q0 = np.load(a.source / "checkpoint.npz")["joints"]
    wrist = robot.wrist(q0, "right")
    rng = np.random.default_rng(a.seed)
    proposals = [dict(id=0, joints=q0.tolist())]
    for i in range(1, a.samples):
        q = q0.copy()
        # Index lowering creates access; wrist pitch/translation change the
        # supporting little finger's orientation relative to the pulled strand.
        q[robot.indices["left_index_MCP_joint"]] = rng.uniform(0.65, 1.15)
        q[robot.indices["left_index_PIP_joint"]] = rng.uniform(-0.1, 0.15)
        q[robot.indices["left_index_DIP_joint"]] = rng.uniform(-0.2, 0.1)
        position = wrist[:3, 3] + np.clip(rng.normal(0, 0.009, 3), -0.018, 0.018)
        rotation = (
            Rotation.from_rotvec(np.clip(rng.normal(0, 0.16, 3), -0.3, 0.3)).as_matrix()
            @ wrist[:3, :3]
        )
        try:
            q = robot.ik(q, "right", position, rotation)
        except ValueError:
            continue
        proposals.append(dict(id=i, joints=q.tolist()))
    if a.proposals:
        proposals = json.loads(a.proposals.read_text())
        if not proposals or len({x["id"] for x in proposals}) != len(proposals):
            raise ValueError("Proposal IDs must be unique and list must not be empty")
    (a.output / "proposals.json").write_text(json.dumps(proposals, indent=2))
    results = []
    with ProcessPoolExecutor(
        max_workers=a.workers,
        initializer=initialize,
        initargs=(str(a.source), str(a.support_reference), a.duration),
    ) as pool:
        for result in pool.map(rollout, proposals):
            results.append(result)
            (a.output / "search_history.json").write_text(json.dumps(results, indent=2))
            print(
                json.dumps(
                    {k: v for k, v in result.items() if k != "access"}
                    | {"access_cost_mm": result.get("access", {}).get("cost_mm")}
                ),
                flush=True,
            )
    valid = [r for r in results if r["score"] is not None]
    if not valid:
        raise RuntimeError("No collision-free preparation candidate")
    winner = min(valid, key=lambda r: r["score"])
    proposal = next(p for p in proposals if p["id"] == winner["id"])
    initialize(str(a.source), str(a.support_reference), a.duration)
    replay = rollout(proposal, keep=True)
    if not np.isclose(replay["score"], winner["score"], atol=1e-5):
        raise RuntimeError("Independent replay differs from selected preparation")
    c = engine.CONTEXT
    c["motion"].metadata.update(
        status="experimental", figure="Preparation for thumb transfer"
    )
    frames = c["payload"]["frames"] + replay.pop("frames")
    metrics = json.loads((a.source / "metrics.json").read_text())[
        "metrics"
    ] + replay.pop("metrics")
    export_bimanual(c["sim"], frames, metrics, a.output)
    c["sim"].save_checkpoint(a.output / "checkpoint.npz")
    (a.output / "transfer_plan.json").write_text(json.dumps(c["plan"], indent=2))
    (a.output / "joint_commands.json").write_text(
        json.dumps(
            dict(
                times=c["motion"].times,
                targets=[q.tolist() for q in c["motion"].targets],
                phases=c["motion"].phases,
            )
        )
    )
    (a.output / "winner.json").write_text(
        json.dumps(dict(proposal=proposal, evaluation=replay), indent=2)
    )
    print(
        "Selected preparation",
        proposal["id"],
        "score",
        replay["score"],
        "not a certified pickup",
        flush=True,
    )


if __name__ == "__main__":
    main()
