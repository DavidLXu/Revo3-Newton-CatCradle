"""Replay a saved motion interval at a different Newton solver resolution.

Solver resolution and optional contact relaxation change after loading the exact checkpoint.
This partial-run diagnostic does not certify the full trajectory.
"""

import argparse
import json
import hashlib
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.bimanual_checks import segment_separation, gaps
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--end", type=float, required=True)
    p.add_argument("--substeps", type=int, required=True)
    p.add_argument("--iterations", type=int, required=True)
    p.add_argument("--contact-relaxation", type=float)
    p.add_argument(
        "--rest-length-max-scale",
        type=float,
        help="Optional local payout cap, relative to each initial rest length",
    )
    a = p.parse_args()
    if a.contact_relaxation is not None and not 0 < a.contact_relaxation <= 1:
        p.error("contact relaxation must be in (0, 1]")
    if min(a.substeps, a.iterations) <= 0:
        raise ValueError("Resolution must be positive")
    if a.rest_length_max_scale is not None and (
        not np.isfinite(a.rest_length_max_scale) or a.rest_length_max_scale < 1
    ):
        p.error("rest-length-max-scale must be finite and at least one")
    a.output.mkdir(parents=True, exist_ok=False)
    payload = json.loads((a.source / "trajectory.json").read_text())
    command = json.loads((a.source / "joint_commands.json").read_text())
    saved = np.load(a.checkpoint)
    original_config = json.loads(str(saved["config"]))
    robot = RevoTron()
    motion = Motion(robot)
    motion.times = command["times"]
    motion.targets = [np.array(q) for q in command["targets"]]
    motion.phases = command["phases"]
    motion.metadata = payload["motion"].copy()
    motion.metadata.update(
        status="experimental", source="Solver-resolution interval probe"
    )
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        Config(**original_config),
        palm_geometry="hull",
        knuckle_geometry=payload.get("collision_geometry", {}).get("knuckles", "none"),
        robot_body_geometry=payload.get("collision_geometry", {}).get("robot_body", "none"),
        length_control=payload.get("length_control", "local"),
        rest_length_max_scale=payload.get("rest_length_max_scale", 2),
    )
    sim.load_checkpoint(a.checkpoint)
    if not sim.t < a.end <= motion.times[-1]:
        raise ValueError("Probe interval must lie within the saved motion")
    sim.config.substeps = a.substeps
    sim.config.iterations = a.iterations
    # Newton exposes this as the loop count in SolverXPBD.step; no state reset.
    sim.solver.iterations = a.iterations
    if a.contact_relaxation is not None:
        sim.config.contact_relaxation = a.contact_relaxation
        sim.solver.soft_contact_relaxation = a.contact_relaxation
    if a.rest_length_max_scale is not None:
        sim.rest_length_max_scale = a.rest_length_max_scale
    start = sim.t
    count = round(start * 30) + 1
    frames = payload["frames"][:count]
    metrics = json.loads((a.source / "metrics.json").read_text())["metrics"][:count]
    rows = []
    for frame in range(count, round(a.end * 30) + 1):
        sim.advance(frame / 30)
        snapshot = sim.snapshot()
        metric = sim.metrics()
        rows.append(
            dict(
                time=sim.t,
                original_support_gaps_m={
                    f"{side}_{finger}": float(
                        gaps(
                            sim.transforms, np.asarray(snapshot["rope"]), side, finger
                        ).min()
                    )
                    for side in ("left", "right")
                    for finger in ("thumb", "little")
                },
                stretch=metric["max_stretch_ratio"],
                separation_m=segment_separation(np.asarray(snapshot["rope"])),
                nonlocal_separation_m=segment_separation(
                    np.asarray(snapshot["rope"]),
                    minimum_arc_distance=2 * sim.config.radius,
                ),
            )
        )
        frames.append(snapshot)
        metrics.append(metric)
        if frame % 15 == 0:
            print(rows[-1], flush=True)
    export_bimanual(sim, frames, metrics, a.output)
    sim.save_checkpoint(a.output / "checkpoint.npz")
    (a.output / "joint_commands.json").write_text(json.dumps(command))
    for name in ("plan.json", "transfer_plan.json"):
        if (a.source / name).exists():
            shutil.copyfile(a.source / name, a.output / name)
    report = dict(
        source=str(a.source),
        checkpoint=str(a.checkpoint),
        start=start,
        end=sim.t,
        original_substeps=original_config["substeps"],
        original_iterations=original_config["iterations"],
        substeps=a.substeps,
        iterations=a.iterations,
        contact_relaxation=sim.config.contact_relaxation,
        rest_length_max_scale=sim.rest_length_max_scale,
        interval_peak_stretch=max(r["stretch"] for r in rows),
        interval_min_separation_m=min(r["separation_m"] for r in rows),
        interval_min_nonlocal_separation_m=min(
            r["nonlocal_separation_m"] for r in rows
        ),
        segment_contacts_sha256=hashlib.sha256(
            (ROOT / "revo3_cat_cradle/segment_contacts.py").read_bytes()
        ).hexdigest(),
        samples=rows,
        scope="Resolution comparison on this interval only; full motion remains unverified",
    )
    (a.output / "resolution_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}), flush=True)


if __name__ == "__main__":
    main()
