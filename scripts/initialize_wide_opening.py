"""Start a separate, wider Opening A fixture and let its unanchored rope settle.

This is initial placement, NOT a continuation of the source's dynamic state.
Only initialization geometry is deformed; recorded frames use Newton dynamics.
"""

import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.bimanual_checks import gaps
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.mesh_geometry import (
    load_knuckle_meshes,
    support_material_distances,
)


def seed_wider_figure(robot, joints, points, span, meshes=None):
    """Translate contact neighborhoods rigidly; interpolate free string spans."""
    tf = robot.fk(joints)
    support = {}
    for side in ("left", "right"):
        distances = [
            support_material_distances(tf, points, side, finger, meshes)
            if meshes
            else gaps(tf, points, side, finger)
            for finger in ("thumb", "index", "little")
        ]
        if any(d.min() > 0.0015 for d in distances):
            raise ValueError("Source Opening A is missing an original support")
        support[side] = np.min(distances, axis=0) < 0.004
    if np.any(support["left"] & support["right"]):
        raise ValueError("Ambiguous seed: both hands contact the same material")
    labels = np.full(len(points), np.nan)
    labels[support["left"]] = 1
    labels[support["right"]] = 0
    lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    arc = np.r_[0, np.cumsum(lengths)[:-1]]
    ids = np.flatnonzero(np.isfinite(labels))
    period = lengths.sum()
    weights = np.interp(
        arc,
        np.r_[arc[ids] - period, arc[ids], arc[ids] + period],
        np.tile(labels[ids], 3),
    )
    wrists = {s: robot.wrist(joints, s) for s in ("left", "right")}
    vector = wrists["left"][:3, 3] - wrists["right"][:3, 3]
    distance = np.linalg.norm(vector)
    if span <= distance:
        raise ValueError("Requested seed must be wider than the source")
    delta = (span - distance) / 2 * vector / distance
    q = joints.copy()
    for side, sign in [("left", 1), ("right", -1)]:
        w = wrists[side]
        q = robot.ik(q, side, w[:3, 3] + sign * delta, w[:3, :3])
    placed = points + (2 * weights[:, None] - 1) * delta
    return q, placed


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--span", type=float, default=0.24)
    p.add_argument("--friction", type=float, default=2)
    p.add_argument("--settle", type=float, default=3)
    p.add_argument("--take-up", type=float, default=0.01)
    p.add_argument("--tension", type=float)
    p.add_argument("--spacing", type=float)
    p.add_argument("--substeps", type=int)
    p.add_argument("--iterations", type=int)
    p.add_argument("--stretch-stiffness", type=float)
    p.add_argument("--rest-length-max-scale", type=float, default=2)
    p.add_argument("--length-control", choices=["local", "global"], default="local")
    p.add_argument(
        "--preserve-sampling",
        action="store_true",
        help="Keep the seed's dense wrap samples and existing spring count",
    )
    a = p.parse_args()
    if (
        not np.isfinite([a.span, a.friction, a.settle, a.take_up]).all()
        or not 0.15 < a.span <= 0.6
        or a.friction < 0
        or a.settle < 1
        or a.take_up < 0
    ):
        p.error("Invalid fixture parameters")
    state = np.load(a.source / "checkpoint.npz")
    source_payload = json.loads((a.source / "trajectory.json").read_text())
    robot = RevoTron()
    source_meshes = (
        load_knuckle_meshes(robot)
        if source_payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else None
    )
    q, points = seed_wider_figure(
        robot, state["joints"], state["particle_q"], a.span, source_meshes
    )
    m = Motion(robot)
    m.targets = [q]
    m.phases = ["Independent Opening A placement"]
    m.metadata.update(
        status="experimental",
        figure="Wider Opening A initialization",
        initialization="Independent placement, not replay of Position 1 or source dynamics",
        source_placement=str(a.source),
        initial_rope_points=points.tolist(),
        initial_half_span_m=a.span / 2,
        opening_half_span_m=a.span / 2,
        transfer_reference_time_s=a.settle,
        preserve_initial_sampling=a.preserve_sampling,
    )
    m.append(a.settle, q, "Free settling of wider Opening A")
    cfg = Config(**json.loads(str(state["config"])))
    cfg.friction = a.friction
    cfg.adaptive_take_up_m = a.take_up
    for name, value in (
        ("spacing", a.spacing),
        ("substeps", a.substeps),
        ("iterations", a.iterations),
        ("stretch_stiffness", a.stretch_stiffness),
    ):
        if value is not None:
            if not np.isfinite(value) or value <= 0:
                p.error(f"{name} must be finite and positive")
            setattr(cfg, name, value)
    if a.tension is not None:
        if not np.isfinite(a.tension) or a.tension < 0:
            p.error("Tension must be finite and nonnegative")
        cfg.adaptive_tension_n = a.tension
    sim = BimanualSimulation(
        robot,
        m,
        initial_loop(robot, m),
        cfg,
        palm_geometry="hull",
        knuckle_geometry="mesh",
        length_control=a.length_control,
        rest_length_max_scale=a.rest_length_max_scale,
    )
    m.metadata["initial_rest_length_m"] = float(sim.initial_rest_lengths.sum())
    frames, metrics = [sim.snapshot()], [sim.metrics()]
    for frame in range(1, round(a.settle * cfg.fps) + 1):
        sim.advance(frame / cfg.fps)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
    export_bimanual(sim, frames, metrics, a.output)
    (a.output / "joint_commands.json").write_text(
        json.dumps(
            dict(
                times=m.times, targets=[x.tolist() for x in m.targets], phases=m.phases
            )
        )
    )
    meshes = load_knuckle_meshes(robot)
    distances = {
        s + "_" + f: support_material_distances(
            sim.transforms, sim.state.particle_q.numpy(), s, f, meshes
        )
        for s in ("left", "right")
        for f in ("thumb", "index", "little")
    }
    patches = {}
    for label, ds in distances.items():
        ids = np.flatnonzero(ds < 0.0015)
        patches[label] = [
            x.tolist()
            for x in np.split(ids, np.flatnonzero(np.diff(ids) > 1) + 1)
            if len(x)
        ]
    report = dict(
        scope="Independent fixture settling, not completed Opening A manipulation",
        span_m=a.span,
        initial_rest_length_m=m.metadata["initial_rest_length_m"],
        final_rest_length_m=float(sim.rest_lengths.sum()),
        peak_stretch=max(x["max_stretch_ratio"] for x in metrics),
        final_support_gaps_m={
            label: float(ds.min()) for label, ds in distances.items()
        },
        final_contact_patches=patches,
        final_max_speed_m_s=float(
            np.linalg.norm(sim.state.particle_qd.numpy(), axis=1).max()
        ),
    )
    (a.output / "initialization_audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
