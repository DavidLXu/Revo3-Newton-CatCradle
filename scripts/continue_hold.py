"""Hold a saved motion boundary with unchanged joints and continuous rope dynamics."""

import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=3)
    parser.add_argument("--label", default="Continue free settling")
    parser.add_argument("--segment-hand-contacts", action="store_true",
                        help="Enable interior rope-edge contact with finger capsules after restore")
    parser.add_argument("--stretch-damping", type=float,
                        help="Change longitudinal spring damping after exact checkpoint restore")
    args = parser.parse_args()
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("Hold duration must be finite and positive")
    if args.stretch_damping is not None and (
        not np.isfinite(args.stretch_damping) or args.stretch_damping < 0
    ):
        parser.error("Stretch damping must be finite and nonnegative")
    payload = json.loads((args.source / "trajectory.json").read_text())
    commands = json.loads((args.source / "joint_commands.json").read_text())
    saved = np.load(args.source / "checkpoint.npz")
    cfg = Config(**json.loads(str(saved["config"])))
    start = float(saved["time"])
    if abs(commands["times"][-1] - start) > 1e-5:
        raise ValueError("Hold must start at the final saved motion boundary")
    if not np.allclose(commands["targets"][-1], saved["joints"], atol=1e-10, rtol=0):
        raise ValueError("Saved command and physical boundary disagree")
    robot = RevoTron()
    motion = Motion(robot)
    motion.times = commands["times"].copy()
    motion.targets = [np.asarray(q) for q in commands["targets"]]
    motion.phases = commands["phases"].copy()
    motion.metadata = payload["motion"].copy()
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        cfg,
        palm_geometry=payload["collision_geometry"]["palm"],
        knuckle_geometry=payload["collision_geometry"].get("knuckles", "none"),
        length_control=payload.get("length_control", "local"),
        rest_length_max_scale=payload.get("rest_length_max_scale", 2),
    )
    sim.load_checkpoint(args.source / "checkpoint.npz")
    if args.segment_hand_contacts and not cfg.segment_hand_contacts:
        motion.metadata.setdefault("initial_segment_hand_contacts", False)
        motion.metadata.setdefault("physics_events", []).append(dict(
            time=start, parameter="segment_hand_contacts", previous_value=False, value=True,
        ))
        sim.set_segment_hand_contacts(True)
    if args.stretch_damping is not None:
        motion.metadata.setdefault("initial_stretch_damping", cfg.stretch_damping)
        motion.metadata.setdefault("physics_events", []).append(dict(
            time=start, parameter="stretch_damping",
            previous_value=cfg.stretch_damping, value=args.stretch_damping,
        ))
        sim.set_stretch_damping(args.stretch_damping)
    end = round((start + args.seconds) * cfg.fps) / cfg.fps
    if end <= start + 1e-6:
        raise ValueError("Hold must include at least one output frame")
    motion.append(end, sim.q.copy(), args.label)
    motion.metadata.setdefault("static_holds", []).append(
        dict(start_s=start, end_s=end, label=args.label)
    )
    if (
        "transfer_reference_time_s" in motion.metadata
        and not (args.source / "transfer_plan.json").exists()
    ):
        motion.metadata["transfer_reference_time_s"] = end
    frames = payload["frames"].copy()
    metrics = json.loads((args.source / "metrics.json").read_text())["metrics"].copy()
    if abs(frames[-1]["time"] - start) > 1e-5:
        raise ValueError("Exported trajectory and checkpoint time disagree")
    args.output.mkdir(parents=True, exist_ok=False)
    hold = [
        dict(
            time=start,
            rope=sim.state.particle_q.numpy().copy(),
            velocities=sim.state.particle_qd.numpy().copy(),
            rest_lengths=sim.rest_lengths.copy(),
        )
    ]
    for frame in range(round(start * cfg.fps) + 1, round(end * cfg.fps) + 1):
        sim.advance(frame / cfg.fps)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        positions = sim.state.particle_q.numpy().copy()
        hold.append(
            dict(
                time=sim.t,
                rope=positions,
                velocities=sim.state.particle_qd.numpy().copy(),
                rest_lengths=sim.rest_lengths.copy(),
            )
        )
        if (
            not np.isfinite(positions).all()
            or np.linalg.norm(np.roll(positions, -1, axis=0) - positions, axis=1).max()
            >= MAX_SEGMENT_LENGTH
        ):
            export_bimanual(sim, frames, metrics, args.output)
            raise RuntimeError("Static hold left the valid rope geometry range")
        if frame % cfg.fps == 0:
            print(
                dict(
                    time=sim.t,
                    stretch=metrics[-1]["max_stretch_ratio"],
                    max_speed_m_s=float(
                        np.linalg.norm(hold[-1]["velocities"], axis=1).max()
                    ),
                ),
                flush=True,
            )
    export_bimanual(sim, frames, metrics, args.output)
    (args.output / "joint_commands.json").write_text(
        json.dumps(
            dict(
                times=motion.times,
                targets=[q.tolist() for q in motion.targets],
                phases=motion.phases,
            )
        )
    )
    np.savez_compressed(
        args.output / "hold.npz", **{k: [h[k] for h in hold] for k in hold[0]}
    )
    for name in ("transfer_plan.json", "plan.json"):
        if (args.source / name).exists():
            shutil.copyfile(args.source / name, args.output / name)


if __name__ == "__main__":
    main()
