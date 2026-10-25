"""Recompute a recorded transfer experiment from its initial placement.

Recipes contain robot commands and one initial rope seed. Newton integrates
every subsequent rope state. A recorded feedback command sequence is replayed
open-loop; run the validators again before accepting a result on another setup.
"""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.single_player import Motion, initial_loop


def recipe_motion(recipe, robot):
    """Validate command identity and keep interrupted quintics intact."""
    if recipe.get("schema_version") != 1:
        raise ValueError("Unsupported transfer recipe version")
    if recipe["joint_names"] != robot.names:
        raise ValueError("Recipe joint order does not match the robot")
    commands = recipe["commands"]
    times = np.asarray(commands["times"], dtype=float)
    targets = np.asarray(commands["targets"], dtype=float)
    phases = commands["phases"]
    if (
        times.ndim != 1
        or len(times) < 2
        or not np.isfinite(times).all()
        or times[0] != 0
        or np.any(np.diff(times) <= 0)
        or targets.shape != (len(times), len(robot.names))
        or not np.isfinite(targets).all()
        or len(phases) != len(times)
        or not all(isinstance(p, str) for p in phases)
    ):
        raise ValueError("Invalid transfer commands")
    if np.any(targets < robot.limits[:, 0] - 1e-10) or np.any(
        targets > robot.limits[:, 1] + 1e-10
    ):
        raise ValueError("Recipe exceeds robot joint limits")
    metadata = deepcopy(recipe["motion"])
    if metadata.get("physics_events"):
        raise ValueError("Recipes with runtime physics events are not supported")
    if "initial_rope_points" not in metadata:
        raise ValueError("Recipe requires an explicit initial rope seed")
    if not np.isclose(metadata["duration"], times[-1], atol=1e-8, rtol=0):
        raise ValueError("Recipe duration does not match commands")
    for segment in metadata.get("interrupted_segments", []):
        bounds = np.array([segment[k] for k in ("start", "stop", "end")])
        endpoints = np.array([segment[k] for k in ("initial", "target")])
        if (
            not np.isfinite(bounds).all()
            or not 0 <= bounds[0] < bounds[1] <= bounds[2]
            or bounds[1] > times[-1] + 1e-8
            or endpoints.shape != (2, len(robot.names))
            or not np.isfinite(endpoints).all()
            or np.any(endpoints < robot.limits[:, 0] - 1e-10)
            or np.any(endpoints > robot.limits[:, 1] + 1e-10)
        ):
            raise ValueError("Invalid interrupted command segment")
    motion = Motion.__new__(Motion)
    motion.hand = robot
    motion.times = times.tolist()
    motion.targets = [q.copy() for q in targets]
    motion.phases = phases.copy()
    motion.metadata = metadata
    initial_loop(robot, motion)  # Validate the seed before creating any output.
    return motion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recipe", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, help="Stop early for a smoke run")
    args = parser.parse_args()
    recipe = json.loads(args.recipe.read_text())
    robot = RevoTron()
    motion = recipe_motion(recipe, robot)
    config = Config(**recipe["config"])
    duration = motion.times[-1] if args.duration is None else args.duration
    if not np.isfinite(duration) or not 0 < duration <= motion.times[-1] + 1e-8:
        parser.error("Duration must be positive and within the recorded motion")
    final_frame = round(duration * config.fps)
    if final_frame < 1 or abs(final_frame / config.fps - duration) > 1e-8:
        parser.error("Duration must end on an output frame")
    if args.output.exists():
        parser.error("Output already exists; choose a new directory")
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        config,
        palm_geometry=recipe["collision_geometry"]["palm"],
        knuckle_geometry=recipe["collision_geometry"]["knuckles"],
        length_control=recipe["length_control"],
        rest_length_max_scale=recipe["rest_length_max_scale"],
    )
    args.output.mkdir(parents=True)
    (args.output / "joint_commands.json").write_text(json.dumps(recipe["commands"]))
    (args.output / "transfer_plan.json").write_text(json.dumps(recipe["transfer_plan"]))
    (args.output / "recipe.json").write_text(json.dumps(recipe))
    frames, metrics = [sim.snapshot()], [sim.metrics()]
    for frame in range(1, final_frame + 1):
        sim.advance(frame / config.fps)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        rope = sim.state.particle_q.numpy()
        if (
            not np.isfinite(rope).all()
            or np.max(np.linalg.norm(np.roll(rope, -1, axis=0) - rope, axis=1))
            >= MAX_SEGMENT_LENGTH
        ):
            export_bimanual(sim, frames, metrics, args.output)
            raise RuntimeError("Replay exceeded the rope contact search bound")
        if frame % config.fps == 0 or frame == final_frame:
            export_bimanual(sim, frames, metrics, args.output)
            print(
                json.dumps(
                    dict(time_s=sim.t, stretch=metrics[-1]["max_stretch_ratio"])
                ),
                flush=True,
            )


if __name__ == "__main__":
    main()
