"""Run a bimanual cat's-cradle recipe as a reset/observe/step/score episode."""

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.skill_episode import BimanualSkillEpisode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recipe", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument(
        "--active-side", choices=["left", "right", "both"], default="both"
    )
    args = parser.parse_args()
    if not math.isfinite(args.step) or args.step <= 0:
        parser.error("--step must be finite and positive")
    active = ("left", "right") if args.active_side == "both" else (args.active_side,)
    episode = BimanualSkillEpisode.from_recipe(
        args.recipe, args.output, active_sides=active
    )
    duration = episode.duration if args.duration is None else args.duration
    if not 0 < duration <= episode.duration + 1e-8:
        parser.error("--duration must be within the recipe duration")
    fps = episode.config.fps
    step_frames = round(args.step * fps)
    if step_frames < 1 or abs(step_frames / fps - args.step) > 1e-8:
        parser.error("--step must be a whole number of output frames")
    target_frame = round(duration * fps)
    if abs(target_frame / fps - duration) > 1e-8:
        parser.error("--duration must end on an output frame")
    for next_frame in range(step_frames, target_frame + 1, step_frames):
        print(json.dumps(episode.step(next_frame / fps)), flush=True)
    if episode.sim.t < target_frame / fps - 1e-8:
        print(json.dumps(episode.step(target_frame / fps)), flush=True)
    result = episode.finish()
    print(json.dumps(result, indent=2), flush=True)
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
