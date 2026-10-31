"""Small episode interface for bimanual transfer experiments.

The episode deliberately separates the replay policy from the Newton state:
``reset`` builds a fresh physical scene, ``step`` advances a recorded command
schedule, ``observe`` returns task-level contact measurements, ``score`` reads
the private trajectory, and ``finish`` writes an auditable result bundle.

This is an experiment harness rather than a learned policy. A future policy can
use :meth:`observe` without receiving particle coordinates or joint vectors,
while the scorer can still enforce the existing physical gates.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .bimanual import BimanualSimulation, RevoTron, export_bimanual
from .bimanual_checks import gaps, segment_separation
from .sim import Config
from .single_player import initial_loop


def _finite(value: Any) -> bool:
    return bool(np.isfinite(np.asarray(value, dtype=float)).all())


class BimanualSkillEpisode:
    """Run one fixed-command transfer as a reset/observe/step/score episode."""

    def __init__(
        self,
        recipe: dict[str, Any],
        output: Path | None = None,
        *,
        active_sides: Iterable[str] = ("left", "right"),
    ) -> None:
        self.recipe = recipe
        self.output = Path(output) if output is not None else None
        self.robot = RevoTron()
        self.active_sides = tuple(active_sides)
        if not self.active_sides or not set(self.active_sides).issubset(
            {"left", "right"}
        ):
            raise ValueError("active_sides must contain left and/or right")
        self.plan = recipe.get("transfer_plan", {})
        if not set(self.active_sides).issubset(self.plan):
            raise ValueError("recipe has no transfer plan for the active side")
        self.reset()

    @classmethod
    def from_recipe(
        cls,
        recipe_path: str | Path,
        output: Path | None = None,
        *,
        active_sides: Iterable[str] = ("left", "right"),
    ) -> "BimanualSkillEpisode":
        path = Path(recipe_path)
        return cls(
            json.loads(path.read_text(encoding="utf-8")),
            output,
            active_sides=active_sides,
        )

    def reset(self) -> dict[str, Any]:
        """Restore the recipe's initial placement and clear the episode log."""
        commands = self.recipe["commands"]
        # Import the existing recipe validator lazily to keep the core package
        # independent of the command-line entry point at import time.
        from scripts.replay_transfer import recipe_motion

        self.motion = recipe_motion(self.recipe, self.robot)
        self.config = Config(**self.recipe["config"])
        self.sim = BimanualSimulation(
            self.robot,
            self.motion,
            initial_loop(self.robot, self.motion),
            self.config,
            palm_geometry=self.recipe["collision_geometry"]["palm"],
            knuckle_geometry=self.recipe["collision_geometry"]["knuckles"],
            length_control=self.recipe["length_control"],
            rest_length_max_scale=self.recipe["rest_length_max_scale"],
        )
        self.frames = [self.sim.snapshot()]
        self.metrics = [self.sim.metrics()]
        self.events: list[dict[str, Any]] = []
        self._log_event("reset", {"time_s": 0.0, "recipe_duration_s": self.duration})
        # The command list is retained for exact action replay, but actions
        # advance only to an absolute output-frame time.
        if not _finite(commands["times"]):
            raise ValueError("recipe command times must be finite")
        return self.observe()

    @property
    def duration(self) -> float:
        return float(self.recipe["commands"]["times"][-1])

    def _log_event(self, kind: str, payload: dict[str, Any]) -> None:
        self.events.append({"event": kind, **payload})

    def _side_observation(self, side: str, frame: dict[str, Any]) -> dict[str, float]:
        tf = self.robot.fk(np.asarray(frame["joints"], dtype=float))
        rope = np.asarray(frame["rope"], dtype=float)
        plan = self.plan[side]
        result: dict[str, float] = {}
        for name in ("far_indices", "near_indices", "old_thumb_indices"):
            ids = np.asarray(plan[name], dtype=int)
            result[name.replace("_indices", "_gap_m")] = float(
                gaps(tf, rope[ids], side, "thumb").min()
            )
        return result

    def observe(self) -> dict[str, Any]:
        """Return policy-facing measurements without particle or joint state."""
        frame = self.frames[-1]
        metric = self.metrics[-1]
        return {
            "time_s": float(frame["time"]),
            "phase": str(frame["phase"]),
            "active_sides": list(self.active_sides),
            "max_stretch_ratio": float(metric["max_stretch_ratio"]),
            "rope_rest_length_m": float(metric["rope_rest_length_m"]),
            "thumb_contacts": {
                side: self._side_observation(side, frame)
                for side in self.active_sides
            },
        }

    def step(
        self, until_s: float, *, action: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        """Advance to an output-frame boundary and return the new observation."""
        until_s = float(until_s)
        if not np.isfinite(until_s) or until_s <= self.sim.t:
            raise ValueError("step time must be finite and greater than current time")
        if until_s > self.duration + 1e-8:
            raise ValueError("step time exceeds recipe duration")
        frame = round(until_s * self.config.fps)
        if abs(frame / self.config.fps - until_s) > 1e-8:
            raise ValueError("step time must end on an output frame")
        for i in range(round(self.sim.t * self.config.fps) + 1, frame + 1):
            self.sim.advance(i / self.config.fps)
            self.frames.append(self.sim.snapshot())
            self.metrics.append(self.sim.metrics())
        observation = self.observe()
        self._log_event(
            "step",
            {
                "time_s": until_s,
                "action": action or {"kind": "replay", "until_s": until_s},
                "observation": observation,
            },
        )
        return observation

    def score(self) -> dict[str, Any]:
        """Apply the existing contact, dynamic-rope and strain gates."""
        if not self.frames:
            raise RuntimeError("episode has no frames")
        times = np.asarray([f["time"] for f in self.frames], dtype=float)
        ropes = np.asarray([f["rope"] for f in self.frames], dtype=float)
        joints = np.asarray([f["joints"] for f in self.frames], dtype=float)
        last = np.flatnonzero(times >= max(0.0, times[-1] - 1.0))
        stages: dict[str, dict[str, Any]] = {}
        checks: dict[str, bool] = {
            "finite": bool(np.isfinite(ropes).all()),
            "joint_limits": bool(
                np.all(joints >= self.robot.limits[:, 0] - 1e-7)
                and np.all(joints <= self.robot.limits[:, 1] + 1e-7)
            ),
            "dynamic_rope": bool(
                np.all(self.sim.model.particle_mass.numpy() > 0)
                and not len(self.sim.anchor_indices)
            ),
            "completed_hold": bool(times[-1] >= self.duration - 1e-8),
        }
        for side in self.active_sides:
            plan = self.plan[side]
            far, near, old = (
                np.asarray(plan[k], dtype=int)
                for k in ("far_indices", "near_indices", "old_thumb_indices")
            )
            # A pickup is a held contact: a single touching frame is not
            # enough, so use the worst far-gap over the final hold window.
            far_gap = max(
                gaps(self.robot.fk(joints[k]), ropes[k, far], side, "thumb").min()
                for k in last
            )
            near_gap = min(
                gaps(self.robot.fk(joints[k]), ropes[k, near], side, "thumb").min()
                for k in last
            )
            old_gap = max(
                gaps(self.robot.fk(joints[k]), ropes[k, old], side, "thumb").min()
                for k in last
            )
            stages[side] = {
                "maximum_far_gap_m": float(far_gap),
                "minimum_near_gap_m": float(near_gap),
                "maximum_old_loop_gap_m": float(old_gap),
                "passed": bool(
                    far_gap < 0.0015 and near_gap > 0.002 and old_gap < 0.0015
                ),
            }
            checks[f"{side}_new_far_bight"] = bool(far_gap < 0.0015)
            checks[f"{side}_near_string_clear"] = bool(near_gap > 0.002)
            checks[f"{side}_old_loop_retained"] = bool(old_gap < 0.0015)
        ratios = np.asarray([m["max_stretch_ratio"] for m in self.metrics])
        checks["strain"] = bool(np.isfinite(ratios).all() and ratios.max() < 1.2)
        separation = min(
            segment_separation(rope, minimum_arc_distance=2 * self.config.radius)
            for rope in ropes
        )
        checks["self_contact"] = bool(separation >= 0.0018)
        return {
            "passed": bool(all(checks.values())),
            "checks": checks,
            "stages": stages,
            "duration_s": float(times[-1]),
            "max_stretch_ratio": float(ratios.max()),
            "minimum_segment_distance_m": float(separation),
            "active_sides": list(self.active_sides),
        }

    def finish(self) -> dict[str, Any]:
        """Export physics, append persistent logs, and return the score."""
        result = self.score()
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=False)
            export_bimanual(self.sim, self.frames, self.metrics, self.output)
            (self.output / "recipe.json").write_text(
                json.dumps(self.recipe, indent=2), encoding="utf-8"
            )
            manifest = {
                "schema_version": 1,
                "task": "revotron-bimanual-cat-cradle-transfer",
                "active_sides": list(self.active_sides),
                "recipe_duration_s": self.duration,
                "observation_fields": [
                    "time_s",
                    "phase",
                    "max_stretch_ratio",
                    "rope_rest_length_m",
                    "thumb_contacts",
                ],
            }
            (self.output / "episode.json").write_text(
                json.dumps(manifest, indent=2), encoding="utf-8"
            )
            with (self.output / "log.jsonl").open("w", encoding="utf-8") as stream:
                for event in self.events:
                    stream.write(json.dumps(event, ensure_ascii=False) + "\n")
            (self.output / "result.json").write_text(
                json.dumps(result, indent=2), encoding="utf-8"
            )
            (self.output / "summary.json").write_text(
                json.dumps(
                    {
                        "task": manifest["task"],
                        "passed": result["passed"],
                        "duration_s": result["duration_s"],
                        "stages": result["stages"],
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
        return result
