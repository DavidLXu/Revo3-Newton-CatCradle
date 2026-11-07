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
from .bimanual_checks import gaps
from .segment_contacts import MAX_SEGMENT_LENGTH
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
        from copy import deepcopy

        self.recipe = deepcopy(recipe)
        self.output = Path(output) if output is not None else None
        if self.output is not None and self.output.exists():
            raise FileExistsError(f"Episode output already exists: {self.output}")
        self.robot = RevoTron()
        self.active_sides = tuple(active_sides)
        if not self.active_sides or not set(self.active_sides).issubset(
            {"left", "right"}
        ):
            raise ValueError("active_sides must contain left and/or right")
        self.plan = self.recipe.get("transfer_plan", {})
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
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output / "log.jsonl").write_text("", encoding="utf-8")
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
        event = {"event": kind, **payload}
        self.events.append(event)
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            with (self.output / "log.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")

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
                side: self._side_observation(side, frame) for side in self.active_sides
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
            rope = np.asarray(self.frames[-1]["rope"])
            if (
                not np.isfinite(rope).all()
                or np.linalg.norm(np.roll(rope, -1, axis=0) - rope, axis=1).max()
                >= MAX_SEGMENT_LENGTH
            ):
                self._log_event(
                    "abort", {"time_s": self.sim.t, "reason": "invalid rope geometry"}
                )
                if self.output is not None:
                    export_bimanual(self.sim, self.frames, self.metrics, self.output)
                raise RuntimeError("Episode exceeded the rope contact search bound")
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
        """Use the same evaluator as validate_transfer, including other supports."""
        from scripts.validate_transfer import evaluate_transfer, transfer_window

        data = {
            "time": np.asarray([f["time"] for f in self.frames]),
            "joints": np.asarray([f["joints"] for f in self.frames]),
            "rope": np.asarray([f["rope"] for f in self.frames]),
            "rest_lengths": np.asarray([f["rest_lengths"] for f in self.frames]),
            "particle_masses": self.sim.model.particle_mass.numpy(),
            "anchors": self.sim.anchor_indices,
        }
        try:
            transfer_window(data["time"], self.motion.metadata)
        except ValueError as error:
            return {
                "passed": False,
                "status": "incomplete",
                "reason": str(error),
                "duration_s": float(data["time"][-1]),
                "checks": {"transfer_history": False},
                "sides": {},
            }
        payload = {
            "config": self.recipe["config"],
            "motion": self.motion.metadata,
            "collision_geometry": self.recipe["collision_geometry"],
            "self_contact_search_bound_m": float(MAX_SEGMENT_LENGTH),
            "frames": self.frames,
        }
        return evaluate_transfer(
            data,
            self.plan,
            payload,
            side="both"
            if set(self.active_sides) == {"left", "right"}
            else self.active_sides[0],
            robot=self.robot,
        )

    def finish(self) -> dict[str, Any]:
        """Export physics, append persistent logs, and return the score."""
        result = self.score()
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=True)
            export_bimanual(self.sim, self.frames, self.metrics, self.output)
            (self.output / "joint_commands.json").write_text(
                json.dumps(self.recipe["commands"]), encoding="utf-8"
            )
            (self.output / "transfer_plan.json").write_text(
                json.dumps(self.plan), encoding="utf-8"
            )
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
                        "sides": result["sides"],
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
        return result
