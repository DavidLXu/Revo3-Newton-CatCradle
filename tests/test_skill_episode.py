"""The episode must never certify a transfer with weaker gates than the CLI."""

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from revo3_cat_cradle.skill_episode import BimanualSkillEpisode
from scripts.validate_transfer import evaluate_transfer


@pytest.fixture(scope="module")
def episode():
    recipe = Path(__file__).resolve().parents[1] / "examples/transfers/left_pickup.json"
    return BimanualSkillEpisode.from_recipe(recipe, active_sides=("left",))


def test_early_episode_is_incomplete_and_observations_are_detached(episode):
    assert episode.score()["status"] == "incomplete"
    observation = episode.observe()
    assert not {"rope", "joints", "rest_lengths"}.intersection(observation)
    observation["thumb_contacts"]["left"]["far_gap_m"] = -100
    assert episode.observe()["thumb_contacts"]["left"]["far_gap_m"] > 0


def test_episode_uses_native_gates_and_checks_inactive_thumb(episode):
    original_frames, original_motion = episode.frames, deepcopy(episode.motion.metadata)
    try:
        # A synthetic stationary window isolates evaluator wiring; this is not
        # represented as a simulated pickup or a successful physical run.
        episode.frames = [deepcopy(original_frames[0]) for _ in range(61)]
        for frame, time in zip(episode.frames, np.arange(180, 241) / 30):
            frame.update(time=float(time), phase="Hold new thumb bights")
        episode.motion.metadata.update(transfer_reference_time_s=6, duration=8)
        data = {
            "time": np.array([f["time"] for f in episode.frames]),
            "joints": np.array([f["joints"] for f in episode.frames]),
            "rope": np.array([f["rope"] for f in episode.frames]),
            "rest_lengths": np.array([f["rest_lengths"] for f in episode.frames]),
            "particle_masses": episode.sim.model.particle_mass.numpy(),
            "anchors": episode.sim.anchor_indices,
        }
        payload = dict(
            config=episode.recipe["config"],
            motion=episode.motion.metadata,
            collision_geometry=episode.recipe["collision_geometry"],
            frames=episode.frames,
            self_contact_search_bound_m=0.016,
        )
        expected = evaluate_transfer(data, episode.plan, payload, side="left")
        assert episode.score() == expected
        assert "right_near_string_not_caught" in expected["checks"]
        assert "other_supports_retained_during_transfer" in expected["checks"]
        assert expected["checks"]["completed_hold"]

        # A final 'hold' label cannot hide commands that still move in its window.
        data["joints"][-2, 0] += 0.01
        assert not evaluate_transfer(data, episode.plan, payload, side="left")[
            "checks"
        ]["completed_hold"]
        data["joints"][-2, 0] -= 0.01
        payload["frames"][-2]["phase"] = "Still picking up"
        assert not evaluate_transfer(data, episode.plan, payload, side="left")[
            "checks"
        ]["completed_hold"]
    finally:
        episode.frames = original_frames
        episode.motion.metadata = original_motion


def test_episode_rejects_existing_output_before_starting_physics(tmp_path):
    with pytest.raises(FileExistsError):
        BimanualSkillEpisode({}, output=tmp_path)


def test_episode_finish_exports_continuation_inputs_and_incomplete_score(
    episode, tmp_path
):
    original = episode.output
    episode.output = tmp_path / "smoke"
    try:
        result = episode.finish()
        assert not result["passed"]
        for name in (
            "joint_commands.json",
            "transfer_plan.json",
            "result.json",
            "summary.json",
        ):
            json.loads((episode.output / name).read_text())
        assert (episode.output / "checkpoint.npz").exists()
    finally:
        episode.output = original
