"""Published recipes must preserve robot identity and interrupted commands."""

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.single_player import initial_loop
from scripts.replay_transfer import recipe_motion

EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "transfers"


@pytest.fixture(scope="module")
def robot():
    return RevoTron()


@pytest.mark.parametrize("name", ["left_pickup", "right_pickup_attempt"])
def test_published_recipe_preserves_clipped_quintic_and_initial_seed(robot, name):
    recipe = json.loads((EXAMPLES / f"{name}.json").read_text())
    original = deepcopy(recipe)
    motion = recipe_motion(recipe, robot)
    np.testing.assert_array_equal(
        initial_loop(robot, motion), recipe["motion"]["initial_rope_points"]
    )
    for segment in recipe["motion"]["interrupted_segments"]:
        time = (segment["start"] + segment["stop"]) / 2
        u = (time - segment["start"]) / (segment["end"] - segment["start"])
        u = u**3 * (10 + u * (-15 + 6 * u))
        expected = (1 - u) * np.array(segment["initial"]) + u * np.array(
            segment["target"]
        )
        np.testing.assert_array_equal(motion.sample(time)[0], expected)
    np.testing.assert_array_equal(
        motion.sample(motion.times[-1])[0], motion.targets[-1]
    )
    motion.metadata["status"] = "test"
    assert recipe == original


@pytest.mark.parametrize(
    "invalid", ["joint_order", "time", "limit", "events", "segment"]
)
def test_replay_rejects_incompatible_recipe(robot, invalid):
    recipe = json.loads((EXAMPLES / "left_pickup.json").read_text())
    if invalid == "joint_order":
        recipe["joint_names"].reverse()
    elif invalid == "time":
        recipe["commands"]["times"][1] = 0
    elif invalid == "limit":
        recipe["commands"]["targets"][0][0] = 100
    elif invalid == "events":
        recipe["motion"]["physics_events"] = [
            {"time": 1, "parameter": "friction", "value": 2}
        ]
    else:
        recipe["motion"]["interrupted_segments"][0]["target"][0] = 100
    with pytest.raises(ValueError):
        recipe_motion(recipe, robot)
