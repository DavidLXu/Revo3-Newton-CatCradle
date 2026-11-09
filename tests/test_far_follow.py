"""A moving helper may be scripted while feedback follows only its known bight."""
from copy import deepcopy

import numpy as np
import pytest

from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF
from revo3_cat_cradle.single_player import Motion
from revo3_cat_cradle.transfers import fingertip
from scripts.continue_cup import far_follow_step

pytestmark = pytest.mark.skipif(not ROBOT_URDF.exists(), reason="Optional robot assets missing")


def fixture():
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    tip = fingertip(robot, q, "right", "thumb")
    rope = np.tile(tip + [0, 0, .02], (40, 1))
    rope[:, 0] += np.linspace(0, .003, 40)
    rope[22:26] += [0, 0, .05]
    plan = {"right": dict(far_indices=[*range(35, 40), *range(6), 20],
                          near_indices=[22, 23], old_loop_indices=[24, 25])}
    nominal = q.copy()
    little = robot.indices["right_little_MCP_joint"]
    nominal[little] = min(q[little]+.3, robot.limits[little, 1])
    return robot, q, nominal, rope, plan, little


def test_joint_feedback_bounds_and_immutable_observations():
    robot, q, nominal, rope, plan, little = fixture()
    before = [x.copy() for x in (q, nominal, rope)]
    old_plan = deepcopy(plan)
    target, report = far_follow_step(robot, q, nominal, rope, np.zeros_like(rope),
                                     plan, "right", 39, 1/30, .7)
    thumb = [i for i,n in enumerate(robot.names) if n.startswith("right_thumb_")]
    fixed = [i for i in range(len(q)) if i not in [*thumb, little]]
    np.testing.assert_array_equal(target[fixed], q[fixed])
    assert np.max(abs(target-q))*30*1.875 <= 3+1e-10
    assert np.max(abs(target[thumb]-q[thumb]))*30*1.875 <= .7+1e-10
    assert np.all(target >= robot.limits[:, 0]) and np.all(target <= robot.limits[:, 1])
    for now, original in zip((q, nominal, rope), before):
        np.testing.assert_array_equal(now, original)
    assert plan == old_plan and report["tracked_material"] == 39
    assert set(report["material_window"]) == {*range(35, 40), *range(6)}


def test_feedback_does_not_switch_to_distant_far_material_or_retain_old_loops():
    robot, q, nominal, rope, plan, _ = fixture()
    altered = rope.copy()
    altered[[20, 24, 25]] += 10
    results = [far_follow_step(robot, q, nominal, points, np.zeros_like(points),
                              plan, "right", 39, 1/30)[0]
               for points in (rope, altered)]
    np.testing.assert_array_equal(*results)


@pytest.mark.parametrize("center", [-1, 40, 2.5, True, 10])
def test_feedback_requires_a_valid_member_of_the_original_far_arc(center):
    robot, q, nominal, rope, plan, _ = fixture()
    with pytest.raises(ValueError, match="far-material"):
        far_follow_step(robot, q, nominal, rope, np.zeros_like(rope), plan,
                        "right", center, 1/30)
