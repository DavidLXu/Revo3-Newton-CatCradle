"""Feedback may move thumb joints, but not arm joints or rope observations."""

import numpy as np
import pytest

from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF
from revo3_cat_cradle.bimanual_checks import hand_clearance, self_clearance_margin
from revo3_cat_cradle.single_player import Motion
from revo3_cat_cradle.transfers import fingertip
from scripts.control_thumb_pickup import feedback_target, thumb_pair_clearances

pytestmark = pytest.mark.skipif(
    not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing"
)


def test_thumb_distances_match_independent_full_hand_query():
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    for angle in (0, 0.3):
        q[robot.indices["left_thumb_MCP_joint"]] += angle
        tf = robot.fk(q)
        expected = [
            g
            for g, a, b in hand_clearance(tf, all_pairs=True)
            if a.startswith("thumb_") and b.startswith("thumb_")
        ]
        np.testing.assert_allclose(
            np.sort(thumb_pair_clearances(tf)), expected, atol=1e-12
        )


@pytest.mark.parametrize("sides", [("left", "right"), ("left",)])
@pytest.mark.parametrize(
    "retained", [("far_indices", "old_loop_indices"), ("old_loop_indices",)]
)
def test_feedback_scope_speed_and_observation_immutability(sides, retained):
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    points, plan = [], {}
    for side in ("left", "right"):
        tip = fingertip(robot, q, side, "thumb")
        n = len(points)
        points.extend([tip + [0, 0, 0.02], tip + [0, 0, 0.05], tip])
        plan[side] = dict(
            far_indices=[n], near_indices=[n + 1], old_loop_indices=[n + 2]
        )
    rope = np.asarray(points)
    velocity = np.zeros_like(rope)
    original = rope.copy()
    before = q.copy()
    target, _, _ = feedback_target(
        robot,
        q,
        rope,
        velocity,
        plan,
        q,
        1 / 30,
        1.5,
        0.003,
        1,
        sides=sides,
        retained_strands=retained,
    )
    other = [
        i
        for i, name in enumerate(robot.names)
        if not any(name.startswith(f"{side}_thumb_") for side in sides)
    ]
    np.testing.assert_array_equal(target[other], before[other])
    np.testing.assert_array_equal(q, before)
    np.testing.assert_array_equal(rope, original)
    np.testing.assert_array_equal(velocity, np.zeros_like(rope))
    assert np.max(abs(target - q)) * 30 * 1.875 <= 1.5 + 1e-10
    assert np.all(target >= robot.limits[:, 0]) and np.all(target <= robot.limits[:, 1])


@pytest.mark.parametrize("seat_old_support", [False, True])
def test_assisting_thumb_does_not_chase_or_seat_an_uncaptured_far_strand(
    seat_old_support,
):
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    points, plan = [], {}
    for side in ("left", "right"):
        tip = fingertip(robot, q, side, "thumb")
        n = len(points)
        points.extend([tip + [0, 0, 0.02], tip + [0, 0, 0.05], tip])
        plan[side] = dict(
            far_indices=[n], near_indices=[n + 1], old_loop_indices=[n + 2]
        )
    original = np.array(points)
    changed = original.copy()
    changed[plan["right"]["far_indices"]] += 10
    results = []
    for rope in (original, changed):
        result, _, _ = feedback_target(
            robot,
            q,
            rope,
            np.zeros_like(rope),
            plan,
            q,
            1 / 30,
            0.8,
            0.0025,
            0.25,
            seat_depth=0.004,
            retained_by_side={
                "left": ("far_indices", "old_loop_indices"),
                "right": ("old_loop_indices",),
            },
            pose_weights={"left": 0.001, "right": 0.02},
            seated_strands={"right": "old_loop_indices"} if seat_old_support else None,
        )
        results.append(result)
    np.testing.assert_array_equal(results[0], results[1])
    np.testing.assert_array_equal(original, points)


def test_near_strand_feedback_cannot_accumulate_thumb_index_overlap():
    """A recorded near-collision geometry must not deepen on successive steps."""
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    for finger in ("middle", "ring"):
        q[robot.indices[f"left_{finger}_MPR_joint"]] = 0
    q[robot.indices["left_little_MCP_joint"]] = 0.4
    for joint, angle in zip(
        ("CMP", "CMR", "MCP", "PIP", "DIP"),
        (0.36199, 1.28521, 0.79859, 1.09104, 1.24549),
    ):
        q[robot.indices[f"left_thumb_{joint}_joint"]] = angle
    for joint, angle in zip(("MPR", "MCP", "PIP", "DIP"), (0, -0.08, -0.1, -0.2)):
        q[robot.indices[f"left_index_{joint}_joint"]] = angle
    local = np.array(
        [
            [0.011422249, -0.022445004, 0.108070015],
            [0.018780958, -0.021042666, 0.106546045],
            [0.001683510, -0.024302550, 0.110126331],
            [0.022177415, -0.045507773, 0.097438885],
            [0.014750618, -0.046414352, 0.099323947],
            [0.030065810, -0.044545944, 0.095472744],
            [0.012167314, -0.067175734, 0.090249103],
            [0.012648692, -0.059115983, 0.088838578],
            [0.015659219, -0.073125013, 0.090916962],
        ]
    )
    base = robot.fk(q)["left_hand_base_link"]
    rope = local @ base[:3, :3].T + base[:3, 3]
    nominal = q.copy()
    plan = {
        "left": dict(
            far_indices=[0, 1, 2], near_indices=[3, 4, 5], old_loop_indices=[6, 7, 8]
        )
    }
    assert self_clearance_margin(robot.fk(q), "left") > 0
    for _ in range(12):
        target, _, _ = feedback_target(
            robot,
            q,
            rope,
            np.zeros_like(rope),
            plan,
            nominal,
            1 / 30,
            0.8,
            0.004,
            0.25,
            contact_weight=10,
            seat_depth=0.004,
            sides=("left",),
            pose_weight=0.001,
        )
        for u in np.linspace(0, 1, 21):
            assert self_clearance_margin(robot.fk(q + u * (target - q)), "left") >= 0
        q = target
