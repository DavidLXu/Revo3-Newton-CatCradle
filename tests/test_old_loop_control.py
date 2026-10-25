"""The helper controller manipulates robot joints, not rope observations."""

import numpy as np
import pytest

from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF
from revo3_cat_cradle.bimanual_checks import digit_clearances, hand_clearance
from revo3_cat_cradle.single_player import Motion
from revo3_cat_cradle.transfers import fingertip
from scripts.control_old_loop import (
    nearest_helper_surface,
    surface_position,
    lift_target,
    retain_right_thumb_target,
)
from scripts.plan_old_loop_support import support_geometry
from revo3_cat_cradle.contact_geometry import finger_capsule

pytestmark = pytest.mark.skipif(
    not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing"
)


@pytest.mark.parametrize("side,finger", [("right", "middle"), ("left", "thumb")])
def test_digit_collision_barrier_matches_full_hand_query(side, finger):
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    tf = robot.fk(q)
    opposite = [
        g
        for g, a, b in hand_clearance(tf, all_pairs=True)
        if (a if side == "left" else b).startswith(finger + "_")
    ]
    own = [
        g
        for g, a, b in hand_clearance(tf, side, all_pairs=True)
        if a.startswith(finger + "_") or b.startswith(finger + "_")
    ]
    np.testing.assert_allclose(
        np.sort(digit_clearances(tf, side, finger)), sorted(opposite + own), atol=1e-12
    )


def test_helper_step_preserves_rope_and_nonhelper_joints():
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    q[robot.indices["right_middle_MCP_joint"]] = 1
    q[robot.indices["right_middle_PIP_joint"]] = 1.6
    q[robot.indices["right_middle_DIP_joint"]] = -0.3
    rope = np.array([fingertip(robot, q, "right", "middle") + [0.001, 0, 0], [2, 2, 2]])
    original = rope.copy()
    before = q.copy()
    link, local = nearest_helper_surface(robot.fk(q), rope[0])
    goal = surface_position(robot, q, link, local) + [0, 0, 0.002]
    target, _, _ = lift_target(
        robot, q, rope, np.array([0]), rope[1:], link, local, goal, 1 / 30, 1.2
    )
    other = [i for i, n in enumerate(robot.names) if not n.startswith("right_middle_")]
    np.testing.assert_array_equal(target[other], q[other])
    np.testing.assert_array_equal(q, before)
    np.testing.assert_array_equal(rope, original)
    assert np.max(abs(target - q)) * 30 * 1.875 <= 1.2 + 1e-10


def test_support_normal_distinguishes_above_from_below_in_world_frame():
    # Rotate the local cylinder axis into world X. Its upper/lower surface must
    # be classified by world Z, not the URDF local frame or distance alone.
    tf = np.eye(4)
    tf[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    tf[:3, 3] = [0.4, 0.1, 1.2]
    center, _, radius, _ = finger_capsule("middle", "PIP", "right")
    world_center = tf[:3, :3] @ center + tf[:3, 3]
    for sign in (-1, 1):
        point = world_center + [0, 0, sign * (radius + 0.001)]
        gap, normal, axial = support_geometry({"right_middle_PIP_link": tf}, point)
        assert abs(gap) < 1e-12
        assert abs(axial) < 1e-12
        np.testing.assert_allclose(normal, [0, 0, sign], atol=1e-12)


@pytest.mark.parametrize("hold_surface", [False, True])
def test_original_thumb_retention_changes_only_bounded_thumb_commands(hold_surface):
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    rope = np.array([fingertip(robot, q, "right", "thumb") + [0, 0, 0.001]])
    before_q, before_rope = q.copy(), rope.copy()
    goal = None
    if hold_surface:
        link, local = nearest_helper_surface(robot.fk(q), rope[0], finger="thumb")
        goal = (link, local, surface_position(robot, q, link, local) + [0, 0, 0.001])
    target = retain_right_thumb_target(robot, q, rope, np.array([0]), 1 / 30, 1.2, goal)
    other = [
        i for i, name in enumerate(robot.names) if not name.startswith("right_thumb_")
    ]
    np.testing.assert_array_equal(target[other], q[other])
    np.testing.assert_array_equal(q, before_q)
    np.testing.assert_array_equal(rope, before_rope)
    assert np.max(abs(target - q)) * 30 * 1.875 <= 1.2 + 1e-10
