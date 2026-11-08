"""Transfer planning must preserve joint scope and rope material identity."""

import numpy as np
import pytest
from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF
from revo3_cat_cradle.single_player import Motion
from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.transfers import fingertip, digit_target

pytestmark = pytest.mark.skipif(
    not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing"
)


def test_open_arc_sweep_does_not_invent_a_closing_chord():
    from revo3_cat_cradle.transfers import thumb_arc_sweep

    center, _, _, _ = finger_capsule("thumb", "DIP", "right")
    tf = np.eye(4)
    tf[:3, 3] = [0.4, -0.08, 1.2]
    transforms = {"right_thumb_DIP_link": tf}
    for sweep in (np.pi / 2, 1.5 * np.pi):
        angles = np.linspace(-0.8, -0.8 + sweep, 30)
        points = (
            center + np.c_[0.02 * np.cos(angles), np.zeros(30), 0.02 * np.sin(angles)]
        )
        points += tf[:3, 3]
        assert thumb_arc_sweep(transforms, points, "right") == pytest.approx(sweep)


def test_overpass_curtain_interpolates_slope_and_keeps_multiple_branches():
    from revo3_cat_cradle.transfers import over_strand_margin

    strand = np.array([[0.0, -1.0, 1.0], [0.0, 1.0, 2.0], [0.01, -1.0, 3.0]])
    assert over_strand_margin([0.0, 0.0, 2.51], strand) == pytest.approx(0.007)
    assert over_strand_margin([0.0, 0.0, 2.49], strand) < 0
    assert np.isinf(over_strand_margin([0.1, 0.0, 0.0], strand))
    flat = np.array([[-0.1, 0.0, 1.0], [0.1, 0.0, 1.0]])
    assert over_strand_margin([0.0, 0.0, 1.02], flat) == pytest.approx(0.017)


def test_thumb_tip_mirrors_local_direction_and_digit_ik_leaves_arms_fixed():
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    tf = robot.fk(q)
    for side, sign in [("left", -1), ("right", 1)]:
        c, axis, radius, half = finger_capsule("thumb", "DIP", side)
        expected = (
            tf[f"{side}_thumb_DIP_link"] @ np.r_[c + sign * axis * (half + radius), 1]
        )[:3]
        np.testing.assert_allclose(fingertip(robot, q, side, "thumb"), expected)
        desired = q.copy()
        ids = [i for i, n in enumerate(robot.names) if n.startswith(f"{side}_thumb_")]
        desired[ids[2]] -= 0.1
        target = fingertip(robot, desired, side, "thumb")
        solved = digit_target(robot, q, side, "thumb", target)
        np.testing.assert_allclose(
            fingertip(robot, solved, side, "thumb"), target, atol=1e-5
        )
        untouched = [i for i in range(len(q)) if i not in ids]
        np.testing.assert_array_equal(solved[untouched], q[untouched])
        assert np.all(solved >= robot.limits[:, 0]) and np.all(
            solved <= robot.limits[:, 1]
        )


def test_digit_path_with_no_forbidden_rope_still_preserves_joint_scope():
    from revo3_cat_cradle.transfers import plan_digit_path

    robot = RevoTron()
    initial = Motion(robot, upright=True).targets[0]
    target = initial.copy()
    target[robot.indices["left_thumb_MCP_joint"]] -= .02
    route = plan_digit_path(robot, initial, target, "left", "thumb", np.empty((0, 3)))
    np.testing.assert_array_equal(route[-1], target)
    fixed = [i for i, name in enumerate(robot.names) if not name.startswith("left_thumb_")]
    for q in route:
        np.testing.assert_array_equal(q[fixed], initial[fixed])
    # An actual forbidden strand is still tested at the source endpoint.
    obstacle = fingertip(robot, initial, "left", "thumb")[None]
    with pytest.raises(ValueError, match="endpoints are not clear"):
        plan_digit_path(robot, initial, target, "left", "thumb", obstacle)


def test_support_arc_crosses_closed_seam_without_passing_another_support(monkeypatch):
    import revo3_cat_cradle.transfers as transfers
    from types import SimpleNamespace

    contacts = {
        ("left", "thumb"): 0,
        ("left", "index"): 2,
        ("left", "little"): 4,
        ("right", "little"): 6,
        ("right", "index"): 8,
        ("right", "thumb"): 10,
    }

    def fake_gaps(tf, points, side, finger):
        distances = np.ones(12)
        distances[contacts[side, finger]] = 0
        return distances

    monkeypatch.setattr(transfers, "gaps", fake_gaps)
    robot = SimpleNamespace(fk=lambda q: {})
    points = np.zeros((12, 3))
    arc = transfers.support_arc(
        robot, np.zeros(1), points, ("left", "thumb"), ("right", "thumb")
    )
    np.testing.assert_array_equal(arc, [0, 11, 10])
    with pytest.raises(ValueError, match="Ambiguous"):
        transfers.support_arc(
            robot, np.zeros(1), points, ("left", "index"), ("right", "little")
        )
    # Opening A's thumb has the opposite thumb and opposite index as neighbors.
    # A release audit must include both branches, including the seam-crossing one.
    contacts["left", "index"] = 8
    contacts["right", "index"] = 2
    loop = transfers.original_thumb_loop(robot, np.zeros(1), points, "left")
    np.testing.assert_array_equal(loop, [0, 1, 2, 10, 11])


def test_support_patch_ignores_only_isolated_grazes_and_handles_loop_seam():
    from revo3_cat_cradle.transfers import support_contact_patch

    np.testing.assert_array_equal(
        support_contact_patch([1, 2, 3, 4, 9], 12), [1, 2, 3, 4]
    )
    np.testing.assert_array_equal(
        support_contact_patch([0, 1, 6, 10, 11], 12), [10, 11, 0, 1]
    )
    with pytest.raises(ValueError, match="Ambiguous"):
        support_contact_patch([1, 2, 3, 7, 8], 12)
    with pytest.raises(ValueError, match="Ambiguous"):
        support_contact_patch([1, 7], 12)


def test_digit_avoidance_rejects_a_sweep_starting_on_forbidden_strand():
    robot = RevoTron()
    q = Motion(robot, upright=True).targets[0]
    tip = fingertip(robot, q, "left", "thumb")
    with pytest.raises(ValueError, match="misses|clearance"):
        digit_target(robot, q, "left", "thumb", tip, avoid_points=tip[None, :])
