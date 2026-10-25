"""Near-string pickup targets must lie between the live thumb contact patches."""

import numpy as np
import pytest

from scripts.plan_bridge_transfer import free_bridge_span


def test_bridge_excludes_material_already_on_either_thumb():
    bridge = free_bridge_span(np.arange(144, 153), np.arange(163, 172), 257)
    np.testing.assert_array_equal(bridge, np.arange(153, 163))
    assert not np.intersect1d(bridge, [164, 168, 175, 180]).size


def test_bridge_follows_current_patches_across_closed_seam():
    np.testing.assert_array_equal(
        free_bridge_span([38, 39, 0], [5, 6], 40), [1, 2, 3, 4]
    )


@pytest.mark.parametrize(
    "right,left", [([], [20]), ([10], []), ([10, 11], [11, 12]), ([20, 21], [10, 11])]
)
def test_missing_overlapping_or_reversed_support_is_not_a_bridge(right, left):
    with pytest.raises(ValueError):
        free_bridge_span(right, left, 40)


@pytest.mark.parametrize("helper_mode", [None, "lift", "curl", "fixed_wrist"])
def test_thumb_pose_refinement_only_changes_permitted_robot_joints(helper_mode):
    from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF
    from revo3_cat_cradle.single_player import Motion
    from scripts.plan_thumb_pose_path import preserve_thumb_pose

    if not ROBOT_URDF.exists():
        pytest.skip("Optional RevoTron assets missing")
    robot = RevoTron()
    reference = Motion(robot, upright=True).targets[0].copy()
    desired = reference.copy()
    desired[robot.arm_indices["right"][0]] += 0.01
    desired[robot.indices["right_thumb_DIP_joint"]] += 0.2
    obstacles = np.array([[2.0, 2.0, 2.0]])
    before = (reference.copy(), desired.copy(), obstacles.copy())
    from scripts.control_old_loop import nearest_helper_surface, surface_position
    from revo3_cat_cradle.transfers import fingertip

    helper_goal = None
    if helper_mode:
        point = fingertip(robot, reference, "right", "middle") + [0, 0, 0.001]
        link, local = nearest_helper_surface(robot.fk(reference), point)
        helper_goal = (
            link,
            local,
            surface_position(robot, reference, link, local) + [0, 0, 0.001],
        )
    result = preserve_thumb_pose(
        robot,
        reference,
        desired,
        obstacles,
        evaluations=15,
        helper_goal=helper_goal,
        helper_curl=None if helper_mode != "curl" else 0.2,
        fixed_wrist=helper_mode == "fixed_wrist",
    )
    active = robot.arm_indices["right"] + [
        robot.indices[f"right_thumb_{joint}_joint"] for joint in ("CMP", "CMR", "MCP")
    ]
    if helper_mode == "fixed_wrist":
        active = []
    if helper_mode:
        active += [
            i for i, name in enumerate(robot.names) if name.startswith("right_middle_")
        ]
    fixed = [robot.indices[f"right_thumb_{joint}_joint"] for joint in ("PIP", "DIP")]
    other = [i for i in range(len(reference)) if i not in active + fixed]
    np.testing.assert_array_equal(result[fixed], reference[fixed])
    np.testing.assert_array_equal(
        result[other], (reference if helper_mode == "fixed_wrist" else desired)[other]
    )
    for actual, original in zip((reference, desired, obstacles), before):
        np.testing.assert_array_equal(actual, original)
    assert np.isfinite(result).all()
    assert np.all(result >= robot.limits[:, 0])
    assert np.all(result <= robot.limits[:, 1])
