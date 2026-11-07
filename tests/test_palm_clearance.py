import numpy as np
import trimesh
import json
from pathlib import Path

from revo3_cat_cradle.palm_clearance import (
    capsule_hull_clearance,
    OpposingPalmClearance,
)


def test_axis_intersection_is_detected_even_when_endpoints_are_outside():
    box = trimesh.creation.box(extents=(1, 1, 1))
    bound, sampled = capsule_hull_clearance(box, [-2, 0, 0], [2, 0, 0], 0.1)
    assert sampled < -0.59
    assert bound <= sampled


def test_separated_capsule_has_a_conservative_clearance_bound():
    box = trimesh.creation.box(extents=(1, 1, 1))
    bound, sampled = capsule_hull_clearance(box, [-0.4, 1, 0], [0.4, 1, 0], 0.1)
    np.testing.assert_allclose(sampled, 0.4, atol=1e-12)
    assert 0.3997 < bound <= sampled


def test_capsule_radius_catches_an_axis_that_misses_the_palm():
    box = trimesh.creation.box(extents=(1, 1, 1))
    _, sampled = capsule_hull_clearance(box, [0, 0.55, -0.2], [0, 0.55, 0.2], 0.1)
    assert sampled < 0


def test_recorded_finger_clear_pose_still_intersects_opposite_palm():
    from revo3_cat_cradle.bimanual import RevoTron
    from revo3_cat_cradle.bimanual_checks import hand_clearance

    path = (
        Path(__file__).resolve().parents[1]
        / "examples/transfers/rejected_stagger_roll.json"
    )
    q = np.asarray(json.loads(path.read_text())[-1]["joints"])
    robot = RevoTron()
    poses = robot.fk(q)
    assert hand_clearance(poses)[0] > 0.003
    checker = OpposingPalmClearance(robot, poses)
    for quick in (False, True):
        report = checker.measure(poses, quick=quick)
        assert report["sampled_finger_palm_gap_m"] < -0.008
        assert report["palm_palm_sphere_lower_bound_m"] > 0


def test_separating_plane_bound_certifies_the_opening_fixture():
    from revo3_cat_cradle.bimanual import RevoTron

    path = (
        Path(__file__).resolve().parents[1]
        / "examples/transfers/global_opening_seed.json"
    )
    q = np.asarray(json.loads(path.read_text())["commands"]["targets"][0])
    robot = RevoTron()
    poses = robot.fk(q)
    checker = OpposingPalmClearance(robot, poses)
    precise = checker.measure(poses)
    quick = checker.measure(poses, quick=True)
    # The analytic face bound can be tighter than a sampled bound reduced by
    # half a sample interval; it must still lie below the sampled distance.
    assert (
        0.05
        < quick["finger_palm_lower_bound_m"]
        <= precise["sampled_finger_palm_gap_m"]
    )
    assert quick["palm_palm_sphere_lower_bound_m"] > 0.05
