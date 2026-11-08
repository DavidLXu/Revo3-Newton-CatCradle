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


def test_deeper_index_hook_clears_opposing_palm_during_release():
    from revo3_cat_cradle.bimanual import RevoTron

    path = (
        Path(__file__).resolve().parents[1]
        / "examples/transfers/release_both_little_loops.json"
    )
    route = json.loads(path.read_text())
    robot = RevoTron()
    checker = OpposingPalmClearance(robot, robot.fk(np.asarray(route[0]["joints"])))
    # The final 30 degrees of wrist roll previously drove the index through
    # the opposite palm, although opposing finger capsules stayed clear.
    start, end = [np.asarray(row["joints"]) for row in route[-2:]]
    for u in np.linspace(0, 1, 11):
        report = checker.measure(robot.fk((1 - u) * start + u * end), quick=True)
        assert report["finger_palm_lower_bound_m"] > 0.001
        assert report["palm_palm_sphere_lower_bound_m"] > 0.02


def test_overfolded_little_passes_opposing_check_but_overlaps_own_palm():
    from revo3_cat_cradle.bimanual import RevoTron

    path = (Path(__file__).resolve().parents[1]
            / "examples/transfers/attempt_roll_released_back40.json")
    robot = RevoTron()
    q = np.asarray(json.loads(path.read_text())[-1]["joints"])
    checker = OpposingPalmClearance(robot, robot.fk(q))
    initial = checker.own_distal_clearance(robot.fk(q), "left", "little")
    assert initial["lower_bound_m"] > 0
    for joint, angle in [("MCP", 1.583), ("PIP", 1.6001), ("DIP", 1.55)]:
        q[robot.indices[f"left_little_{joint}_joint"]] = angle
    poses = robot.fk(q)
    assert checker.measure(poses, quick=True)["finger_palm_lower_bound_m"] > 0
    own = checker.own_distal_clearance(poses, "left", "little")
    assert own["link"] == "left_little_DIP_link"
    assert own["sampled_gap_m"] < -.004
    # Arm motion is deliberately absent from the continuation/audit cache key.
    from scipy.spatial.transform import Rotation
    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler("xyz", [30, -40, 70], degrees=True).as_matrix()
    world[:3, 3] = [.4, -.2, .1]
    moved = checker.own_distal_clearance(
        {name: world @ pose for name, pose in poses.items()}, "left", "little")
    np.testing.assert_allclose(moved["lower_bound_m"], own["lower_bound_m"], atol=1e-12)
