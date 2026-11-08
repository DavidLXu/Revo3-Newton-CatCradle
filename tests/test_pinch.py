"""A shared near-strand sample is required; two separate contacts do not pinch."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.pinch import (
    near_pinch_samples, near_pinch_edge_samples,
    bilateral_grip_observation, bilateral_grip_checks,
)


def fixture(gap=.002, cooriented=False, side="left"):
    transforms = {}
    for finger, sign in (("thumb", -1), ("index", -1 if cooriented else 1)):
        for k, segment in enumerate(("DIP", "PIP", "MCP")):
            center, _, radius, _ = finger_capsule(finger, segment, side)
            tf = np.eye(4)
            tf[:3, 3] = [sign * (radius + gap / 2), k, 0]
            tf[:3, 3] -= center
            transforms[f"{side}_{finger}_{segment}_link"] = tf
    return transforms


def test_same_material_contact_is_invariant_under_world_rotation():
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [.3, .7, -.8]).as_matrix()
    pose[:3, 3] = [.2, -.4, 1.3]
    transforms = {name: pose @ tf for name, tf in fixture().items()}
    points = np.array([[0, 0, 0], [0, .2, 0]]) @ pose[:3, :3].T + pose[:3, 3]
    samples = near_pinch_samples(transforms, points, "left", [0, 1])
    assert samples[0]["paired"] and not samples[1]["paired"]
    assert samples[0]["normal_dot"] == pytest.approx(-1)
    assert samples[0]["thumb_gap_m"] == pytest.approx(0, abs=1e-12)
    assert not near_pinch_samples(transforms, points, "left", [1])[0]["paired"]


def test_separate_material_contacts_do_not_count_as_a_pinch():
    points = np.array([[-.002, 0, 0], [.002, 0, 0]])
    samples = near_pinch_samples(fixture(gap=.006), points, "left", [0, 1])
    assert samples[0]["thumb_gap_m"] == pytest.approx(0)
    assert samples[1]["index_gap_m"] == pytest.approx(0)
    assert not any(row["paired"] for row in samples)


@pytest.mark.parametrize("transforms", [fixture(cooriented=True), fixture(gap=.001)])
def test_same_side_contact_and_excess_penetration_are_rejected(transforms):
    assert not near_pinch_samples(transforms, np.zeros((1, 3)), "left", [0])[0]["paired"]


@pytest.mark.parametrize("edge", [0, 1])
def test_edge_midpoint_can_be_pinched_when_both_particle_samples_miss(edge):
    rope = np.array([[0, -.002, 0], [0, .002, 0]])
    assert not any(row["paired"] for row in near_pinch_samples(fixture(), rope, "left", [0, 1]))
    rows = near_pinch_edge_samples(fixture(), rope, "left", [edge])
    middle = next(row for row in rows if row["fraction"] == .5)
    assert middle["paired"] and middle["edge"] == edge
    assert middle["thumb_gap_m"] == pytest.approx(0, abs=1e-12)
    assert middle["index_gap_m"] == pytest.approx(0, abs=1e-12)


def test_separate_contacts_on_one_edge_do_not_pin_a_common_cross_section():
    rope = np.array([[-.002, 0, 0], [.002, 0, 0]])
    rows = near_pinch_edge_samples(fixture(gap=.006), rope, "left", [0])
    assert rows[0]["thumb_gap_m"] == pytest.approx(0)
    assert rows[-1]["index_gap_m"] == pytest.approx(0)
    assert not any(row["paired"] for row in rows)


def test_proximity_is_not_two_sided_nonpositive_surface_contact():
    point = np.zeros((1, 3))
    loose = fixture(gap=.0021)
    assert near_pinch_samples(loose, point, "left", [0])[0]["paired"]
    assert not near_pinch_samples(loose, point, "left", [0], max_gap_m=0)[0]["paired"]
    compressed = near_pinch_samples(fixture(gap=.00196), point, "left", [0], max_gap_m=0)[0]
    assert compressed["paired"]
    assert compressed["thumb_gap_m"] < 0 and compressed["index_gap_m"] < 0


def test_edge_sampling_uses_the_same_strict_contact_threshold():
    rope = np.array([[0, -.002, 0], [0, .002, 0]])
    assert any(row["paired"] for row in near_pinch_edge_samples(fixture(gap=.0021), rope, "left", [0]))
    assert not any(row["paired"] for row in near_pinch_edge_samples(
        fixture(gap=.0021), rope, "left", [0], max_gap_m=0))
    assert any(row["paired"] for row in near_pinch_edge_samples(
        fixture(gap=.00196), rope, "left", [0], max_gap_m=0))


@pytest.mark.parametrize("gap", [float("nan"), -.0001, .0002])
def test_contact_threshold_cannot_weaken_the_existing_proximity_gate(gap):
    with pytest.raises(ValueError):
        near_pinch_samples(fixture(), np.zeros((1, 3)), "left", [0], max_gap_m=gap)


def test_bilateral_guard_rejects_extra_old_contact_while_near_grip_remains():
    transforms = fixture() | fixture(side="right")
    rope = np.array([[0, -.002, 0], [0, .002, 0], [0, .2, 0], [0, .3, 0]])
    plan = {side: dict(near_indices=[0, 1], old_loop_indices=[2, 3])
            for side in ("left", "right")}
    observed = bilateral_grip_observation(transforms, rope, plan)
    assert all(bilateral_grip_checks(observed).values())
    before = rope.copy()
    rope[2:] = [[0, -.001, 0], [0, .001, 0]]
    observed = bilateral_grip_observation(transforms, rope, plan)
    checks = bilateral_grip_checks(observed)
    assert checks["left_near_proximity_each_frame"]
    assert checks["right_near_proximity_each_frame"]
    assert not checks["left_no_old_strand_paired_samples"]
    assert not checks["right_no_old_strand_paired_samples"]
    assert observed["right"]["old_loop_indices"] == [2]
    np.testing.assert_array_equal(rope[:2], before[:2])


def test_bilateral_guard_identifies_the_hand_that_lost_contact():
    transforms = fixture() | fixture(side="right", gap=.006)
    rope = np.array([[0, -.002, 0], [0, .002, 0], [0, .2, 0], [0, .3, 0]])
    plan = {side: dict(near_indices=[0, 1], old_loop_indices=[2, 3])
            for side in ("left", "right")}
    unchanged = rope.copy()
    checks = bilateral_grip_checks(bilateral_grip_observation(transforms, rope, plan))
    assert checks == dict(left_near_proximity_each_frame=True,
                         right_near_proximity_each_frame=False,
                         left_no_old_strand_paired_samples=True,
                         right_no_old_strand_paired_samples=True)
    np.testing.assert_array_equal(rope, unchanged)
