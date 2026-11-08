"""A shared near-strand sample is required; two separate contacts do not pinch."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.pinch import near_pinch_samples


def fixture(gap=.002, cooriented=False):
    transforms = {}
    for finger, sign in (("thumb", -1), ("index", -1 if cooriented else 1)):
        for k, segment in enumerate(("DIP", "PIP", "MCP")):
            center, _, radius, _ = finger_capsule(finger, segment)
            tf = np.eye(4)
            tf[:3, 3] = [sign * (radius + gap / 2), k, 0]
            tf[:3, 3] -= center
            transforms[f"left_{finger}_{segment}_link"] = tf
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
