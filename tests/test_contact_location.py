"""Tip contact and shaft contact need different pickup motions."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from revo3_cat_cradle.contact_geometry import finger_capsule
from scripts.audit_finger_contacts import audit_frame


class RigidFixture:
    def __init__(self, pose):
        self.pose = pose

    def fk(self, _):
        return {
            f"{side}_{finger}_{segment}_link": self.pose
            for side in ("left", "right")
            for finger in ("thumb", "index", "middle", "ring", "little")
            for segment in ("MCP", "PIP", "DIP")
        }


@pytest.mark.parametrize("region,along", [("shaft", 0), ("positive_cap", .0195),
                                           ("negative_cap", -.0195)])
def test_contact_location_distinguishes_tip_from_shaft_in_world_frame(region, along):
    center, axis, radius, _ = finger_capsule("middle", "DIP")
    contact = center + along * axis
    if region == "shaft":
        contact += np.array([radius + .001, 0, 0])
    points = contact + np.array([[0, -.02, 0], [0, .02, 0]])
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", [.4, -.8, 1.1]).as_matrix()
    pose[:3, 3] = [.3, -.2, 1.4]
    world = points @ pose[:3, :3].T + pose[:3, 3]
    rows = audit_frame(RigidFixture(pose), None, world, .001)
    row = next(item for item in rows if item["finger"] == "left_middle_DIP_link")
    assert row["edge_gap_m"] == pytest.approx(0, abs=1e-12)
    assert row["rope_axial_m"] == pytest.approx(along, abs=1e-12)
    assert row["closest_region"] == region


def test_material_filter_does_not_confuse_an_existing_shaft_contact_with_new_tip_contact():
    center, axis, radius, half = finger_capsule("middle", "DIP")
    shaft = center + np.array([radius + .001, 0, 0])
    tip = center + (half + radius + .001) * axis
    ends = np.array([[0, -.02, 0], [0, .02, 0]])
    points = np.vstack([shaft + ends, tip + ends])
    for edge, region in [(0, "shaft"), (2, "positive_cap")]:
        rows = audit_frame(RigidFixture(np.eye(4)), None, points, .001,
                           edge_indices=[edge])
        row = next(item for item in rows if item["finger"] == "left_middle_DIP_link")
        assert row["edge"] == edge
        assert row["closest_region"] == region
        assert row["edge_gap_m"] == pytest.approx(0, abs=1e-12)
        assert row["minimum_particle_gap_m"] > .008
