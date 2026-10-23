"""Optional RevoTron checks; download assets with setup_revotron.sh first."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF, resample_loop
from revo3_cat_cradle.hand import Hand


def test_loop_sampling_uses_arc_length_instead_of_source_vertex_density():
    coarse = np.array([[0, 0, 0], [0.1, 0, 0], [0.1, 0.1, 0], [0, 0.1, 0]])
    dense = np.r_[
        np.column_stack((np.linspace(0, 0.1, 101), np.zeros(101), np.zeros(101))),
        coarse[2:],
    ]
    a = resample_loop(coarse, 0.005)
    b = resample_loop(dense, 0.005)
    np.testing.assert_allclose(a, b, atol=1e-8)
    assert len(a) == 80
    np.testing.assert_allclose(
        np.linalg.norm(np.roll(a, -1, axis=0) - a, axis=1), 0.005, atol=1e-7
    )


@pytest.mark.skipif(
    not ROBOT_URDF.exists(), reason="Optional official RevoTron assets not installed"
)
def test_revotron_fk_matches_hand_subtree_and_ik_reaches_both_wrists():
    robot = RevoTron()
    hand = Hand()
    q = np.zeros(len(robot.names))
    assert len(q) == 59
    for side, sign in [("left", 1), ("right", -1)]:
        rotation = Rotation.from_euler("x", 90 * sign, degrees=True).as_matrix()
        target = np.array([0.44, 0.30 * sign, 1.2])
        q = robot.ik(q, side, target, rotation)
        pose = robot.wrist(q, side)
        np.testing.assert_allclose(pose[:3, 3], target, atol=0.002)
        assert (
            np.linalg.norm(Rotation.from_matrix(pose[:3, :3] @ rotation.T).as_rotvec())
            < 0.01
        )
    q[[robot.indices[n] for n in hand.names]] = hand.limits.mean(axis=1)
    poses = robot.fk(q)
    single = hand.fk(hand.limits.mean(axis=1))
    transform = poses["left_hand_base_link"] @ np.linalg.inv(
        single["left_hand_base_link"]
    )
    for name in single:
        if name.startswith("left") and name in poses:
            np.testing.assert_allclose(
                poses[name], transform @ single[name], atol=1e-12
            )
    previous = poses["left_index_DIP_link"].copy()
    q[robot.indices["right_index_MCP_joint"]] += 0.2
    np.testing.assert_array_equal(robot.fk(q)["left_index_DIP_link"], previous)
    q[robot.indices["left_index_MCP_joint"]] += 0.2
    assert (
        np.linalg.norm(robot.fk(q)["left_index_DIP_link"][:3, 3] - previous[:3, 3])
        > 0.001
    )
