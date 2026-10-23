"""Geometry checks for the experimental single-player sequence."""

import numpy as np
from scipy.spatial.transform import Rotation
from revo3_cat_cradle.sim import Simulation
from revo3_cat_cradle.bimanual_checks import triangle_coordinates


def test_batched_body_motion_matches_com_velocity_in_world_frame():
    rng = np.random.default_rng(14)
    current = np.repeat(np.eye(4)[None], 9, axis=0)
    previous = current.copy()
    current[:, :3, :3] = Rotation.random(9, random_state=rng).as_matrix()
    previous[:, :3, :3] = Rotation.random(9, random_state=rng).as_matrix()
    current[:, :3, 3] = rng.normal(size=(9, 3))
    previous[:, :3, 3] = rng.normal(size=(9, 3))
    centers = rng.normal(size=(9, 3)) * 0.02
    poses, velocities = Simulation.body_kinematics(current, previous, centers, 0.001)
    expected = [
        Simulation.body_velocity(a, b, c, 0.001)
        for a, b, c in zip(current, previous, centers)
    ]
    np.testing.assert_allclose(velocities, expected, atol=1e-10)
    np.testing.assert_allclose(poses[:, :3], current[:, :3, 3])
    np.testing.assert_allclose(
        Rotation.from_quat(poses[:, 3:]).as_matrix(), current[:, :3, :3], atol=1e-14
    )


def test_threading_distinguishes_inside_bight_from_outside_plane_crossing():
    triangle = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    front = triangle_coordinates(np.array([0.2, 0.3, 0.1]), triangle)
    back = triangle_coordinates(np.array([0.2, 0.3, -0.1]), triangle)
    outside = triangle_coordinates(np.array([1.0, 1.0, 0.0]), triangle)
    assert front[0] * back[0] < 0
    assert front[1].min() > 0.03
    assert outside[1].min() < 0
    assert triangle_coordinates(np.zeros(3), np.zeros((3, 3))) is None


def test_self_clearance_uses_requested_hands_thumb_geometry():
    from revo3_cat_cradle.bimanual_checks import hand_clearance

    transforms = {}
    for side in ("left", "right"):
        for i, finger in enumerate(("thumb", "index", "middle", "ring", "little")):
            for segment in ("MCP", "PIP", "DIP"):
                t = np.eye(4)
                t[0, 3] = -0.1 if finger == "thumb" else (i - 1) * 0.1
                t[1, 3] = 0.02 if finger == "index" else 0.0
                transforms[f"{side}_{finger}_{segment}_link"] = t
    for side, dy in [("left", 0.0249), ("right", 0.0)]:
        pairs = hand_clearance(transforms, self_side=side, all_pairs=True)
        gap = next(g for g, a, b in pairs if {a, b} == {"index_DIP", "thumb_DIP"})
        expected = np.linalg.norm([0.1007, dy, 0.0032]) - 0.022
        np.testing.assert_allclose(gap, expected, atol=1e-10)
