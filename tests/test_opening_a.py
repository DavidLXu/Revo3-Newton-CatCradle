"""Geometry checks for the experimental single-player sequence."""

import numpy as np
from scipy.spatial.transform import Rotation
from revo3_cat_cradle.sim import Simulation
from revo3_cat_cradle.bimanual_checks import triangle_coordinates


def test_self_clearance_margin_only_exempts_adjacent_mcp_overlap():
    from revo3_cat_cradle.bimanual_checks import self_clearance_margin

    transforms = {}
    for i, finger in enumerate(("thumb", "index", "middle", "ring", "little")):
        for j, segment in enumerate(("MCP", "PIP", "DIP")):
            t = np.eye(4)
            t[:3, 3] = [10 * i, 0, 10 * j]
            transforms[f"right_{finger}_{segment}_link"] = t
    transforms["right_index_MCP_link"][:3, 3] = [0, 0, 50]
    transforms["right_middle_MCP_link"][:3, 3] = [0.022, 0, 50]
    assert self_clearance_margin(transforms, "right") > 0
    transforms["right_index_PIP_link"][:3, 3] = [0, 0, 60]
    transforms["right_middle_PIP_link"][:3, 3] = [0.019, 0, 60]
    assert self_clearance_margin(transforms, "right") < 0


def test_wider_initial_layout_adds_rope_without_scaling_fingers():
    from revo3_cat_cradle.bimanual import RevoTron
    from revo3_cat_cradle.single_player import Motion, initial_loop

    robot = RevoTron()
    narrow = Motion(robot, upright=True)
    wide = Motion(robot, upright=True, half_span=0.20)
    arms = robot.arm_indices["left"] + robot.arm_indices["right"]
    fingers = [i for i in range(len(robot.names)) if i not in arms]
    np.testing.assert_array_equal(narrow.targets[0][fingers], wide.targets[0][fingers])
    lengths = []
    for motion, span in [(narrow, 0.30), (wide, 0.40)]:
        q = motion.targets[0]
        distance = np.linalg.norm(
            robot.wrist(q, "left")[:3, 3] - robot.wrist(q, "right")[:3, 3]
        )
        assert abs(distance - span) < 0.0005
        points = initial_loop(robot, motion)
        lengths.append(
            np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1).sum()
        )
    assert 0.19 < lengths[1] - lengths[0] < 0.21


def test_extra_initial_material_preserves_geometry_and_checkpoint_rebuild(tmp_path):
    from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation
    from revo3_cat_cradle.single_player import Motion, initial_loop
    from revo3_cat_cradle.sim import Config

    robot = RevoTron()
    motion = Motion(robot, upright=True, half_span=0.175)
    config = Config(substeps=1, iterations=1)
    original = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    motion.metadata["initial_rope_extra_m"] = 0.03
    longer = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    np.testing.assert_array_equal(longer.initial_positions, original.initial_positions)
    np.testing.assert_array_equal(
        longer.model.particle_mass.numpy(), original.model.particle_mass.numpy()
    )
    assert (
        abs(
            float(
                longer.initial_rest_lengths.sum() - original.initial_rest_lengths.sum()
            )
            - 0.03
        )
        < 1e-6
    )
    assert np.all(longer.model.particle_mass.numpy() > 0)
    assert len(longer.anchor_indices) == 0
    longer.advance(1 / 30)
    longer.save_checkpoint(tmp_path / "longer.npz")
    rebuilt = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    rebuilt.load_checkpoint(tmp_path / "longer.npz")
    for name in ("particle_q", "particle_qd"):
        np.testing.assert_array_equal(
            getattr(rebuilt.state, name).numpy(), getattr(longer.state, name).numpy()
        )
    np.testing.assert_array_equal(rebuilt.rest_lengths, longer.rest_lengths)
    np.testing.assert_array_equal(
        rebuilt.initial_rest_lengths, longer.initial_rest_lengths
    )


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
