"""Root collision diagnostics must distinguish surfaces from their convex hull."""

import numpy as np
import pytest
import trimesh

from revo3_cat_cradle.mesh_geometry import (
    signed_surface_clearance,
    support_gap,
    support_material_distances,
    support_contact_indices,
)
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, ROBOT_URDF
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config


def test_triangle_clearance_and_orientation():
    mesh = trimesh.creation.box(extents=[2, 2, 2])
    points = np.array([[0, 0, 0], [2, 0, 0], [1, 1, 2]])
    expected = [-1, 1, 1]
    np.testing.assert_allclose(signed_surface_clearance(mesh, points), expected)
    mesh.invert()
    np.testing.assert_allclose(signed_surface_clearance(mesh, points), expected)


@pytest.mark.skipif(not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing")
def test_mpr_bodies_follow_fk_and_reject_legacy_checkpoint(tmp_path):
    robot = RevoTron()
    motion = Motion(robot, upright=True)
    points = initial_loop(robot, motion)
    config = Config(substeps=1, iterations=1)
    legacy = BimanualSimulation(robot, motion, points, config, palm_geometry="hull")
    checkpoint = tmp_path / "legacy.npz"
    legacy.save_checkpoint(checkpoint)
    sim = BimanualSimulation(
        robot, motion, points, config, palm_geometry="hull", knuckle_geometry="mesh"
    )
    assert len(sim.proxy_links) == len(legacy.proxy_links) + 8
    assert np.all(sim.model.particle_mass.numpy() > 0)
    assert len(sim.anchor_indices) == 0
    with pytest.raises(ValueError, match="collision bodies"):
        sim.load_checkpoint(checkpoint)
    q = motion.targets[0].copy()
    q[robot.indices["right_index_MPR_joint"]] += 0.1
    motion.append(1, q, "Rotate knuckle")
    sim.advance(1)
    index = sim.proxy_links.index("right_index_MPR_link")
    expected = np.asarray(sim.as_warp(robot.fk(q)["right_index_MPR_link"]))
    np.testing.assert_allclose(sim.state.body_q.numpy()[index], expected, atol=1e-6)
    sim.save_checkpoint(tmp_path / "mesh.npz")
    positions = sim.state.particle_q.numpy().copy()
    sim.load_checkpoint(tmp_path / "mesh.npz")
    np.testing.assert_array_equal(sim.state.particle_q.numpy(), positions)


def test_root_support_includes_concave_mesh_surface_not_only_mcp_capsule():
    mesh = trimesh.creation.box(extents=[0.02, 0.02, 0.02])
    mesh.apply_translation([0.04, 0, 0.02])
    tf = {
        f"right_index_{segment}_link": np.eye(4)
        for segment in ["MPR", "MCP", "PIP", "DIP"]
    }
    meshes = {"right_index_MPR_link": [mesh]}
    points = np.array([[0.04, 0, 0.02], [0.2, 0.1, 0.2]])
    assert support_gap(tf, points, "right", "index", meshes) == pytest.approx(-0.011)
    outside = np.array([[0.055, 0, 0.02]])
    assert support_gap(tf, outside, "right", "index", meshes) == pytest.approx(0.004)
    batch = np.vstack([points, outside])
    np.testing.assert_allclose(
        support_material_distances(tf, batch, "right", "index", meshes),
        [support_gap(tf, point[None], "right", "index", meshes) for point in batch],
        atol=1e-12,
    )
    # Include an interior mesh point that capsule-only tracking would miss;
    # widening the query must also include the separate outside contact.
    np.testing.assert_array_equal(
        support_contact_indices(tf, batch, "right", "index", meshes), [0]
    )
    np.testing.assert_array_equal(
        support_contact_indices(tf, batch, "right", "index", meshes, threshold=0.006),
        [0, 2],
    )


@pytest.mark.skipif(not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing")
def test_explicit_figure_seed_preserves_sampling_across_checkpoint_rebuild(tmp_path):
    import json

    robot = RevoTron()
    motion = Motion(robot, upright=True)
    points = initial_loop(robot, motion)[::2]
    motion.metadata.update(
        initial_rope_points=points.tolist(), preserve_initial_sampling=True
    )
    config = Config(substeps=1, iterations=1)
    sim = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    assert sim.model.particle_count == len(points)
    assert np.all(sim.model.particle_mass.numpy() > 0)
    assert len(sim.anchor_indices) == 0
    sim.advance(1 / 30)
    sim.save_checkpoint(tmp_path / "seed.npz")
    rebuilt_motion = Motion(robot, upright=True)
    rebuilt_motion.metadata = json.loads(json.dumps(motion.metadata))
    rebuilt = BimanualSimulation(
        robot, rebuilt_motion, initial_loop(robot, rebuilt_motion), config
    )
    rebuilt.load_checkpoint(tmp_path / "seed.npz")
    for field in ("particle_q", "particle_qd"):
        np.testing.assert_array_equal(
            getattr(rebuilt.state, field).numpy(), getattr(sim.state, field).numpy()
        )
    np.testing.assert_array_equal(rebuilt.rest_lengths, sim.rest_lengths)
