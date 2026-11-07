"""Arm collisions must be physical and survive exact checkpoint continuation."""

import numpy as np
import pytest
import trimesh

from revo3_cat_cradle.bimanual import BimanualSimulation, RevoTron, ROBOT_URDF
from revo3_cat_cradle.robot_geometry import (
    edges_enter_hull, hull_planes, load_robot_hulls, ROBOT_BODY_LINKS,
)
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.single_player import Motion, initial_loop


def test_complete_edges_detect_solid_crossing_with_endpoints_outside():
    hull = trimesh.creation.box(extents=[2, 2, 2])
    planes = hull_planes(hull)
    assert edges_enter_hull([[-2, 0, 0], [2, 0, 0]], planes).all()
    assert not edges_enter_hull([[-2, 1, 0], [2, 1, 0]], planes).any()
    # Unlike a world AABB check, clipping each plane resolves diagonal misses.
    assert not edges_enter_hull([[0.5, 2.5, 0], [2.5, 0.5, 0]], planes).any()


def test_hull_clipping_rotation_and_inset():
    hull = trimesh.creation.box(extents=[2, 2, 2])
    hull.apply_transform(trimesh.transformations.rotation_matrix(0.7, [0, 0, 1]))
    points = np.array([[-2, 0, 0], [2, 0, 0]])
    assert edges_enter_hull(points, hull_planes(hull)).all()
    box = trimesh.creation.box(extents=[2, 2, 2])
    shallow = [[-2, 0.999, 0], [2, 0.999, 0]]
    assert edges_enter_hull(shallow, hull_planes(box)).all()
    assert not edges_enter_hull(shallow, hull_planes(box), inset=0.002).any()
    with pytest.raises(ValueError, match="Invalid hull"):
        edges_enter_hull(points, hull_planes(box), inset=-1)


@pytest.mark.skipif(not ROBOT_URDF.exists(), reason="Optional RevoTron assets missing")
def test_robot_hulls_follow_fk_and_checkpoint_rejects_missing_bodies(tmp_path):
    robot = RevoTron()
    motion = Motion(robot, upright=True)
    config = Config(substeps=1, iterations=1)
    points = initial_loop(robot, motion)
    legacy = BimanualSimulation(robot, motion, points, config)
    legacy.save_checkpoint(tmp_path / "legacy.npz")
    sim = BimanualSimulation(robot, motion, points, config, robot_body_geometry="hulls")
    assert set(ROBOT_BODY_LINKS).issubset(sim.proxy_links)
    assert len(sim.proxy_links) == len(legacy.proxy_links) + len(ROBOT_BODY_LINKS)
    assert np.all(sim.model.particle_mass.numpy() > 0)
    assert len(sim.anchor_indices) == 0
    with pytest.raises(ValueError, match="collision bodies"):
        sim.load_checkpoint(tmp_path / "legacy.npz")
    # Probe a real forearm face with one particle sphere. This verifies that
    # the added shapes participate in soft contacts, not just visual/FK data.
    hull = load_robot_hulls(robot)["Link6_L"]
    face = int(hull.area_faces.argmax())
    local = hull.triangles_center[face] + 0.0005 * hull.face_normals[face]
    tf = robot.fk(motion.targets[0])["Link6_L"]
    positions = sim.state.particle_q.numpy().copy()
    probe = positions.copy()
    probe[0] = tf[:3, :3] @ local + tf[:3, 3]
    sim.state.particle_q.assign(probe)
    sim.pipeline.collide(sim.state, sim.contacts)
    count = int(sim.contacts.soft_contact_count.numpy()[0])
    particles = sim.contacts.soft_contact_particle.numpy()[:count]
    shapes = sim.contacts.soft_contact_shape.numpy()[:count]
    bodies = sim.model.shape_body.numpy()[shapes]
    assert np.any((particles == 0) & (bodies == sim.proxy_links.index("Link6_L")))
    sim.state.particle_q.assign(positions)
    q = motion.targets[0].copy()
    q[robot.indices["Joint4_L"]] += 0.03
    motion.append(1, q, "Move the left forearm")
    sim.advance(1)
    index = sim.proxy_links.index("Link6_L")
    expected = np.asarray(sim.as_warp(robot.fk(q)["Link6_L"]))
    np.testing.assert_allclose(sim.state.body_q.numpy()[index], expected, atol=1e-6)
    sim.save_checkpoint(tmp_path / "hulls.npz")
    before = sim.state.particle_q.numpy().copy()
    sim.load_checkpoint(tmp_path / "hulls.npz")
    np.testing.assert_array_equal(sim.state.particle_q.numpy(), before)
