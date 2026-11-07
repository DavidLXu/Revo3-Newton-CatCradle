"""A rope edge can hit a finger even when both particle spheres miss it."""

import numpy as np
import warp as wp
import newton

from revo3_cat_cradle.segment_contacts import (
    SegmentRopeSolverXPBD,
    segment_capsule_contacts,
)


def contact_delta(
    *,
    height=0.0105,
    friction=0.0,
    body_velocity=0.0,
    body_angular_velocity=0.0,
    rotation=None,
):
    builder = newton.ModelBuilder(gravity=(0, 0, 0))
    body = builder.add_body(is_kinematic=True)
    cfg = builder.default_shape_cfg.copy()
    cfg.mu = friction
    builder.add_shape_capsule(body, radius=0.01, half_height=0.02, cfg=cfg)
    points = np.array(
        [[-0.009, height, 0], [0.009, height, 0], [0.03, 0.04, 0]], np.float32
    )
    for point in points:
        builder.add_particle(point, (0, 0, 0), mass=1.0, radius=0.001)
    model = builder.finalize(device="cpu")
    state = model.state()
    delta = wp.zeros(3, dtype=wp.vec3, device="cpu")
    if rotation is not None:
        q = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), rotation)
        state.body_q.assign(wp.array([wp.transform(wp.vec3(), q)], dtype=wp.transform))
        state.particle_q.assign(
            wp.array(
                [wp.quat_rotate(q, wp.vec3(*p)) for p in points],
                dtype=wp.vec3,
            )
        )
    state.body_qd.assign(
        wp.array(
            [
                wp.spatial_vector(
                    body_velocity, 0.0, 0.0, 0.0, 0.0, body_angular_velocity
                )
            ],
            dtype=wp.spatial_vector,
        )
    )
    wp.launch(
        segment_capsule_contacts,
        dim=(1, 1),
        inputs=[
            state.particle_q,
            state.particle_qd,
            model.particle_inv_mass,
            model.particle_radius,
            wp.array([0], dtype=int),
            model.shape_body,
            model.shape_transform,
            model.shape_scale,
            model.shape_material_mu,
            state.body_q,
            state.body_qd,
            model.body_com,
            friction,
            0.01,
            1.0,
        ],
        outputs=[delta],
        device="cpu",
    )
    return points, delta.numpy(), model


def test_edge_contact_removes_interior_penetration_when_nodes_miss():
    points, delta, model = contact_delta()
    assert np.linalg.norm(points[:2, :2], axis=1).min() > 0.011
    corrected_midpoint = (points + delta)[:2].mean(axis=0)
    np.testing.assert_allclose(corrected_midpoint[1], 0.011, atol=1e-8)
    np.testing.assert_allclose(delta[:2, 1], 0.0005, atol=1e-8)
    solver = SegmentRopeSolverXPBD(model, segment_hand_contacts=True)
    assert solver.segment_hand_contacts
    assert len(solver.capsule_shapes) == 1


def test_edge_contact_is_unilateral_and_rotates_with_finger():
    _, delta, _ = contact_delta(height=0.02)
    np.testing.assert_array_equal(delta, 0)
    _, delta, _ = contact_delta(rotation=np.pi / 2)
    np.testing.assert_allclose(delta[:2, 0], -0.0005, atol=1e-8)
    np.testing.assert_allclose(delta[:2, 1:], 0, atol=1e-8)


def test_edge_friction_uses_body_velocity_and_coulomb_bound():
    _, delta, _ = contact_delta(friction=0.5, body_velocity=0.1)
    # Contact follows the moving finger, limited by mu times normal correction.
    np.testing.assert_allclose(delta[:2, 0], 0.00025, atol=1e-8)
    np.testing.assert_allclose(delta[:2, 1], 0.0005, atol=1e-8)
    _, delta, _ = contact_delta(friction=0.0, body_velocity=0.1)
    np.testing.assert_array_equal(delta[:, 0], 0)


def test_solver_applies_edge_contact_inside_xpbd_iterations():
    _, _, model = contact_delta()
    model.particle_mu = 0.0
    before = model.state()
    after = model.state()
    solver = SegmentRopeSolverXPBD(model, segment_hand_contacts=True, iterations=4)
    solver.step(before, after, model.control(), None, 0.001)
    assert after.particle_q.numpy()[:2, 1].mean() > 0.0108
    np.testing.assert_array_equal(after.body_q.numpy(), before.body_q.numpy())


def test_rotating_capsule_drags_edge_with_surface_velocity():
    _, delta, _ = contact_delta(friction=0.5, body_angular_velocity=1.0)
    # omega x r at the 10 mm radial surface is -10 mm/s in X.
    np.testing.assert_allclose(delta[:2, 0], -0.0001, atol=1e-8)
    np.testing.assert_allclose(delta[:2, 1], 0.0005, atol=1e-8)
