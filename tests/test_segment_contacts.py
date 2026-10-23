"""Regression: two rope edges can collide while all endpoint spheres miss."""

import numpy as np
import warp as wp
from revo3_cat_cradle.segment_contacts import segment_contacts


def test_segment_contact_separates_crossing_and_preserves_center_of_mass():
    points = np.array(
        [
            [-0.003, 0, 0],
            [0.003, 0, 0],
            [0.006, 0.006, 0.006],
            [0, -0.003, 0.0005],
            [0, 0.003, 0.0005],
            [-0.006, 0.006, 0.006],
        ],
        dtype=np.float32,
    )
    # The crossing segments' endpoints are more than two radii apart.
    assert np.linalg.norm(points[0] - points[3]) > 0.002
    positions = wp.array(points, dtype=wp.vec3, device="cpu")
    velocity = wp.zeros(6, dtype=wp.vec3, device="cpu")
    invmass = wp.ones(6, device="cpu")
    radius = wp.full(6, 0.001, device="cpu")
    flags = wp.ones(6, dtype=wp.int32, device="cpu")
    deltas = wp.zeros(6, dtype=wp.vec3, device="cpu")
    grid = wp.HashGrid(32, 32, 32, device="cpu")
    grid.build(positions, 0.002)
    wp.launch(
        segment_contacts,
        dim=6,
        inputs=[
            grid.id,
            positions,
            velocity,
            invmass,
            radius,
            flags,
            0.25,
            0.0,
            0.001,
            0.001,
            1.0,
            deltas,
        ],
        device="cpu",
    )
    correction = deltas.numpy()
    np.testing.assert_allclose(correction.sum(axis=0), 0, atol=1e-9)
    corrected = points + correction
    first_midpoint = corrected[:2].mean(axis=0)
    second_midpoint = corrected[3:5].mean(axis=0)
    np.testing.assert_allclose(
        np.linalg.norm(first_midpoint - second_midpoint), 0.002, atol=1e-8
    )


def test_long_segment_start_outside_old_search_radius_is_not_missed():
    points = np.array(
        [
            [-0.003, 0, 0],
            [0.003, 0, 0],
            [0.04, 0.04, 0.04],
            [0, -0.014, 0.0005],
            [0, 0.001, 0.0005],
            [-0.04, 0.04, 0.04],
        ],
        dtype=np.float32,
    )
    # The 15 mm target edge starts outside the old 13 mm query radius.
    assert np.linalg.norm(points[3]) > 0.013
    positions = wp.array(points, dtype=wp.vec3, device="cpu")
    grid = wp.HashGrid(32, 32, 32, device="cpu")
    grid.build(positions, 0.01)
    deltas = wp.zeros(6, dtype=wp.vec3, device="cpu")
    wp.launch(
        segment_contacts,
        dim=6,
        inputs=[
            grid.id,
            positions,
            wp.zeros(6, dtype=wp.vec3, device="cpu"),
            wp.ones(6, device="cpu"),
            wp.full(6, 0.001, device="cpu"),
            wp.ones(6, dtype=wp.int32, device="cpu"),
            0.25,
            0.0,
            0.005,
            0.001,
            1.0,
            deltas,
        ],
        device="cpu",
    )
    correction = deltas.numpy()
    assert correction[3, 2] > 0.00001
    np.testing.assert_allclose(correction.sum(axis=0), 0, atol=1e-8)
