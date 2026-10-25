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


def test_dense_straight_rope_does_not_repel_its_own_local_neighbors():
    # Several edges within one diameter belong to a single continuous tube.
    straight = np.c_[-0.003 + 0.006 * np.linspace(0, 1, 13) ** 1.5, np.zeros((13, 2))]
    points = np.vstack([straight, [[0.03, 0.03, 0], [-0.03, 0.03, 0]]]).astype(
        np.float32
    )
    for shift in (0, 7):
        p = np.roll(points, shift, axis=0)
        count = len(p)
        positions = wp.array(p, dtype=wp.vec3, device="cpu")
        grid = wp.HashGrid(32, 32, 32, device="cpu")
        grid.build(positions, 0.002)
        deltas = wp.zeros(count, dtype=wp.vec3, device="cpu")
        wp.launch(
            segment_contacts,
            dim=count,
            inputs=[
                grid.id,
                positions,
                wp.zeros(count, dtype=wp.vec3, device="cpu"),
                wp.ones(count, device="cpu"),
                wp.full(count, 0.001, device="cpu"),
                wp.ones(count, dtype=wp.int32, device="cpu"),
                0.25,
                0.0,
                0.001,
                0.001,
                1.0,
                deltas,
            ],
            device="cpu",
        )
        correction = np.roll(deltas.numpy(), -shift, axis=0)
        np.testing.assert_allclose(correction[4:9], 0, atol=1e-10)
