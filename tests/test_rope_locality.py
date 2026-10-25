"""A dense continuous tube is not a collision with another part of the rope."""

import numpy as np
import pytest
from revo3_cat_cradle.bimanual_checks import segment_separation


def test_dense_smooth_loop_excludes_only_the_local_tube_neighborhood():
    angles = np.linspace(0, 2 * np.pi, 128, endpoint=False)
    points = np.c_[0.01 * np.cos(angles), 0.01 * np.sin(angles), np.zeros(128)]
    # Three-index separation is still less than one rope diameter here.
    assert segment_separation(points) < 0.0012
    assert segment_separation(points, minimum_arc_distance=0.002) > 0.0018
    assert segment_separation(
        np.roll(points, 17, axis=0), minimum_arc_distance=0.002
    ) == pytest.approx(segment_separation(points, minimum_arc_distance=0.002))


def test_locality_filter_keeps_a_true_nonlocal_crossing():
    points = np.array(
        [
            [-0.003, 0, 0],
            [0.003, 0, 0],
            [0.006, 0.006, 0.006],
            [0, -0.003, 0.0005],
            [0, 0.003, 0.0005],
            [-0.006, 0.006, 0.006],
        ]
    )
    for shift in range(len(points)):
        assert segment_separation(
            np.roll(points, shift, axis=0), minimum_arc_distance=0.002
        ) == pytest.approx(0.0005)
