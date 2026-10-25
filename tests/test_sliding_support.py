"""A support may slide along material, but tracking must not jump between loops."""

import numpy as np
import pytest

from revo3_cat_cradle.transfers import track_support_patch, directed_support_arc


def test_sliding_support_follows_continuity_and_reports_extra_contact():
    center = 10
    for shift in range(1, 5):
        center, patch, extras = track_support_patch(
            list(range(9 + shift, 12 + shift)) + [25, 26], 40, center, max_shift=2
        )
        assert center == 10 + shift
        np.testing.assert_array_equal(patch, np.arange(9 + shift, 12 + shift))
        np.testing.assert_array_equal(extras[0], [25, 26])
    with pytest.raises(ValueError, match="continuity"):
        track_support_patch([25, 26], 40, center, max_shift=2)


def test_tracker_rejects_missing_or_equidistant_support_patches():
    with pytest.raises(ValueError, match="no contact"):
        track_support_patch([], 40, 10)
    with pytest.raises(ValueError, match="Ambiguous"):
        track_support_patch([8, 12], 40, 10)


def test_closed_seam_and_arc_direction_survive_sliding():
    center, patch, extras = track_support_patch([0, 1, 39], 40, 39)
    assert center == 0
    np.testing.assert_array_equal(patch, [39, 0, 1])
    assert extras == []
    np.testing.assert_array_equal(directed_support_arc(1, 38, 40, -1), [1, 0, 39, 38])
    np.testing.assert_array_equal(directed_support_arc(38, 1, 40, 1), [38, 39, 0, 1])
