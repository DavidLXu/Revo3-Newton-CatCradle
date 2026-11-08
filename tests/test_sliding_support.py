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


def test_merging_patch_does_not_lose_a_still_contacting_material_sample():
    # Recorded right-index contact merger: the midpoint moves 599 -> 589,
    # while material 599 remains in contact. The eight-particle gate is unchanged.
    contacts = list(range(388, 403)) + list(range(568, 612))
    center, patch, extras = track_support_patch(contacts, 689, 599)
    assert center == 599
    np.testing.assert_array_equal(patch, np.arange(568, 612))
    np.testing.assert_array_equal(extras[0], np.arange(388, 403))
    with pytest.raises(ValueError, match="continuity"):
        track_support_patch(range(568, 585), 689, center)


def test_retained_material_beats_a_closer_unrelated_patch_midpoint():
    center, patch, extras = track_support_patch(list(range(10, 31)) + [32], 100, 30, 2)
    assert center == 30
    np.testing.assert_array_equal(patch, np.arange(10, 31))
    np.testing.assert_array_equal(extras[0], [32])


def test_patch_growth_across_closed_seam_preserves_retained_material():
    center, patch, extras = track_support_patch(list(range(20, 40)) + [0, 1], 40, 0, 2)
    assert center == 0
    assert 0 in patch and extras == []
