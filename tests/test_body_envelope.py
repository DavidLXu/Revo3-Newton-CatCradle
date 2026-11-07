import numpy as np

from revo3_cat_cradle.body_envelope import edge_box_face_margins


def test_edge_crossing_body_box_is_not_missed_by_outside_endpoints():
    margins = edge_box_face_margins([[-2, 0, 0], [2, 0, 0]], [[-1]*3, [1]*3], 0.1)
    assert np.all(margins < 0)


def test_separating_face_includes_rope_radius():
    margins = edge_box_face_margins([[-2, 1.4, 0], [2, 1.4, 0]], [[-1]*3, [1]*3], 0.1)
    np.testing.assert_allclose(margins, 0.3)


def test_diagonal_miss_can_be_conservatively_rejected():
    # The line x+y=3 misses the unit box, but neither X nor Y separates
    # its whole span. The negative bound must not be called penetration.
    margins = edge_box_face_margins([[0.5, 2.5, 0], [2.5, 0.5, 0]], [[-1]*3, [1]*3], 0)
    assert np.all(margins < 0)
