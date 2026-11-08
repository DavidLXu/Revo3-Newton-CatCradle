"""Temporary helpers do not excuse failed physics or complete final release."""

from copy import deepcopy

import pytest

from revo3_cat_cradle.free_loop import validate_bilateral_release
from revo3_cat_cradle.helper_phase import validate_temporary_helper_source


def reports():
    physical = dict(passed_diagnostics=False, interval_s=[10, 12],
                    released_sides=["left", "right"], checks={key: True for key in (
                        "thumb_and_index_contacts_retained", "left_little_clear",
                        "interval_little_own_palm_clearance", "full_history_strain",
                        "full_history_sampled_self_contact", "contact_search_bound",
                        "dynamic_rope", "joint_limits", "full_history_opposing_palm_clearance",
                        "full_history_finger_clearance", "full_history_central_body_envelopes_clear",
                        "full_history_robot_hull_centerlines_clear", "completed_hold")})
    physical["checks"]["right_little_clear"] = False
    helper = dict(helper_track_complete=True, failure=None, sampling_fps=30,
                  expected_interval_s=[11, 12], intent=dict(temporary_helper=dict(
                      side="right", finger="little", expected_contact_interval_s=[11, 12])),
                  samples=[dict(time_s=11 + i / 30, center=448, patch=[447, 448, 449])
                           for i in range(31)])
    return physical, helper


def test_declared_helper_is_an_intermediate_state_and_not_final_release():
    physical, helper = reports()
    assert validate_temporary_helper_source(physical, helper, 12) == "right_little"
    with pytest.raises(ValueError, match="bilateral little-loop release"):
        validate_bilateral_release(physical, 12)


@pytest.mark.parametrize("gate", [
    "full_history_strain", "full_history_sampled_self_contact", "dynamic_rope",
    "interval_little_own_palm_clearance", "full_history_robot_hull_centerlines_clear",
    "left_little_clear", "completed_hold", "thumb_and_index_contacts_retained",
])
def test_helper_cannot_excuse_failed_physics_or_the_other_hand(gate):
    physical, helper = reports()
    physical["checks"][gate] = False
    with pytest.raises(ValueError, match="physical or nonhelper"):
        validate_temporary_helper_source(physical, helper, 12)


def test_stale_missing_and_relabelled_tracks_fail():
    physical, original = reports()
    mutations = [
        lambda h: h.update(helper_track_complete=False),
        lambda h: h["samples"].pop(15),
        lambda h: h["samples"][-1].update(center=500),
        lambda h: h["intent"]["temporary_helper"].update(side="left"),
        lambda h: h.update(expected_interval_s=[11, 11.9]),
    ]
    for mutate in mutations:
        helper = deepcopy(original)
        mutate(helper)
        with pytest.raises(ValueError):
            validate_temporary_helper_source(physical, helper, 12)
    with pytest.raises(ValueError, match="stale"):
        validate_temporary_helper_source(physical, original, 12.1)
