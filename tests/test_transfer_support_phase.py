import pytest

from revo3_cat_cradle.transfers import retained_support_centers, captured_far_centers


def tracking(names):
    return {"samples": [{"time": 3., "supports": {
        name: {"center": i + 10} for i, name in enumerate(names)
    }}]}


def test_default_phase_still_requires_all_four_original_supports():
    report = tracking(["left_index", "right_index", "left_thumb", "right_thumb"])
    assert len(retained_support_centers(report, 3.)) == 4
    del report["samples"][0]["supports"]["right_thumb"]
    with pytest.raises(ValueError, match="required original"):
        retained_support_centers(report, 3.)


def test_one_thumb_transfer_keeps_the_other_thumb_and_both_indices():
    report = tracking(["left_index", "right_index", "left_thumb"])
    assert set(retained_support_centers(report, 3., ["right"])) == {
        "left_index", "right_index", "left_thumb"}
    del report["samples"][0]["supports"]["left_thumb"]
    with pytest.raises(ValueError, match="required original"):
        retained_support_centers(report, 3., ["right"])


def test_releasing_both_thumbs_never_excuses_a_missing_index():
    report = tracking(["left_index", "right_index"])
    assert set(retained_support_centers(report, 3., ["left", "right"])) == {
        "left_index", "right_index"}
    del report["samples"][0]["supports"]["right_index"]
    with pytest.raises(ValueError, match="required original"):
        retained_support_centers(report, 3., ["left", "right"])


def test_phase_source_must_match_one_checkpoint_row():
    report = tracking(["left_index", "right_index"])
    for time in (2., float("nan")):
        with pytest.raises(ValueError):
            retained_support_centers(report, time, ["left", "right"])
    report["samples"] *= 2
    with pytest.raises(ValueError):
        retained_support_centers(report, 3., ["left", "right"])
    with pytest.raises(ValueError):
        retained_support_centers(report, 3., ["unknown"])


def test_exchange_audit_keeps_the_nonmoving_thumb_and_every_frame():
    from copy import deepcopy
    from scripts.audit_thumb_exchange import retained_phase_tracks

    required = {"left_index", "right_index", "left_thumb"}
    report = tracking(required)
    report["samples"].append(deepcopy(report["samples"][0]))
    report["samples"][1]["time"] = 4.
    assert retained_phase_tracks(report, [3., 4.], 3., required)
    # A departing right thumb may disappear, but the left support may not.
    report["errors"] = {"right_thumb": {"time_s": 3.5}}
    assert retained_phase_tracks(report, [3., 4.], 3., required)
    del report["samples"][1]["supports"]["left_thumb"]
    assert not retained_phase_tracks(report, [3., 4.], 3., required)
    report["samples"].pop()
    assert not retained_phase_tracks(report, [3., 4.], 3., required)


def test_captured_far_guard_requires_both_held_material_tracks():
    from copy import deepcopy

    plan = {"left": {"far_indices": [7, 8, 9]}, "right": {"far_indices": [1, 2, 3]}}
    report = dict(interval_s=[3., 4.], checks=dict(
        stationary_hold=True, original_indices_tracked=True,
        left_new_far_contact_held=True, left_same_far_material=True,
        right_new_far_contact_held=True, right_same_far_material=True),
        summary={"left": {"persistent_far_particles": [7, 8]},
                 "right": {"persistent_far_particles": [1, 2, 3]}})
    assert captured_far_centers(report, 4., plan, 10) == {"left": 7, "right": 2}
    with pytest.raises(ValueError, match="matching held"):
        captured_far_centers(report, 5., plan, 10)
    for particles in ([], [4, 5], [-1], [10]):
        bad = deepcopy(report)
        bad["summary"]["right"]["persistent_far_particles"] = particles
        with pytest.raises(ValueError, match="Both captured"):
            captured_far_centers(bad, 4., plan, 10)
    report["checks"]["left_new_far_contact_held"] = False
    with pytest.raises(ValueError, match="Both captured"):
        captured_far_centers(report, 4., plan, 10)


def test_existing_far_track_is_kept_when_a_second_contact_patch_appears():
    plan = {"left": {"far_indices": [7, 8, 12, 13]}, "right": {"far_indices": [1, 2]}}
    report = dict(interval_s=[3., 4.], checks=dict(
        stationary_hold=True, original_indices_tracked=True,
        left_new_far_contact_held=True, left_same_far_material=True,
        right_new_far_contact_held=True, right_same_far_material=True),
        summary={"left": {"persistent_far_particles": [7, 8, 12, 13]},
                 "right": {"persistent_far_particles": [1, 2]}})
    with pytest.raises(ValueError, match="Ambiguous"):
        captured_far_centers(report, 4., plan, 20)
    assert captured_far_centers(report, 4., plan, 20,
                                {"left": 12, "right": 2}) == {"left": 12, "right": 1}
    with pytest.raises(ValueError, match="outside"):
        captured_far_centers(report, 4., plan, 20, {"left": 19, "right": 2})


def handoff_fixture():
    plan = {"left": {"far_indices": list(range(70, 85))},
            "right": {"far_indices": list(range(10, 55))}}
    report = dict(interval_s=[3., 4.], checks=dict(
        stationary_hold=True, original_indices_tracked=True,
        left_new_far_contact_held=True, left_same_far_material=True,
        right_new_far_contact_held=True, right_same_far_material=True),
        summary={"left": {"persistent_far_particles": [75, 76, 77]},
                 "right": {"persistent_far_particles": [11, 12, 13, 41, 42, 43]}})
    return report, plan, {"left": 76, "right": 12}


def test_explicit_handoff_retains_a_patch_already_held_before_the_other_leaves():
    from revo3_cat_cradle.transfers import handoff_far_centers, track_support_patch

    report, plan, previous = handoff_fixture()
    centers, declaration = handoff_far_centers(
        report, 4., plan, 100, previous, {"right": 42})
    assert centers == {"left": 76, "right": 42}
    assert declaration["right"]["departing_held_patch"] == [11, 12, 13]
    assert declaration["right"]["receiving_held_patch"] == [41, 42, 43]
    assert previous["right"] == 12
    # Without the advance declaration the ordinary tracker still rejects this
    # disappearance. The receiver was in contact throughout the earlier hold.
    with pytest.raises(ValueError, match="continuity"):
        track_support_patch([41, 42, 43], 100, previous["right"])
    assert track_support_patch([41, 42, 43], 100, centers["right"])[0] == 42


@pytest.mark.parametrize("handoff", [
    {}, {"other": 42}, {"right": 42.5}, {"right": True},
    {"right": 12}, {"right": 13}, {"right": 40}, {"right": 75},
])
def test_handoff_rejects_unheld_wrong_strand_and_same_patch_requests(handoff):
    from revo3_cat_cradle.transfers import handoff_far_centers

    report, plan, previous = handoff_fixture()
    with pytest.raises(ValueError):
        handoff_far_centers(report, 4., plan, 100, previous, handoff)


def test_handoff_does_not_recover_an_already_lost_or_unheld_source():
    from revo3_cat_cradle.transfers import handoff_far_centers

    report, plan, previous = handoff_fixture()
    with pytest.raises(ValueError, match="matching held"):
        handoff_far_centers(report, 4.1, plan, 100, previous, {"right": 42})
    report["summary"]["right"]["persistent_far_particles"] = [11, 41, 42, 43]
    with pytest.raises(ValueError, match="simultaneously"):
        handoff_far_centers(report, 4., plan, 100, previous, {"right": 42})
    report["summary"]["right"]["persistent_far_particles"] = [41, 42, 43]
    with pytest.raises(ValueError, match="continuity"):
        handoff_far_centers(report, 4., plan, 100, previous, {"right": 42})
