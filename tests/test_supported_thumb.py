from copy import deepcopy

import pytest

from scripts.plan_supported_thumb import validate_observation


def observation():
    return {
        "tracking_complete": True,
        "errors": {},
        "final": {"time": 48.566667, "supports": {
            f"{side}_{finger}": {"center": 5, "patch": [4, 5, 6]}
            for side in ("left", "right") for finger in ("thumb", "index")
        }},
    }


def test_observation_accepts_export_rounding_without_mutating_evidence():
    track = observation()
    before = deepcopy(track)
    validate_observation(1457 / 30, 1457 / 30, track, [{"max_stretch_ratio": 1.17}])
    assert track == before


def test_observation_rejects_old_loop_loss_even_when_other_supports_survive():
    track = observation()
    track["tracking_complete"] = False
    track["errors"]["left_thumb"] = {"time_s": 48.4, "reason": "No contact"}
    with pytest.raises(ValueError, match="support tracking failed"):
        validate_observation(1457 / 30, 1457 / 30, track, [{"max_stretch_ratio": 1.17}])


def test_observation_rejects_stale_report_and_an_interrupted_motion():
    with pytest.raises(ValueError, match="saved checkpoint"):
        validate_observation(48.6, 48.6, observation(), [{"max_stretch_ratio": 1.17}])
    with pytest.raises(ValueError, match="did not finish"):
        validate_observation(1457 / 30, 49, observation(), [{"max_stretch_ratio": 1.17}])


def test_observation_checks_full_strain_history_not_only_final_frame():
    with pytest.raises(ValueError, match="strain gate"):
        validate_observation(1457 / 30, 1457 / 30, observation(), [
            {"max_stretch_ratio": 1.208}, {"max_stretch_ratio": 1.01},
        ])


def test_near_failure_at_phase_endpoint_is_not_a_completed_source():
    # Exported duration can equal the saved time even when a guard rejected
    # that exact phase endpoint. Equal timestamps alone cannot certify it.
    with pytest.raises(ValueError, match="failed continuation"):
        validate_observation(1457 / 30, 1457 / 30, observation(),
                             [{"max_stretch_ratio": 1.17}], stop_reports=["near_stop.json"])
    with pytest.raises(ValueError, match="forbidden near"):
        validate_observation(1457 / 30, 1457 / 30, observation(),
                             [{"max_stretch_ratio": 1.17}], near_gaps=[.02, .0014])
