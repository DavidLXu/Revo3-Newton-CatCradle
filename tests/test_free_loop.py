import numpy as np
import pytest

from revo3_cat_cradle.free_loop import (
    ApexDetector, BottomCrossingDetector, free_loop_material,
    common_wrist_translation, validate_bilateral_release,
    validate_bilateral_near_hold, observed_phase_time,
)


def test_clamped_swing_requires_current_bilateral_held_edge_evidence():
    report = dict(passed_diagnostics=True, sampling="edges", interval_s=[11, 12],
                  maximum_surface_gap_m=.0001, persistent_material=dict(left=[3], right=[17]),
                  checks=dict(completed_stationary_hold=True, left_same_material_pinch=True,
                              right_same_material_pinch=True))
    validate_bilateral_near_hold(report, 12)
    with pytest.raises(ValueError, match="checkpoint"):
        validate_bilateral_near_hold(report, 12.1)
    report["persistent_material"]["right"] = []
    with pytest.raises(ValueError, match="bilateral"):
        validate_bilateral_near_hold(report, 12)
    report["persistent_material"]["right"] = [17]
    report["maximum_surface_gap_m"] = .001
    with pytest.raises(ValueError, match="bilateral"):
        validate_bilateral_near_hold(report, 12)


@pytest.mark.parametrize("released", [[], ["left"], ["right"]])
def test_a_passing_retention_or_one_sided_release_cannot_start_a_free_loop_swing(released):
    report = dict(passed_diagnostics=True, released_sides=released, interval_s=[6, 12],
                  checks={side + "_little_clear": side in released for side in ("left", "right")})
    with pytest.raises(ValueError, match="bilateral"):
        validate_bilateral_release(report, 12)


def test_bilateral_release_must_describe_the_current_checkpoint_and_clear_both_littles():
    report = dict(passed_diagnostics=True, released_sides=["left", "right"],
                  interval_s=[6, 12], checks=dict(left_little_clear=True, right_little_clear=True))
    validate_bilateral_release(report, 12)
    with pytest.raises(ValueError, match="checkpoint"):
        validate_bilateral_release(report, 12.1)
    report["checks"]["right_little_clear"] = False
    with pytest.raises(ValueError, match="bilateral"):
        validate_bilateral_release(report, 12)


@pytest.mark.parametrize("direction", [-1, 1])
def test_apex_requires_outward_motion_on_the_intended_side(direction):
    detector = ApexDetector(direction)
    cue = lambda x, v: detector.update(direction * x, direction * v)
    assert not cue(-0.12, 0.2)
    assert not cue(-0.12, -0.1)
    assert not cue(0.1, -0.1)  # No preceding outward motion.
    assert not cue(0.1, 0.15)
    assert not cue(0.15, 0.01)
    assert cue(0.15, -0.01)
    assert not cue(0.01, 0.0)  # Returning below the displacement gate disarms it.
    assert not cue(0.1, -0.01)


def test_apex_rejects_invalid_or_nonfinite_observations():
    with pytest.raises(ValueError):
        ApexDetector(0)
    with pytest.raises(ValueError):
        ApexDetector(1, minimum_speed=np.nan)
    with pytest.raises(ValueError):
        ApexDetector(1).update(np.nan, 0)


@pytest.mark.parametrize("direction", [-1, 1])
def test_center_crossing_requires_a_directed_approach_and_cannot_repeat(direction):
    detector = BottomCrossingDetector(direction)
    cue = lambda x, v: detector.update(direction * x, direction * v)
    assert not cue(.1, .2)  # Already past the center: no observed approach.
    assert not cue(-.01, .2)  # Small noise cannot arm the trigger.
    assert not cue(.01, .2)
    assert not cue(-.1, .3)
    assert not cue(-.01, .2)
    assert cue(.01, .2)
    assert not cue(.02, .2)
    assert not cue(-.1, -.3)
    assert not cue(.01, -.2)  # Wrong measured velocity rejects a jump.
    assert not cue(.02, .2)  # Cannot turn that rejected jump into a late event.


def test_center_crossing_rejects_invalid_detector_or_observation():
    with pytest.raises(ValueError):
        BottomCrossingDetector(0)
    with pytest.raises(ValueError):
        BottomCrossingDetector(1, minimum_offset=np.nan)
    with pytest.raises(ValueError):
        BottomCrossingDetector(1).update(0, np.nan)


def phase_sample(time, x, velocity):
    return dict(time_s=time, relative_position_m=[x, 0, -.35],
                timing_velocity_m_s=[velocity, 0, 0])


def test_approach_during_stroke_arms_the_next_crossing():
    # Rounded measurements from the second 100 mm stroke: at the final
    # waypoint the approach is already inside the 10 mm arming threshold.
    samples = [phase_sample(43.133333, .0262, -.456),
               phase_sample(43.166667, .0093, -.415),
               phase_sample(43.2, -.013, -.588)]
    assert observed_phase_time(BottomCrossingDetector(-1, minimum_offset=.01),
                               samples[1:]) is None
    assert observed_phase_time(BottomCrossingDetector(-1, minimum_offset=.01),
                               samples, not_before=43.166667) == 43.2


def test_crossing_inside_motion_is_not_replayed_after_motion():
    samples = [phase_sample(0, -.1, .4), phase_sample(.1, .01, .4),
               phase_sample(.2, .04, .3), phase_sample(.3, .08, .1)]
    detector = BottomCrossingDetector(1)
    assert observed_phase_time(detector, samples, not_before=.2) is None
    assert observed_phase_time(detector, [phase_sample(.4, .09, .1)]) is None


def test_observed_phase_state_survives_a_separate_probe():
    detector = BottomCrossingDetector(1)
    assert observed_phase_time(detector, [phase_sample(0, -.1, .3),
                                         phase_sample(.1, -.01, .3)],
                               not_before=.1) is None
    assert observed_phase_time(detector, [phase_sample(.2, .01, .3)]) == .2


def test_free_material_wraps_without_using_a_thumb_arc():
    plan = {"left": {"far_indices": [8], "old_thumb_indices": [12]},
            "right": {"far_indices": [24], "old_thumb_indices": [20]}}
    band, thumbs = free_loop_material(plan, 32)
    np.testing.assert_array_equal(band, np.r_[np.arange(24, 32), np.arange(9)])
    np.testing.assert_array_equal(thumbs, [12, 20])
    plan["left"]["old_thumb_indices"] = [0]
    with pytest.raises(ValueError, match="original free arc"):
        free_loop_material(plan, 32)


def test_cartesian_segments_prevent_observed_two_hand_spacing_drift():
    import json
    from pathlib import Path
    from revo3_cat_cradle.bimanual import RevoTron, ROBOT_URDF

    if not ROBOT_URDF.exists():
        pytest.skip("Optional RevoTron assets missing")
    path = Path(__file__).resolve().parents[1] / "examples/transfers/attempt_raised_thumb_spread.json"
    q = np.array(json.loads(path.read_text())[-1]["joints"])
    robot = RevoTron()
    centers = {s: robot.wrist(q, s)[:3, 3].copy() for s in ("left", "right")}
    forward = {s: p + [0.08, 0, 0] for s, p in centers.items()}
    backward = {s: p - [0.08, 0, 0] for s, p in centers.items()}
    # Reproduce the problematic direct endpoint interpolation: both endpoints
    # are feasible, but the interpolated relative wrist pose is not constant.
    for side in centers:
        q = robot.ik(q, side, forward[side], robot.wrist(q, side)[:3, :3])
    with pytest.raises(ValueError, match="relative wrist tolerance"):
        common_wrist_translation(robot, q, backward, 0.6, 1)
    route, report = common_wrist_translation(robot, q, backward, 0.6, 9)
    assert len(route) == 9
    assert report["maximum_relative_translation_drift_m"] < 0.001
    assert report["maximum_relative_rotation_drift_deg"] < 0.2
    assert report["peak_joint_speed_rad_s"] < 3
