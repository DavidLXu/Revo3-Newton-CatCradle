import numpy as np
import pytest

from revo3_cat_cradle.free_loop import ApexDetector, free_loop_material, common_wrist_translation


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
