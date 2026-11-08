"""Validate an explicitly declared intermediate helper-contact phase."""

import numpy as np


def validate_temporary_helper_source(physical, helper, checkpoint_time):
    """Keep physical gates while allowing the declared little finger to touch.

    Final little release remains a separate task condition. This validator is
    for intentional helper phases only, and is not used by free-loop motion.
    Original index and new far-material guards are required separately by the
    continuation command.
    """
    required = {
        "thumb_and_index_contacts_retained", "left_little_clear", "right_little_clear",
        "interval_little_own_palm_clearance", "full_history_strain",
        "full_history_sampled_self_contact", "contact_search_bound", "dynamic_rope",
        "joint_limits", "full_history_opposing_palm_clearance",
        "full_history_finger_clearance", "full_history_central_body_envelopes_clear",
        "full_history_robot_hull_centerlines_clear", "completed_hold",
    }
    declared = helper.get("intent", {}).get("temporary_helper", {})
    side, finger = declared.get("side"), declared.get("finger")
    # Only this experimentally checked helper role is supported here.
    if side not in ("left", "right") or finger != "little":
        raise ValueError("A temporary source requires a declared little-finger helper")
    checks = physical.get("checks", {})
    exempt = side + "_little_clear"
    if (not required.issubset(checks)
            or not all(value for key, value in checks.items() if key != exempt)):
        raise ValueError("Helper source fails physical or nonhelper support checks")
    interval = physical.get("interval_s", [])
    expected = helper.get("expected_interval_s", [])
    declared_interval = declared.get("expected_contact_interval_s", [])
    fps = helper.get("sampling_fps", 0)
    rows = helper.get("samples", [])
    if (len(interval) != 2 or len(expected) != 2 or len(declared_interval) != 2
            or not np.isfinite([*interval, *expected, *declared_interval,
                                checkpoint_time, fps]).all()
            or fps <= 0 or expected[0] >= expected[1]
            or interval[0] > expected[0] + 1e-5
            or abs(interval[-1] - checkpoint_time) > 1e-5
            or abs(expected[-1] - checkpoint_time) > 1e-5
            or not np.allclose(expected, declared_interval, atol=1e-5, rtol=0)
            or not helper.get("helper_track_complete") or helper.get("failure")
            or not rows):
        raise ValueError("Helper source has an incomplete or stale phase audit")
    times = np.array([row["time_s"] for row in rows])
    grid = np.arange(round(expected[0] * fps), round(expected[1] * fps) + 1) / fps
    if (len(times) != len(grid) or not np.isfinite(times).all()
            or not np.allclose(times, grid, atol=1e-5, rtol=0)
            or any(row["center"] not in row["patch"] for row in rows)):
        raise ValueError("Helper source lacks a continuous recorded material track")
    return side + "_" + finger
