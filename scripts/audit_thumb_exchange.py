"""Check necessary material/contact conditions after intentional thumb exchange.

Passing these conditions does not certify a threaded bight, Cup, or any other
figure. The original index tracks, new far strands and complete old-loop arcs
are inspected separately; captions and generic thumb contact are insufficient.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.mesh_geometry import (
    load_knuckle_meshes, support_contact_indices, support_gap,
)
from revo3_cat_cradle.transfers import thumb_arc_sweep
from scripts.audit_finger_contacts import audit_frame
from scripts.audit_loop_release import completed_stationary_hold


def retained_phase_tracks(report, times, start, names):
    """Require every recorded frame of each nondeparting original support."""
    expected = np.asarray(times)[np.asarray(times) >= start - 1e-5]
    rows = [row for row in report.get("samples", []) if row["time"] >= start - 1e-5]
    return bool(len(rows) == len(expected)
                and np.allclose([row["time"] for row in rows], expected, atol=1e-5, rtol=0)
                and not set(names).intersection(report.get("errors", {}))
                and all(set(names).issubset(row["supports"]) for row in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    args = parser.parse_args()
    source = args.source
    with np.load(source / "trajectory.npz") as archive:
        times, joints, rope = (archive[k].copy() for k in ("time", "joints", "rope"))
    if (not np.isfinite(args.start) or not times[0] <= args.start < times[-1]
            or np.min(abs(times - args.start)) > 1e-5):
        parser.error("Start must match a recorded phase boundary")
    payload = json.loads((source / "trajectory.json").read_text())
    plan = json.loads((source / "transfer_plan.json").read_text())
    physical = json.loads((source / "loop_release_audit.json").read_text())
    tracking = json.loads((source / "index_material_support_tracking.json").read_text())
    original = json.loads((source / "material_support_tracking.json").read_text())
    guard = json.loads((source / "support_guard.json").read_text())
    for report in (physical, tracking, original):
        if (abs(report["interval_s"][-1] - times[-1]) > 1e-5
                or report["interval_s"][0] > args.start + 1e-5):
            parser.error("Run the physical and index-material audits for this complete phase first")
    required_physics = {
        "full_history_strain", "full_history_sampled_self_contact", "contact_search_bound",
        "dynamic_rope", "joint_limits", "full_history_opposing_palm_clearance",
        "full_history_finger_clearance", "full_history_central_body_envelopes_clear",
        "full_history_robot_hull_centerlines_clear", "interval_little_own_palm_clearance",
        "left_little_clear", "right_little_clear", "completed_hold",
    }
    if not required_physics.issubset(physical["checks"]):
        parser.error("Physical audit is missing required collision, strain or release checks")
    active = ("left", "right") if args.side == "both" else (args.side,)
    required_supports = {"left_index", "right_index"} | {
        side + "_thumb" for side in ("left", "right") if side not in active}
    checks = {"physics_" + k: bool(physical["checks"][k]) for k in sorted(required_physics)}
    checks.update(
        original_indices_tracked=bool(tracking["tracking_complete"]
                                     and set(tracking["tracked_supports"])
                                     == {"left_index", "right_index"}),
        intentional_phase_declared=(set(guard["intentionally_departing_thumb_sides"])
                                    == set(active)
                                    and set(guard["tracked_original_supports"])
                                    == required_supports
                                    and abs(guard["source_time_s"] - args.start) < 1e-5),
        nondeparting_original_supports_tracked=retained_phase_tracks(
            original, times, args.start, required_supports),
        stationary_hold=completed_stationary_hold(times, joints, payload["motion"]["duration"]),
    )
    held = np.flatnonzero(times >= times[-1] - 1 - 1e-5)
    robot = RevoTron()
    meshes = (load_knuckle_meshes(robot)
              if payload["collision_geometry"].get("knuckles") == "mesh" else {})
    radius = float(payload["config"]["radius"])
    observations, summary = [], {}
    persistent = {side: set(plan[side]["far_indices"]) for side in active}
    for k in held:
        transforms = robot.fk(joints[k])
        row = dict(time_s=float(times[k]), sides={})
        for side in active:
            values = {}
            for strand in ("far_indices", "near_indices", "old_loop_indices"):
                ids = np.asarray(plan[side][strand])
                edges = ids[np.isin((ids + 1) % len(rope[k]), ids)]
                contacts = audit_frame(robot, joints[k], rope[k], radius, edge_indices=edges)
                edge_gap = min(v["edge_gap_m"] for v in contacts
                               if v["finger"].startswith(side + "_thumb_"))
                values[strand + "_gap_m"] = min(
                    edge_gap, support_gap(transforms, rope[k, ids], side, "thumb", meshes, radius))
            contacts = support_contact_indices(transforms, rope[k], side, "thumb", meshes)
            persistent[side].intersection_update(contacts.tolist())
            values["far_arc_projected_sweep_rad"] = thumb_arc_sweep(
                transforms, rope[k, plan[side]["far_indices"]], side)
            row["sides"][side] = values
        observations.append(row)
    for side in active:
        values = [row["sides"][side] for row in observations]
        summary[side] = dict(
            maximum_far_gap_m=max(v["far_indices_gap_m"] for v in values),
            minimum_near_gap_m=min(v["near_indices_gap_m"] for v in values),
            minimum_old_loop_gap_m=min(v["old_loop_indices_gap_m"] for v in values),
            persistent_far_particles=sorted(persistent[side]),
        )
        checks.update({
            side + "_new_far_contact_held": summary[side]["maximum_far_gap_m"] < .0015,
            side + "_same_far_material": bool(persistent[side]),
            side + "_near_strand_clear": summary[side]["minimum_near_gap_m"] > .002,
            side + "_old_loop_clear": summary[side]["minimum_old_loop_gap_m"] > .004,
        })
    report = dict(scope=__doc__, passed_diagnostics=all(checks.values()), checks=checks,
                  completed_figure=False, active_sides=list(active),
                  interval_s=[args.start, float(times[-1])], summary=summary, samples=observations)
    (source / "thumb_exchange_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
    return 0 if report["passed_diagnostics"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
