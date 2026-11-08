"""Check necessary contact conditions for releasing Opening A little loops.

This diagnostic does not certify threading, material identity or a finished
figure. Capsule gaps include rope edges; knuckle meshes are sampled at particles.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import (
    segment_separation,
    hand_clearance,
    self_clearance_margin,
)
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_gap
from revo3_cat_cradle.palm_clearance import OpposingPalmClearance
from revo3_cat_cradle.body_envelope import CentralBodyEnvelope
from revo3_cat_cradle.robot_geometry import RobotHullAudit
from scripts.audit_finger_contacts import audit_frame


def completed_stationary_hold(times, joints, duration):
    """Check one recorded second of stationary joints, independently of captions."""
    times, joints = np.asarray(times), np.asarray(joints)
    if (times.ndim != 1 or len(times) < 2 or joints.ndim != 2
            or joints.shape[0] != len(times) or not joints.shape[1]
            or not np.isfinite(times).all() or not np.isfinite(joints).all()
            or not np.isfinite(duration) or np.any(np.diff(times) <= 0)
            or abs(times[-1] - duration) > 1e-6
            or times[-1] - times[0] < 1 - 1e-5):
        return False
    held = times >= times[-1] - 1 - 1e-5
    return bool(
        times[held][0] <= times[-1] - 1 + 1e-5
        and np.max(np.ptp(joints[held], axis=0)) < 1e-8
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--released", choices=("none", "left", "right", "both"), default="both",
                        help="Use none before release to require both little contacts")
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        data = {key: archive[key] for key in archive.files}
    payload = json.loads((args.source / "trajectory.json").read_text())
    times, rope, joints = (data[key] for key in ("time", "rope", "joints"))
    if not np.isfinite(args.start) or args.start < times[0]:
        parser.error("Start must be finite and inside the recorded interval")
    selected = np.flatnonzero(times >= args.start - 1e-5)
    if not len(selected) or times[-1] - args.start < 1 - 1e-5:
        parser.error("The requested interval must contain at least one second")
    if abs(times[selected[0]] - args.start) > 1e-5:
        parser.error("Start must match a recorded frame")
    if not all(np.isfinite(x).all() for x in (rope, joints, data["rest_lengths"])):
        parser.error("Nonfinite physical state")
    if np.any(data["rest_lengths"] <= 0):
        parser.error("Rest lengths must be positive")
    robot = RevoTron()
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    radius = float(payload["config"]["radius"])
    names = [(s, f) for s in ("left", "right") for f in ("thumb", "index", "little")]
    contacts = {s + "_" + f: [] for s, f in names}
    worst_capsule_gap = float("inf")
    for k in selected:
        poses = robot.fk(joints[k])
        rows = audit_frame(robot, joints[k], rope[k], radius)
        worst_capsule_gap = min(worst_capsule_gap, rows[0]["edge_gap_m"])
        for side, finger in names:
            name = side + "_" + finger
            edge_gap = min(
                row["edge_gap_m"]
                for row in rows
                if row["finger"].startswith(name + "_")
            )
            contacts[name].append(
                min(edge_gap, support_gap(poses, rope[k], side, finger, meshes, radius))
            )
    tail = times[selected] >= times[-1] - 1 - 1e-5
    supports = {
        name: dict(
            minimum_hold_gap_m=float(np.asarray(values)[tail].min()),
            maximum_hold_gap_m=float(np.asarray(values)[tail].max()),
            maximum_transfer_gap_m=float(max(values)),
        )
        for name, values in contacts.items()
    }
    released = (() if args.released == "none" else
                ("left", "right") if args.released == "both" else (args.released,))
    checks = {
        "thumb_and_index_contacts_retained": all(
            supports[s + "_" + f]["maximum_transfer_gap_m"] < 0.0015
            for s in ("left", "right")
            for f in ("thumb", "index")
        )
    }
    for side in ("left", "right"):
        value = supports[side + "_little"]
        if side in released:
            checks[side + "_little_clear"] = value["minimum_hold_gap_m"] > 0.002
        else:
            checks[side + "_little_retained"] = value["maximum_transfer_gap_m"] < 0.0015
    edges = np.linalg.norm(np.roll(rope, -1, axis=1) - rope, axis=2)
    stretch = float((edges / data["rest_lengths"]).max())
    separation = np.array(
        [segment_separation(points, minimum_arc_distance=2 * radius) for points in rope]
    )
    palms = OpposingPalmClearance(
        robot, robot.fk(joints[0]), geometry=payload["collision_geometry"]["palm"]
    )
    palm_cache = {}
    palm_results = []
    body_envelopes = CentralBodyEnvelope(robot)
    body_results = []
    robot_hulls = RobotHullAudit(robot)
    hull_intrusions = []
    for k, pose in enumerate(joints):
        body_results.append(body_envelopes.measure(robot.fk(pose), rope[k], radius))
        intrusions = robot_hulls.measure(robot.fk(pose), rope[k])
        if intrusions:
            hull_intrusions.append(dict(time_s=float(times[k]), links=intrusions))
        key = pose.tobytes()
        if key not in palm_cache:
            transforms = robot.fk(pose)
            palm_cache[key] = dict(
                **palms.measure(transforms, quick=True),
                opposing_finger_gap_m=hand_clearance(transforms)[0],
                same_hand_finger_margin_m=min(
                    self_clearance_margin(transforms, side)
                    for side in ("left", "right")
                ),
            )
        palm_results.append(palm_cache[key])
    worst_palm_frame = min(
        range(len(times)),
        key=lambda k: palm_results[k]["finger_palm_lower_bound_m"],
    )
    palm_bound = min(row["palm_palm_sphere_lower_bound_m"] for row in palm_results)
    finger_gap = min(row["opposing_finger_gap_m"] for row in palm_results)
    self_margin = min(row["same_hand_finger_margin_m"] for row in palm_results)
    worst_body_frame = min(
        range(len(times)), key=lambda k: body_results[k]["minimum_face_margin_m"]
    )
    checks.update(
        full_history_strain=stretch < 1.2,
        full_history_sampled_self_contact=bool(separation.min() >= 0.0018),
        contact_search_bound=bool(edges.max() < payload["self_contact_search_bound_m"]),
        dynamic_rope=bool(
            np.all(data["particle_masses"] > 0) and not len(data["anchors"])
        ),
        joint_limits=bool(
            np.all(joints >= robot.limits[:, 0] - 1e-7)
            and np.all(joints <= robot.limits[:, 1] + 1e-7)
        ),
        full_history_opposing_palm_clearance=bool(
            palm_results[worst_palm_frame]["finger_palm_lower_bound_m"] >= 0
            and palm_bound >= 0
        ),
        full_history_finger_clearance=bool(finger_gap >= -0.0005 and self_margin >= 0),
        full_history_central_body_envelopes_clear=bool(
            body_results[worst_body_frame]["minimum_face_margin_m"] >= 0
        ),
        full_history_robot_hull_centerlines_clear=not hull_intrusions,
        completed_hold=completed_stationary_hold(times, joints, payload["motion"]["duration"]),
    )
    report = dict(
        scope="Necessary release diagnostics only; not a threading or figure certificate. "
        "Excludes swept collisions, robot self-collisions, rope-radius clearance "
        "at robot-body meshes and material identity. Central-body boxes and "
        "complete-edge arm/body hull checks are conservative: overlap does not "
        "by itself prove penetration of the original mesh.",
        passed_diagnostics=all(checks.values()),
        checks=checks,
        full_history_robot_hull_intrusions=hull_intrusions,
        released_sides=list(released),
        interval_s=[float(times[selected[0]]), float(times[-1])],
        supports=supports,
        full_history_peak_stretch=stretch,
        full_history_minimum_rope_separation_m=float(separation.min()),
        minimum_separation_time_s=float(times[separation.argmin()]),
        interval_minimum_capsule_gap_m=worst_capsule_gap,
        full_history_palm_clearance=dict(
            time_s=float(times[worst_palm_frame]),
            **palm_results[worst_palm_frame],
            minimum_palm_palm_sphere_lower_bound_m=palm_bound,
            minimum_opposing_finger_gap_m=finger_gap,
            minimum_same_hand_finger_margin_m=self_margin,
        ),
        full_history_central_body_clearance=dict(
            time_s=float(times[worst_body_frame]), **body_results[worst_body_frame]
        ),
        initialization=payload["motion"].get("initialization"),
    )
    filename = "pre_release_audit.json" if args.released == "none" else "loop_release_audit.json"
    (args.source / filename).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report["passed_diagnostics"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
