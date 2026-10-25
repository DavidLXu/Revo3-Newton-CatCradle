"""Check the first Cup & Saucer transfer: new thumb bight plus retained old loop."""

import argparse, json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps, segment_separation
from revo3_cat_cradle.transfers import support_arc, original_thumb_loop
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_gap


def transfer_window(times, metadata):
    """Use the saved Opening A boundary, including independently initialized runs."""
    reference_time = float(metadata.get("transfer_reference_time_s", 21))
    original = int(np.argmin(abs(times - reference_time)))
    if abs(times[original] - reference_time) > 1e-5:
        raise ValueError("Opening A reference is not present in the physical history")
    transfer = np.flatnonzero(times > reference_time + 1e-5)
    if not len(transfer) or times[-1] - reference_time < 1:
        raise ValueError("At least one second of transfer history is required")
    last = np.flatnonzero(times >= max(reference_time, times[-1] - 1))
    return original, transfer, last


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--release-old", action="store_true")
    parser.add_argument(
        "--side",
        choices=["left", "right", "both"],
        default="both",
        help="Thumb pickup/release being assessed; inactive thumbs must retain their original loops",
    )
    args = parser.parse_args()
    out = args.output
    data = np.load(out / "trajectory.npz")
    plan = json.loads((out / "transfer_plan.json").read_text())
    active = set(plan) if args.side == "both" else {args.side}
    if not active.issubset(plan):
        raise ValueError("Requested side has no saved material reference")
    payload = json.loads((out / "trajectory.json").read_text())
    robot = RevoTron()
    times = data["time"]
    original, transfer, last = transfer_window(times, payload["motion"])
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    poses = {int(k): robot.fk(data["joints"][k]) for k in np.union1d(transfer, last)}
    stages = {}
    checks = {}
    for side, p in plan.items():
        result = {
            name: float(
                max(
                    gaps(poses[int(k)], data["rope"][k, p[name]], side, "thumb").min()
                    for k in last
                )
            )
            for name in ["far_indices", "old_thumb_indices"]
        }
        result["minimum_near_string_gap_m"] = float(
            min(
                gaps(
                    poses[int(k)], data["rope"][k, p["near_indices"]], side, "thumb"
                ).min()
                for k in last
            )
        )
        if side in active:
            checks[side + "_new_far_bight"] = result["far_indices"] < 0.0015
        old_loop = original_thumb_loop(
            robot, data["joints"][original], data["rope"][original], side, meshes=meshes
        )
        old_gaps = [
            gaps(poses[int(k)], data["rope"][k, old_loop], side, "thumb").min()
            for k in last
        ]
        result["minimum_old_loop_gap_m"] = float(min(old_gaps))
        result["maximum_old_loop_gap_m"] = float(max(old_gaps))
        if args.release_old and side in active:
            other = "right" if side == "left" else "left"
            bridge = support_arc(
                robot,
                data["joints"][original],
                data["rope"][original],
                (side, "thumb"),
                (other, "thumb"),
                meshes=meshes,
            )
            result["minimum_old_bridge_gap_m"] = float(
                min(
                    gaps(poses[int(k)], data["rope"][k, bridge], side, "thumb").min()
                    for k in last
                )
            )
            assist = [
                int(k)
                for k in transfer
                if "old" in payload["frames"][k]["phase"].lower()
            ]
            if not assist:
                raise ValueError("No old-loop assistance phase is recorded")
            result["minimum_helper_gap_m"] = float(
                min(
                    gaps(
                        poses[int(k)], data["rope"][k, old_loop], other, "middle"
                    ).min()
                    for k in assist
                )
            )
            checks[side + "_old_loop_released"] = (
                result["minimum_old_loop_gap_m"] > 0.004
            )
            checks[side + "_helper_contacted_old_loop"] = (
                result["minimum_helper_gap_m"] < 0.0015
            )
        else:
            checks[side + "_old_thumb_loop_retained"] = (
                result["maximum_old_loop_gap_m"] < 0.0015
            )
        checks[side + "_near_string_not_caught"] = (
            result["minimum_near_string_gap_m"] > 0.002
        )
        stages[side] = result
    checks["other_supports_retained"] = all(
        max(support_gap(poses[int(k)], data["rope"][k], s, f, meshes) for k in last)
        < 0.0015
        for s in ("left", "right")
        for f in ("index", "little")
    )
    checks["other_supports_retained_during_transfer"] = all(
        max(support_gap(poses[int(k)], data["rope"][k], s, f, meshes) for k in transfer)
        < 0.0015
        for s in ("left", "right")
        for f in ("index", "little")
    )
    edges = np.linalg.norm(
        np.roll(data["rope"][transfer], -1, axis=1) - data["rope"][transfer], axis=2
    )
    stretch = float((edges / data["rest_lengths"][transfer]).max())
    full_edges = np.linalg.norm(
        np.roll(data["rope"], -1, axis=1) - data["rope"], axis=2
    )
    full_stretch = float((full_edges / data["rest_lengths"]).max())
    local_arc = 2 * float(payload["config"]["radius"])
    separation = min(
        segment_separation(data["rope"][k], minimum_arc_distance=local_arc)
        for k in transfer
    )
    checks["strain"] = stretch < 1.2
    checks["full_history_strain"] = full_stretch < 1.2
    checks["self_contact"] = separation >= 0.0018 and edges.max() < payload.get(
        "self_contact_search_bound_m", 0.008
    )
    checks["dynamic_rope"] = bool(
        np.all(data["particle_masses"] > 0) and not len(data["anchors"])
    )
    checks["completed_hold"] = (
        payload["frames"][-1]["phase"]
        == (
            "Hold assisted old-loop release"
            if args.release_old
            else "Hold new thumb bights"
        )
        and times[-1] >= payload["motion"]["duration"] - 1e-6
    )
    report = dict(
        passed=all(checks.values()),
        checks={k: bool(v) for k, v in checks.items()},
        sides=stages,
        maximum_stretch_ratio=stretch,
        full_history_maximum_stretch_ratio=full_stretch,
        minimum_segment_distance_m=separation,
        local_arc_exclusion_m=local_arc,
        duration_s=float(times[-1]),
        active_sides=sorted(active),
        reference_time_s=float(times[original]),
        initialization=payload["motion"].get(
            "initialization", "Continuous Opening A history"
        ),
        scope="One-sided old-loop release"
        if args.release_old
        else "Thumb pickup only; Cup & Saucer and Star are not yet verified",
    )
    (out / "transfer_validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
