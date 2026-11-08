"""Time common-wrist reversals from measured free-loop turning points or crossings.

This experimental controller reuses continue_cup's exact-state continuation and
collision guards. It observes privileged simulated rope state and commands only
robot joints. An elevated free band is a timing candidate, not a successful Cup.
Optional near-strand checks stop integration at the first rejected output
frame; they are not substep contact feedback.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.free_loop import (
    ApexDetector, BottomCrossingDetector, free_loop_material,
    common_wrist_translation, validate_bilateral_release,
    validate_bilateral_near_hold, observed_phase_time,
)
from revo3_cat_cradle.pinch import bilateral_grip_observation
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_contact_indices
from revo3_cat_cradle.transfers import track_support_patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--amplitude", type=float, default=0.08)
    parser.add_argument("--center-offset", type=float, default=0,
                        help="Forward offset of the stroke center from the initial wrists")
    parser.add_argument("--timing", choices=("apex", "bottom"), default="apex",
                        help="Reverse at an X turning point or a directed X=0 crossing; "
                             "both are measured relative to the original thumb bridge")
    parser.add_argument("--strokes", type=int, default=5)
    parser.add_argument("--stroke-duration", type=float, default=0.6)
    parser.add_argument("--cartesian-steps", type=int, default=9)
    parser.add_argument("--probe-step", type=float, default=0.1)
    parser.add_argument("--apex-timeout", type=float, default=2)
    parser.add_argument("--apex-min-offset", type=float, default=0.03)
    parser.add_argument("--hold-duration", type=float, default=1.2)
    parser.add_argument("--require-near-pinch", action="store_true",
                        help="Require a held bilateral near-strand proximity source and "
                             "reject lost proximity or accidental old-loop pairing")
    args = parser.parse_args()
    durations = [args.stroke_duration, args.probe_step, args.apex_timeout, args.hold_duration]
    if (not np.isfinite([args.amplitude, args.center_offset, *durations]).all()
            or not 0 < args.amplitude <= .12
            or abs(args.center_offset) + args.amplitude > .12):
        parser.error("Amplitude must be positive and both stroke targets within 0.12 m of the initial wrists")
    if args.strokes < 1 or min(durations) <= 0 or any(abs(t * 30 - round(t * 30)) > 1e-7 for t in durations):
        parser.error("Use positive durations on the 30 Hz grid and at least one stroke")
    if args.cartesian_steps < 1 or abs(args.stroke_duration / args.cartesian_steps * 30 - round(args.stroke_duration / args.cartesian_steps * 30)) > 1e-7:
        parser.error("Cartesian segments must lie on the 30 Hz grid")
    if not np.isfinite(args.apex_min_offset) or not 0 < args.apex_min_offset <= args.amplitude:
        parser.error("Turning-point displacement gate must lie in (0, amplitude]")
    source, out = args.source.resolve(), args.output.resolve()
    saved = np.load(source / "checkpoint.npz")
    payload = json.loads((source / "trajectory.json").read_text())
    tracking = json.loads((source / "material_support_tracking.json").read_text())
    release = json.loads((source / "loop_release_audit.json").read_text())
    if not tracking["tracking_complete"] or abs(tracking["final"]["time"] - float(saved["time"])) > 1e-5:
        raise ValueError("Source needs completed material tracking through its checkpoint")
    validate_bilateral_release(release, float(saved["time"]))
    if args.require_near_pinch:
        near_hold = json.loads((source / "near_pinch_edge_audit.json").read_text())
        validate_bilateral_near_hold(near_hold, float(saved["time"]))
    if payload["config"]["fps"] != 30 or payload["config"]["adaptive_length"]:
        raise ValueError("Controller expects a passive rope at 30 Hz")
    if abs(payload["config"]["radius"] - 0.001) > 1e-10:
        raise ValueError("Material tracking expects a 1 mm rope radius")
    if payload["collision_geometry"].get("robot_body") != "hulls":
        raise ValueError("Controller requires physical arm/body hull contacts")
    plan = json.loads((source / "transfer_plan.json").read_text())
    band, bridge = free_loop_material(plan, len(saved["particle_q"]))
    robot = RevoTron()
    meshes = load_knuckle_meshes(robot) if payload["collision_geometry"].get("knuckles") == "mesh" else {}
    q0 = saved["joints"].copy()
    wrists = {side: robot.wrist(q0, side).copy() for side in ("left", "right")}
    q_current = q0.copy()
    out.mkdir(parents=True, exist_ok=False)
    centers = {name: item["center"] for name, item in tracking["final"]["supports"].items()}
    last_checked = float(saved["time"])
    report = dict(
        scope=__doc__, source=str(source),
        source_checkpoint_sha256=hashlib.sha256((source / "checkpoint.npz").read_bytes()).hexdigest(),
        amplitude_m=args.amplitude, stroke_duration_s=args.stroke_duration,
        center_offset_m=args.center_offset,
        timing_trigger=args.timing,
        phase_observation="Approach tracked during strokes; cues accepted after motion",
        probe_step_s=args.probe_step, apex_timeout_s=args.apex_timeout,
        apex_minimum_offset_m=args.apex_min_offset,
        timing_velocity="Three-frame finite difference of fixed-material relative positions",
        maximum_strokes=args.strokes, cartesian_steps=args.cartesian_steps,
        fixed_free_material=band.tolist(), fixed_thumb_material=bridge.tolist(),
        events=[], samples=[], retention_errors=[], height_candidates=[],
        require_near_pinch=args.require_near_pinch, near_pinch_errors=[],
        old_pinch_errors=[],
        near_pinch_maximum_gap_m=.0001,
        completed_figure=False, status="running",
    )
    current_source = source
    history = []
    for frame in payload["frames"][-3:]:
        rope = np.asarray(frame["rope"])
        history.append((float(frame["time"]), rope[band].mean(axis=0) - rope[bridge].mean(axis=0)))

    def flush():
        pending = out / "swing_control.pending.json"
        pending.write_text(json.dumps(report, indent=2) + "\n")
        pending.replace(out / "swing_control.json")

    def advance(waypoints=None, duration=None, phase=None):
        nonlocal current_source, last_checked, q_current
        phase = phase or ("Hold for the free-loop turning point" if args.timing == "apex"
                          else "Hold for the directed free-loop center crossing")
        moving_until = last_checked + sum(row["duration"] for row in waypoints) if waypoints else last_checked
        feedback = []
        command = [sys.executable, str(ROOT / "scripts/continue_cup.py"),
                   "--source", str(current_source), "--output", str(out), "--side", "both",
                   "--check-self-collision", "--check-robot-hulls", "--check-body-envelope",
                   "--max-stretch-ratio", "1.2",
                   "--checkpoint-every-frame", "--hold-label", phase]
        if args.require_near_pinch:
            command += ["--stop-on-grip-loss"]
        if waypoints is None:
            command += ["--hold-only", "--hold-duration", str(duration or args.probe_step)]
        else:
            waypoint = out / "control_waypoint.json"
            waypoint.write_text(json.dumps(waypoints))
            command += ["--route-only", "--approach-path", str(waypoint),
                        "--hold-duration", str(args.probe_step)]
        with (out / "physics.log").open("a") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        current_source = out
        with np.load(out / "trajectory.npz") as trajectory:
            for k in np.flatnonzero(trajectory["time"] > last_checked + 1e-5):
                time = float(trajectory["time"][k])
                rope = trajectory["rope"][k]
                transforms = robot.fk(trajectory["joints"][k])
                if args.require_near_pinch:
                    observed = bilateral_grip_observation(transforms, rope, plan)
                    pairs = {side: observed[side]["near_indices"] for side in observed}
                    old_pairs = {side: observed[side]["old_loop_indices"] for side in observed}
                    if any(old_pairs.values()):
                        report["old_pinch_errors"].append(dict(time_s=time, paired_edges=old_pairs))
                        raise RuntimeError("An old loop entered the near-strand grip during the stroke")
                    if not all(pairs.values()):
                        report["near_pinch_errors"].append(dict(time_s=time, paired_edges=pairs))
                        raise RuntimeError("Near-strand proximity was lost during the stroke")
                for name in centers:
                    side, finger = name.split("_")
                    contacts = support_contact_indices(transforms, rope, side, finger, meshes)
                    try:
                        centers[name], _, _ = track_support_patch(contacts, len(rope), centers[name])
                    except ValueError as exc:
                        report["retention_errors"].append(dict(time_s=time, support=name, reason=str(exc)))
                        raise RuntimeError("Original support material tracking failed") from exc
                strain = float((np.linalg.norm(np.roll(rope, -1, axis=0) - rope, axis=1) / trajectory["rest_lengths"][k]).max())
                if strain >= 1.2:
                    report["strain_stop"] = dict(time_s=time, ratio=strain)
                    raise RuntimeError("Rope stretch reached the unchanged 1.2 gate")
                relative = rope[band].mean(axis=0) - rope[bridge].mean(axis=0)
                history.append((time, relative.copy()))
                del history[:-3]
                estimated_velocity = (history[-1][1] - history[0][1]) / (history[-1][0] - history[0][0])
                with np.load(out / f"state_{round(time * 30):04d}.npz") as frame_state:
                    velocity = frame_state["particle_qd"][band].mean(axis=0) - frame_state["particle_qd"][bridge].mean(axis=0)
                sample = dict(time_s=time, relative_position_m=relative.tolist(),
                              relative_velocity_m_s=velocity.tolist(), timing_velocity_m_s=estimated_velocity.tolist())
                report["samples"].append(sample)
                if time >= moving_until - 1e-5:
                    feedback.append(sample)
                if relative[2] > 0.02:
                    report["height_candidates"].append(dict(time_s=time, relative_position_m=relative.tolist(), checkpoint=f"state_{round(time * 30):04d}.npz"))
            last_checked = float(trajectory["time"][-1])
        state = np.load(out / "checkpoint.npz")
        q_current = state["joints"].copy()
        flush()
        print(report["samples"][-1], flush=True)
        return feedback

    try:
        for stroke in range(args.strokes):
            direction = 1 if stroke % 2 == 0 else -1
            offset = args.center_offset + direction * args.amplitude
            targets = {s: pose[:3, 3] + [offset, 0, 0] for s, pose in wrists.items()}
            waypoints, kinematics = common_wrist_translation(robot, q_current, targets, args.stroke_duration, args.cartesian_steps)
            report["events"].append(dict(kind="stroke", number=stroke + 1, direction=direction, start_s=last_checked, kinematics=kinematics))
            flush()
            first_sample = len(report["samples"])
            moving_until = last_checked + args.stroke_duration
            samples = advance(waypoints=waypoints)
            if report["height_candidates"]:
                report["status"] = "elevated_band_candidate"
                break
            if stroke + 1 == args.strokes:
                report["status"] = "stroke_limit"
                break
            detector_type = ApexDetector if args.timing == "apex" else BottomCrossingDetector
            detector = detector_type(direction, minimum_offset=args.apex_min_offset)
            until = last_checked + args.apex_timeout
            apex = observed_phase_time(detector, report["samples"][first_sample:],
                                       not_before=moving_until)
            while apex is None:
                if last_checked >= until - 1e-5:
                    report["status"] = "apex_timeout" if args.timing == "apex" else "crossing_timeout"
                    break
                samples = advance()
                if report["height_candidates"]:
                    report["status"] = "elevated_band_candidate"
                    break
                apex = observed_phase_time(detector, samples)
            if report["status"] != "running":
                break
            report["events"].append(dict(kind=("observed_apex" if args.timing == "apex"
                                              else "observed_center_crossing"), direction=direction,
                                         observed_time_s=apex, reversal_time_s=last_checked))
        advance(duration=args.hold_duration, phase="Hold after the measured-phase loop swings")
        if report["height_candidates"]:
            report["status"] = "elevated_band_candidate"
    except Exception as exc:
        report["status"] = "rejected"
        report["error"] = str(exc)
        stop = out / "stretch_stop.json"
        if stop.exists():
            report["strain_stop"] = json.loads(stop.read_text())
        grip_stop = out / "grip_stop.json"
        if grip_stop.exists():
            stopped = json.loads(grip_stop.read_text())
            report["grip_stop"] = stopped
            for strand, field, failed in (
                ("near_indices", "near_pinch_errors", lambda values: not all(values.values())),
                ("old_loop_indices", "old_pinch_errors", lambda values: any(values.values())),
            ):
                values = {side: stopped["sides"][side][strand] for side in ("left", "right")}
                if failed(values):
                    report[field].append(dict(time_s=stopped["time_s"], paired_edges=values))
        raise
    finally:
        flush()


if __name__ == "__main__":
    main()
