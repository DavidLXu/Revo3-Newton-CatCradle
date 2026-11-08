"""Continue the dynamic Opening A checkpoint into thumb pickup experiments."""

import argparse, json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.transfers import (
    digit_target,
    fingertip,
    support_arc,
    original_thumb_loop,
    track_support_patch,
)
from revo3_cat_cradle.bimanual_checks import gaps, hand_clearance, self_clearance_margin
from revo3_cat_cradle.mesh_geometry import load_knuckle_meshes, support_contact_indices
from revo3_cat_cradle.palm_clearance import OpposingPalmClearance
from revo3_cat_cradle.body_envelope import CentralBodyEnvelope
from revo3_cat_cradle.robot_geometry import RobotHullAudit
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH
from revo3_cat_cradle.pinch import near_pinch_samples


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=ROOT / "output/publish_opening_a")
    p.add_argument("--output", type=Path, default=ROOT / "output/cup_pickup")
    p.add_argument("--checkpoint", type=Path)
    p.add_argument("--checkpoint-every-frame", action="store_true",
                   help="Save exact dynamic states for subsequent contact-timing branches")
    p.add_argument("--max-stretch-ratio", type=float,
                   help="Stop at the first recorded frame reaching this stretch ratio")
    p.add_argument("--support-reference", type=Path)
    p.add_argument(
        "--pickup-only",
        action="store_true",
        help="Hold after approach and reach, before optional scoop probes",
    )
    p.add_argument("--hold-only", action="store_true")
    p.add_argument("--hold-duration", type=float, default=2)
    p.add_argument("--hold-label", default="Hold transfer pose")
    p.add_argument(
        "--stop-on-support-loss", action="store_true",
        help="Track all four original thumb/index patches at every output frame; "
             "requires a source material_support_tracking.json matching the checkpoint",
    )
    p.add_argument(
        "--hold-on-near-pinch", choices=("left", "right", "both"),
        help="During a route, stop closing at opposing thumb/index contact on the "
             "same near-strand sample, then hold the measured joint pose",
    )
    p.add_argument("--pinch-max-gap", type=float, default=.0001,
                   help="Maximum gap to each pinch surface in metres (0 to 0.0001); "
                        "zero requires nonpositive gaps, not merely proximity")
    p.add_argument(
        "--stop-on-near-contact",
        action="store_true",
        help="Stop before continuing a transfer that contacts its forbidden near strand",
    )
    p.add_argument(
        "--friction",
        type=float,
        help="Hand-rope friction after restoring the exact checkpoint state",
    )
    p.add_argument(
        "--fixed-rest-lengths",
        action="store_true",
        help="After restoring the full state, stop active take-up without changing rope positions or velocities",
    )
    p.add_argument(
        "--reach-only",
        action="store_true",
        help="Continue an existing approach with reach and hold only",
    )
    p.add_argument(
        "--route-only",
        action="store_true",
        help="Hold immediately after supplied joint waypoints",
    )
    p.add_argument("--assist-old", action="store_true")
    p.add_argument("--assist-local", action="store_true")
    p.add_argument("--assist-return", action="store_true")
    p.add_argument(
        "--assist-far",
        action="store_true",
        help="Raise the right far-index strand with its middle finger before thumb pickup",
    )
    p.add_argument("--far-approach", choices=["under", "direct"], default="under")
    p.add_argument("--assist-index-curl", type=float, default=0.0)
    p.add_argument("--side", choices=["left", "right", "both"], default="left")
    p.add_argument("--depth", type=float, default=0.015)
    p.add_argument("--reach", type=float, default=0.012)
    p.add_argument("--lift", type=float, default=0.018)
    p.add_argument("--avoid-near", action="store_true")
    p.add_argument("--approach-path", type=Path)
    p.add_argument(
        "--skip-approach",
        action="store_true",
        help="Start from an already completed thumb overpass and continue reach, lower, scoop and lift",
    )
    p.add_argument(
        "--check-self-collision",
        action="store_true",
        help="Reject same-hand finger overlap and opposing palm intersections before replay",
    )
    p.add_argument(
        "--check-body-envelope", action="store_true",
        help="Stop if a rope edge lacks separation from a conservative central-body box",
    )
    p.add_argument(
        "--check-robot-hulls", action="store_true",
        help="Stop if a complete rope centerline edge enters an arm or body collision hull",
    )
    p.add_argument("--index-curl", type=float, default=0.0)
    p.add_argument("--index-pip", type=float, default=0.0)
    p.add_argument("--index-dip", type=float, default=0.0)
    p.add_argument(
        "--little-curl",
        type=float,
        help="Optional little MCP angle during index-bight preparation",
    )
    p.add_argument("--little-pip", type=float)
    p.add_argument("--little-dip", type=float)
    p.add_argument(
        "--prepare-only",
        action="store_true",
        help="Lower index bights and hold, then replan from the resulting rope state",
    )
    p.add_argument("--over-height", type=float, default=0.015)
    p.add_argument("--reach-duration", type=float, default=2.0)
    p.add_argument(
        "--retarget-before-reach",
        action="store_true",
        help="Update the selected far-strand point after the approach, using current rope state",
    )
    p.add_argument(
        "--target-index", type=int, help="Material point on the intended far strand"
    )
    a = p.parse_args()
    if not np.isfinite(a.pinch_max_gap) or not 0 <= a.pinch_max_gap <= .0001:
        p.error("--pinch-max-gap must be finite and in [0, 0.0001] metres")
    if a.pinch_max_gap != .0001 and not a.hold_on_near_pinch:
        p.error("--pinch-max-gap requires --hold-on-near-pinch")
    if a.hold_on_near_pinch and (
        not a.route_only or not a.approach_path or a.hold_only
        or a.stop_on_near_contact or a.side not in ("both", a.hold_on_near_pinch)
    ):
        p.error("--hold-on-near-pinch requires a matching --side, --route-only and "
                "--approach-path, without --hold-only or --stop-on-near-contact")
    if a.max_stretch_ratio is not None and (
        not np.isfinite(a.max_stretch_ratio) or a.max_stretch_ratio <= 1
    ):
        p.error("--max-stretch-ratio must be finite and greater than one")
    if a.friction is not None and (not np.isfinite(a.friction) or a.friction < 0):
        p.error("--friction must be finite and nonnegative")
    if a.prepare_only and not a.index_curl:
        p.error("--prepare-only requires a nonzero --index-curl")
    if a.target_index is not None and a.side == "both":
        p.error("--target-index requires a single side")
    if not np.isfinite(a.reach_duration) or a.reach_duration <= 0:
        p.error("--reach-duration must be finite and positive")
    if not np.isfinite(a.hold_duration) or a.hold_duration <= 0:
        p.error("--hold-duration must be finite and positive")
    out = a.output
    out.mkdir(parents=True, exist_ok=True)
    data = json.loads((a.source / "trajectory.json").read_text())
    commands = json.loads((a.source / "joint_commands.json").read_text())
    checkpoint = a.checkpoint or a.source / "checkpoint.npz"
    seed = np.load(checkpoint)
    r = RevoTron()
    meshes = (
        load_knuckle_meshes(r)
        if data.get("collision_geometry", {}).get("knuckles") == "mesh"
        else None
    )
    m = Motion(r)
    m.resume_at(commands, data["motion"], float(seed["time"]), seed["joints"])
    m.metadata.update(
        source="Opening A to Cup and Saucer transfer development",
        status="experimental",
        figure="Thumb-loop transfer",
    )
    sim = BimanualSimulation(
        r,
        m,
        initial_loop(r, m),
        Config(**json.loads(str(seed["config"]))),
        palm_geometry="hull",
        knuckle_geometry=data.get("collision_geometry", {}).get("knuckles", "none"),
        robot_body_geometry=data.get("collision_geometry", {}).get("robot_body", "none"),
        length_control=data.get("length_control", "local"),
        rest_length_max_scale=data.get("rest_length_max_scale", 2),
    )
    sim.load_checkpoint(checkpoint)
    support_centers = {}
    if a.stop_on_support_loss:
        if abs(sim.config.radius - .001) > 1e-10:
            raise ValueError("Original material tracking requires a 1 mm rope radius")
        tracking = json.loads((a.source / "material_support_tracking.json").read_text())
        samples = [row for row in tracking["samples"]
                   if abs(row["time"] - float(seed["time"])) < 1e-5]
        names = {f"{s}_{f}" for s in ("left", "right") for f in ("thumb", "index")}
        if len(samples) != 1 or set(samples[0]["supports"]) != names:
            raise ValueError("Source needs four original material tracks at the checkpoint")
        support_centers = {name: row["center"] for name, row in samples[0]["supports"].items()}
    if a.friction is not None:
        m.metadata.setdefault("initial_friction", data["config"]["friction"])
        m.metadata.setdefault("physics_events", []).append(
            dict(
                time=sim.t,
                parameter="friction",
                previous_value=sim.config.friction,
                value=a.friction,
            )
        )
        sim.set_contact_friction(a.friction)
    if a.fixed_rest_lengths:
        m.metadata.setdefault(
            "initial_adaptive_length", data["config"]["adaptive_length"]
        )
        m.metadata.setdefault("physics_events", []).append(
            dict(time=sim.t, parameter="adaptive_length", value=False)
        )
        # Keep every existing rest length, spring and dynamic state unchanged.
        # Subsequent frames are a passive rope with this material length.
        sim.config.adaptive_length = False
    count = round(sim.t * 30) + 1
    frames = data["frames"][:count]
    metrics = json.loads((a.source / "metrics.json").read_text())["metrics"][:count]
    q = sim.q.copy()
    palm_checker = OpposingPalmClearance(r, r.fk(q)) if a.check_self_collision else None
    body_checker = CentralBodyEnvelope(r) if a.check_body_envelope else None
    hull_checker = RobotHullAudit(r) if a.check_robot_hulls else None
    # Subsequent arm-assisted transfers start in the actual saved wrist frames,
    # including any orientation chosen by the contact-aware search.
    m.rotations = {side: r.wrist(q, side)[:3, :3].copy() for side in ("left", "right")}
    rope = sim.state.particle_q.numpy()
    sides = ("left", "right") if a.side == "both" else (a.side,)
    previous_plan = a.source / "transfer_plan.json"
    plan = json.loads(previous_plan.read_text()) if previous_plan.exists() else {}
    audit = []
    if a.hold_only or a.assist_old:
        plan = json.loads((a.source / "transfer_plan.json").read_text())
    # A route-only continuation must retain material identities from its source.
    preserve_plan = (
        a.hold_only or a.assist_old or (a.route_only and previous_plan.exists())
    )
    for s in () if preserve_plan else sides:
        other = "right" if s == "left" else "left"
        sign = 1 if s == "left" else -1
        reference = (
            np.load(a.support_reference / "checkpoint.npz")
            if a.support_reference
            else seed
        )
        far = support_arc(
            r,
            reference["joints"],
            reference["particle_q"],
            (s, "index"),
            (other, "little"),
            meshes=meshes,
        )
        near = support_arc(
            r,
            reference["joints"],
            reference["particle_q"],
            (s, "index"),
            (other, "thumb"),
            meshes=meshes,
        )
        old = np.flatnonzero(
            gaps(r.fk(reference["joints"]), reference["particle_q"], s, "thumb")
            < 0.0015
        )
        old_loop = original_thumb_loop(
            r, reference["joints"], reference["particle_q"], s, meshes=meshes
        )
        k = (
            int(far[np.argmin(abs(rope[far, 1] - sign * 0.025))])
            if a.target_index is None
            else a.target_index
        )
        if k not in far:
            raise ValueError("Target material point must belong to the far strand")
        target = rope[k].copy()
        plan[s] = dict(
            far_indices=far.tolist(),
            near_indices=near.tolist(),
            old_thumb_indices=old.tolist(),
            old_loop_indices=old_loop.tolist(),
            target=target.tolist(),
            target_index=k,
        )
    (out / "transfer_plan.json").write_text(json.dumps(plan, indent=2))

    def save():
        export_bimanual(sim, frames, metrics, out)
        (out / "joint_commands.json").write_text(
            json.dumps(
                dict(
                    times=m.times,
                    targets=[q.tolist() for q in m.targets],
                    phases=m.phases,
                )
            )
        )
        (out / "transfer_audit.json").write_text(json.dumps(audit, indent=2))

    def move(t, q, phase, detect_pinch=False):
        if (
            not np.isfinite(q).all()
            or np.any(q < r.limits[:, 0] - 1e-6)
            or np.any(q > r.limits[:, 1] + 1e-6)
        ):
            raise ValueError("Transfer target violates joint limits")
        for u in np.linspace(0, 1, 21):
            trial = (1 - u) * sim.q + u * q
            if hand_clearance(r.fk(trial))[0] < -0.0005:
                raise ValueError(f"Transfer phase crosses opposing hand: {phase}")
            if a.check_self_collision:
                for side in ("left", "right"):
                    if self_clearance_margin(r.fk(trial), side) < 0:
                        raise ValueError(
                            f"Transfer phase self-collides on {side}: {phase}"
                        )
                clearance = palm_checker.measure(r.fk(trial), quick=True)
                if (
                    clearance["finger_palm_lower_bound_m"] < 0
                    or clearance["palm_palm_sphere_lower_bound_m"] < 0
                ):
                    raise ValueError(
                        f"Transfer phase lacks opposing palm clearance: {phase}; "
                        f"{clearance}"
                    )
        m.append(t, q, phase)
        for i in range(round(sim.t * 30) + 1, round(t * 30) + 1):
            sim.advance(i / 30)
            frames.append(sim.snapshot())
            metrics.append(sim.metrics())
            if (
                not np.isfinite(sim.state.particle_q.numpy()).all()
                or metrics[-1]["max_coordinate_m"] > 3
            ):
                save()
                raise RuntimeError("Diverged")
            positions = sim.state.particle_q.numpy()
            if a.max_stretch_ratio is not None and metrics[-1]["max_stretch_ratio"] >= a.max_stretch_ratio:
                save()
                sim.save_checkpoint(out / f"state_{i:04d}.npz")
                (out / "stretch_stop.json").write_text(json.dumps(dict(
                    time_s=sim.t, ratio=metrics[-1]["max_stretch_ratio"],
                    threshold=a.max_stretch_ratio,
                ), indent=2) + "\n")
                raise RuntimeError("Rope stretch reached the requested gate")
            if hull_checker is not None:
                intrusions = hull_checker.measure(sim.transforms, positions)
                if intrusions:
                    save()
                    (out / "robot_hull_stop.json").write_text(json.dumps(dict(
                        time_s=sim.t,
                        scope="Rope centerline inside a conservative arm/body hull",
                        links=intrusions,
                    ), indent=2))
                    raise RuntimeError("Rope centerline enters a robot collision hull")
            if body_checker is not None:
                body_clearance = body_checker.measure(sim.transforms, positions, sim.config.radius)
                if body_clearance["minimum_face_margin_m"] < 0:
                    save()
                    (out / "body_envelope_stop.json").write_text(json.dumps(dict(
                        time_s=sim.t,
                        scope="Conservative box overlap; not proof of mesh penetration",
                        **body_clearance,
                    ), indent=2))
                    raise RuntimeError("Rope lacks certified central-body envelope clearance")
            if (
                np.linalg.norm(np.roll(positions, -1, axis=0) - positions, axis=1).max()
                >= MAX_SEGMENT_LENGTH
            ):
                save()
                raise RuntimeError(
                    "Transfer exceeded the rope self-contact search bound"
                )
            if a.stop_on_near_contact:
                near_gaps = {
                    s: float(gaps(
                        sim.transforms, positions[plan[s]["near_indices"]], s, "thumb"
                    ).min()) for s in sides
                }
                if min(near_gaps.values()) < 0.0015:
                    save()
                    sim.save_checkpoint(out / f"state_{i:04d}.npz")
                    (out / "near_stop.json").write_text(json.dumps(dict(
                        time_s=sim.t, gap_m=near_gaps, threshold_m=0.0015,
                        phase=phase, commanded_phase_end_s=t,
                    ), indent=2) + "\n")
                    raise RuntimeError("Transfer contacted the forbidden near strand")
            if a.checkpoint_every_frame:
                sim.save_checkpoint(out / f"state_{i:04d}.npz")
            for name in support_centers:
                side, finger = name.split("_")
                contacts = support_contact_indices(sim.transforms, positions, side,
                                                   finger, meshes or {})
                try:
                    support_centers[name], _, _ = track_support_patch(
                        contacts, len(positions), support_centers[name])
                except ValueError as exc:
                    save()
                    sim.save_checkpoint(out / f"state_{i:04d}.npz")
                    (out / "support_stop.json").write_text(json.dumps(dict(
                        time_s=sim.t, support=name, previous_center=support_centers[name],
                        reason=str(exc), contacts=contacts.tolist(),
                    ), indent=2) + "\n")
                    raise RuntimeError("Original support material tracking failed") from exc
            if detect_pinch:
                pinch_sides = (("left", "right") if a.hold_on_near_pinch == "both"
                               else (a.hold_on_near_pinch,))
                paired = {side: [row for row in near_pinch_samples(
                    sim.transforms, positions, side, plan[side]["near_indices"],
                    sim.config.radius, max_gap_m=a.pinch_max_gap,
                ) if row["paired"]] for side in pinch_sides}
                if all(paired.values()):
                    # Clip the current quintic without rewriting its executed
                    # prefix. Only the robot command changes; rope state is kept.
                    m.resume_at(dict(times=m.times, targets=m.targets, phases=m.phases),
                                m.metadata, sim.t, sim.q)
                    (out / "pinch_trigger.json").write_text(json.dumps(dict(
                        time_s=sim.t, sides=list(pinch_sides), commanded_phase_end_s=t,
                        maximum_surface_gap_m=a.pinch_max_gap,
                        samples=paired,
                        scope="Sampled capsule pinch geometry; validate the subsequent "
                              "hold, original supports and physical gates separately.",
                    ), indent=2) + "\n")
                    save()
                    sim.save_checkpoint(out / f"state_{i:04d}.npz")
                    return True
            if i % 30 == 0:
                rope = sim.state.particle_q.numpy()
                record = dict(
                    time=sim.t, phase=phase, strain=metrics[-1]["max_stretch_ratio"]
                )
                for s in sides:
                    record[s] = {
                        name: float(
                            gaps(sim.transforms, rope[plan[s][name]], s, "thumb").min()
                        )
                        for name in ("far_indices", "near_indices", "old_thumb_indices")
                    }
                    if "old_loop_indices" in plan[s]:
                        record[s]["old_loop_gap"] = float(
                            gaps(
                                sim.transforms,
                                rope[plan[s]["old_loop_indices"]],
                                s,
                                "thumb",
                            ).min()
                        )
                    if a.assist_far:
                        record[s]["middle_far_gap"] = float(
                            gaps(
                                sim.transforms,
                                rope[plan[s]["far_indices"]],
                                s,
                                "middle",
                            ).min()
                        )
                audit.append(record)
                print(record, flush=True)
                save()
        save()
        sim.save_checkpoint(out / f"state_{round(t * 30):04d}.npz")

    if a.assist_far:
        if a.far_approach == "under":
            q = digit_target(r, q, "right", "middle", np.array([0.390, -0.035, 1.205]))
            move(sim.t + 2, q, "Curl middle behind and below the far strand")
        for target, label in [
            ([0.388, 0.009, 1.218], "Fold right middle below far-index strand"),
            ([0.388, 0.009, 1.238], "Pick far-index strand up with right middle"),
            ([0.388, 0.009, 1.268], "Raise far-index strand for thumb handoff"),
        ]:
            q = digit_target(r, q, "right", "middle", np.array(target))
            move(sim.t + 2, q, label)
        move(sim.t + 2, q, "Hold middle-finger far-strand assistance")
        return
    if a.assist_old:
        if a.assist_index_curl:
            q = m.finger(q, "right", "index", [a.assist_index_curl, 0.3, 0.3])
            move(sim.t + 2, q, "Hook right index to retain its existing bight")
        if a.assist_return:
            # Lift the other branch of the old loop, accessible to the right
            # middle finger without translating the wrist or its index bight.
            # These are robot fingertip targets only; the rope stays dynamic.
            for target, label in [
                ([0.390, 0.015, 1.230], "Curl below old thumb return strand"),
                ([0.390, 0.015, 1.250], "Pick up old thumb return strand"),
                ([0.390, 0.015, 1.280], "Lift old return strand above thumb"),
                ([0.410, 0.015, 1.300], "Carry old return strand beyond thumb"),
            ]:
                q = digit_target(r, q, "right", "middle", np.array(target))
                move(sim.t + 2, q, label)
            move(sim.t + 2, q, "Hold assisted old-loop release")
            return
        if a.assist_local:
            q = digit_target(r, q, "left", "thumb", np.array([0.42, 0.025, 1.27]))
            move(sim.t + 2, q, "Present old thumb bridge to the opposite middle finger")
            q[r.indices["right_middle_MPR_joint"]] = 0
            q = m.finger(q, "right", "middle", [1.58, 0.6, 0.1])
            move(
                sim.t + 2,
                q,
                "Curl right middle below old bridge without moving its wrist",
            )
            q = m.finger(q, "right", "middle", [1.58, 0.15, 0.1])
            move(sim.t + 2, q, "Scoop old bridge with right middle")
            wrist = r.wrist(q, "right")[:3, 3].copy()
            for offset, label in [
                ([0, 0, 0.06], "Raise old bridge over left thumb"),
                ([0.055, 0, 0.06], "Carry old loop beyond thumb tip"),
            ]:
                q = r.ik(q, "right", wrist + offset, m.rotations["right"])
                move(sim.t + 2, q, label)
            move(sim.t + 2, q, "Hold assisted old-loop release")
            return
        # The opposite middle finger helps lift the lower thumb loop over its tip.
        q[r.indices["right_middle_MPR_joint"]] = 0
        q = m.finger(q, "right", "middle", [1.58, 0.15, 0.1])
        move(sim.t + 2, q, "Fold right middle finger for old-loop assistance")
        for target, label in [
            ([0.364, 0.017, 1.224], "Approach below old thumb bridge"),
            ([0.364, 0.027, 1.224], "Pass right middle under old bridge"),
            ([0.370, 0.027, 1.259], "Lift old thumb bridge"),
            ([0.395, 0.027, 1.300], "Raise old loop above left thumb"),
            ([0.435, 0.027, 1.300], "Carry old loop beyond left thumb tip"),
        ]:
            q = m.aim_tip(q, "right", "middle", np.array(target))
            move(sim.t + 2, q, label)
        move(sim.t + 2, q, "Hold assisted old-loop release")
        return
    if a.hold_only:
        move(sim.t + a.hold_duration, q, a.hold_label)
        return
    if a.index_curl:
        for s in sides:
            q = m.finger(q, s, "index", [a.index_curl, a.index_pip, a.index_dip])
            if a.little_curl is not None:
                q[r.indices[f"{s}_little_MCP_joint"]] = a.little_curl
            for segment, value in (("PIP", a.little_pip), ("DIP", a.little_dip)):
                if value is not None:
                    q[r.indices[f"{s}_little_{segment}_joint"]] = value
        move(sim.t + 2, q, "Lower index bights to provide thumb clearance")
        rope = sim.state.particle_q.numpy()
        for s in sides:
            sign = 1 if s == "left" else -1
            ids = np.array(plan[s]["far_indices"])
            k = (
                int(ids[np.argmin(abs(rope[ids, 1] - sign * 0.025))])
                if a.target_index is None
                else a.target_index
            )
            plan[s]["target"] = rope[k].tolist()
            plan[s]["target_index"] = k
        (out / "transfer_plan.json").write_text(json.dumps(plan, indent=2))
    if a.prepare_only:
        move(sim.t + 2, q, "Hold lowered index bights for thumb approach")
        return
    if a.approach_path:
        for j, waypoint in enumerate(json.loads(a.approach_path.read_text())):
            if isinstance(waypoint, dict):
                q = np.asarray(waypoint["joints"])
                duration = float(waypoint["duration"])
                phase = waypoint.get("phase", f"Route thumb around near string {j + 1}")
                if not np.isfinite(duration) or duration <= 0:
                    raise ValueError("Waypoint duration must be finite and positive")
            else:
                q, duration, phase = (
                    np.asarray(waypoint),
                    1.5,
                    f"Route thumb around near string {j + 1}",
                )
            if move(sim.t + duration, q, phase, detect_pinch=bool(a.hold_on_near_pinch)):
                q = sim.q.copy()
                break
    if a.route_only:
        if not a.approach_path:
            raise ValueError("--route-only requires --approach-path")
        move(sim.t + a.hold_duration, q, a.hold_label)
        return
    for label, offset, dt in [
        ("Move thumbs over near index strings", [-0.012, 0, a.over_height], 2),
        (
            "Reach beyond far index strings",
            [a.reach, 0, a.over_height],
            a.reach_duration,
        ),
        ("Lower thumbs under far index strings", [a.reach, 0, -a.depth], 2),
        ("Scoop far index strings", [-0.008, 0, -a.depth], 2),
        ("Raise new thumb bights", [-0.008, 0, a.lift], 2),
    ]:
        if (
            a.approach_path or a.reach_only or a.skip_approach
        ) and label == "Move thumbs over near index strings":
            continue
        if a.retarget_before_reach and label == "Reach beyond far index strings":
            current_rope = sim.state.particle_q.numpy()
            for s in sides:
                plan[s]["target"] = current_rope[plan[s]["target_index"]].tolist()
            (out / "transfer_plan.json").write_text(json.dumps(plan, indent=2))
        for s in sides:
            avoid = (
                sim.state.particle_q.numpy()[plan[s]["near_indices"]]
                if a.avoid_near
                else None
            )
            q = digit_target(
                r,
                q,
                s,
                "thumb",
                np.array(plan[s]["target"]) + offset,
                avoid_points=avoid,
            )
        print(
            "Plan",
            label,
            "tip",
            [(s, fingertip(r, q, s, "thumb").tolist()) for s in sides],
            "handgap",
            hand_clearance(r.fk(q)),
            flush=True,
        )
        move(sim.t + dt, q, label)
        if (
            a.pickup_only or a.reach_only
        ) and label == "Reach beyond far index strings":
            break
    move(sim.t + 2, q, "Hold new thumb bights")


if __name__ == "__main__":
    main()
