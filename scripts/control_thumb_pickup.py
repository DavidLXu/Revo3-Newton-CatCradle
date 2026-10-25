"""Experimental joint-only feedback for thumb pickup of a moving strand.

All observations come from Newton's current rope state. Feedback changes
only the selected thumb joint commands, with bounded joint speed. Optional scripted
wrist translations create clearance or carry an assisting finger. It never resets,
pins, projects or teleports rope particles. Geometry losses are planning signals,
not evidence of a completed figure; audit the exported physical trajectory.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.bimanual_checks import (
    digit_clearances,
    gaps,
    hand_clearance,
    self_clearance_margin,
)
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.contact_geometry import finger_capsule


def thumb_pair_clearances(transforms):
    """Nine capsule distances between opposing thumbs, vectorized for IK."""
    ends, radii = {}, {}
    for side in ("left", "right"):
        ends[side], radii[side] = [], []
        for segment in ("MCP", "PIP", "DIP"):
            c, axis, radius, half = finger_capsule("thumb", segment, side)
            tf = transforms[f"{side}_thumb_{segment}_link"]
            ends[side].append(
                np.array([c - axis * half, c + axis * half]) @ tf[:3, :3].T + tf[:3, 3]
            )
            radii[side].append(radius)
    x = np.repeat(ends["left"], 3, axis=0)
    y = np.tile(ends["right"], (3, 1, 1))
    u, v, w = x[:, 1] - x[:, 0], y[:, 1] - y[:, 0], x[:, 0] - y[:, 0]
    dot = lambda a, b: np.einsum("ij,ij->i", a, b)
    a, b, c, d, e = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    den = a * c - b * b
    s = np.clip(
        np.divide(b * e - c * d, den, out=np.zeros_like(den), where=den > 1e-16), 0, 1
    )
    t = (b * s + e) / np.maximum(c, 1e-16)
    s = np.where(t < 0, np.clip(-d / np.maximum(a, 1e-16), 0, 1), s)
    s = np.where(t > 1, np.clip((b - d) / np.maximum(a, 1e-16), 0, 1), s)
    distance = np.linalg.norm(
        x[:, 0] + s[:, None] * u - y[:, 0] - np.clip(t, 0, 1)[:, None] * v, axis=1
    )
    return distance - np.repeat(radii["left"], 3) - np.tile(radii["right"], 3)


def feedback_target(
    robot,
    q,
    rope,
    velocity,
    plan,
    nominal,
    dt,
    speed,
    margin,
    predict,
    contact_weight=3.0,
    seat_depth=None,
    sides=("left", "right"),
    pose_weight=0.0005,
    retained_strands=("far_indices", "old_loop_indices"),
    retained_by_side=None,
    pose_weights=None,
    seated_strands=None,
):
    ids = [
        i
        for i, name in enumerate(robot.names)
        if any(name.startswith(f"{s}_thumb_") for s in sides)
    ]
    # Motion.sample uses quintic interpolation, whose maximum slope is 1.875.
    step = speed * dt / 1.875
    lower = np.maximum(robot.limits[ids, 0], q[ids] - step)
    upper = np.minimum(robot.limits[ids, 1], q[ids] + step)
    near = {
        side: np.concatenate(
            [
                rope[plan[side]["near_indices"]],
                (rope + predict * dt * velocity)[plan[side]["near_indices"]],
            ]
        )
        for side in sides
    }
    weights = np.array(
        [
            (pose_weights or {}).get(robot.names[i].split("_", 1)[0], pose_weight)
            for i in ids
        ]
    )

    def residual(x):
        trial = q.copy()
        trial[ids] = x
        tf = robot.fk(trial)
        result = [weights * (x - nominal[ids]), 0.0002 * (x - q[ids])]
        result.append(15 * np.maximum(0.002 - thumb_pair_clearances(tf), 0))
        for side in sides:
            result.append(
                30 * np.maximum(0.0005 - digit_clearances(tf, side, "thumb"), 0)
            )
            result.append(
                8 * np.maximum(margin - gaps(tf, near[side], side, "thumb"), 0)
            )
            required = (retained_by_side or {}).get(side, retained_strands)
            for name in required:
                contact = gaps(tf, rope[plan[side][name]], side, "thumb").min()
                result.append(
                    np.array(
                        [
                            contact_weight * max(contact - 0.0001, 0),
                            8 * max(-contact - 0.0002, 0),
                        ]
                    )
                )
            seat_name = (seated_strands or {}).get(side, "far_indices")
            if seat_depth is not None and seat_name in required:
                c, axis, radius, half = finger_capsule("thumb", "DIP", side)
                t = tf[f"{side}_thumb_DIP_link"]
                local = (rope[plan[side][seat_name]] - t[:3, 3]) @ t[:3, :3] - c
                axial = local @ axis
                distance = (
                    np.linalg.norm(
                        local - np.clip(axial, -half, half)[:, None] * axis, axis=1
                    )
                    - radius
                    - 0.001
                )
                nearest = int(np.argmin(distance))
                outward = axial[nearest] * (1 if side == "right" else -1)
                # Move the finger farther through its retained strand, so contact is
                # on the distal link's side rather than its rounded tip.
                result.append(np.array([3 * max(outward - seat_depth, 0)]))
        return np.concatenate(result)

    fit = least_squares(
        residual,
        q[ids],
        bounds=(lower, upper),
        max_nfev=60,
        ftol=1e-7,
        xtol=1e-7,
        gtol=1e-9,
    )
    target = q.copy()
    target[ids] = fit.x

    for _ in range(8):
        clear = True
        for u in (0.25, 0.5, 0.75, 1):
            tf = robot.fk((1 - u) * q + u * target)
            if hand_clearance(tf)[0] < -0.0005 or any(
                self_clearance_margin(tf, side) < 0 for side in ("left", "right")
            ):
                clear = False
                break
        if clear:
            return target, float(fit.cost), int(fit.nfev)
        target = (q + target) / 2
    tf = robot.fk(q)
    if hand_clearance(tf)[0] >= -0.0005 and all(
        self_clearance_margin(tf, side) >= 0 for side in ("left", "right")
    ):
        return q.copy(), float(fit.cost), int(fit.nfev)
    raise RuntimeError("Current pose already violates hand clearance")


def scripted_support_target(current, desired, ids, dt, speed):
    """Limit the actual per-frame quintic, not only its slower nominal route."""
    result = np.array(current, dtype=float, copy=True)
    step = speed * dt / 1.875
    if ids:
        delta = desired[ids] - current[ids]
        # A common fraction retains the planned multi-joint posture during lag.
        fraction = min(1.0, step / max(float(np.max(abs(delta))), 1e-15))
        result[ids] += fraction * delta
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument(
        "--plan",
        type=Path,
        help="Material strand identities from the saved Opening A reference",
    )
    nominal_source = parser.add_mutually_exclusive_group(required=True)
    nominal_source.add_argument(
        "--nominal",
        type=Path,
        help="Checkpoint providing thumb pose preference only",
    )
    nominal_source.add_argument(
        "--nominal-route",
        type=Path,
        help="Joint route providing time-varying thumb pose preferences only",
    )
    parser.add_argument(
        "--retain-old-only",
        action="store_true",
        help="Approach around the near strand before asking the thumb to capture the far strand",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=4)
    parser.add_argument(
        "--fixed-rest-lengths",
        action="store_true",
        help="Freeze the restored material lengths while continuing joint-only feedback",
    )
    parser.add_argument(
        "--stop-on-near-contact",
        action="store_true",
        help="Stop the approach experiment when the measured near-string gap drops below 1.5 mm",
    )
    parser.add_argument("--speed", type=float, default=1.5)
    parser.add_argument("--margin", type=float, default=0.003)
    parser.add_argument("--predict", type=float, default=1)
    parser.add_argument("--contact-weight", type=float, default=3)
    parser.add_argument("--pose-weight", type=float, default=0.0005)
    parser.add_argument(
        "--active-side", choices=["both", "left", "right"], default="both"
    )
    parser.add_argument(
        "--assist-other-thumb",
        action="store_true",
        help="Control the other thumb's old loop and near clearance without asking it to pick up a far strand",
    )
    parser.add_argument("--support-pose-weight", type=float, default=0.02)
    parser.add_argument(
        "--seat-old-support",
        action="store_true",
        help="Seat the assisting thumb's old loop away from its rounded tip before moving the wrist",
    )
    parser.add_argument(
        "--scripted-support-joints",
        nargs="+",
        default=[],
        help="Non-feedback finger joints that follow --nominal-route to reposition a supporting strand",
    )
    parser.add_argument(
        "--seat-depth",
        type=float,
        help="Maximum distal axial contact coordinate from DIP capsule center, meters",
    )
    parser.add_argument(
        "--wrist-spread",
        type=float,
        default=0,
        help="Outward Y translation of each wrist in meters",
    )
    parser.add_argument("--spread-duration", type=float, default=2)
    parser.add_argument(
        "--right-wrist-offset",
        type=float,
        nargs=3,
        default=[0, 0, 0],
        help="World XYZ wrist translation while the selected thumb retains its bight",
    )
    a = parser.parse_args()
    capture_sides = ("left", "right") if a.active_side == "both" else (a.active_side,)
    if a.assist_other_thumb and a.active_side == "both":
        parser.error("Other-thumb assistance requires a single pickup side")
    if a.seat_old_support and (not a.assist_other_thumb or a.seat_depth is None):
        parser.error("Old-loop seating requires an assisting thumb and --seat-depth")
    if a.seat_depth is not None and not np.isfinite(a.seat_depth):
        parser.error("Seat depth must be finite")
    sides = ("left", "right") if a.assist_other_thumb else capture_sides
    if (
        min(a.duration, a.speed, a.margin, a.contact_weight, a.spread_duration) <= 0
        or min(a.predict, a.wrist_spread) < 0
        or not np.isfinite(a.right_wrist_offset).all()
        or not np.isfinite([a.pose_weight, a.support_pose_weight]).all()
        or min(a.pose_weight, a.support_pose_weight) < 0
    ):
        parser.error(
            "Duration, speed and margin must be positive; prediction nonnegative"
        )
    a.output.mkdir(parents=True, exist_ok=False)
    saved = np.load(a.checkpoint)
    payload = json.loads((a.source / "trajectory.json").read_text())
    commands = json.loads((a.source / "joint_commands.json").read_text())
    plan = json.loads((a.plan or a.source / "transfer_plan.json").read_text())
    robot = RevoTron()
    if a.scripted_support_joints and not a.nominal_route:
        parser.error("Scripted support joints require --nominal-route")
    support_ids = []
    for name in a.scripted_support_joints:
        if (
            name not in robot.indices
            or not any(
                name.startswith(f"{side}_{finger}_")
                for side in ("left", "right")
                for finger in ("thumb", "index", "middle", "ring", "little")
            )
            or any(name.startswith(f"{side}_thumb_") for side in sides)
        ):
            parser.error("Scripted support joints must be non-feedback finger joints")
        support_ids.append(robot.indices[name])
    motion = Motion(robot)
    motion.resume_at(commands, payload["motion"], float(saved["time"]), saved["joints"])
    motion.metadata.update(
        source="Joint-only moving-strand thumb feedback", status="experimental"
    )
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        Config(**json.loads(str(saved["config"]))),
        palm_geometry=payload["collision_geometry"]["palm"],
        knuckle_geometry=payload["collision_geometry"].get("knuckles", "none"),
        length_control=payload["length_control"],
        rest_length_max_scale=payload["rest_length_max_scale"],
    )
    sim.load_checkpoint(a.checkpoint)
    start = sim.t
    starting_q = sim.q.copy()
    if a.fixed_rest_lengths:
        motion.metadata.setdefault(
            "initial_adaptive_length", sim.config.adaptive_length
        )
        motion.metadata.setdefault("physics_events", []).append(
            dict(time=start, parameter="adaptive_length", value=False)
        )
        sim.config.adaptive_length = False
    retained = (
        ("old_loop_indices",)
        if a.retain_old_only
        else ("far_indices", "old_loop_indices")
    )
    retained_by_side = {
        side: retained if side in capture_sides else ("old_loop_indices",)
        for side in sides
    }
    pose_weights = {
        side: a.pose_weight if side in capture_sides else a.support_pose_weight
        for side in sides
    }
    seated_strands = {
        side: "old_loop_indices"
        if a.seat_old_support and side not in capture_sides
        else "far_indices"
        for side in sides
    }
    reference_motion = None
    if a.nominal_route:
        reference_motion = Motion(robot)
        reference_motion.times = [start]
        reference_motion.targets = [starting_q.copy()]
        reference_motion.phases = ["Approach preference"]
        for waypoint in json.loads(a.nominal_route.read_text()):
            q = np.asarray(
                waypoint["joints"] if isinstance(waypoint, dict) else waypoint
            )
            duration = (
                float(waypoint.get("duration", 1.5))
                if isinstance(waypoint, dict)
                else 1.5
            )
            if (
                not np.isfinite(duration)
                or duration <= 0
                or q.shape != starting_q.shape
                or not np.isfinite(q).all()
            ):
                raise ValueError("Invalid nominal waypoint")
            if np.any(q < robot.limits[:, 0]) or np.any(q > robot.limits[:, 1]):
                raise ValueError("Nominal waypoint violates joint limits")
            if support_ids and (
                1.875
                * np.max(
                    abs(q[support_ids] - reference_motion.targets[-1][support_ids])
                )
                / duration
                > a.speed + 1e-9
            ):
                raise ValueError("Scripted support route exceeds the thumb speed limit")
            reference_motion.append(
                reference_motion.times[-1] + duration, q, "Approach preference"
            )
        nominal = starting_q.copy()
    else:
        nominal = np.load(a.nominal)["joints"]
    spread_q = starting_q.copy()
    for side, sign in (("left", 1), ("right", -1)):
        wrist = robot.wrist(starting_q, side)
        offset = np.array([0, sign * a.wrist_spread, 0], dtype=float)
        if side == "right":
            offset += a.right_wrist_offset
        if np.any(offset):
            spread_q = robot.ik(
                spread_q,
                side,
                wrist[:3, 3] + offset,
                wrist[:3, :3],
            )
    arm_ids = robot.arm_indices["left"] + robot.arm_indices["right"]
    count = round(start * sim.config.fps) + 1
    frames = payload["frames"][:count]
    metrics = json.loads((a.source / "metrics.json").read_text())["metrics"][:count]
    rows = []
    (a.output / "transfer_plan.json").write_text(json.dumps(plan, indent=2))

    def save():
        export_bimanual(sim, frames, metrics, a.output)
        (a.output / "joint_commands.json").write_text(
            json.dumps(
                dict(
                    times=motion.times,
                    targets=[q.tolist() for q in motion.targets],
                    phases=motion.phases,
                )
            )
        )
        (a.output / "feedback_report.json").write_text(
            json.dumps(
                dict(
                    source=str(a.source),
                    checkpoint=str(a.checkpoint),
                    nominal=str(a.nominal or a.nominal_route),
                    retained_strands=retained,
                    start=start,
                    end=sim.t,
                    max_joint_speed_rad_s=a.speed,
                    margin_m=a.margin,
                    prediction_frames=a.predict,
                    contact_weight=a.contact_weight,
                    pose_weight=a.pose_weight,
                    active_sides=sides,
                    pickup_sides=capture_sides,
                    observed_sides=list(plan),
                    retained_strands_by_side=retained_by_side,
                    pose_weights_by_side=pose_weights,
                    seated_strands_by_side=seated_strands,
                    scripted_support_joints=a.scripted_support_joints,
                    seat_depth_m=a.seat_depth,
                    scripted_wrist_spread_m=a.wrist_spread,
                    scripted_right_wrist_offset_m=a.right_wrist_offset,
                    spread_duration_s=a.spread_duration,
                    physics_config=asdict(sim.config),
                    scope="Joint-only feedback experiment; contact does not certify topology",
                    samples=rows,
                ),
                indent=2,
            )
        )

    for frame in range(count, count + round(a.duration * sim.config.fps)):
        if reference_motion is not None:
            nominal = reference_motion.sample(frame / sim.config.fps)[0]
        base_q = scripted_support_target(
            sim.q, nominal, support_ids, 1 / sim.config.fps, a.speed
        )
        u = np.clip((frame / sim.config.fps - start) / a.spread_duration, 0, 1)
        u = u**3 * (10 + u * (-15 + 6 * u))
        base_q[arm_ids] = (1 - u) * starting_q[arm_ids] + u * spread_q[arm_ids]
        try:
            q, cost, evaluations = feedback_target(
                robot,
                base_q,
                sim.state.particle_q.numpy(),
                sim.state.particle_qd.numpy(),
                plan,
                nominal,
                1 / sim.config.fps,
                a.speed,
                a.margin,
                a.predict,
                a.contact_weight,
                a.seat_depth,
                sides,
                a.pose_weight,
                retained,
                retained_by_side,
                pose_weights,
                seated_strands,
            )
        except Exception:
            save()
            raise
        joint_step = float(np.max(abs(q - sim.q)))
        if any(
            hand_clearance(robot.fk((1 - u) * sim.q + u * q))[0] < -0.0005
            or any(
                self_clearance_margin(robot.fk((1 - u) * sim.q + u * q), side) < 0
                for side in ("left", "right")
            )
            for u in np.linspace(0, 1, 5)
        ):
            save()
            raise RuntimeError("Scripted wrist/feedback step crosses the opposing hand")
        motion.append(
            frame / sim.config.fps,
            q,
            "Approach far strand with near-string feedback"
            if a.retain_old_only
            else "Track far strands while clearing near strands",
        )
        sim.advance(frame / sim.config.fps)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        row = dict(
            time=sim.t,
            cost=cost,
            evaluations=evaluations,
            stretch=metrics[-1]["max_stretch_ratio"],
            maximum_joint_step_rad=joint_step,
            scripted_support_lag_rad=float(
                np.max(abs(base_q[support_ids] - nominal[support_ids]))
            )
            if support_ids
            else 0.0,
            thumb_pair_clearance_m=float(thumb_pair_clearances(sim.transforms).min()),
        )
        rope = sim.state.particle_q.numpy()
        if (
            not np.isfinite(rope).all()
            or np.linalg.norm(np.roll(rope, -1, axis=0) - rope, axis=1).max()
            >= MAX_SEGMENT_LENGTH
        ):
            save()
            raise RuntimeError("Feedback exceeded the valid rope geometry range")
        for side in plan:
            row[side] = {
                name: float(
                    gaps(sim.transforms, rope[plan[side][name]], side, "thumb").min()
                )
                for name in ("far_indices", "near_indices", "old_loop_indices")
            }
        rows.append(row)
        if a.stop_on_near_contact and any(
            row[side]["near_indices"] < 0.0015 for side in sides
        ):
            save()
            raise RuntimeError("Approach contacted the forbidden near strand")
        if frame % 15 == 0:
            print(row, flush=True)
            save()
            sim.save_checkpoint(a.output / f"state_{frame:04d}.npz")
    save()


if __name__ == "__main__":
    main()
