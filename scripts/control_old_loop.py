"""Lift a measured old-loop contact using only right-middle joint commands.

The moving target belongs to the finger surface, never to a rope particle.
Newton alone advances the rope, including its friction, contact and velocity.
This experimental continuation does not certify a completed old-loop release.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, export_bimanual
from revo3_cat_cradle.bimanual_checks import gaps, hand_clearance, digit_clearances
from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.single_player import Motion, initial_loop
from revo3_cat_cradle.sim import Config


def nearest_helper_surface(transforms, point, side="right", finger="middle"):
    best = None
    for segment in ("MCP", "PIP", "DIP"):
        c, axis, radius, half = finger_capsule(finger, segment, side)
        name = f"{side}_{finger}_{segment}_link"
        tf = transforms[name]
        local = (point - tf[:3, 3]) @ tf[:3, :3]
        center = c + np.clip((local - c) @ axis, -half, half) * axis
        direction = local - center
        distance = np.linalg.norm(direction)
        if distance < 1e-10:
            raise ValueError("Contact point lies on capsule axis")
        candidate = (distance - radius, name, center + radius * direction / distance)
        if best is None or candidate[0] < best[0]:
            best = candidate
    return best[1], best[2]


def surface_position(robot, q, link, local):
    return (robot.fk(q)[link] @ np.r_[local, 1])[:3]


def lift_target(
    robot, q, rope, old_ids, obstacles, link, local, goal, dt, speed, curl=None
):
    ids = [i for i, name in enumerate(robot.names) if name.startswith("right_middle_")]
    step = speed * dt / 1.875
    lower = np.maximum(robot.limits[ids, 0], q[ids] - step)
    upper = np.minimum(robot.limits[ids, 1], q[ids] + step)

    def residual(x):
        trial = q.copy()
        trial[ids] = x
        tf = robot.fk(trial)
        surface = (tf[link] @ np.r_[local, 1])[:3]
        old = gaps(tf, rope[old_ids], "right", "middle").min()
        result = np.r_[
            2 * (surface - goal),
            0.0002 * (x - q[ids]),
            20 * np.maximum(0.001 - gaps(tf, obstacles, "right", "middle"), 0),
            20 * np.maximum(0.002 - digit_clearances(tf, "right", "middle"), 0),
            5 * max(old - 0.0003, 0),
            5 * max(-old - 0.0003, 0),
        ]
        if curl is not None:
            result = np.r_[
                result, 0.01 * (trial[robot.indices["right_middle_DIP_joint"]] - curl)
            ]
        return result

    fit = least_squares(
        residual,
        q[ids],
        bounds=(lower, upper),
        max_nfev=70,
        ftol=1e-8,
        xtol=1e-8,
        gtol=1e-10,
    )
    target = q.copy()
    target[ids] = fit.x
    for _ in range(10):
        valid = True
        for u in (0.25, 0.5, 0.75, 1):
            tf = robot.fk((1 - u) * q + u * target)
            own = min(
                g
                for g, a, b in hand_clearance(tf, "right", all_pairs=True)
                if a.startswith("middle_") or b.startswith("middle_")
            )
            if hand_clearance(tf)[0] < -0.0005 or own < -0.0005:
                valid = False
                break
        if valid:
            return target, float(fit.cost), False
        target = (q + target) / 2
    return q.copy(), float(fit.cost), True


def retain_right_thumb_target(robot, q, rope, material, dt, speed, surface_goal=None):
    """Keep the original right-thumb support while its wrist carries a helper.

    The material window belongs to the original support, excluding a separate
    return-strand graze. This changes only thumb joints, never particle state.
    """
    ids = [i for i, name in enumerate(robot.names) if name.startswith("right_thumb_")]
    step = speed * dt / 1.875
    lower = np.maximum(robot.limits[ids, 0], q[ids] - step)
    upper = np.minimum(robot.limits[ids, 1], q[ids] + step)

    def residual(x):
        trial = q.copy()
        trial[ids] = x
        tf = robot.fk(trial)
        contact = gaps(tf, rope[material], "right", "thumb").min()
        center, axis, radius, half = finger_capsule("thumb", "DIP", "right")
        t = tf["right_thumb_DIP_link"]
        local = (rope[material] - t[:3, 3]) @ t[:3, :3] - center
        axial = local @ axis
        distance = (
            np.linalg.norm(local - np.clip(axial, -half, half)[:, None] * axis, axis=1)
            - radius
        )
        nearest = int(np.argmin(distance))
        result = np.r_[
            0.0002 * (x - q[ids]),
            10 * max(contact - 0.0001, 0),
            10 * max(-contact - 0.0002, 0),
            3 * max(axial[nearest] - 0.004, 0) if surface_goal is None else 0,
            20 * np.maximum(0.001 - digit_clearances(tf, "right", "thumb"), 0),
        ]
        if surface_goal is not None:
            link, local_point, world_point = surface_goal
            result = np.r_[
                result, 5 * ((tf[link] @ np.r_[local_point, 1])[:3] - world_point)
            ]
        return result

    fit = least_squares(residual, q[ids], bounds=(lower, upper), max_nfev=60)
    target = q.copy()
    target[ids] = fit.x
    for _ in range(10):
        if all(
            digit_clearances(robot.fk(q + u * (target - q)), "right", "thumb").min()
            >= -0.0005
            for u in (0.25, 0.5, 0.75, 1)
        ):
            return target
        target = (q + target) / 2
    return q.copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material", type=int, default=175)
    parser.add_argument("--offset", type=float, nargs=3, default=[0, 0, 0.015])
    parser.add_argument("--duration", type=float, default=3)
    parser.add_argument("--speed", type=float, default=1.2)
    parser.add_argument(
        "--curl", type=float, help="Optional middle DIP hook angle in radians"
    )
    parser.add_argument("--wrist-offset", type=float, nargs=3, default=[0, 0, 0])
    parser.add_argument("--wrist-duration", type=float, default=3)
    parser.add_argument("--retain-right-thumb", action="store_true")
    parser.add_argument("--hold-right-thumb-surface", action="store_true")
    parser.add_argument("--fixed-helper", action="store_true")
    args = parser.parse_args()
    if (
        min(args.duration, args.speed, args.wrist_duration) <= 0
        or not np.isfinite(args.offset + args.wrist_offset).all()
    ):
        parser.error("Duration/speed must be positive and offset finite")
    args.output.mkdir(parents=True, exist_ok=False)
    payload = json.loads((args.source / "trajectory.json").read_text())
    commands = json.loads((args.source / "joint_commands.json").read_text())
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    saved = np.load(args.source / "checkpoint.npz")
    if args.material not in plan["left"]["old_loop_indices"]:
        raise ValueError("Selected material must belong to the old left thumb loop")
    robot = RevoTron()
    motion = Motion(robot)
    motion.times = commands["times"]
    motion.targets = [np.array(q) for q in commands["targets"]]
    motion.phases = commands["phases"]
    motion.metadata = payload["motion"].copy()
    motion.metadata.update(
        source="Right-middle old-loop lift experiment", status="experimental"
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
    sim.load_checkpoint(args.source / "checkpoint.npz")
    start = sim.t
    starting_q = sim.q.copy()
    wrist_q = starting_q.copy()
    if np.any(args.wrist_offset):
        wrist = robot.wrist(starting_q, "right")
        wrist_q = robot.ik(
            starting_q, "right", wrist[:3, 3] + args.wrist_offset, wrist[:3, :3]
        )
    arm_ids = robot.arm_indices["right"]
    rope = sim.state.particle_q.numpy()
    link, local = nearest_helper_surface(sim.transforms, rope[args.material])
    initial = surface_position(robot, sim.q, link, local)
    count = round(start * sim.config.fps) + 1
    frames = payload["frames"][:count]
    metrics = json.loads((args.source / "metrics.json").read_text())["metrics"][:count]
    selected = np.array([(args.material + i) % len(rope) for i in range(-4, 5)])
    old_ids = np.intersect1d(selected, plan["left"]["old_loop_indices"])
    obstacles = np.union1d(plan["left"]["far_indices"], plan["left"]["near_indices"])
    # Restrict to the original support near the near-strand endpoint. A distant
    # new graze must never silently substitute for the original right thumb loop.
    support_center = plan["left"]["near_indices"][-1]
    thumb_material = (support_center + np.arange(-8, 9)) % len(rope)
    thumb_surface_goal = None
    if args.hold_right_thumb_surface:
        if not args.retain_right_thumb:
            parser.error("--hold-right-thumb-surface requires --retain-right-thumb")
        contact_ids = thumb_material[
            gaps(sim.transforms, rope[thumb_material], "right", "thumb") < 0.0015
        ]
        if not len(contact_ids):
            raise ValueError("Original right thumb support is absent")
        material = int(contact_ids[(len(contact_ids) - 1) // 2])
        support_link, support_local = nearest_helper_surface(
            sim.transforms, rope[material], finger="thumb"
        )
        thumb_surface_goal = (
            support_link,
            support_local,
            surface_position(robot, sim.q, support_link, support_local),
        )
    rows = []
    (args.output / "transfer_plan.json").write_text(json.dumps(plan, indent=2))

    def save():
        export_bimanual(sim, frames, metrics, args.output)
        (args.output / "joint_commands.json").write_text(
            json.dumps(
                dict(
                    times=motion.times,
                    targets=[q.tolist() for q in motion.targets],
                    phases=motion.phases,
                )
            )
        )
        (args.output / "helper_lift_report.json").write_text(
            json.dumps(
                dict(
                    source=str(args.source),
                    start=start,
                    end=sim.t,
                    material=args.material,
                    moving_link=link,
                    local_surface_point=local.tolist(),
                    initial_surface_point=initial.tolist(),
                    requested_offset_m=args.offset,
                    requested_dip_curl_rad=args.curl,
                    wrist_offset_m=args.wrist_offset,
                    wrist_duration_s=args.wrist_duration,
                    retain_right_thumb=args.retain_right_thumb,
                    hold_right_thumb_surface=args.hold_right_thumb_surface,
                    fixed_helper=args.fixed_helper,
                    right_thumb_material=thumb_material.tolist(),
                    selected_old_material=old_ids.tolist(),
                    scope="Joint-only helper motion; release and topology remain unverified",
                    samples=rows,
                ),
                indent=2,
            )
        )

    for frame in range(count, count + round(args.duration * sim.config.fps)):
        t = frame / sim.config.fps
        u = np.clip((t - start) / args.duration, 0, 1)
        u = u**3 * (10 + u * (-15 + 6 * u))
        goal = initial + u * np.array(args.offset)
        rope = sim.state.particle_q.numpy()
        base_q = sim.q.copy()
        wrist_u = np.clip((t - start) / args.wrist_duration, 0, 1)
        wrist_u = wrist_u**3 * (10 + wrist_u * (-15 + 6 * wrist_u))
        base_q[arm_ids] = starting_q[arm_ids] + wrist_u * (
            wrist_q[arm_ids] - starting_q[arm_ids]
        )
        if args.retain_right_thumb:
            base_q = retain_right_thumb_target(
                robot,
                base_q,
                rope,
                thumb_material,
                1 / sim.config.fps,
                args.speed,
                thumb_surface_goal,
            )
        try:
            if args.fixed_helper:
                q, cost, stopped = base_q, 0.0, False
            else:
                q, cost, stopped = lift_target(
                    robot,
                    base_q,
                    rope,
                    old_ids,
                    rope[obstacles],
                    link,
                    local,
                    goal,
                    1 / sim.config.fps,
                    args.speed,
                    args.curl,
                )
        except Exception:
            save()
            raise
        if any(
            hand_clearance(robot.fk(sim.q + u * (q - sim.q)))[0] < -0.0005
            for u in (0.25, 0.5, 0.75, 1)
        ):
            save()
            raise RuntimeError("Combined helper/wrist step crosses the opposite hand")
        motion.append(t, q, "Lift old return strand with right middle")
        sim.advance(t)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        rope = sim.state.particle_q.numpy()
        row = dict(
            time=t,
            cost=cost,
            collision_limited=stopped,
            surface_displacement_m=(
                surface_position(robot, sim.q, link, local) - initial
            ).tolist(),
            strain=metrics[-1]["max_stretch_ratio"],
            right_original_thumb_gap_m=float(
                gaps(sim.transforms, rope[thumb_material], "right", "thumb").min()
            ),
        )
        for name in ("old_loop_indices", "far_indices", "near_indices"):
            points = rope[plan["left"][name]]
            row["helper_" + name] = float(
                gaps(sim.transforms, points, "right", "middle").min()
            )
            row["thumb_" + name] = float(
                gaps(sim.transforms, points, "left", "thumb").min()
            )
        rows.append(row)
        if frame % 15 == 0:
            print(row, flush=True)
            save()
            sim.save_checkpoint(args.output / f"state_{frame:04d}.npz")
    save()


if __name__ == "__main__":
    main()
