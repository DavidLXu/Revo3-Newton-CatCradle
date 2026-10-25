"""Generate joint-only hand separation, keeping wrist orientations and finger angles."""

import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import hand_clearance


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--distance",
        type=float,
        default=0.02,
        help="Total added distance between wrist origins, meters",
    )
    p.add_argument("--duration", type=float, default=3)
    p.add_argument("--steps", type=int, default=6)
    a = p.parse_args()
    if (
        not np.isfinite([a.distance, a.duration]).all()
        or a.distance < 0
        or a.duration <= 0
        or a.steps < 1
    ):
        p.error("Invalid distance, duration or steps")
    robot = RevoTron()
    state = np.load(a.source / "checkpoint.npz")
    q = state["joints"].copy()
    original = q.copy()
    wrists = {s: robot.wrist(q, s) for s in ("left", "right")}
    vector = wrists["left"][:3, 3] - wrists["right"][:3, 3]
    distance = np.linalg.norm(vector)
    if distance < 1e-6:
        raise ValueError("Cannot infer separation direction from coincident wrists")
    direction = vector / distance
    fps = json.loads(str(state["config"]))["fps"]
    frames = np.rint(np.linspace(0, a.duration * fps, a.steps + 1)).astype(int)
    if np.any(np.diff(frames) <= 0):
        raise ValueError("Every waypoint requires one output frame")
    poses = [q]
    for u in np.linspace(0, 1, a.steps + 1)[1:]:
        for side, sign in [("left", 1), ("right", -1)]:
            w = wrists[side]
            target = w[:3, 3] + sign * u * a.distance / 2 * direction
            q = robot.ik(q, side, target, w[:3, :3])
            actual = robot.wrist(q, side)
            if (
                np.linalg.norm(actual[:3, 3] - target) > 0.0005
                or np.linalg.norm(
                    Rotation.from_matrix(actual[:3, :3] @ w[:3, :3].T).as_rotvec()
                )
                > 0.002
            ):
                raise ValueError("Wrist pose target missed")
        poses.append(q.copy())
    clearance = min(
        hand_clearance(robot.fk(x + u * (y - x)))[0]
        for x, y in zip(poses, poses[1:])
        for u in np.linspace(0, 1, 9)
    )
    if clearance < -0.0005:
        raise ValueError("Hand spread crosses opposing fingers")
    arms = robot.arm_indices["left"] + robot.arm_indices["right"]
    fixed = [i for i in range(len(q)) if i not in arms]
    assert all(np.array_equal(v[fixed], original[fixed]) for v in poses)
    a.output.write_text(
        json.dumps(
            [
                dict(
                    joints=v.tolist(),
                    duration=float(frames[i + 1] - frames[i]) / fps,
                    phase=f"Separate wrists by {1000 * a.distance:.0f} mm with fingers unchanged",
                )
                for i, v in enumerate(poses[1:])
            ],
            indent=2,
        )
    )
    report = dict(
        initial_wrist_distance_m=float(distance),
        added_distance_m=a.distance,
        final_wrist_distance_m=float(
            np.linalg.norm(
                robot.wrist(q, "left")[:3, 3] - robot.wrist(q, "right")[:3, 3]
            )
        ),
        direction=direction.tolist(),
        min_opposing_finger_gap_m=clearance,
        scope="Robot-only proposal; continuous physics and support audits required",
    )
    a.output.with_suffix(".diagnostics.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
