"""Two hands turn a shared rectangle into a crossed string figure."""

import sys, json, argparse
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, resample_loop
from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.sim import Config
from revo3_cat_cradle.bimanual import export_bimanual as export


class CrossMotion:
    def __init__(self, robot):
        self.hand = robot
        self.duration = 17.0
        self.times = np.array([0, 2, 5, 8, 10, 13, 15, 17])
        self.targets = []
        self.metadata = {
            "source": "Bimanual wrist turn and hooked finger supports",
            "direction": "双手",
            "duration": 17,
            "robot": "RevoTron",
            "stages": [
                "Seat loop",
                "Turn wrist",
                "Cross strands",
                "Hold crossed loop",
                "Pick up with middle fingers",
                "Spread",
                "Hold",
            ],
        }
        q = np.zeros(len(robot.names))
        for side in ["left", "right"]:
            for f in ["index", "little"]:
                for seg, v in [("MCP", 0.30), ("PIP", 1.2), ("DIP", 1.0)]:
                    q[robot.indices[f"{side}_{f}_{seg}_joint"]] = v
            for f in ["middle", "ring"]:
                for seg, v in [("MCP", 1.4), ("PIP", 1.4), ("DIP", 1.4)]:
                    q[robot.indices[f"{side}_{f}_{seg}_joint"]] = v
            for n in robot.names:
                if n.startswith(side + "_thumb"):
                    q[robot.indices[n]] = min(1.2, robot.limits[robot.indices[n], 1])
        # Track the mean support position as the wrist rolls, so the arm supplies
        # the small compensating translation rather than rotating about a free hand.
        origins = {}
        for side in ["left", "right"]:
            tf = robot.fk(q)
            base = tf[f"{side}_hand_base_link"]
            centers = []
            for f in ["index", "little"]:
                c, ax, rad, h = finger_capsule(f, "PIP", side)
                p = (tf[f"{side}_{f}_PIP_link"] @ np.r_[c, 1])[:3]
                centers.append((p - base[:3, 3]) @ base[:3, :3])
            origins[side] = np.mean(centers, axis=0)
        for angle, distance, x in [
            (0, 0.14, 0.47),
            (0, 0.14, 0.47),
            (90, 0.14, 0.47),
            (180, 0.14, 0.47),
            (180, 0.14, 0.47),
            (180, 0.14, 0.47),
            (180, 0.20, 0.47),
            (180, 0.20, 0.47),
        ]:
            for side, sign in [("left", 1), ("right", -1)]:
                rot = (
                    Rotation.from_euler("x", 90, degrees=True).as_matrix()
                    if side == "left"
                    else Rotation.from_euler("y", angle, degrees=True).as_matrix()
                    @ Rotation.from_euler("x", -90, degrees=True).as_matrix()
                )
                center = np.array(
                    [x if side == "right" else 0.47, sign * distance, 1.20]
                )
                q = robot.ik(q, side, center - rot @ origins[side], rot)
            if len(self.targets) >= 5:
                for side in ["left", "right"]:
                    for seg, v in [("MCP", 0.05), ("PIP", 0.3), ("DIP", 0.7)]:
                        q[robot.indices[f"{side}_middle_{seg}_joint"]] = v
            self.targets.append(q.copy())
        self.targets = np.array(self.targets)

    def sample(self, t):
        i = min(
            max(np.searchsorted(self.times, t, side="right") - 1, 0),
            len(self.times) - 2,
        )
        u = np.clip((t - self.times[i]) / (self.times[i + 1] - self.times[i]), 0, 1)
        u = u * u * u * (10 + u * (-15 + 6 * u))
        return (1 - u) * self.targets[i] + u * self.targets[i + 1], self.metadata[
            "stages"
        ][i]


def loop(robot, motion):
    tf = robot.fk(motion.sample(0)[0])
    points = []
    centers = []
    for side in ["left", "right"]:
        for f in ["index", "little"]:
            c, ax, rad, h = finger_capsule(f, "PIP", side)
            p = (tf[f"{side}_{f}_PIP_link"] @ np.r_[c, 1])[:3]
            centers.append(p)
            for theta in np.linspace(0, 2 * np.pi, 60, endpoint=False):
                points.append(
                    p + np.array([0, np.cos(theta), np.sin(theta)]) * (rad + 0.0013)
                )
    points = np.array(points)
    hull = ConvexHull(points[:, 1:])
    p = points[hull.vertices]
    p[:, 0] = np.mean(np.array(centers)[:, 0])
    return resample_loop(p, 0.003)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "output/revotron_bimanual")
    p.add_argument("--duration", type=float, default=17)
    p.add_argument("--resume", action="store_true")
    a = p.parse_args()
    if not 0 <= a.duration <= 17:
        p.error("--duration must be between 0 and 17 seconds")
    r = RevoTron()
    m = CrossMotion(r)
    sim = BimanualSimulation(
        r, m, loop(r, m), Config(substeps=128, iterations=48, adaptive_take_up_m=0.003)
    )
    if a.resume:
        sim.load_checkpoint(a.output / "checkpoint.npz")
        frames = json.loads((a.output / "trajectory.json").read_text())["frames"]
        metrics = json.loads((a.output / "metrics.json").read_text())["metrics"]
    else:
        for i in range(60):
            sim.advance(0, move=False)
        frames = [sim.snapshot()]
        metrics = [sim.metrics()]
        export(sim, frames, metrics, a.output)
    for i in range(round(sim.t * 30) + 1, round(a.duration * 30) + 1):
        sim.advance(i / 30)
        frames.append(sim.snapshot())
        metrics.append(sim.metrics())
        if i % 30 == 0:
            export(sim, frames, metrics, a.output)
            sim.save_checkpoint(a.output / f"state_{i:04d}.npz")
            print(sim.t, sim.phase, metrics[-1]["max_stretch_ratio"], flush=True)
    export(sim, frames, metrics, a.output)


if __name__ == "__main__":
    main()
