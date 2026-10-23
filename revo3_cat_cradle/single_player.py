"""Scripted Position 1 poses and cross-hand fingertip targeting for Opening A."""

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from .bimanual import resample_loop
from .contact_geometry import finger_capsule


class Motion:
    def __init__(self, robot, upright=False, symmetric=False):
        self.hand = robot
        self.times = [0.0]
        self.targets = []
        self.phases = ["Position 1"]
        self.metadata = dict(
            source="Single-player Position 1 and Opening A",
            direction="双手",
            robot="RevoTron",
            reference="https://www.alysion.org/figures/introduc.htm",
        )
        q = np.zeros(len(robot.names))
        for side in ("left", "right"):
            for finger in ("middle", "ring"):
                for j, v in [("MCP", 0.0), ("PIP", 0.0), ("DIP", 0.0)]:
                    q[robot.indices[f"{side}_{finger}_{j}_joint"]] = v
            for j, v in [("MCP", 1.58), ("PIP", 1.5), ("DIP", 1.5)]:
                q[robot.indices[f"{side}_index_{j}_joint"]] = (
                    v if side == "right" else 0
                )
            for j, v in [
                ("CMP", 0.1),
                ("CMR", 1.1),
                ("MCP", 1.1),
                ("PIP", 0.3),
                ("DIP", 0.2),
            ]:
                q[robot.indices[f"{side}_thumb_{j}_joint"]] = v
            for j, v in [("MCP", 0.7), ("PIP", 0.0), ("DIP", 0.0)]:
                q[robot.indices[f"{side}_little_{j}_joint"]] = v
        for side in ("left", "right"):
            for finger, angle in [
                ("index", 0.35),
                ("middle", 0.4),
                ("ring", -0.4),
                ("little", -0.2),
            ]:
                q[robot.indices[f"{side}_{finger}_MPR_joint"]] = angle
        if upright:
            for side in ("left", "right") if symmetric else ("right",):
                for j, v in [("CMP", 0.5), ("CMR", 1.3), ("MCP", 0.9)]:
                    q[robot.indices[f"{side}_thumb_{j}_joint"]] = v
                for j, v in [("MPR", 0.25), ("MCP", 0.8), ("PIP", 0.0), ("DIP", 0.0)]:
                    q[robot.indices[f"{side}_index_{j}_joint"]] = v
                q[robot.indices[f"{side}_middle_MPR_joint"]] = 0
                q[robot.indices[f"{side}_ring_MPR_joint"]] = 0
        self.rotations = {
            side: Rotation.from_euler("z", sign * 90, degrees=True).as_matrix()
            for side, sign in [("left", -1), ("right", 1)]
        }
        for side, sign in [("left", 1), ("right", -1)]:
            q = robot.ik(
                q, side, np.array([0.4, sign * 0.15, 1.13]), self.rotations[side]
            )
        self.targets = [q.copy()]

    def append(self, time, q, phase):
        assert time > self.times[-1]
        self.times.append(float(time))
        self.targets.append(q.copy())
        self.phases.append(phase)
        self.metadata["duration"] = self.times[-1]

    def sample(self, t):
        if t >= self.times[-1]:
            return self.targets[-1].copy(), self.phases[-1]
        i = max(0, np.searchsorted(self.times, t, side="right") - 1)
        u = np.clip((t - self.times[i]) / (self.times[i + 1] - self.times[i]), 0, 1)
        u = u**3 * (10 + u * (-15 + 6 * u))
        return (1 - u) * self.targets[i] + u * self.targets[i + 1], self.phases[i + 1]

    def finger(self, q, side, finger, values):
        q = q.copy()
        for seg, value in zip(("MCP", "PIP", "DIP"), values):
            q[self.hand.indices[f"{side}_{finger}_{seg}_joint"]] = value
        return q

    def aim_center(self, q, side, finger, target):
        tf = self.hand.fk(q)
        base = tf[f"{side}_hand_base_link"]
        c, axis, r, h = finger_capsule(finger, "DIP", side)
        point = (tf[f"{side}_{finger}_DIP_link"] @ np.r_[c, 1])[:3]
        local = (point - base[:3, 3]) @ base[:3, :3]
        rot = self.rotations[side]
        return self.hand.ik(q, side, np.asarray(target) - rot @ local, rot)

    def aim_tip(self, q, side, finger, target):
        tf = self.hand.fk(q)
        base = tf[f"{side}_hand_base_link"]
        c, axis, r, h = finger_capsule(finger, "DIP", side)
        point = (tf[f"{side}_{finger}_DIP_link"] @ np.r_[c + axis * (h + r), 1])[:3]
        local = (point - base[:3, 3]) @ base[:3, :3]
        rot = self.rotations[side]
        return self.hand.ik(q, side, np.asarray(target) - rot @ local, rot)


def initial_loop(robot, motion):
    tf = robot.fk(motion.targets[0])
    centers = []
    radii = []
    for side in ("left", "right"):
        for finger in ("thumb", "little"):
            c, axis, r, h = finger_capsule(finger, "DIP", side)
            centers.append((tf[f"{side}_{finger}_DIP_link"] @ np.r_[c, 1])[:3])
            radii.append(r + 0.0014)
    centers = np.array(centers)
    origin = centers.mean(axis=0)
    u = np.array([0, 1, 0])
    v = centers[1] - centers[0]
    v[1] = 0
    v /= np.linalg.norm(v)
    cloud = np.array(
        [
            c + r * (u * np.cos(a) + v * np.sin(a))
            for c, r in zip(centers, radii)
            for a in np.linspace(0, 2 * np.pi, 60, endpoint=False)
        ]
    )
    hull = ConvexHull((cloud - origin) @ np.column_stack((u, v)))
    return resample_loop(cloud[hull.vertices], 0.003)
