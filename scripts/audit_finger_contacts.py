"""Compare rope-edge and particle gaps to all finger capsules at saved frames."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.contact_geometry import finger_capsule


def audit_frame(robot, joints, points, rope_radius):
    poses = robot.fk(joints)
    u = np.roll(points, -1, axis=0) - points
    aa = np.einsum("ij,ij->i", u, u)
    rows = []
    for side in ("left", "right"):
        for finger in ("thumb", "index", "middle", "ring", "little"):
            for segment in ("MCP", "PIP", "DIP"):
                name = f"{side}_{finger}_{segment}_link"
                center, axis, radius, half = finger_capsule(finger, segment, side)
                tf = poses[name]
                a, b = (
                    np.array([center - axis * half, center + axis * half])
                    @ tf[:3, :3].T
                    + tf[:3, 3]
                )
                v = b - a
                w = points - a
                bb, cc, dd, ee = u @ v, v @ v, np.einsum("ij,ij->i", u, w), w @ v
                den = aa * cc - bb * bb
                s = np.clip(
                    np.divide(
                        bb * ee - cc * dd,
                        den,
                        out=np.zeros_like(den),
                        where=den > 1e-16,
                    ),
                    0,
                    1,
                )
                t = (bb * s + ee) / max(cc, 1e-16)
                s = np.where(t < 0, np.clip(-dd / np.maximum(aa, 1e-16), 0, 1), s)
                s = np.where(t > 1, np.clip((bb - dd) / np.maximum(aa, 1e-16), 0, 1), s)
                t = np.clip(t, 0, 1)
                if cc < 1e-16:
                    s = np.clip(-dd / np.maximum(aa, 1e-16), 0, 1)
                gaps = (
                    np.linalg.norm(w + s[:, None] * u - t[:, None] * v, axis=1)
                    - radius
                    - rope_radius
                )
                node_gaps = (
                    np.linalg.norm(
                        w - np.clip(ee / max(cc, 1e-16), 0, 1)[:, None] * v, axis=1
                    )
                    - radius
                    - rope_radius
                )
                i = int(gaps.argmin())
                rows.append(
                    dict(
                        finger=name,
                        edge=i,
                        edge_fraction=float(s[i]),
                        edge_length_m=float(np.sqrt(aa[i])),
                        edge_gap_m=float(gaps[i]),
                        minimum_particle_gap_m=float(node_gaps.min()),
                    )
                )
    return sorted(rows, key=lambda item: item["edge_gap_m"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--time", type=float, nargs="*", help="Defaults to final frame")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[key] for key in ("time", "joints", "rope"))
    payload = json.loads((args.source / "trajectory.json").read_text())
    robot = RevoTron()
    report = dict(
        scope="Discrete edge/capsule gaps; excludes palm, meshes and swept collisions",
        frames=[],
    )
    for requested in args.time or [times[-1]]:
        i = int(np.abs(times - requested).argmin())
        if abs(times[i] - requested) > 1e-5:
            parser.error(f"No saved frame at {requested}")
        report["frames"].append(
            dict(
                time_s=float(times[i]),
                contacts=audit_frame(
                    robot, joints[i], rope[i], payload["config"]["radius"]
                ),
            )
        )
    output = args.output or args.source / "finger_contact_audit.json"
    output.write_text(json.dumps(report, indent=2))
    for frame in report["frames"]:
        print(
            json.dumps(
                dict(time_s=frame["time_s"], worst_contacts=frame["contacts"][:5])
            )
        )


if __name__ == "__main__":
    main()
