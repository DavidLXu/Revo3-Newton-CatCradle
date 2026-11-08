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


def audit_frame(robot, joints, points, rope_radius, *, edge_indices=None):
    """Inspect a closed rope, optionally restricting the material edges.

    Region names describe the closest point only; even shaft contact cannot
    certify a secured bight or a completed string figure.
    """
    selected = np.arange(len(points)) if edge_indices is None else np.asarray(edge_indices)
    if (selected.ndim != 1 or not len(selected)
            or not np.issubdtype(selected.dtype, np.integer)
            or np.any(selected < 0) or np.any(selected >= len(points))):
        raise ValueError("Select existing material edge indices")
    selected_nodes = np.unique(np.r_[selected, (selected + 1) % len(points)])
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
                i = int(selected[gaps[selected].argmin()])
                closest_rope = points[i] + s[i] * u[i]
                axial = float((closest_rope - (a + b) / 2) @ (tf[:3, :3] @ axis))
                region = (
                    "negative_cap" if axial < -half else
                    "positive_cap" if axial > half else "shaft"
                )
                rows.append(
                    dict(
                        finger=name,
                        edge=i,
                        edge_fraction=float(s[i]),
                        edge_length_m=float(np.sqrt(aa[i])),
                        edge_gap_m=float(gaps[i]),
                        minimum_particle_gap_m=float(node_gaps[selected_nodes].min()),
                        rope_axial_m=axial,
                        capsule_half_length_m=float(half),
                        closest_region=region,
                    )
                )
    return sorted(rows, key=lambda item: item["edge_gap_m"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--time", type=float, nargs="*", help="Defaults to final frame")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--material-range", type=int, nargs=2, metavar=("START", "END"),
                        help="Inspect only edges START <= i < END, preserving material IDs")
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        times, joints, rope = (archive[key] for key in ("time", "joints", "rope"))
    payload = json.loads((args.source / "trajectory.json").read_text())
    selected = None
    if args.material_range is not None:
        start, end = args.material_range
        if not 0 <= start < end <= rope.shape[1]:
            parser.error("Material range must select existing rope edges")
        selected = np.arange(start, end)
    robot = RevoTron()
    report = dict(
        scope="Discrete edge/capsule gaps and contact location; not a pickup certificate. "
              "Excludes palm, meshes and swept collisions",
        material_edge_range=args.material_range,
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
                    robot, joints[i], rope[i], payload["config"]["radius"],
                    edge_indices=selected,
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
