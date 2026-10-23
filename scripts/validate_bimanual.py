"""Check this scripted shared-loop figure, including physical finger pickup.

Projected crossings describe this camera view, not a general knot invariant.
"""

import json, sys, argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps, segment_separation


def crossings(points):
    p = points[:, 1:]
    n = len(p)
    hits = []

    def cross(a, b):
        return a[0] * b[1] - a[1] * b[0]

    for i in range(n):
        a = p[i]
        u = p[(i + 1) % n] - a
        for j in range(i + 3, n):
            if min(j - i, n - j + i) <= 2:
                continue
            b = p[j]
            v = p[(j + 1) % n] - b
            den = cross(u, v)
            if abs(den) < 1e-12:
                continue
            t = cross(b - a, v) / den
            s = cross(b - a, u) / den
            if 0 < t < 1 and 0 < s < 1:
                h = a + t * u
                if (
                    abs(h[0]) < 0.10
                    and 1.1 < h[1] < 1.3
                    and all(np.linalg.norm(h - other) > 0.004 for other in hits)
                ):
                    hits.append(h)
    return len(hits)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_bimanual"
    )
    a = parser.parse_args()
    d = np.load(a.output / "trajectory.npz")
    payload = json.loads((a.output / "trajectory.json").read_text())
    r = RevoTron()
    tf = [r.fk(q) for q in d["joints"]]
    contacts = {side: [] for side in ("left", "right")}
    for k, points in enumerate(d["rope"]):
        for side in contacts:
            contacts[side].append(
                [
                    float(gaps(tf[k], points, side, f).min())
                    for f in ("index", "middle", "little")
                ]
            )
    contacts = {s: np.array(v) for s, v in contacts.items()}
    pickup = {}
    before = int(np.argmin(abs(d["time"] - 10)))
    after = int(np.argmin(abs(d["time"] - 13)))
    for side in contacts:
        g = gaps(tf[after], d["rope"][after], side, "middle")
        ids = np.flatnonzero(g < 0.001)
        if len(ids):
            t0 = tf[before][f"{side}_hand_base_link"]
            t1 = tf[after][f"{side}_hand_base_link"]
            p0 = (d["rope"][before, ids] - t0[:3, 3]) @ t0[:3, :3]
            p1 = (d["rope"][after, ids] - t1[:3, 3]) @ t1[:3, :3]
            lift = float(np.median(p1[:, 2] - p0[:, 2]))
            distance_before = float(
                gaps(tf[before], d["rope"][before, ids], side, "middle").min()
            )
        else:
            lift = 0.0
            distance_before = 0.0
        pickup[side] = dict(
            material_indices=ids.tolist(),
            finger_gap_before_m=distance_before,
            median_material_lift_in_hand_frame_m=lift,
            contact_after_m=float(g.min()),
            passed=bool(len(ids) >= 2 and distance_before > 0.001 and lift > 0.01),
        )
    arm = r.arm_indices["left"] + r.arm_indices["right"]
    velocity = np.diff(d["joints"], axis=0) / np.diff(d["time"])[:, None]
    max_velocity = float(abs(velocity[:, arm]).max())
    limit_ok = bool(
        np.all(d["joints"] >= r.limits[:, 0] - 1e-7)
        and np.all(d["joints"] <= r.limits[:, 1] + 1e-7)
    )
    held = all(np.all(np.min(v[:, [0, 2]], axis=1) < 0.0015) for v in contacts.values())
    end_cross = crossings(d["rope"][-1])
    start_cross = crossings(d["rope"][0])
    separations = np.array([segment_separation(points) for points in d["rope"]])
    max_edge = float(
        np.linalg.norm(np.roll(d["rope"], -1, axis=1) - d["rope"], axis=2).max()
    )
    report = dict(
        minimum_nonadjacent_segment_distance_m=float(separations.min()),
        segment_contact_worst_time_s=float(d["time"][separations.argmin()]),
        max_edge_length_m=max_edge,
        completed_seconds=float(d["time"][-1]),
        joint_limits_passed=limit_ok,
        max_arm_speed_rad_s=max_velocity,
        all_particles_dynamic=bool(np.all(d["particle_masses"] > 0)),
        anchors=payload["anchor_indices"],
        both_hands_keep_support_contact=bool(held),
        middle_finger_pickup=pickup,
        start_central_crossings=start_cross,
        final_central_crossings=end_cross,
        final_finger_gaps_m={
            s: dict(zip(("index", "middle", "little"), v[-1].tolist()))
            for s, v in contacts.items()
        },
        max_stretch_ratio=payload["validation"]["max_stretch_ratio"],
        finite=bool(np.isfinite(d["rope"]).all()),
        minimum_rope_height_m=float(d["rope"][:, :, 2].min()),
    )
    report["passed"] = bool(
        report["completed_seconds"] >= 17
        and limit_ok
        and max_velocity < 3.1416
        and held
        and all(x["passed"] for x in pickup.values())
        and start_cross == 0
        and end_cross >= 1
        and report["max_stretch_ratio"] < 1.2
        and report["finite"]
        and report["all_particles_dynamic"]
        and not report["anchors"]
        and separations.min() >= 0.0018
        and max_edge < 0.008
    )
    (a.output / "validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise RuntimeError("Bimanual figure validation did not pass")


if __name__ == "__main__":
    main()
