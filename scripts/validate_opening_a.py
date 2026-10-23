"""Validate cross-hand strand pickup and threading, rather than a projected X."""

import argparse, json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.contact_geometry import finger_capsule
from revo3_cat_cradle.bimanual_checks import (
    gaps,
    segment_separation,
    triangle_coordinates,
)
from revo3_cat_cradle.bimanual_checks import palmar_arc
from revo3_cat_cradle.bimanual_checks import hand_clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_opening_a"
    )
    args = parser.parse_args()
    out = args.output
    d = np.load(out / "trajectory.npz")
    payload = json.loads((out / "trajectory.json").read_text())
    search_bound = float(payload.get("self_contact_search_bound_m", 0.008))
    r = RevoTron()
    times = d["time"]
    tf = [r.fk(q) for q in d["joints"]]
    arcs = {
        s: palmar_arc(r, d["joints"][0], d["rope"][0], s) for s in ("left", "right")
    }
    contacts = {}
    for side in ("left", "right"):
        contacts[side] = {
            f: np.array([gaps(t, p, side, f).min() for t, p in zip(tf, d["rope"])])
            for f in ("thumb", "index", "little")
        }
    foreign = {
        s: np.array(
            [gaps(t, p[arcs[other]], s, "index").min() for t, p in zip(tf, d["rope"])]
        )
        for s, other in [("left", "right"), ("right", "left")]
    }
    first = (times >= 8) & (times <= 10)
    last = times >= times[-1] - 1
    pick_up = {}
    for side, mask in [("right", first), ("left", last)]:
        ids = arcs["left" if side == "right" else "right"]
        k = int(np.flatnonzero(mask)[-1])
        g = gaps(tf[k], d["rope"][k, ids], side, "index")
        pick_up[side] = dict(
            source_hand="left" if side == "right" else "right",
            source_arc_indices=ids.tolist(),
            final_contact_material_indices=ids[g < 0.0015].tolist(),
            initial_source_distance_m=float(foreign[side][0]),
            worst_contact_gap_during_hold_m=float(foreign[side][mask].max()),
            passed=bool(
                foreign[side][0] > 0.1 and np.max(foreign[side][mask]) < 0.0015
            ),
        )
    thread = []
    c, axis, rad, half = finger_capsule("index", "DIP", "left")
    for k in np.flatnonzero((times >= 11.5) & (times <= 13)):
        p = d["rope"][k]
        t = tf[k]
        a = p[gaps(t, p, "left", "thumb") < 0.0015]
        b = p[gaps(t, p, "left", "little") < 0.0015]
        ids = arcs["left"]
        other = p[ids[gaps(t, p[ids], "right", "index") < 0.0015]]
        if not (len(a) and len(b) and len(other)):
            continue
        triangle = np.array([a.mean(0), b.mean(0), other.mean(0)])
        center = (t["left_index_DIP_link"] @ np.r_[c, 1])[:3]
        result = triangle_coordinates(center, triangle)
        if result is not None:
            thread.append((k, result[0], result[1]))
    crossings = []
    for before, after in zip(thread, thread[1:]):
        if before[1] * after[1] < 0:
            fraction = abs(before[1]) / (abs(before[1]) + abs(after[1]))
            weights = (1 - fraction) * before[2] + fraction * after[2]
            if weights.min() > 0.03:
                crossings.append(
                    dict(
                        time_s=float(
                            (1 - fraction) * times[before[0]]
                            + fraction * times[after[0]]
                        ),
                        barycentric_weights=weights.tolist(),
                    )
                )
    separation = np.array([segment_separation(p) for p in d["rope"]])
    edges = np.linalg.norm(np.roll(d["rope"], -1, axis=1) - d["rope"], axis=2)
    opposing = np.array([hand_clearance(t)[0] for t in tf])
    fingers = ("thumb", "index", "middle", "ring", "little")
    self_margin = []
    self_worst = []
    for t in tf:
        for side in ("left", "right"):
            pairs = hand_clearance(t, side, all_pairs=True)
            self_worst.append(pairs[0][0])
            for gap, a, b in pairs:
                adjacent = (
                    abs(fingers.index(a.split("_")[0]) - fingers.index(b.split("_")[0]))
                    == 1
                )
                # Adjacent MCP proxies share a small volume at the joint bases.
                tolerance = (
                    0.003
                    if adjacent and a.endswith("MCP") and b.endswith("MCP")
                    else 0.001
                )
                self_margin.append(gap + tolerance)
    arm = r.arm_indices["left"] + r.arm_indices["right"]
    speed = np.max(abs(np.diff(d["joints"][:, arm], axis=0) / np.diff(times)[:, None]))
    support = {
        s: {f: float(contacts[s][f][last].max()) for f in ("thumb", "little")}
        for s in ("left", "right")
    }
    checks = dict(
        completed=bool(times[-1] >= 21),
        finite=bool(np.isfinite(d["rope"]).all() and abs(d["rope"]).max() < 3),
        all_particles_dynamic=bool(
            np.all(d["particle_masses"] > 0) and not len(d["anchors"])
        ),
        joint_limits=bool(
            np.all(d["joints"] >= r.limits[:, 0] - 1e-6)
            and np.all(d["joints"] <= r.limits[:, 1] + 1e-6)
        ),
        arm_speed=bool(speed < np.pi),
        right_cross_hand_pickup=pick_up["right"]["passed"],
        left_cross_hand_pickup=pick_up["left"]["passed"],
        left_index_threads_right_bight=bool(len(crossings) == 1),
        both_index_bights_retained=bool(
            all(foreign[s][last].max() < 0.0015 for s in foreign)
        ),
        four_original_supports_retained=bool(
            all(v < 0.0015 for x in support.values() for v in x.values())
        ),
        rope_self_contact=bool(
            separation.min() >= 0.0018 and edges.max() < search_bound
        ),
        strain=bool((edges / d["rest_lengths"]).max() < 1.2),
        opposing_hands_clear=bool(opposing.min() >= -0.001),
        finger_proxies_clear=bool(min(self_margin) >= 0),
    )
    report = dict(
        passed=all(checks.values()),
        checks=checks,
        pickup=pick_up,
        threading_crossings=crossings,
        final_support_gaps_m=support,
        minimum_segment_distance_m=float(separation.min()),
        maximum_edge_length_m=float(edges.max()),
        self_contact_search_bound_m=search_bound,
        maximum_stretch_ratio=float((edges / d["rest_lengths"]).max()),
        minimum_opposing_hand_gap_m=float(opposing.min()),
        minimum_self_capsule_gap_m=float(min(self_worst)),
        maximum_arm_speed_rad_s=float(speed),
        duration_s=float(times[-1]),
        physics="Newton XPBD, contact and friction, active rest-length control",
    )
    (out / "validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
