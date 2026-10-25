"""Diagnose Opening A support on the collision geometry actually simulated.

Contact is necessary, not sufficient, for a correctly threaded figure. Keep
the independent threading, strand identity and complete-trajectory validator.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.mesh_geometry import (
    load_knuckle_meshes,
    support_gap,
    support_material_distances,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, default=17.2)
    args = parser.parse_args()
    data = np.load(args.source / "trajectory.npz")
    payload = json.loads((args.source / "trajectory.json").read_text())
    robot = RevoTron()
    meshes = (
        load_knuckle_meshes(robot)
        if payload.get("collision_geometry", {}).get("knuckles") == "mesh"
        else {}
    )
    selected = np.flatnonzero(data["time"] >= args.start - 1e-5)
    if not len(selected):
        raise ValueError("No frames in the requested interval")
    tail_start = max(args.start, float(data["time"][-1]) - 1)
    names = [(s, f) for s in ("left", "right") for f in ("thumb", "index", "little")]
    contacts = {s + "_" + f: [] for s, f in names}
    final = {}
    for k in selected:
        tf = robot.fk(data["joints"][k])
        for side, finger in names:
            name = side + "_" + finger
            contacts[name].append(
                support_gap(tf, data["rope"][k], side, finger, meshes)
            )
            if k == selected[-1]:
                distances = support_material_distances(
                    tf, data["rope"][k], side, finger, meshes
                )
                final[name] = dict(
                    gap_m=float(distances.min()),
                    material_indices=np.flatnonzero(distances < 0.0015).tolist(),
                )
    times = data["time"][selected]
    edges = np.linalg.norm(np.roll(data["rope"], -1, axis=1) - data["rope"], axis=2)
    support = {}
    for name, values in contacts.items():
        values = np.asarray(values)
        lost = np.flatnonzero(values >= 0.0015)
        support[name] = dict(
            final=final[name],
            worst_hold_gap_m=float(values[times >= tail_start - 1e-5].max()),
            first_missing_contact_s=float(times[lost[0]]) if len(lost) else None,
        )
    report = dict(
        scope="Contact diagnostics; not a threading, topology or full physics certificate",
        interval_s=[float(times[0]), float(times[-1])],
        simulated_knuckle_meshes=bool(meshes),
        supports=support,
        all_six_contacts_during_final_second=all(
            item["worst_hold_gap_m"] < 0.0015 for item in support.values()
        ),
        interval_peak_stretch=float(
            (edges[selected] / data["rest_lengths"][selected]).max()
        ),
        full_peak_stretch=float((edges / data["rest_lengths"]).max()),
        interval_maximum_edge_m=float(edges[selected].max()),
        self_contact_search_bound_m=payload["self_contact_search_bound_m"],
    )
    (args.source / "opening_support_audit.json").write_text(
        json.dumps(report, indent=2)
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
