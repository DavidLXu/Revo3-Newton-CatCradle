"""Audit a static Position 1 pre-roll before continuing into cross-hand pickup.

The stability measurements concern the saved 30 Hz samples. They do not turn
a contact test into a topological proof or hide any initial transient.
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import (
    gaps,
    segment_separation,
    self_clearance_margin,
)
from revo3_cat_cradle.segment_contacts import MAX_SEGMENT_LENGTH


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()
    state = np.load(args.source / "initial.npz")
    data = np.load(args.source / "settling.npz")
    times, rope = data["time"], data["rope"]
    robot = RevoTron()
    tf = robot.fk(state["joints"])
    radius = json.loads(str(state["config"]))["radius"]
    edge = np.linalg.norm(np.roll(rope, -1, axis=1) - rope, axis=2)
    strain = edge / data["rest_lengths"]
    speed = np.linalg.norm(data["velocities"], axis=2)
    tail = times >= times[-1] - 0.5 - 1e-6
    contacts = {}
    for side in ("left", "right"):
        for finger in ("thumb", "little"):
            distance = np.asarray([gaps(tf, points, side, finger) for points in rope])
            minimum = distance.min(axis=1)
            missing = np.flatnonzero(minimum >= 0.0015)
            contacts[side + "_" + finger] = dict(
                final_gap_m=float(minimum[-1]),
                final_material=np.flatnonzero(distance[-1] < 0.0015).tolist(),
                worst_tail_gap_m=float(minimum[tail].max()),
                first_missing_contact_s=float(times[missing[0]])
                if len(missing)
                else None,
            )
    separation = [segment_separation(p, minimum_arc_distance=2 * radius) for p in rope]
    tail_excursion = float(np.linalg.norm(rope[tail] - rope[tail][0], axis=2).max())
    own_margin = min(self_clearance_margin(tf, s) for s in ("left", "right"))
    checks = dict(
        finite=bool(np.isfinite(rope).all() and np.isfinite(speed).all()),
        initial_supports_retained=all(
            v["first_missing_contact_s"] is None for v in contacts.values()
        ),
        supports_in_final_hold=all(
            v["worst_tail_gap_m"] < 0.0015 for v in contacts.values()
        ),
        strain=bool(strain.max() < 1.2),
        self_contact=bool(
            min(separation) >= 0.0018 and edge.max() < MAX_SEGMENT_LENGTH
        ),
        own_fingers_clear=bool(own_margin >= 0),
        final_half_second_position_excursion_under_1mm=tail_excursion < 0.001,
        final_half_second_max_speed_under_1cm_s=bool(speed[tail].max() < 0.01),
    )
    report = dict(
        scope="Static initialization diagnostics at recorded frames; not a full figure certificate",
        ready_for_pickup=all(checks.values()),
        checks=checks,
        settling_duration_s=float(times[-1]),
        rest_length_m=float(data["rest_lengths"][-1].sum()),
        contacts=contacts,
        peak_stretch=float(strain.max()),
        tail_peak_stretch=float(strain[tail].max()),
        maximum_edge_m=float(edge.max()),
        minimum_nonlocal_separation_m=float(min(separation)),
        minimum_own_finger_margin_m=float(own_margin),
        tail_position_excursion_m=tail_excursion,
        tail_max_speed_m_s=float(speed[tail].max()),
        samples=[
            dict(
                time_s=float(t),
                peak_stretch=float(s.max()),
                max_speed_m_s=float(v.max()),
                minimum_nonlocal_separation_m=float(d),
            )
            for t, s, v, d in zip(times, strain, speed, separation)
        ],
    )
    (args.source / "position_one_audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))


if __name__ == "__main__":
    main()
