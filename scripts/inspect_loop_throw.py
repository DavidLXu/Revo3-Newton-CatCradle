"""Plot fixed free-loop material relative to the original thumb bridge.

The X/Z projection is a motion diagnostic, not a crossing or topology test.
Use it together with contact tracking, physical audits and the rendered motion.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--plot", action="store_true", help="Also write a PNG; requires matplotlib")
    args = parser.parse_args()
    with np.load(args.source / "trajectory.npz") as archive:
        times, rope = archive["time"], archive["rope"]
    if not np.isfinite(times).all() or not np.isfinite(rope).all():
        parser.error("Motion diagnostics require finite recorded states")
    if not np.isfinite(args.start) or not times[0] <= args.start < times[-1]:
        parser.error("Start must lie inside the recorded interval")
    selected = np.flatnonzero(times >= args.start - 1e-5)
    if abs(times[selected[0]] - args.start) > 1e-5:
        parser.error("Start must match a recorded frame")
    plan = json.loads((args.source / "transfer_plan.json").read_text())
    count = rope.shape[1]
    left = int(plan["left"]["far_indices"][0])
    right = int(plan["right"]["far_indices"][0])
    thumbs = [
        int(p["old_thumb_indices"][(len(p["old_thumb_indices"]) - 1) // 2])
        for p in (plan["left"], plan["right"])
    ]
    arcs = [
        (right + np.arange((left - right) % count + 1)) % count,
        (right - np.arange((right - left) % count + 1)) % count,
    ]
    free = [arc for arc in arcs if not np.intersect1d(arc, thumbs).size]
    if len(free) != 1 or len(free[0]) < 17:
        parser.error("Cannot identify a unique original free arc between the indices")
    middle = len(free[0]) // 2
    band = free[0][middle - 8 : middle + 9]
    bridge = rope[selected][:, thumbs].mean(axis=1)
    probe = rope[selected][:, band].mean(axis=1)
    relative = probe - bridge
    angle = np.unwrap(np.arctan2(relative[:, 2], relative[:, 0]))
    peak = int(relative[:, 2].argmax())
    report = dict(
        scope=__doc__,
        interval_s=[float(times[selected[0]]), float(times[selected[-1]])],
        fixed_free_loop_material=band.tolist(),
        fixed_thumb_bridge_material=thumbs,
        maximum_relative_height_m=float(relative[peak, 2]),
        peak_height_time_s=float(times[selected[peak]]),
        signed_projected_sweep_deg=float(np.rad2deg(angle[-1] - angle[0])),
        projected_angle_range_deg=float(np.rad2deg(np.ptp(angle))),
        initial_relative_position_m=relative[0].tolist(),
        final_relative_position_m=relative[-1].tolist(),
    )
    (args.source / "loop_throw_motion.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not args.plot:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5), layout="constrained")
    axes[0].plot(times[selected] - args.start, relative[:, 2] * 100, color="#b31548")
    axes[0].axhline(0, color="#666666", linewidth=0.8)
    axes[0].set(xlabel="Time since throw start (s)", ylabel="Height above thumb bridge (cm)")
    axes[1].plot(relative[:, 0] * 100, relative[:, 2] * 100, color="#b31548")
    axes[1].scatter(relative[[0, -1], 0] * 100, relative[[0, -1], 2] * 100,
                    c=["#17805c", "#313b45"], zorder=3)
    axes[1].scatter([0], [0], c="#777777", marker="+", s=70)
    axes[1].set(xlabel="Forward offset from thumb bridge (cm)", ylabel="Vertical offset (cm)")
    axes[1].axis("equal")
    for ax in axes:
        ax.grid(alpha=0.18)
    fig.suptitle("Free-loop motion — fixed material, not a topology certificate")
    fig.savefig(args.source / "loop_throw_motion.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
