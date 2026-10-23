"""Inspect source-strand pickup and opposing-hand capsule clearance."""

import sys, json, argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from revo3_cat_cradle.bimanual import RevoTron
from revo3_cat_cradle.bimanual_checks import gaps, hand_clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "output/revotron_opening_a"
    )
    args = parser.parse_args()
    out = args.output
    d = np.load(out / "trajectory.npz")
    robot = RevoTron()
    ids = json.loads((out / "plan.json").read_text())["left_palmar_indices"]
    for k in range(0, len(d["time"]), 15):
        tf = robot.fk(d["joints"][k])
        p = d["rope"][k]
        g = gaps(tf, p, "right", "index")
        clearance = hand_clearance(tf)
        support = [
            round(gaps(tf, p, s, f).min() * 1000, 1)
            for s in ("left", "right")
            for f in ("thumb", "little")
        ]
        print(
            round(float(d["time"][k]), 2),
            "source",
            round(g[ids].min() * 1000, 1),
            "index ids",
            np.flatnonzero(g < 0.001).tolist(),
            "supports",
            support,
            "hands",
            round(clearance[0] * 1000, 1),
            *clearance[1:],
        )


if __name__ == "__main__":
    main()
