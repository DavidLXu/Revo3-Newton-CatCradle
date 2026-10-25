"""Extend contact-aware search checkpoints toward a right-thumb bight.

This bounded offline search never declares a completed figure. Its angular
criterion only identifies a candidate for subsequent hold, topology and physics
validation. Each round preserves the selected predecessor's full dynamic state.
"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--support-reference", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--rounds", type=int, default=6)
    p.add_argument("--samples", type=int, default=40)
    p.add_argument("--workers", type=int, default=3)
    p.add_argument("--scale", type=float, default=0.5)
    p.add_argument("--seed", type=int, default=71)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    source = a.source.resolve()
    scale = a.scale
    history = []
    for i in range(a.rounds):
        out = a.output.resolve() / f"round_{i + 1:02d}"
        command = [
            sys.executable,
            str(Path(__file__).with_name("search_thumb_transfer.py")),
            "--source",
            str(source),
            "--support-reference",
            str(a.support_reference.resolve()),
            "--output",
            str(out),
            "--samples",
            str(a.samples),
            "--workers",
            str(a.workers),
            "--scale",
            str(scale),
            "--seed",
            str(a.seed + 17 * i),
            "--wrist-rotation-std",
            ".12",
            "--score-mode",
            "wrap",
        ]
        print(
            json.dumps(
                dict(round=i + 1, status="running", source=str(source), output=str(out))
            ),
            flush=True,
        )
        with (a.output / f"round_{i + 1:02d}.log").open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        winner = json.loads((out / "winner.json").read_text())
        evaluation = winner["evaluation"]
        ready = (
            evaluation["target_gap_mm"] < 1.5
            and evaluation["support_loss_mm"] == 0
            and evaluation["minimum_target_sweep_rad"] >= math.pi
            and evaluation["minimum_retained_sweep_rad"] >= math.pi
            and evaluation["maximum_near_sweep_rad"] < math.pi
        )
        record = dict(
            round=i + 1,
            output=str(out),
            candidate_ready=ready,
            selected=winner["proposal"]["id"],
            evaluation=evaluation,
        )
        history.append(record)
        (a.output / "history.json").write_text(json.dumps(history, indent=2))
        print(json.dumps(record), flush=True)
        if ready:
            print(
                "Candidate found; hold and full-trajectory validation are still required.",
                flush=True,
            )
            break
        source = out
        scale = min(scale * 1.4, 1.2) if record["selected"] == 0 else a.scale


if __name__ == "__main__":
    main()
