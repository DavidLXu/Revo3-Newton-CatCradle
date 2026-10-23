# Single-player Opening A on RevoTron — experimental

This sequence follows **Position 1 → Opening A**: one pair of hands manipulates one closed string. It is not the two-person handoff game, a five-point star, or Cup & Saucer.

1. Seat the loop on both thumbs and little fingers; settle for two seconds before recording.
2. Lower the right index under the left palmar string, lift it on the back of the finger, and withdraw with the bight.
3. Pass the left index through that right-index bight, then approach the right palmar string from below.
4. Lift the right palmar string and extend both hands. Hold the resulting six-support figure for two seconds.

The 21-second recorded sequence uses official RevoTron FK and bounded arm IK. The hands and arms are kinematic. The string has positive-mass particles throughout, no hand attachments, no adhesion, and no runtime position targets. Newton XPBD computes contact and friction. Finger collisions use fitted capsules; palm collisions use convex hulls of the official palm and rubber meshes.

## Current status

**This is a motion/topology draft, not a fully validated physics result.** The publication-check export ([report](validation/opening-a.json)) passes both cross-hand material-strand pickup checks, one left-index crossing inside the existing right-index bight, retention of both index bights and all four original thumb/little supports, joint limits, arm speed, and approximate finger-clearance checks.

Two strict checks fail:

- Peak edge stretch is approximately **1.902**, above the **1.2** limit.
- Minimum nonadjacent segment distance is **1.724 mm** for a **2 mm** rope diameter, below the **1.8 mm** acceptance limit. The longest edge reaches **10.862 mm** and is within the current **16 mm** search bound, but the remaining penetration still fails the self-contact check. The discrete solver has no continuous collision detection.

The current code expands the previous 8 mm search bound to **16 mm**, with a regression test for a long edge whose starting point lies outside the old query radius. The full 21-second publication run still fails the two checks above. The optional `--rest-length-max-scale 4` and `--length-control global` modes are ongoing experiments and are not recommended as a validated replacement.

The default active controller adjusts local rest lengths. It is an ideal numerical take-up model, not a passive rope or a physical spool. An experimental global controller keeps relative material segment lengths fixed, but does not yet retain the initial loop reliably; it is not the reference result. Friction is an assumed coefficient of 1.2, not a measured material calibration. Accurate finger surfaces, converged contact, and uniform material-length control remain necessary before claiming physical reproduction or hardware transfer.

## Reproduce the draft

Run from the repository root after `bash scripts/setup.sh` and `bash scripts/setup_revotron.sh`. This exports an experimental result and its failure report; it does not guarantee a physically validated figure:

```bash
.venv/bin/python scripts/run_single_player.py \
  --approach upright --substeps 32 --iterations 24 --length-control local \
  --output output/revotron_opening_a_stage1
.venv/bin/python scripts/continue_opening_a.py \
  --source output/revotron_opening_a_stage1 --output output/revotron_opening_a
.venv/bin/python scripts/validate_opening_a.py --output output/revotron_opening_a
# The validator intentionally exits nonzero while either strict check fails.
.venv/bin/python scripts/render_revotron.py \
  --simulation-dir output/revotron_opening_a --single-player --draft
bash scripts/serve.sh
```

Open `/web/opening-a-video.html` for the labelled draft movie. The page also links to synchronized whole-robot and overhead hand-detail playback. Without `--draft`, the renderer refuses to record a trajectory whose validation fails. The render manifest preserves the complete validation report.

The first stage exports checkpoints each second. The second stage measures the right palmar material strand after threading and searches a small set of arm targets for a clear opposing-finger sweep. These are scripted geometric choices, not autonomous perception or a learned controller. The threading test uses a triangle spanned by the actual support contacts; it is evidence for this motion, not a general rope-topology proof.

## Procedure references

- [Alysion — introductions and Position 1](https://www.alysion.org/figures/introduc.htm)
- [Alysion — openings](https://www.alysion.org/figures/openings.htm)
- [Caroline Furness Jayne, String Figures and How to Make Them, scan page 37](https://en.wikisource.org/wiki/Page:String_Figures_and_How_to_Make_Them.djvu/37)

The source procedure is adapted to robot geometry; the joint trajectory is original code. No tutorial illustrations or reference video are distributed.

## Code organization

- `revo3_cat_cradle/bimanual.py`: robot FK/IK, collision model, simulation export.
- `revo3_cat_cradle/single_player.py`: Position 1 and finger/wrist target construction.
- `revo3_cat_cradle/bimanual_checks.py`: shared support, clearance and threading geometry.
- `scripts/run_single_player.py` and `scripts/continue_opening_a.py`: first and second cross-hand pickups.
- `scripts/validate_opening_a.py`: independent trajectory checks, nonzero exit on failure.
- `scripts/render_revotron.py`: whole-robot/detail rendering; `--draft` retains the failed report and labels the movie.

Generated trajectories, checkpoints, movies, reference media, hardware calibration, and downloaded model meshes are excluded from Git. The setup scripts fetch pinned dependencies and external robot assets. Source code remains Apache-2.0; see [third-party attribution](../THIRD_PARTY.md).
