# Alternative Cup release experiments

The standard thumb-pickup route still fails the right near-strand clearance
gate. A second route follows the alternative method in
[Neil Parker's Cup and Saucer instructions](https://llx.com/Neil/string/playground/cup_and_saucer.html):
release the little-finger loops, swing the hanging loop over the other strings,
then exchange the thumb loops. Only the first operation is under test here.
Cup & Saucer, boat and Star remain incomplete.

The new comparison starts from the same independently placed Opening A with
global rest-length control and interior rope-edge/capsule contacts enabled.
After six seconds, all six supports remain, peak stretch is 1.0872, maximum
edge length is 2.609 mm and minimum sampled nonlocal rope separation is
1.934 mm. The rope is not certified stationary. At the start of release,
rest lengths are frozen without changing positions or velocities. There are
no hand anchors or runtime rope resets.

| Trial | Result |
| --- | --- |
| Both little MCP/PIP/DIP to 1/1/1 rad | Both loops remain caught. |
| Both then to 1.4/1.5/0.8 rad | Left releases; right stays caught. Peak stretch 1.2165 fails the 1.2 gate. |
| Left alone to 1.4/1.5/0.8 rad | Necessary left-release checks pass after a 1.2 s hold. Minimum left little clearance is 41.012 mm; the other five supports remain. |
| Right alone to 1.4/1.5/0.8 rad | Right stays caught; peak stretch 1.2307 fails. |
| After left release, right little MPR to −0.5 rad, then deeper flexion | Right remains caught. Left stays clear and full-history stretch stays at 1.0885. |
| Hook both indices, rotate wrists 100° about world Y | All six supports remain; neither little loop releases. |
| After left release and right abduction, hook right index, move right wrist 35 mm outward, roll −60° about world X | Both little loops release and remain clear during the hold, but full-history peak stretch is 1.3618. Rejected. |
| Fresh fixture with right little MPR −0.5 rad and its initial bight shifted 30 mm distally | All six supports survive four seconds. Subsequent finger flexion releases only the left little loop; the right still catches. |
| Replace the 35 mm outward wrist movement with 45 mm forward staggering, retaining the index hook and −60° roll | Both little loops release and stretch stays below 1.2, but the right index penetrates the opposite palm. Rejected. |

The left-only result has full-history peak stretch 1.0885 and sampled nonlocal
separation of at least 1.934 mm. These checks establish a useful local release
checkpoint, not the material threading of the remaining figure. A freed strand
also reaches the right thumb, which must be accounted for before a loop exchange.
The [recorded diagnostics](alternate_cup_20261108.json) retain the checkpoint
hashes and the limits of each audit.

The outward-wrist trial finishes at 24.6 s. Final-second little clearances are
at least 47.373 mm on the left and 63.137 mm on the right; thumb and index
contacts remain throughout the transfer. The failure occurs earlier, at
18.7667 s, on rope edge 632 beside the right index. Checking only the final
pose would miss this stretch spike. The front-to-back staggering comparison
does avoid the spike. At 24.6 s, final-second little clearances are at least
33.344 mm on the left and 60.252 mm on the right. The four thumb/index contacts
remain throughout, and full-history nonlocal separation is at least 1.934 mm.
The initial contact audit missed an 8.740 mm right-index/left-palm intersection
at the final pose. Opposing-palm clearance is now required by the release audit
and by `continue_cup.py --check-self-collision`. The continuation from this
checkpoint was stopped. This is an invalid trajectory, despite its retained
rope supports and acceptable deformation. The loop throw and thumb exchange
remain unfinished.

## Reproduce the left-only release

After the normal setup, run these commands in order with fresh output folders:

```bash
.venv/bin/python scripts/replay_transfer.py examples/transfers/global_opening_seed.json \
  --output output/global_opening
.venv/bin/python scripts/continue_cup.py --source output/global_opening \
  --output output/little_fold --side both --route-only --check-self-collision \
  --approach-path examples/transfers/fold_little_loops.json --fixed-rest-lengths \
  --hold-duration 1 --hold-label 'Hold hanging little-finger loop'
.venv/bin/python scripts/continue_cup.py --source output/little_fold \
  --output output/left_little_release --side both --route-only --check-self-collision \
  --approach-path examples/transfers/release_left_little.json \
  --hold-duration 1.2 --hold-label 'Hold left little loop released'
.venv/bin/python scripts/audit_loop_release.py --source output/left_little_release \
  --start 10 --released left

# Reach the checkpoint before the rejected bilateral route.
.venv/bin/python scripts/continue_cup.py --source output/left_little_release \
  --output output/right_little_abduct --side both --route-only --check-self-collision \
  --approach-path examples/transfers/abduct_right_little.json \
  --hold-duration 1.2 --hold-label 'Hold after right little abduction'
```

`rejected_stagger_roll.json` records the rejected 45 mm staggering route.
With the new palm check, it cannot complete its preflight. Do not disable that
check to treat the recorded release as a valid result.

The release audit includes continuous contact retention at exported frames,
interior edge/capsule gaps, full-history stretch and rope separation, positive
particle masses, no anchors, joint limits and a completed second of fixed
joints. Opposing finger/palm gaps use the simulated convex palm surfaces and
a conservative bound between capsule-axis samples. Palm/palm separation uses
bounding spheres. Rope/mesh contacts are sampled at particles. The audit does
not establish arm clearance, material identity, swept collision freedom or the
final figure's topology. Use
`--released both` to require both little fingers to remain clear; the left-only
checkpoint must fail that request.

The portable six-second recipe contains the original initialization and trimmed
commands. It does not load a recorded rope trajectory. Subsequent commands
restore the exact simulated state and stop active length adjustment at six
seconds. Revalidate every fresh run; these results were measured on Apple
Silicon CPU.

Verification: the portable recipe's first 0.1 s exactly matches the recorded
global-fixture joints, rope positions and rest lengths. All 94 non-integration
tests and F-rule lint pass. The rotating-capsule test checks tangential dragging
from angular velocity. Palm regressions include the recorded pose whose fingers
clear each other while an index intersects the other palm. The existing
single-hand integration failure on GitHub is still unresolved: uplift is
24.576 mm against a 25 mm threshold. This is not a green full-CI claim.
