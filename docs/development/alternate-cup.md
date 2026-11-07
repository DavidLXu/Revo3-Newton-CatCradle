# Alternative Cup release experiments

The standard thumb-pickup route still fails the right near-strand clearance
gate. A second route follows the alternative method in
[Neil Parker's Cup and Saucer instructions](https://llx.com/Neil/string/playground/cup_and_saucer.html):
release the little-finger loops, swing the hanging loop over the other strings,
then exchange the thumb loops.
The experiments now reach bilateral release and test the loop throw.
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
| Move the wrist 25 mm forward, 15 mm outward and 25 mm down, with index MCP/PIP/DIP 0.2/0.6/0.6 rad | Palm clearance passes, but full-history stretch reaches 1.2292. Rejected. |
| Keep index MCP at −0.08 rad on that balanced wrist route | Both loops release, but stretch reaches 1.2877. Rejected. |
| Return to 45 mm forward staggering with index MCP/PIP/DIP −0.08/0.9/1.0 rad | Both little loops release. Necessary release, palm and finger-clearance checks pass over the recorded history. |

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

The deeper index hook removes the palm intersection without repeating the
stretch spike. At 24.6 s, the final-second little clearances are at least
43.574 mm on the left and 60.175 mm on the right. All four thumb/index contacts
remain throughout the 17.4–24.6 s interval. Full-history peak stretch is 1.1105,
minimum nonlocal rope separation is 1.934 mm, and the opposing finger/palm
clearance bound is at least 1.960 mm. Opposing finger capsules stay 4.201 mm
apart; same-hand finger margins pass as well.

This is a release checkpoint, not a completed Cup. A separate material audit
tracks all four original thumb/index contact patches from 6 to 24.6 s without
a lost or discontinuous track. Additional contact patches appear on the thumbs
and right index during the wrist roll and remain listed in the report. A
continuous contact track does not establish threading. Wrist restoration, the
hanging-loop throw and thumb exchange still need dynamic validation and
material inspection.

## Preparing and throwing the hanging loop

Restoring the right wrist fully upright fails the stretch gate. Adding a left
index hook reaches 1.2095 during the hold; leaving the left index unchanged
reaches 1.2663 beside the right index before the hold is complete. Both
branches are rejected. Retaining a −30° right-wrist roll avoids this problem.
At 27.3 s, the tilted preparation passes the release checks and all four
original material tracks remain continuous. Final-second little clearances
are at least 42.556 mm left and 53.747 mm right. The history's peak stretch
remains 1.1105. The loop is hanging and moving, not certified stationary.

Two throws from that checkpoint retain the contacts and pass the necessary
physical checks, but neither completes the throw. Both lower the wrists 50 mm,
raise them 120 mm above the starting position, then return. The first pitches
the wrists 15° during loading; the second uses 30° followed by −20° during
the lift. Durations keep peak arm-joint speed below 3 rad/s under quintic
interpolation. Tracked central free-loop material remains at least 235 mm and
223 mm below the midpoint of the original thumb bridge, respectively. These
motion diagnostics do not establish a crossing or a finished figure.
The first throw also exceeds the material tracker's eight-particle-per-frame
continuity bound at the left index (28.0 s) and right thumb (28.1 s). Contact
retention alone therefore cannot certify those original supports. The larger
wrist-pitch comparison keeps all four original tracks continuous.

Three forward/backward swings increase the free-loop excursion. With wrist
offsets of ±80 mm, 0.5 s travel between extremes, and 0.4 s entry/return,
central material briefly reaches 21.878 mm above the thumb-bridge midpoint.
It still swings back rather than completing a loop throw. The hand/rope checks
pass after the hold, all four original tracks remain continuous, and peak
stretch is 1.1186. The subsequently added body-envelope check rejects this
route. This comparison is `attempt_loop_swing.json` and starts
from the same 27.3 s preparation checkpoint.

Extending to five swings still does not complete the throw. Keeping the 0.5 s
half-cycle raises the peak central-material height to 51.292 mm above the
bridge, versus 20.170 mm when the last two cycles use 0.6 s. Both pass the
hand/rope contact and deformation checks, with peak stretch 1.1186. However,
the left-index material track exceeds its continuity bound at 33.3667 s in
the fixed-cadence branch and 31.3333 s in the slower branch. These are
incomplete attempts, not accepted figure transitions.

The swing experiments also enter conservative torso/head envelopes, first at
28.3 s. Their earlier hand/rope checks did not include the central body. All
three- and five-swing branches are now rejected by the expanded release audit.
Lowering the bridge 120 mm and retracting it 40 mm during the fifth swing
raises the tracked material to 95.158 mm above the bridge, but still fails
this body-clearance check and does not complete a throw.

`CentralBodyEnvelope` uses boxes enclosing the official base, torso, neck and
head collision meshes. Several meshes are not watertight, so an inside/outside
mesh sign would be unreliable. A positive edge/box separation certifies that
the rope tube is outside the box; an overlap is inconclusive and rejects the
route without claiming a measured mesh penetration. The initial bilateral
release, tilted preparation and two short throws clear these envelopes. Their
minimum bounds are 150.015, 150.015, 72.515 and 113.153 mm respectively.
Separate triangle-intersection spot checks confirm actual surface crossings:
the three-swing rope intersects eight official waist collision triangles at
30.3333 s; the lowered-bridge attempt intersects six at 32.4667 s. Edge indices,
triangle indices and intersection coordinates are retained in the data report.

New continuations should pass `--check-body-envelope`. This optional runtime
guard saves `body_envelope_stop.json` and stops on the first exported frame
without certified body clearance. The historical swing paths below are
retained to reproduce rejected experiments. Arm collisions and swept motion
remain outside the diagnostics; no completed figure is claimed.

## Reproduce release and throw attempts

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

# Curl the index farther to clear the opposing palm during the release.
.venv/bin/python scripts/continue_cup.py --source output/right_little_abduct \
  --output output/both_little_release --side both --route-only --check-self-collision \
  --approach-path examples/transfers/release_both_little_loops.json \
  --hold-duration 1.2 --hold-label 'Hold bilateral release with deeper index hook'
.venv/bin/python scripts/audit_loop_release.py --source output/both_little_release \
  --start 17.4 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/both_little_release \
  --start 6

# Prepare the hanging loop while retaining a 30-degree right-wrist tilt.
.venv/bin/python scripts/continue_cup.py --source output/both_little_release \
  --output output/throw_ready --side both --route-only --check-self-collision \
  --approach-path examples/transfers/prepare_loop_throw.json \
  --hold-duration 1.2 --hold-label 'Hold released loop with right wrist tilted 30 degrees'
.venv/bin/python scripts/audit_loop_release.py --source output/throw_ready \
  --start 24.6 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/throw_ready --start 6

# This attempt keeps the supports but does not throw the loop over the bridge.
.venv/bin/python scripts/continue_cup.py --source output/throw_ready \
  --output output/throw_attempt --side both --route-only --check-self-collision \
  --approach-path examples/transfers/attempt_loop_throw.json \
  --hold-duration 1.2 --hold-label 'Hold after the free-loop throw'
.venv/bin/python scripts/audit_loop_release.py --source output/throw_attempt \
  --start 27.3 --released both
.venv/bin/python scripts/inspect_loop_throw.py --source output/throw_attempt --start 27.3
```

Use `attempt_wrist_loop_throw.json` in a separate output folder to reproduce
the larger wrist-pitch comparison, or `attempt_loop_swing.json` for three
swings. `inspect_loop_throw.py` writes numerical
motion diagnostics by default. Its optional `--plot` needs matplotlib; to
generate the plot without changing the physics environment:

```bash
uv run --isolated --no-project --python .venv/bin/python --with numpy --with matplotlib \
  python scripts/inspect_loop_throw.py --source output/throw_attempt --start 27.3 --plot
```

The five-swing comparison resumes the three-swing run at its 30.2 s checkpoint,
before the return to center and final hold:

```bash
.venv/bin/python scripts/continue_cup.py --source output/throw_ready \
  --output output/swing_attempt --side both --route-only --check-self-collision \
  --approach-path examples/transfers/attempt_loop_swing.json \
  --hold-duration 1.2 --hold-label 'Hold after three loop swings'
.venv/bin/python scripts/continue_cup.py --source output/swing_attempt \
  --checkpoint output/swing_attempt/state_0906.npz \
  --output output/five_swings --side both --route-only --check-self-collision \
  --approach-path examples/transfers/extend_loop_swing.json \
  --hold-duration 1.2 --hold-label 'Hold after five swings at the original cadence'
.venv/bin/python scripts/audit_retained_supports.py --source output/five_swings --start 6
```

The last command is expected to report the continuity failure described above.
To reproduce the slower comparison, copy the extension route and change its
first four durations from 0.5 to 0.6 s; use a fresh output directory.

To inspect a transient pose rather than only the final still, the renderer
accepts repeatable `--preview-time` arguments alongside `--preview --draft`.
Each requested time produces an additional image without changing the replay.

`rejected_stagger_roll.json` records the rejected 45 mm staggering route.
With the new palm check, it cannot complete its preflight. Do not disable that
check to treat the recorded release as a valid result.

The release audit includes continuous contact retention at exported frames,
interior edge/capsule gaps, full-history stretch and rope separation, positive
particle masses, no anchors, joint limits and a completed second of fixed
joints. Same-hand and opposing finger clearances and central-body envelopes are
checked over the full exported history. Opposing finger/palm gaps use the simulated convex palm surfaces and
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
global-fixture joints, rope positions and rest lengths. All 98 non-integration
tests and F-rule lint pass. The rotating-capsule test checks tangential dragging
from angular velocity. Palm regressions include the recorded pose whose fingers
clear each other while an index intersects the other palm. Body-envelope tests
include edges crossing a box with both endpoints outside. A checkpoint replay
confirms that the runtime body guard stops at 28.3 s, matching the offline audit.
The existing
single-hand integration failure on GitHub is still unresolved: uplift is
24.576 mm against a 25 mm threshold. This is not a green full-CI claim.
