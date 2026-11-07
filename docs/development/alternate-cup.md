# Alternative Cup release experiments

The standard thumb-pickup route still fails the right near-strand clearance
gate. A second route follows the alternative method in
[Neil Parker's Cup and Saucer instructions](https://llx.com/Neil/string/playground/cup_and_saucer.html):
release the little-finger loops, swing the hanging loop over the other strings,
then exchange the thumb loops.
The earlier hand-only checks reported bilateral release, but a subsequent arm
audit invalidates that chain. A new downward-facing preparation now releases
both little loops with arm/body contacts enabled and passes the listed release
and material-tracking checks. Cup & Saucer, boat and Star remain incomplete.

## Correction: missing arm collisions

The old simulation had finger, knuckle and palm contacts but no arm or torso
contact surfaces. Adding a central-body envelope check caught large swings;
it did not catch the released loop entering the forearm. The full preparation
history first enters a conservative right-wrist hull at 10.933333 s. Direct
triangle checks also confirm actual left-forearm surface crossings at 24.6 s
and 27.3 s (four and eight intersections, respectively). The old release and
tilted-preparation checkpoints must not be used as physically valid starts.
The measurements below are historical diagnostics with this omission.

Fresh replays can now enable `--robot-body hulls`. This adds 21 kinematic
collision bodies covering both arms, the base, torso, neck and head. Each
shape is the convex hull of its official URDF link's collision meshes, with
the mesh scale and collision origin applied. Newton supplies contact and
friction; no rope anchors, resets or runtime position edits are added. Hulls
fill mesh cavities, so contact may occur earlier than on the exact surface.
Rigid robot self-collisions and swept collision checking remain separate gaps.

Continuation and episode tools preserve the geometry selection. Checkpoints
from scenes without those bodies are rejected; recompute from the original
placement instead of inserting collision surfaces around an interpenetrating
rope. `audit_robot_hulls.py` clips complete rope edges against every convex
solid, including edges whose endpoints both lie outside. This checks centerline
intrusions at recorded frames, not finite-radius or swept clearance. The release
audit now includes this check, and `--check-robot-hulls` stops a continuation at
its first such intrusion.

```bash
.venv/bin/python scripts/replay_transfer.py examples/transfers/global_opening_seed.json \
  --robot-body hulls --output output/opening_with_robot_contacts
.venv/bin/python scripts/audit_robot_hulls.py --source output/opening_with_robot_contacts
```

The six-second collision-enabled Opening A replay retains all six contacts
through its final second, has no arm/body hull centerline intrusions, and has
peak stretch 1.0872. All 181 recorded joint, rope and rest-length arrays exactly
match the older hand-only fixture: these bodies have not yet contacted the
rope. Passing this initial placement does not establish a successful later
release. The new geometry and audit regressions pass with all 101
non-integration tests. The unrelated single-hand integration failure described
below remains unresolved.

Replaying the old fold and left-release commands with body contacts reaches
13.2 s without a hull-centerline intrusion. Joint commands match exactly; rope
states first diverge at 10.933333 s and differ by up to 96.051 mm in one
coordinate. The left little finger stays at least 40.952 mm clear during the
last second, while the right little and four thumb/index supports remain.
Peak stretch is 1.0885, minimum sampled rope separation is 1.934 mm, and the
four original material contact tracks remain continuous. This is a valid
local release under the listed diagnostics, not a Cup or a free-loop threading
certificate. New extra thumb contacts remain in the material report.

A second branch rotates the strung figure 60° forward about world Y before
releasing the little loops. Four 0.6 s waypoints and a 1.2 s hold end at 9.6 s.
All six contacts remain; original thumb/index tracks are continuous, peak
stretch is 1.1114, and minimum sampled rope separation remains 1.934 mm.
No arm/body hull centerline intrusion occurs. This moves the support region
in front of the forearms without changing rope coordinates directly.

Abducting the right little finger for 1 s and folding both littles over 2 s,
followed by a 1.2 s hold, ends at 13.8 s. The left little is clear in the final
frame but was still touching during part of the last second; the right little
remains caught. Both-release diagnostics therefore fail. The four original
thumb/index tracks, strain, separation and robot-clearance checks pass. The
next attempt rolls the right wrist within this tilted figure plane.

```bash
.venv/bin/python scripts/continue_cup.py --source output/opening_with_robot_contacts \
  --output output/forward_tilt --side both --route-only --fixed-rest-lengths \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --approach-path examples/transfers/tilt_opening_forward.json \
  --hold-duration 1.2 --hold-label 'Hold the forward-tilted Opening A before release'
.venv/bin/python scripts/continue_cup.py --source output/forward_tilt \
  --output output/tilted_release --side both --route-only \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --approach-path examples/transfers/attempt_tilted_little_release.json \
  --hold-duration 1.2 --hold-label 'Hold the little-loop release in front of the forearms'
.venv/bin/python scripts/audit_loop_release.py --source output/tilted_release \
  --start 9.6 --released both
```

The final command is expected to reject the incomplete bilateral release.
`attempt_tilted_right_release.json` then deepens the right index hook, staggers
the wrist by 45 mm within the tilted figure plane, and rolls it to −60° in two
steps. At 21 s the necessary deformation, separation, arm/body and hand checks
pass, with peak stretch still 1.1114 and all four original material tracks
continuous. Both little fingers touch different parts of the free loop during
the final second, however, so the release remains incomplete. The original
little-loop material has moved away; contact alone is not enough to distinguish
that release from subsequent contact with another strand.

The next trial uses a small left-little adjustment selected from collision-free
joint paths: MPR from −0.2 to −0.0872875 rad and MCP from 1.4 to 1.583 rad.
It then folds the right little tip, restores the right wrist from −60° to −30°
within the tilted plane, and advances both wrists 60 mm. Static planning checks
do not modify the rope. At 24.8 s the continuation preserves all four original
thumb/index material tracks and passes the deformation, separation and
robot-clearance checks (peak stretch 1.1141), but both little fingers still
touch the rope in the final second. It is another incomplete release attempt,
recorded as `attempt_tilted_clearance_restore.json`. The next planned comparison
hooks the left index, turns the figure farther forward so the fingers point
downward, then extends both little fingers to test gravity-assisted release.

## Bilateral release with arm and body contacts

The downward turn releases both little loops before the planned finger
extension. The successful branch stops changing the joints at 29.5 s and
continues Newton for another 1.2 s with the little fingers folded. Throughout
the last second, the left and right little clearances are at least 38.091 mm
and 28.859 mm. Four original thumb/index material tracks remain continuous from
6 to 30.7 s, with no extra patches in the final report. Full-history peak
stretch is 1.1141, sampled nonlocal rope separation is at least 1.934 mm, and
the hand, central-body and complete-edge arm/body hull checks pass.

This is the current release preparation. It retains the same passive 1.36345 m
rest-length budget after 6 s, positive particle masses and no anchors or
runtime rope-coordinate edits. It does not yet establish the hanging-loop
throw, thumb-loop exchange, Cup, boat or Star. The initial Opening A is still
an independent placement.

The comparison that extends the little fingers after 29.5 s touches the rope
again and fails the final-second bilateral clearance gates at 32.7 s. Keep
the folded-finger branch for further work.

To reproduce the accepted release preparation, start with the collision-enabled
Opening A and the two forward-tilt commands above, then run:

```bash
.venv/bin/python scripts/continue_cup.py --source output/tilted_release \
  --output output/tilted_right_roll --side both --route-only \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --approach-path examples/transfers/attempt_tilted_right_release.json \
  --hold-duration 1.2 --hold-label 'Hold the tilted right-wrist roll'
.venv/bin/python scripts/continue_cup.py --source output/tilted_right_roll \
  --output output/tilted_restore --side both --route-only \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --approach-path examples/transfers/attempt_tilted_clearance_restore.json \
  --hold-duration 1.2 --hold-label 'Hold after the forward wrist restoration'
.venv/bin/python scripts/continue_cup.py --source output/tilted_restore \
  --output output/downward_ready --side both --route-only \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --approach-path examples/transfers/release_little_loops_downward.json \
  --hold-duration 1.2 --hold-label 'Hold both released little loops with the fingers folded'
.venv/bin/python scripts/audit_loop_release.py --source output/downward_ready \
  --start 24.8 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/downward_ready --start 6
.venv/bin/python scripts/render_revotron.py --simulation-dir output/downward_ready \
  --single-player --preview --fit-rope --draft --label 'Bilateral little-loop release check'
```

The intermediate attempts have incomplete release gates but retain the four
supports and pass the physical diagnostics required for this continuation.
`rejected_downward_extension.json` records the unsuccessful extra extension;
replay it from the 29.5 s boundary, before the folded-finger hold, to reproduce
the comparison. Replaying it after the 30.7 s hold is a different experiment.

The renderer's optional `--fit-rope` detail view encloses the whole recording's
rope bounds with one fixed camera, so the long hanging loop remains visible.
Draft recording also accepts a missing `validation.json` and records its
validation field as null; it never invents a passing result. A two-frame draft
recording exercised that path, and the full release preview was inspected.

## Historical hand-only comparisons

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

Historical continuations used `--check-body-envelope`. This optional runtime
guard saves `body_envelope_stop.json` and stops on the first exported frame
without certified body clearance. The historical swing paths below are
retained to reproduce rejected experiments. Arm collisions were absent from those historical runs; swept motion remains
outside the current diagnostics. No completed figure is claimed.

## Reproduce historical hand-only attempts (invalid for continuation)

These commands preserve the old experiment with missing arm collisions. They
do not reproduce the accepted release above and must not supply a new physical
continuation. Run only for comparison, in fresh output folders:

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
not establish finite-radius arm clearance, material identity, swept collision
freedom or the final figure's topology. The current audit also rejects the
historical arm-hull centerline intrusions described above. Use
`--released both` to require both little fingers to remain clear; the left-only
checkpoint must fail that request.

The portable six-second recipe contains the original initialization and trimmed
commands. It does not load a recorded rope trajectory. Subsequent commands
restore the exact simulated state and stop active length adjustment at six
seconds. Revalidate every fresh run; these results were measured on Apple
Silicon CPU.

Verification: the portable recipe's first 0.1 s exactly matches the recorded
global-fixture joints, rope positions and rest lengths. All 101 non-integration
tests and F-rule lint pass. The rotating-capsule test checks tangential dragging
from angular velocity. Palm regressions include the recorded pose whose fingers
clear each other while an index intersects the other palm. Body-envelope tests
include edges crossing a box with both endpoints outside. A checkpoint replay
confirms that the runtime body guard stops at 28.3 s, matching the offline audit.
Arm/body regressions also check native soft-contact participation, FK tracking,
checkpoint rejection for missing bodies, and complete edges crossing a hull
with their endpoints outside. The existing single-hand integration failure on
GitHub is still unresolved: the latest run measures 23.876 mm uplift against a
25 mm threshold (an earlier run measured 24.576 mm). This is not a green full-CI
claim; the latest run is recorded in the machine-readable experiment report.
