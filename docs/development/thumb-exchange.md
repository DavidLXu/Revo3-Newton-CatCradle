# Releasing the old thumb loops and unloading the helper

The straight-little follow-up reaches 86.766667 s with both old thumb loops
clear, both new far-material tracks and both original indices retained, and
both near strands clear of their thumbs. Every unused middle and ring finger
also clears. The remaining failed release condition is the right little:
several rope patches still lie around its shaft and root. Cup & Saucer, boat
and Star are not complete.

This continues the [orbit and middle-clearance experiments](orbit-transfer.md).
The [numerical record](thumb_exchange_20261109.json) includes the failed
alternatives, source hashes, joint recipes and audit results. All reported
results come from Newton replays; proposed finger positions are not substituted
for rope motion.

## What changed in the motion

The earlier coordinated pickup reaches 67.366667 s with both new far strands
held, the left old loop clear, and the right old loop still around its thumb.
The later helper motions below continue from that state.

A fingertip target approached the far strand but left it ahead of the rounded
end. When that thumb rose, the strand stayed behind and the new contact was
lost. The next proposal put the distal shaft beneath the strand instead.
Achieving that position required wrist and index motion together: the planner
kept the index DIP pose approximately fixed while moving the palm and thumb.
That reference affects joint planning only; it does not attach the rope.

The left-only shaft motion caught the far strand but also met its neighboring
strand. It hit the unchanged 1.2 stretch limit at 65.766667 s, reaching 1.3660
at the rejected frame. The right-only shaft motion lost the other hand's
original thumb support at 64.666667 s. It also fails the full-history sampled
rope self-contact audit. Neither failed state is a continuation source.

The bilateral proposal uses the same 62.966667 s preparation for both hands,
combines their joint curves, and checks the combined hand clearance. Both
old thumb supports are explicitly allowed to leave during this phase; both
original index material tracks remain required. The 3.2 s motion completes
its 1.2 s hold without a runtime stop.

| Final held-second measurement | Left | Right |
| --- | ---: | ---: |
| Maximum far-strand edge/mesh gap | Contact, within 1 µm | 0.435 mm |
| Minimum near-strand gap | 7.125 mm | Contact |
| Minimum whole old-loop gap | 77.808 mm | Contact |
| Persistent far material samples | 209–226 | 636–637 |

The complete physical audit passes its stretch, sampled rope self-contact,
finger clearance, opposing-palm, arm/body hull, little-release and stationary
hold checks. Both original index tracks pass. The exchange audit still fails
right near/old-loop clearance and unused right-finger clearance. These observations establish retained
material contacts and left-side clearance, not a threading or force-closure
certificate.

An extra 1.2 s stationary continuation reaches 68.566667 s. The new far-material
guard follows left sample 217 and right sample 636 through every frame without
losing either track. This longer hold remains a preparation for right-loop
release, not a completed figure.

![Contact and stretch comparison](thumb_exchange_20261109.png)

The plotted gaps use particle/capsule samples to show the motion. The table
uses the separate capsule-edge and enabled knuckle-mesh audit. The shaded
interval is the bilateral stationary hold; the failed left-only endpoint is
included in its stretch trace and excluded from accepted continuation states.

## Opening the clamps

Five preparations branch from the cleared orbit state at 61.166667 s. Only
thumb CMR joints change, over 0.6 s, followed by a 1.2 s hold.

| Preparation | CMR change | Outcome |
| --- | --- | --- |
| Both, small opening | −0.1 rad each | Completes; source for bilateral pickup |
| Both, larger opening | −0.2 rad each | Left original thumb track lost at 62.333333 s |
| Left only | −0.2 rad | Completes; source for left-only comparisons |
| Right only, small | −0.1 rad | Completes |
| Right only, larger | −0.2 rad | Completes |

All completed openings retain four original supports and pass their physical
audits. They intentionally open the near-strand clamp, so they do not claim
continued bilateral near-pinching.

## Phase-specific retention checks

`continue_cup.py` still requires all four original thumb/index tracks by default.
`--release-thumb-supports left`, `right` or `both` declares which old thumb
tracks may depart during a guarded route or hold. It never excuses either
original index track. During a one-sided exchange, the opposite original thumb
remains required. The declaration is recorded and does not count as successful
release.

`audit_retained_supports.py --indices-only` writes a separate index report.
`audit_thumb_exchange.py` combines the physical audit, index continuity,
nondeparting original supports, stationary hold, persistent new far material,
near clearance, complete old-loop clearance and all unused fingers' held
clearance. It always reports
`completed_figure: false`: these are necessary conditions rather than a figure
recognizer. The opened-clamp negative control fails the capture and release
conditions as expected.

Once both new far contacts have passed a stationary hold,
`--stop-on-far-loss` tracks their material through the next route. It requires
a matching physical audit and held far-material report, together with the
original-support guard. A new patch elsewhere on the thumb cannot silently
replace a lost material track. The guard stops the experiment; it applies no
force or constraint to the rope.

If an already tracked strand develops another contact patch, the next phase
uses its previous material center to continue the same track. Without that
history, multiple extended patches remain ambiguous and are rejected.

## Right-hand reorientation follow-up

A 2.5 s right-hand rotation keeps the measured index support point approximately
fixed while directing the thumb upward. Its proposed point drift is at most
0.743 mm and joint-speed bound is 1.975 rad/s. The Newton replay and 1.2 s hold
finish at 71.066667 s. Both new far-material tracks survive, and left-side
clearance remains. However, the right little finger meets the loose old arc:
its first gap below 2 mm occurs at 68.6 s, and capsule-edge contact starts at
68.833333 s. The physical release audit rejects this continuation state.

The recipe is `attempt_thumb_right_contact_pivot.json`, starting from
`output/thumb_bilateral` with both retention guards enabled. It is a rejected
follow-up, not the starting state for further physics. The held 67.366667 s
state also has an extra right middle MCP contact near far-strand edge 627;
clearing unused right fingers remains part of the task. Subsequent planning
uses the rejected replay only to observe where the rope swept past those
fingers, then returns to the earlier validated checkpoint for any new replay.

The next proposal reduces the target thumb tilt to 30 degrees and adjusts all
unused right fingers against that observed rope path. Its 11.333333 s motion
and 1.2 s hold finish at 79.9 s. The ring clears by at least 5.335 mm in the held
second, but the middle and little still touch. Both new far-material tracks and
the original indices survive; left near and old-loop clearances reach 9.190 mm
and 79.593 mm. The physical audit again rejects the endpoint for continued
released-loop motion because the right little has not cleared. The recipe is
`attempt_thumb_right_unload30.json`, with the same bilateral source and guards.

This exposes a limitation of the planner's frozen rope observations: a finger
can move away from the recorded path while the live rope follows it. A further
5 mm clearance proposal against this newer observation misses its index
reference tolerance and is rejected before physics.

The same spatial path also runs in 3 s, followed by a 1.2 s hold. Its interval
speed bound is 2.465 rad/s. This faster replay still retains both far tracks
and leaves the right old loop, middle and little contacts in place. The recipe
is `attempt_thumb_right_unload30_fast.json`.

## Temporary helper and thumb withdrawal

The next probe deliberately uses the right little contact as a temporary
old-loop support. It replays the 53-degree turn from the validated bilateral
source, holds for 1.2 s, then withdraws the thumb 12 mm over 2.6 s. The index
and little surface references guide joint planning. The physical rope remains
free to slide. At 71.066667 s, the replayed positions, velocities, joints and
rest lengths exactly match the earlier turn observation; its rejected endpoint
was not loaded into the simulator.

The temporary old-loop track survives all 115 withdrawal and hold frames,
moving from representative sample 447 to 445. Both new far tracks survive too.
The old thumb loop remains: sample 415 retreats 13.895 mm along the initial
thumb axis while the thumb retreats 12 mm. At the little finger, sample 448
moves 19.859 mm despite the approximately stationary planning reference.
Retained contact therefore does not establish a fixed material grasp. The
little also retains near-strand samples 549–562.

![Material motion during temporary helper withdrawal](thumb_helper_sliding_20261109.png)

Use `attempt_thumb_right_helper_plane12_fast.json` with the same bilateral
source and both retention guards. The temporary-helper audit is separate from
the unchanged final-release audit, which rejects this endpoint:

```bash
.venv/bin/python scripts/audit_temporary_helper.py --source output/helper_probe \
  --intent examples/transfers/attempt_thumb_right_helper_plane12_fast.intent.json
```

The declared helper track passes; a negative control requesting old-loop
sample 299 fails at acquisition. This audit checks every recorded frame and
reports other material contacts. It cannot turn an unfinished release into an
accepted Cup figure. The [human Cup instructions](https://www.alysion.org/figures/introkids1.htm)
specify lifting the lower old thumb string over the thumb while retaining the
upper noose; the next proposal pairs helper lift with thumb withdrawal.

A 30 mm lift paired with 12 mm withdrawal completes at 74.766667 s, retaining
the helper and both far tracks. It still fails final right old-loop release.
The little finger gains another old-loop contact patch, samples 370–380,
alongside the tracked patch 440–448 and near material 552–566. This is an
intermediate helper state, with unresolved extra contacts.

`--source-temporary-helper` permits continuation from such a declared helper
phase when its complete material audit matches the checkpoint. Both original
index and new far-material guards remain required. Every physical gate and
the opposite little-release check must pass; only the declared helper's final
release condition is deferred. The normal free-loop source validator still
rejects this state. Failed strain, hull contact, own-palm clearance, stale
reports, dropped observations and a relabelled helper all fail the new gate.
This option does not mark the helper released or a figure completed.

The recipe is `attempt_thumb_right_helper_lift30_adj.json`; its adjacent intent
file supplies the temporary-helper audit. The planning objective uses the
existing 3 mm adjoining-MCP proxy tolerance only for the ring/little overlap
already accepted at the source. The full path's original clearance checks pass.

The spline speed check now bounds each spatial interval using that interval's
quintic time envelope. Position extrema and the 3 rad/s limit are unchanged.
This avoids combining a large endpoint slope with a mid-motion speed that does
not occur there. Dense derivative checks and the full 176-test suite pass.

## Continuing the temporary helper

Raising the helper 60 mm from the same bilateral preparation retains its old
material track and both new far tracks. The right near strand clears by
2.007 mm during the held second, barely above the 2 mm threshold. The old loop
still surrounds the thumb, and the right middle and little still touch rope.
This state at 75.233333 s passes the temporary-helper source check. It fails
the final-release check.

Three follow-ups show why that distinction matters:

| Motion | Source | Result |
| --- | --- | --- |
| Roll the thumb axis down to horizontal | 30 mm helper lift | Near clearance reaches 4.506 mm, but the old helper track is lost at 76.933333 s. Rejected as a helper source. |
| Move the little away from the recorded near strand | 30 mm helper lift | The live near strand follows the finger. Helper and far tracks survive, but near/old release fails. |
| Separate the wrists another 50 mm in total | 60 mm helper lift | Helper and far tracks survive. Both near-clearance checks fail and the right old loop remains. |

Their recipes use the suffixes `right_helper_roll0`, `right_helper_clear_near`
and `helper60_spread50`. Each has a companion intent file. Continued contact
with the little is allowed only during its declared helper phase; the final
figure still requires that finger to release.

The next 20 mm transverse helper motion stops at 78.233333 s when its tracked
right far patch, near sample 639, disappears. A separate far patch remains at
samples 650–660. The stopped run stays rejected. At its valid 75.233333 s
source, however, both patches had already persisted throughout the stationary
hold: samples 636–638 and 645–662.

`--far-support-handoff right:655` explicitly transfers the next phase's
retention requirement to that already held receiving patch. It requires the
old and new representatives to occur in distinct persistent patches of the
same original far arc, a matching stationary report, and complete previous
tracking. It cannot recover an already lost track, select another strand, or
accept a contact that appears only at the final frame. The other hand and both
original index guards remain active. The declaration is saved in
`far_support_guard.json`; the ordinary frame-by-frame continuity check then
follows the receiver. Neither the joint path nor Newton forces change.

Tests cover a valid overlapping handoff, an absent receiver, an already lost
source, the wrong strand, stale reports and invalid declarations. The full
186-test suite passes. A fresh replay must still pass its physical and helper
audits; changing the tracked patch does not establish old-loop release.

That replay completes at 78.933333 s with both far tracks and the temporary
helper retained. Its right near clearance is at least 4.214 mm during the held
second. At the original stop time, 78.233333 s, particle positions, particle
velocities, joints and rest lengths match the stopped replay exactly. The
handoff changes which already held patch is required, without changing motion
or physics. The right old loop and unused middle/ring/little contacts remain.

A diagonal pull of 40 mm forward and 12 mm upward also completes its hold.
Right near clearance reaches 7.535 mm; left near clearance narrows to 2.226 mm.
Old-loop sample 415 moves from −2.190 mm to +8.281 mm along the thumb's distal
axis. The rounded fingertip ends at +20.5 mm. This is measurable sliding toward
the tip, with the old noose still around the thumb.

![Projected old-noose motion in the thumb frame](thumb_noose_progress_20261109.png)

Doubling that displacement to 80 mm forward and 24 mm upward over 2.5 s reaches
the unchanged stretch limit at 76.3 s: the maximum segment ratio is 1.200799.
Near-strand edge 551 lies between the right ring and little MCP capsules at
that stop. Simultaneously moving the left wrist 40 mm forward and 12 mm up
still stops on stretch, at 76.333333 s with ratio 1.204007. Both stopped states
remain rejected. These observations motivate separate slower-motion and
finger-spacing probes; they do not establish a completed Cup figure.

The slower 5 s replay still jams the near strand, reaching ratio 1.316773 at
edge 550 and stopping at 77.666667 s. Opening only the ring finger cannot meet
the planned clearance without approaching its neighboring middle finger. A
joint adjustment of both fingers clears the observed near-strand passage in
planning and reduces edge 551's ratio to 1.005688 in Newton. That replay reaches
a different jam at 76.9 s: old-loop edge 411 is between the thumb DIP and index
MCP capsules, with ratio 1.228630. It is rejected too. The next planning task
is to keep both passages wide enough for the 2 mm rope while preserving the
required material contacts.

`inspect_stretch_stop.py` reports the terminal checkpoint's most stretched
material edges, their original strand memberships and exact edge/capsule
gaps. It reproduces the two locations above without advancing the simulation:

```bash
.venv/bin/python scripts/inspect_stretch_stop.py --source output/failed_probe --count 5
```

The resulting `stretch_diagnostic.json` includes the checkpoint hash and stop
event. Nearby surfaces help choose the next clearance proposal; the report
does not measure pinch forces or establish a cause by itself.

## Both old thumb loops clear

The next proposal opens both observed rope passages while preserving the
helper pull. It raises the right index reference 5 mm, allowing the thumb DIP
and index MCP/PIP capsules to separate. A 4.5 mm planning target supplies
interpolation margin for the existing 4 mm passage requirement. The right
middle and ring also move to clear the near-strand passage. The left wrist
follows 40 mm forward and 12 mm upward. These references command robot joints;
the rope remains a freely moving Newton object.

The smooth path takes 5.933333 s under its 2.496 rad/s speed bound, followed
by a 1.2 s hold. The whole right old-loop arc clears during the motion and
stays at least 10.905 mm from the thumb during the final held second. Right
near clearance is 15.646 mm. Physical checks pass except the intentionally
engaged right little's final-release condition. The original indices, both
far tracks and the declared helper track all remain continuous.

The recipe is `attempt_thumb_helper60_diagonal80_passages_buffer.json`, with
its companion intent file. Replay from the validated 60 mm helper state using
`--source-temporary-helper --far-support-handoff right:655`, alongside the
existing original-support, far, collision and stretch guards.

This endpoint is accepted only as a temporary-helper source. Left near
clearance is still approximately zero; right middle clearance reaches only
1.457 mm, while right ring clearance is 5.960 mm. The release comparisons below
continue from this validated helper state.

## Unloading the right little

Two little-only motions start at 82.366667 s. They first straighten the DIP
over 1 s, then flex the MCP to 1.55 rad over 2.2 s, followed by a 1.2 s hold.
One also flexes the PIP to 1.2 rad; the other keeps PIP and DIP straight.
All other joints stay fixed.

Both motions preserve the original indices, new far strands and old-loop
clearance. Both clear the left near strand and the unused right middle and
ring. Neither releases the little. The physical audit and exchange audit fail
only their right-little release conditions; the temporary-helper audit passes.

| Final held-second minimum gap | Curled PIP | Straight PIP |
| --- | ---: | ---: |
| Left near strand / thumb | 4.538 mm | 4.380 mm |
| Right near strand / thumb | 14.507 mm | 14.524 mm |
| Left old loop / thumb | 79.575 mm | 79.554 mm |
| Right old loop / thumb | 7.952 mm | 7.979 mm |
| Right middle / rope, including knuckle meshes | 4.226 mm | 4.283 mm |
| Right ring / rope, including knuckle meshes | 7.166 mm | 7.169 mm |

At the straight-PIP endpoint, little contacts remain on material patches
173–182, 359–385, 452–478 and 532–551. The old helper patch remains at the MCP
root, so removing the distal bend alone does not free it. Contact-patch
observations describe where the rope lies; they do not measure friction force.

The recipes are `attempt_thumb_release_helper_unhook_fold.json` and
`attempt_thumb_release_helper_straight_fold.json`, each with a companion intent.
They use the same guards as the preceding helper continuation, without a new
far-patch handoff. Their start time for release/exchange audits is 82.366667 s.
The helper declaration is a release attempt, so continued helper contact is a
failed release even when it permits a further temporary-helper continuation.

Moving the left wrist 50 mm outward during the straight-little fold opens the
planned inter-hand passage to at least 9.972 mm. Its Newton replay still leaves
the little engaged and brings the left near strand back into thumb contact.
Both far tracks and original indices survive. The recipe is
`attempt_thumb_release_helper_straight_spread50.json`; extra hand separation
does not by itself solve this release.

The extended-little comparison keeps the MCP at its starting angle after
straightening PIP and DIP, then uses the same 50 mm left-wrist translation.
It also retains both far tracks and original indices, but fails little release
and left near clearance. Three terminal little-contact patches remain:
353–398, 454–476 and 534–553. Its recipe is
`attempt_thumb_release_helper_extend_spread50.json`.

A further continuation starts from the straight-fold state at 86.766667 s.
It withdraws the little MCP center 30 mm along the negative initial shaft axis,
while allowing the straight shaft to reorient. Thumb center/axis and index
surface references guide the other joints. A bounded cubic B-spline keeps the
joint curve inside its control poses' limits; it takes 3.966667 s under a
1.985 rad/s bound, then holds for 1.2 s.

At 91.933333 s, both far tracks and original indices still hold. Minimum
held near clearances are 5.227 mm left and 17.048 mm right; old-loop clearances
are 104.276 mm and 14.275 mm. Only the right-little release checks fail.
The rope follows the little root during withdrawal, so root displacement
does not establish relative sliding or release. The recipe is
`attempt_thumb_release_helper_base30_bspline.json`, with its companion intent;
use 86.766667 s as its release/exchange audit start.

Read-only planning also rejects a strictly fixed-axis withdrawal and direct
rigid wrist pivots at the arm limits. No rejected planning endpoint is loaded
as a physics state. Reorienting the helper while preserving the new supports
remains the next release problem.

A separate 0.5 s little-MPR roll, with every other joint fixed, stops at
87.133333 s: edge 550 reaches a stretch ratio of 1.576319 against the unchanged
1.2 gate. That edge is on the right near arc, next to the little MCP. The
terminal geometry identifies the affected material and nearby surface; it
does not establish a friction-force diagnosis. The recipe
`attempt_thumb_release_helper_mpr_roll.json` is a rejected experiment, and
its stopped state is excluded from continuation.

A common wrist tilt supplies another release direction without changing the
relative hand arrangement. Both hands rotate 60 degrees and retract 100 mm
over 4 s, leaving the straight right little nearly downward. The planned
relative-pose drift is at most 0.0333 mm and 0.00271 degrees. Finger angles,
rope length and friction stay fixed.

The Newton replay and 1.2 s hold finish at 91.966667 s. Both far tracks,
original indices and the old helper track survive. Two of the four right-little
contact patches leave, but the remaining root patches do not. The right near
strand contacts its thumb again, and right old-loop clearance falls to
0.943 mm during the held second. Left near clearance is 1.973 mm, just below
the unchanged 2 mm requirement.

The physical source check also fails left-little release: that unused finger
briefly contacts rope during the final held-second window. Its terminal frame
is clear, with a capsule-edge gap of 7.912 mm, which does not erase the earlier
contact. This endpoint is rejected as a continuation source. The recipe is
`attempt_thumb_release_helper_gravity60_back100.json`; further experiments
must replay from the validated pre-tilt state and check a complete new hold.

The `release_helper_gravity60_settle24` comparison repeats that tilt from the
valid source with `--hold-duration 2.4`. At the earlier endpoint, 91.966667 s,
particle positions, velocities, joints and rest lengths match the short-hold
run exactly. The longer replay ends at 93.166667 s but still has left-little
contact within its final held second. Extra settling time therefore does not
resolve this contact, and the longer endpoint also stays rejected.

A separate preparation extends only the right thumb DIP from 0.801839 to
0.2 rad over 1 s, then holds for 1.2 s. It starts from the validated
86.766667 s straight-little state and ends at 88.966667 s. Both new far tracks
and original indices survive. Right near clearance is at least 22.213 mm and
right old-loop clearance is 10.714 mm; both near/old-loop and all middle/ring
checks pass. Only final right-little release fails. This preparation is an
accepted temporary-helper source for new tilt trials. Its recipe is
`attempt_thumb_release_helper_thumb_extend20_before_tilt.json`.

The numerical record includes `planned_hold_duration_s` for these release
trials. The final hold is a `continue_cup.py` argument, separate from the motion
recipe; reproduce it with the recorded duration.

### Extended-thumb tilt and downward pull

Repeating the common 60-degree tilt from the extended-thumb preparation
avoids the unused left-little contact. The 4 s motion and 2.4 s hold end at
95.366667 s. Original indices, new far strands and the declared right-little
helper remain tracked. All physical source checks pass except the intentional
right-little contact. This is an accepted temporary-helper source, although
the right old and near thumb strands have returned to contact.

A comparison reduces wrist spacing by 30 mm during the same tilt. It passes
the other physical checks. The original helper tracker rejected it at 92.3 s;
the boundary-erosion correction described below recovers a complete material
track on the unchanged trajectory. It is now an accepted temporary-helper
source. Right-little and right old/near thumb contact still remain.

From the accepted fixed-spacing tilt, lowering only the left wrist by 80 mm
over 2.5 s and holding for 2.4 s ends at 100.266667 s. Both hands retain their
finger angles and the right hand stays fixed. The left support centers finish
below the right little fingertip. The noose contacts extend along the little
shaft, but the finger does not leave the rope. All original-index, new-far and
helper tracks survive; right old/near thumb contact also remains. This state
is valid for another helper phase, not for a released-loop continuation.

A separate preparation from 88.966667 s changes only the unused left little:
MCP 1.55 to 1.0 rad and DIP 0.8 to 1.3 rad, over 1 s with a 1.2 s hold.
It uses earlier tilted-rope frames only as planning obstacles, then replays
from the accepted source. Both old-thumb loops and near strands remain clear;
all support tracks survive. Its terminal state is a backup preparation for
further tilt experiments, not evidence that the subsequent tilt succeeds.

These recipes have suffixes `release_helper_extended_thumb_gravity60`,
`release_helper_extended_thumb_gravity60_close30`,
`release_helper_left_lower80` and `release_helper_left_little_clear` under
`examples/transfers/attempt_thumb_`. Each has a declaration made before replay
and a numerical record with its source, hold duration and acceptance result.
None completes Cup & Saucer.

### Contact-patch boundary erosion

The convergence trial exposed a tracking error. At 92.266667 s, its old-loop
helper patch spans particles 461–482, with representative 482. In the next
frame it spans 460–480. Twenty previously contacting particles remain, but
the midpoint jumps to 470, twelve particles from the representative. The
midpoint rule incorrectly rejects this continuous patch contraction.

The tracker now accepts a unique successor that overlaps the preceding patch,
choosing a contacting representative within the unchanged eight-particle
limit. In this example it chooses 480, a shift of two. Split successors,
excessive shifts and missing contacts still fail. Calls without a previous
patch retain the earlier rule. Runtime transfer guards and the index/helper
audits now carry the preceding patch explicitly. This changes tracking only;
rope forces, trajectories, contact distances and physical thresholds stay
unchanged. The numerical record preserves the original rejected audit beside
the new assessment. All 190 tests pass, including boundary erosion, seam
wrapping and retained failure cases.

The axial withdrawal comparison `release_helper_lower80_base30` is a retained
failure: at 101.566667 s its left far contact changes from 226–234 to 220–223.
Those patches do not overlap, and midpoint 221 exceeds the eight-particle
limit from representative 230. The fix does not recover that run. It stopped
before its hold and is not a continuation source.

The separate `release_helper_lower80_thumb_reclear` trial moves only the right
thumb over 1.5 s, then holds 1.2 s. The 102.966667 s endpoint retains both
original indices, new far strands and the little helper. Old-loop clearance
increases but remains below 4 mm, and the near strand still touches the thumb.
Its valid helper endpoint supports a further thumb-only separation attempt.

The next thumb-only trial, `release_helper_lower80_thumb_reclear10`, ends at
105.666667 s after another 1.5 s motion and 1.2 s hold. Right near clearance
passes at 2.321 mm throughout the held second. Right old-loop clearance is
2.610 mm, below the 4 mm requirement, while the little helper remains engaged.
All other exchange checks pass. This is a valid helper source, not Cup.

Further separation in `release_helper_lower80_thumb_margin10` loses the right
new far bight at 106.5 s. Its preceding frame holds particles 669–678; the next
frame has no right far contact at all. The runtime guard stops the replay.
This is a contact loss, not a patch-boundary tracking error, and the endpoint
is rejected.

Raising only the straight right little MCP from 1.55 to 0.4 rad in
`release_helper_reclear10_little_lift` clears the right old loop by about
18 mm during motion. However, the new far bight slides toward the unchanged
thumb tip and loses contact at 107.6 s during the hold. The preceding frame
holds 670–674; the stopped frame has no right far contact. The little still
touches rope, so this endpoint is also rejected. Further trials need to retain
the new thumb bight while withdrawing the helper.

The independent `release_helper_close30_lower100` comparison starts from the
reassessed converging tilt. It lowers the left wrist 100 mm over 2.5 s and
holds 2.4 s, ending at 100.266667 s. Both original indices, new far strands
and the temporary helper remain tracked. Physical checks pass except for the
intentional right-little contact; right old/near thumb contact remains. It is
a valid helper source, but the larger downward pull alone does not release
the noose.

`release_helper_thumb_hook90_little_lift` tries curling the right thumb DIP
to 0.9 rad before the little lift. It stops in the thumb preparation at
106.466667 s: the tracked 671–672 far patch disappears and only separate
material 645 remains in contact. The little-lift phase never runs. Curling
that joint alone does not preserve the existing bight; subsequent proposals
must account for the moving contact surface.

## Reproduce

First reproduce the cleared orbit follow-up at 61.166667 s using
[the preceding recipes](orbit-transfer.md#reproduce). Call that output
`output/cleared_orbit`. Keep its matching material-support report.

```bash
.venv/bin/python scripts/continue_cup.py --source output/cleared_orbit \
  --output output/thumb_open_both --route-only --side both \
  --approach-path examples/transfers/attempt_thumb_open_both10.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --checkpoint-every-frame --max-stretch-ratio 1.2 --hold-duration 1.2 \
  --stop-on-support-loss
.venv/bin/python scripts/audit_retained_supports.py \
  --source output/thumb_open_both --start 6
.venv/bin/python scripts/audit_loop_release.py \
  --source output/thumb_open_both --start 61.166667 --released both

.venv/bin/python scripts/continue_cup.py --source output/thumb_open_both \
  --output output/thumb_bilateral --route-only --side both \
  --approach-path examples/transfers/attempt_thumb_bilateral_pivot.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --checkpoint-every-frame --max-stretch-ratio 1.2 --hold-duration 1.2 \
  --stop-on-support-loss --release-thumb-supports both
.venv/bin/python scripts/audit_retained_supports.py \
  --source output/thumb_bilateral --start 6
.venv/bin/python scripts/audit_retained_supports.py \
  --source output/thumb_bilateral --start 6 --indices-only
.venv/bin/python scripts/audit_loop_release.py \
  --source output/thumb_bilateral --start 62.966667 --released both
.venv/bin/python scripts/audit_thumb_exchange.py \
  --source output/thumb_bilateral --start 62.966667 --side both
```

The four-original-support audit is expected to fail after intentional left
thumb release. The index-only and physical audits pass; the exchange audit is
expected to fail right near/old-loop clearance and right middle/ring clearance.
Do not ignore any other
new failure in a fresh replay.

To reproduce the extra stationary hold:

```bash
.venv/bin/python scripts/continue_cup.py --source output/thumb_bilateral \
  --output output/thumb_capture_hold --hold-only --side both \
  --hold-duration 1.2 --check-self-collision --check-robot-hulls \
  --check-body-envelope --checkpoint-every-frame --max-stretch-ratio 1.2 \
  --stop-on-support-loss --release-thumb-supports both --stop-on-far-loss
```

Run the same three material/physical audits and the exchange audit on that
output, using 67.366667 s as the phase start for the latter two.

Other continuation recipes use the `attempt_thumb_` prefix. Their source and
expected stop are listed in the numerical record. Fixed-wrist fingertip
proposals, shaft proposals and rejected inverse-kinematics candidates are kept
separate from successful physical holds.

The rope still has 689 positive-mass particles, 1 mm radius, friction 1.2,
96 substeps and 32 iterations. Rest length stays 1.363453388 m after 6 s.
There are no anchors or runtime rope-state replacements. The stretch limit
remains 1.2 and the joint-speed limit remains 3 rad/s.

The original Opening A is independently placed. The earlier cleared-orbit
source has an unresolved conservative right thumb/own-palm proxy overlap of
about 0.661 mm; opening the right CMR removes that overlap in these proposals.
Left-only branches preserve that source limitation. None of the sampled audits
certifies all swept collisions or full robot self-collision.
