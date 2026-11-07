# Transfers after Opening A — work in progress

For the current result, portable inputs and commands, start with the
[experiment guide](../experiments.md). This chronological journal preserves
superseded hypotheses and rejected runs. An `output/` name below is a local
run identifier, not a file distributed by the repository.

The target sequence is Opening A → Cup & Saucer → boat variation → return to Cup & Saucer → Star. The individual transfer operations remain explicit: thumb pickup of the far index strings, removal of the original thumb loops, little-finger release, little-finger pickup of the cup's upper central strand, thumb release and repickup, then right little-finger release. Completion requires retained intended strands, released old strands, clear fingers, stable holds, and passing physics checks over the continuous trajectory. The existing Opening A strain/contact failures are also still open.

## Evidence so far

A left-thumb experiment continues the actual 21-second Opening A checkpoint. A four-waypoint joint-space detour avoids the near index string, followed by a reach toward the far index string. At 29 seconds the extra lowering/scooping stages are unnecessary and actually lose the new bight. Holding the 29-second state through 31 seconds retains both the new far-index bight and original thumb loop. The near-index strand remains at least 15.0 mm away; the other four index/little supports remain in contact.

The contact sequence is promising, **not validated as a completed figure**: transfer-stage peak stretch is 1.358 (limit 1.2). Minimum nonadjacent segment separation is 1.854 mm. The original Opening A still has its own failures. Cup & Saucer, boat and Star have not passed verification.

The next experiment uses the opposite middle finger to lift the old left-thumb loop over the thumb tip. This follows the tutorial's allowed opposite-hand assistance. It is not yet a successful release, and does not establish a complete two-thumb transfer.

## Reproduce the current left-thumb experiment

Use the default Opening A recipe in `docs/opening-a.md` first. Replace source paths below with its output directory if needed.

```bash
.venv/bin/python scripts/plan_thumb_transfer.py \
  --source output/publish_opening_a --output output/cup_thumb_path.json
.venv/bin/python scripts/continue_cup.py \
  --source output/publish_opening_a --output output/cup_left_rrt \
  --approach-path output/cup_thumb_path.json
# Keep the intermediate pickup; later experimental scoops lose this bight.
.venv/bin/python scripts/continue_cup.py \
  --source output/cup_left_rrt --checkpoint output/cup_left_rrt/state_0870.npz \
  --output output/cup_left_pickup_hold --hold-only
.venv/bin/python scripts/validate_transfer.py --output output/cup_left_pickup_hold
# Expected nonzero exit: strain still fails. Other checks are recorded separately.
.venv/bin/python scripts/continue_cup.py \
  --source output/cup_left_pickup_hold --output output/cup_left_assist --assist-old
```

The planner only changes robot joint targets. The Newton continuation retains rope positions, velocities and rest lengths from the saved checkpoint. No rope particles are attached or repositioned at transfer boundaries. A geometric path around a frozen strand is only a proposal: dynamic contacts, wrong-strand catches and old-loop retention must still be checked.

Procedure: [Alysion's Cup & Saucer and Star instructions](https://www.alysion.org/figures/introduc.htm). The hand/rope collision model and active length controller remain approximations described in the main documentation.

## Old-loop assistance probes

The first opposite-hand assist fails: its downward/backward wrist approach loses the right index bight before the old left-thumb loop clears. A curled right index (MCP 0.6, PIP/DIP 0.3 rad) alone does not prevent that loss. These failures are saved separately, not accepted as completed transfers.

The next probe uses `--assist-old --assist-local --assist-index-curl .6`: the left thumb presents its old loop farther forward, the right middle finger curls and scoops with the right wrist stationary, then the arm lifts and carries the loop beyond the left thumb. Run the release-specific audit after it finishes:

```bash
.venv/bin/python scripts/continue_cup.py \
  --source output/cup_left_pickup_hold --output output/cup_left_assist_local \
  --assist-old --assist-local --assist-index-curl .6
.venv/bin/python scripts/validate_transfer.py \
  --output output/cup_left_assist_local --release-old
```

This probe remains unverified until the audit confirms the intended new bight, helper contact with the original bridge, old-loop clearance and retained index/little supports together. The release audit must not infer success merely from an old loop falling off.

## Material identity and subsequent probes

The local-wrist probe finished at 45 seconds and failed. In particular, looking only at the original thumb-to-thumb bridge incorrectly suggested the old loop was released: the other branch still touches the left thumb. `original_thumb_loop` now reconstructs both branches from the Opening A support graph. Release checks use the minimum gap over the entire material arc; retention uses its maximum per-frame nearest gap during the final hold. This also avoids mistaking sliding contact particles for a lost loop. Contact alone is still not proof of encirclement.

The return-branch probe (`--assist-return`) makes contact with the old loop at 35 seconds, with the wrist fixed, but loses helper contact during the lift. It is not an accepted release. The next strategy picks up the right thumb's new bight before attempting to remove either original thumb loop.

Transfer continuation now preserves previously recorded side plans. `--support-reference output/publish_opening_a` specifies the original support graph while all motion starts from the current dynamic checkpoint; it never resets rope state. `--pickup-only` stops after approach/reach, and `--route-only` holds after supplied joint waypoints. Digit-path search now also checks clearance between the two hands.

The initial right-thumb approach catches the wrong near strand and misses the intended far strand. A lower approach (`output/cup_both_under_pickup`) reaches the far strand but also catches the near strand and loses the original right-thumb loop. Neither is accepted. A higher final target (`output/cup_both_upper_pickup`) briefly retains the right new and old loops at 34 seconds, but holding that checkpoint in `output/cup_both_upper_hold` loses the old loop by 36 seconds and retains a wrong near strand. The intermediate fingertip dipped to 1.204 m despite the higher final target. Path search now accepts a minimum fingertip height throughout the route, to test a sweep that avoids this downward withdrawal. This is a geometric planning condition, not proof of retention.

The original Opening A has a lower right-side far-index strand than the left-side counterpart, so simply mirroring the successful left thumb motion is insufficient. New right-thumb probes still start from the unmodified `cup_left_pickup_hold` dynamics, not from failed or manually repaired rope states.

The height-constrained right-thumb search (minimum tip height 1.238 m, candidate at original material point 230 plus `[0.012, 0, 0.015]` m) exhausted its 5,000 attempts without finding a clear route. This is not a proof that no route exists. Further work should change the coordinated hand pose or the endpoint proposal instead of replaying the known downward paths. No Cup & Saucer stage is accepted from these probes.

Verification this iteration: all 24 existing tests passed; the transfer test additionally covers both branches of a seam-crossing thumb loop. The original dynamics failures and the remaining Cup & Saucer / boat / Star operations are still unresolved.

## Contact-aware continuation search

Direct middle-finger assistance and a rear-under approach both touch the right far strand, then lose it during the lift. Segment-level inspection shows the strand moving onto the side of the distal capsule and sliding away. A right-thumb-first run also catches both the near and far strands. These probes are not accepted.

`scripts/search_thumb_transfer.py` now evaluates bounded thumb and wrist proposals with full Newton rollouts. Every candidate starts with the same saved particle positions, velocities and rest lengths. It checks hand clearance, support loss, wrong-strand proximity and stretch while simulating a one-second movement followed by a half-second hold. The selected candidate is replayed independently and its score must agree before exporting a continuous trajectory. Search scores rank experiments; they do not replace the transfer or physics acceptance checks.

```bash
.venv/bin/python scripts/search_thumb_transfer.py \
  --source output/cup_left_pickup_hold --support-reference output/publish_opening_a \
  --output output/cup_search_step1 --samples 24 --workers 3
.venv/bin/python scripts/search_thumb_transfer.py \
  --source output/cup_search_step1 --support-reference output/publish_opening_a \
  --output output/cup_search_step2 --samples 32 --workers 3 --scale .5 --seed 17
```

Step 1 selected candidate 19, reducing the final right far-strand gap to 5.93 mm while retaining supports. Its near-strand gap briefly falls below the 2 mm margin but remains separated (minimum 1.74 mm, final 2.09 mm). Peak stretch is 1.355, so this is still a draft. The exported trajectory preserves the source rope and joint frames exactly through 31 seconds and extends to 32.5 seconds. Step 2 selected candidate 14, reaching a final gap of 2.56 mm (2.69 mm worst in the hold), still with a deficient near-strand clearance margin and excessive stretch.

The search can also sample wrist rotation (`--wrist-rotation-std`), refine around a saved target (`--center-proposal`), or independently replay a particular proposal (`--candidate-file`). Proposals, individual rollout scores, selected target and source-checkpoint hash are retained alongside the exported continuation. Selected scores are checked against a fresh replay; they are not acceptance certificates.

`cup_search_step3` evaluated wrist rotations. Candidate 24 maintained contact with the far strand but also touched the near strand. Refinement around it (`cup_search_refine24`) did not resolve that contact. Candidate 33 from that refinement was replayed separately as `cup_contact33`, then held for two more seconds in `cup_contact33_hold`. Both old loops and the existing supports remain in contact; this is still **not** a completed right-thumb pickup.

The close view and geometry expose a limitation of a distance-only objective: the right far strand touches the PIP capsule, but its open-arc angular sweep about the thumb is only about 94 degrees. The known left pickup has about 185 degrees and distal contact. Mere contact with the near strand likewise does not prove it was caught in a noose. A projected closed-polygon winding number is not a reliable replacement: the artificial closing chord can pass through the finger axis.

`--score-mode wrap` therefore adds an **open-arc angular-sweep planning heuristic** to contact, support and deformation costs. It preserves the left bight and old loops, encourages the right far strand to pass around the thumb, and penalizes wrapping a near strand. This heuristic does not change any final acceptance check and cannot independently prove axial overlap or topology. A regression test verifies its open-arc measurement without inserting a fictitious closing chord.

```bash
.venv/bin/python scripts/search_thumb_transfer.py \
  --source output/cup_contact33_hold --support-reference output/publish_opening_a \
  --output output/cup_search_wrap1 --samples 40 --workers 3 --scale .5 \
  --seed 53 --wrist-rotation-std .12 --score-mode wrap
```

Arm-assisted continuations now retain the actual saved wrist orientations, including those found by the search, instead of silently returning to the original upright wrist frames.

The first wrap-aware round selected candidate 36: support loss remains zero, the far strand remains in contact, and the final right open-arc sweep grows from about 94 to 112 degrees. It is still short of a captured bight. `search_thumb_sequence.py` runs a bounded succession of these searches, starting each round from its selected predecessor's checkpoint. It stops at a *candidate* based on sweep/contact/retention measurements, or at its round limit. It does not certify Cup & Saucer or bypass subsequent validation.

```bash
.venv/bin/python scripts/search_thumb_sequence.py \
  --source output/cup_search_wrap1 --support-reference output/publish_opening_a \
  --output output/cup_wrap_chain --rounds 6 --samples 40 --workers 3
```

This run is in progress. Per-round results are saved in `output/cup_wrap_chain/history.json`, with detailed candidate rollouts in each round directory. The output root must be new to avoid overwriting an existing search. The complete Cup & Saucer, boat and Star sequence remains unverified.

## Solver resolution and local tube contact

The six-round wrap search completed without a captured right bight. Its first two
rounds increased the minimum angular sweep to roughly 135 degrees; the last four
selected a stationary hold. `--horizon 2` now evaluates two sequential targets in
each Newton rollout, including intermediate support loss and a final half-second
hold. This allows a temporary detour rather than requiring every single target to
improve the final-contact heuristic. `output/cup_wrap_horizon2` is an experiment,
not an accepted transfer. Its kernel provenance records the pre-locality-fix
solver imported by that running process.

Resolution probes replay the same saved commands and physical checkpoint. Over
14–16 seconds, the 32-substep/24-iteration control reproduces the original peak
stretch of 1.90168 exactly. Increasing either substeps or iterations reduces this
interval's peak to 1.067–1.112. However, the **full** 0–21 second 64/48 replay loses
both intended pickups and original supports: the original trajectory cannot simply
be reused with higher precision. Its maximum stretch is 1.2354. It fails full
validation and must not replace the published Opening A.

The original minimum segment separation (1.724 mm at 3.6 seconds) is between
closest points only 1.725 mm apart *along the rope*. Fixed index-distance exclusion
misclassifies such densely sampled continuous neighborhoods as self-collisions.
The new kernel excludes a pair only when the shortest arc distance between its
closest points is no more than the sum of their interpolated radii. It retains the
existing broad phase and mass-weighted response for nonlocal crossings. An unevenly
sampled straight-rope regression produces a spurious correction up to 0.587 mm
with the original kernel and zero in the tested interior after the fix; crossing
separation and center-of-mass regressions still pass, including a cyclic seam case.

`segment_separation(..., minimum_arc_distance=...)` offers the corresponding
optional diagnostic. The old default validation gate is unchanged. With a 2 mm
arc exclusion, the original full Opening A minimum is 1.877 mm and the unsuccessful
64/48 replay minimum is 1.949 mm. These values describe nonlocal separation only;
they do not establish successful manipulation. A fresh 32/24 replay from the exact
initial state (`output/opening_locality32`) retains both cross-hand pickups, the
threading event and all original supports over the full 21 seconds. Peak stretch
is still 1.7728, so it is not accepted. Its minimum nonlocal separation is 1.8726 mm.
A 32/48 replay tests additional iterations without doubling the substep rate.
Future exports record the locality rule and diameter; search/probe manifests record
the kernel hash. Saved experiments made with the old kernel remain experimental.

The full test suite passes 28 tests. Successful old-loop removal, Cup & Saucer,
boat and Star have **not** been demonstrated. Motion retuning under the corrected
contact model and whole-trajectory physics checks remain necessary.

## Two-target search result and coordinated preparation

The two-target search completed all 64 candidates and selected candidate 19.
Independent replay matches the score; all source rope/joint frames through 48 s
are bit-identical. The exported targets are at 49 and 50 s with a hold to 50.5 s.
Minimum right far-strand sweep is 140.6 degrees, support loss is zero and the
continuation peak stretch is 1.1843. It is still not a captured bight, and the
whole earlier trajectory still fails physics checks.

The corrected-contact Opening A places the left far strand about 29 mm higher
than the old baseline. The old thumb-only approach is unreachable. Preparatory
motion now supports holding after index flexion (`--prepare-only --index-curl`)
and optionally changing little-finger MCP/PIP/DIP angles. This only changes joint
commands. Each phase preserves the evolving particle state and exports a
checkpoint for replanning; there are no added rope attachments.

The first preparation (`cup_locality_prepare_v2`, both index MCPs at 0.6 radians)
retains supports. The additional `cup_locality_prepare_low` flexes both little
MCPs to 0.75 and indexes to 0.7, but **loses the right little loop** by the final
hold (29 mm contact gap). It is rejected even though it brings the far strand
within thumb reach. `cup_locality_prepare_hook` tests the same little MCP angle
with PIP/DIP extended to -0.1/-0.2, starting again from the retained 25 s state.

The support-arc planner previously treated a single incidental contact particle
as a second support. In corrected Opening A, right-thumb contact comprises its
main run 135–141 and an isolated graze at 209. Planning now recognizes cyclic
contiguous patches, allowing isolated singleton grazes only when a primary patch
has at least three particles. Multiple extended patches still raise ambiguity.
All physical contacts remain active and all gap audits use the complete relevant
material arcs. This classification helps select a planning arc; it is not a loop
or topology acceptance test. A regression covers singleton grazing, seam-spanning
patches and rejection of genuinely ambiguous patches.

Target selection can name a material point on the intended far arc (`--target-index`)
and specify approach height (`--over-height`) in both the geometric planner and
continuation. The named point must belong to that far arc. This avoids forcing
an unreachable world-Y target while keeping the intended strand explicit.

The corrected-contact 32/48 replay loses the left thumb at 7.8 s and reaches
peak stretch 2.3805. Contact relaxation 0.9 at 32/24 instead loses little-finger
supports at 7.7 and 8.1 s, with peak stretch 2.4134. Neither numerical change is
adopted as a successful replacement. The default 32/24, relaxation 0.35 remains
the topology-retaining development baseline, still with excessive strain.

The hooked-little preparation finishes at 29 s with all six support gaps retained
throughout 25–29 s. Its worst per-support gap is below zero under the capsule
contact convention, including the right little finger that failed in the straight
version. This is a successful preparatory hold, **not** a thumb transfer. The fixed
world-Y thumb target remains 12.1 mm outside the digit-only IK tolerance. A further
left-index preparation (`cup_locality_left_index_low`) uses MCP/PIP/DIP angles
1.0/-0.1/-0.2 while retaining the other hand's support pose. It is still pending
physical evaluation. Preparation can specify index PIP/DIP angles separately.
Every new continuation phase now rejects invalid joint targets and opposing-hand
penetration along the interpolated joint path before simulating it.

The complete test suite now passes 29 tests, with lint and whitespace checks clean.
No Cup & Saucer, boat or Star completion has been certified, and no experimental
output has replaced the public demonstration.

The additional left-index lowering finishes at 33 s and is **rejected**: although
the left thumb's geometric approach becomes reachable (one path waypoint), the
right little support is lost, with a final gap of 58.15 mm. Do not continue the
saved `cup_locality_index_low_left_path.json` from that failed state. The last
corrected-kernel preparation that retains all six supports is
`output/cup_locality_prepare_hook/checkpoint.npz` at 29 s; its entire 25–29 s
support audit is saved in that directory. The 25 s `cup_locality_prepare_v2`
checkpoint also retains all six supports.

Next development action: evaluate coordinated preparation with full Newton
rollouts from one of those retained states, including wrist orientation and index
flexion together. Reject any candidate that loses either little loop while making
the thumb approach reachable. Fixed finger-only lowering has now failed twice,
so do not reuse it just because static thumb IK succeeds. The existing wrap-search
engine's left-new-bight penalty does not apply before thumb pickup; preparation
needs its own objective retaining the original six supports and measuring clear,
reachable access to the intended far strand.


## Coordinated approach and missing knuckle collision diagnosis

The 25 s `cup_locality_prepare_v2` state retains all six original supports during
a further five-second hold. In contrast, the 29 s hooked-little preparation above
and the reachable `cup_prepare26` preparation lose the right little loop when
held longer; neither is a stable transfer starting pose. Full Newton preparation
search (`scripts/search_preparation.py`) checks all six supports and evaluates
thumb access after the physical rollout. Its geometric score is not a capture
certificate.

Overlapping the candidate-26 wrist/index preparation and the left-thumb approach,
then retargeting to the current material strand, produces a sustained left-thumb
contact in `cup_overlapped_left_reach`. It is rejected: the right index bight is
lost. All five tested right-index hook poses (MCP/PIP/DIP combinations in
`cup_index_hook_results.json`) also lose that support by approximately 20–22 mm.
Changing only distal joints sometimes produces identical rope motion because
the rope touches only the MCP base.

The right-index bight at the corrected-kernel Opening A checkpoint lies at MCP
local z = -8.2 to -4.2 mm. Its open-arc angular sweep drops from roughly 303° at
25 s to 54° at 26.57 s, before its nearest contact gap becomes positive. Thus
nearest contact alone overstates retention. This sweep is diagnostic, not a
general topology certificate.

The simulation omitted the official MPR knuckle meshes. An exact triangle
winding-number test (not convex-hull containment) finds three rope centers inside
the watertight right-index MPR mesh at 21 s. Their surface penetration depths are
1.48, 2.43 and 1.59 mm, before including the rope's 1 mm radius. The independent
`audit_knuckle_clearance.py` records the actual surfaces and material indices.

An optional `--knuckles mesh` model now includes all eight non-thumb MPR URDF
collision meshes, with original mesh scale and collision origin, as kinematic
bodies driven by FK. Concavities are preserved. No guide surfaces, attachments,
particle targets or massless particles are added. Geometry metadata travels
with exports and continuations; loading a checkpoint with a different body
layout is rejected rather than silently migrating its rope state.

A fresh Position 1 → Opening A rollout is in progress in `opening_mpr_mesh`.
At 7 s there are no rope centers inside any MPR mesh; the minimum rope/surface
clearance is -0.15 mm. This is only a checkpoint result. The first pickup still
has excessive strain (1.73 at 7 s), so it does not establish full physical
validity. Both new regression tests and the complete suite pass (31 tests).
Cup & Saucer, old-loop release, boat and Star remain unverified.


### Fresh knuckle-mesh results and continuation

`opening_mpr_mesh_stage1` completes the first pickup. The old second-pickup
corridor is invalid because the strand now seats lower: its best finger-proxy
clearance is -2.9 mm. The planner now tries shallow depths (13, 8 and 3 mm)
after the existing depths, without changing the clearance gate. Resuming the
exact 13 s state produces `opening_mpr_mesh_shallow`, complete through 21 s.
Both material pickups, one threading crossing, six final supports, joint limits,
proxy clearances and nonlocal rope separation pass. Strain still fails: maximum
2.0615 at 18.4667 s, with the local rest-length cap binding. The final actual-MPR
audit finds zero interior rope centers and a minimum rope/surface gap of -0.156 mm.
The final preview was rendered and inspected; no published video was replaced.

A separately initialized cap-3 run completes as `opening_mpr_cap3`. Although
peak stretch decreases to 1.6948, it loses the left index pickup and fails opposing
finger clearance during the final withdrawal. Reject that motion/parameter pair.
It is not a successful higher-payout replacement.

The cap-2 preparation `cup_mpr_prepare` retains six supports at 25 s. Reusing the
coordinated wrist/thumb route and retargeting the current material point gives
`cup_mpr_overlapped` through 29.0667 s. The left far strand stays within 0.043 mm
of the thumb during the final hold, with at least 167.5° open-arc sweep. The old
thumb loop and original supports remain in contact; nonlocal strand separation
is 1.871 mm. Continuation peak stretch remains excessive at 1.3289. This is
contact/retention evidence, not proof of a transferable new bight or Cup & Saucer.

The right index now bears on the actual MPR housing: capsule-only distance reaches
1.733 mm, but the true MPR surface retains contact. `mesh_geometry.support_gap`
uses exact signed triangle distances plus capsule distance, with conservative
AABB pruning. The temporal audit reads the exported geometry setting and includes
MPR contacts only when those meshes were actually simulated. Its maximum other
support gap is below zero over 25–29.0667 s. Synthetic tests check the signed
interior convention, reversed orientation, and root support missed by capsules.

The first right-thumb attempt (`cup_mpr_right_pickup`) catches both the far and
near branches, and also brings the left near branch onto the left thumb. Reject
it as a completed pickup. The first near-avoiding retry at 30.5667 s is unreachable
by 7 mm; the solver now explicitly rejects a requested near-avoiding sweep if
its sampled clearance is under 1.5 mm instead of trusting an optimization penalty.
A static scan found a higher route (material point 232, +15 mm height) and a
physical retry is running as `cup_mpr_right_avoid_high`. All 33 tests pass, and
F-rule lint plus whitespace checks are clean. Old-loop release, boat and Star
remain incomplete.


The higher near-avoiding right-thumb retry has now completed at 33.0667 s and is
**rejected**. At 31.0 s the right near strand clears by 2.214 mm and the far gap
is 1.940 mm. During the fixed-pose hold the moving rope seats both branches on
the thumb (at 32 s both gaps are negative). Thus static near-string avoidance
was checked correctly but does not survive the physical redistribution. The
final MPR audit still finds zero interior rope centers; deepest rope/surface
overlap is 0.302 mm. This does not fix the wrong-branch pickup.

Next action: develop contact-aware thumb motion that updates from the current
near/far material strands during the pickup and hold. It may move finger joints
but must not change rope positions, attach particles or suppress contacts. Begin
from the exact right-approach checkpoint
`cup_mpr_right_pickup/state_0917.npz` (30.5667 s), or from the retained left-pickup
state at 29.0667 s. Preserve and audit the left new far bight and both original
thumb loops while keeping each unwanted near branch clear. A fixed higher pose
has now failed dynamically, so do not repeat a static endpoint scan as though
it establishes capture. The cup, old-loop release, boat and Star remain undone.


## Moving-strand thumb feedback and left-first old-loop preparation

The preceding goal turn changed authoritative state: optional real MPR collisions
removed the root escape and produced a retained left far-strand contact. This
turn introduced `control_thumb_pickup.py`, a joint-only feedback experiment.
It reads current Newton particle positions and velocities; it changes selected
thumb joints only, optionally combined with a scripted symmetric wrist spread.
It neither edits rope state nor adds attachments. All joint commands are exported
for replay. Rate bounds account for the quintic interpolation's 1.875 peak slope.
The controller penalizes unwanted near-strand contact, loss of old/new contact,
penetration and opposing-thumb collision. An optional seating term encourages
the new strand to contact the side of the DIP capsule rather than the rounded tip.

Rejected two-thumb trials from the exact right-approach checkpoint at 30.5667 s:

- `cup_thumb_feedback`: initial contact feedback without a thumb-pair barrier
  reaches a collision-limited pose after 32 s. Its last exported state is at 32 s.
- `cup_thumb_feedback_clear`: a thumb-pair clearance term avoids that collision,
  but new contacts become intermittent and the left far strand slips.
- `cup_thumb_feedback_return`: a stronger contact term and an original-pose
  preference briefly keep both far strands touching while clearing both near
  strands, but lose left retention later. Right open-arc sweep remains about
  116°, so contact was never treated as proof of a new bight.
- `cup_thumb_feedback_spread`: moving each wrist outward 10 mm improves clearance
  but still loses the left far strand after 33.5 s. An exact 32 s checkpoint was
  preserved separately as `state_0960.npz`.
- `cup_thumb_feedback_seat`: explicit distal axial seating still fails to retain
  the right strand. Do not continue it as a completed two-thumb pickup.

A geometric inspection explains the original right failure: at 32 s its far
strand touches the DIP at distal axial coordinate 15.51 mm, beyond the 9 mm
cylindrical half-length and on the rounded tip. The old loop is farther back
on the same finger. A minimum contact gap alone misses this slipping risk.

The left-only variant is useful. `cup_left_presentation_feedback` starts from
`cup_mpr_overlapped` at 29.0667 s, using the nominal joint pose in
`left_thumb_presentation_nominal_25.npz` (tip target offset +25 mm Y, +20 mm Z),
contact weight 10, pose preference weight 0.002 and distal seating coordinate
6 mm. It completes at 32.0667 s with continuous left far contact, the old loop
and original supports retained, and the near branch well clear. At the final
checkpoint the far contact's distal axial coordinate is -1.34 mm (past the tip
hemisphere and onto the distal link's side).

`cup_left_presentation_hold` then holds every robot joint fixed for two seconds.
The independent 32.0667–34.0667 s audit records a worst final-hold far gap of
0.0233 mm, minimum open-arc sweep 163.87°, maximum other-support gap 0.0157 mm,
no donor-little loss, and minimum nonlocal rope separation 1.968 mm. The MPR
audit finds zero interior rope centers, with worst surface overlap 0.232 mm.
This confirms passive retention of this new left contact, not Cup completion.
Peak stretch during that hold is still 1.4757 and fails the strain criterion.

The original old-loop helper endpoints are unusable. The right middle finger
can reach material point 176 from below with clear hand geometry, but its body
penetrates the new left far branch by 10.7 mm. Thus the first candidate was
rejected before any rollout. Searches of bridge points and fixed wrist shifts
also found no directly usable endpoint. An endpoint IK scan with explicit
new-strand avoidance found a nearby right-middle pose with old-loop gap 0.13 mm,
new-strand clearance 1.86 mm, own-hand clearance 8.36 mm and opposing-hand
clearance 6.63 mm. Its tip misses an arbitrary geometric hint by 4.07 mm; this
is acceptable for the new contact-based target because the task is to pick the
old material strand, not to hit that invented point. No physical gate was relaxed.

`mpr_old_return_clear_approach.json` contains a two-waypoint RRT route to that
pose, checked against the new far and near strands. A continuous physical rollout
is running as `cup_old_return_middle_approach` from the exact held state at
34.0667 s. Before lifting or releasing anything, audit whether the right middle
actually contacts only the old return branch and whether the left new contact and
original supports survive. Old-loop release, the right new bight, Cup & Saucer,
boat and Star are still incomplete.

The full suite passed 35 tests; the added selected-side isolation parameterization
passed its three targeted tests (36 total test cases now). F-rule lint and
whitespace checks pass. No public demonstration or repository push was made.


The helper approach has completed at 38.0667 s. Its old-loop contact at the final
checkpoint is material point 175, with a 0.071 mm gap to the right middle finger.
In the final second that gap ranges from -0.0045 to 0.735 mm. The helper stays
clear of the new left far branch (minimum final-second gap 0.907 mm) and near
branch (5.80 mm). This is an approach/contact candidate, not a lifted or released
loop. The independent 34.0667–38.0667 s audit retains the left far contact (worst
final-hold gap 0.0362 mm), the old left thumb loop and the original supports;
minimum nonlocal rope separation is 1.968 mm. Peak stretch remains 1.4404, so the
full physics gate still fails. The preview was rendered and inspected.

The next physical state to continue is
`output/cup_old_return_middle_approach/checkpoint.npz`, with its complete command
and trajectory prefix. Use the actual contacted old return branch near material
175 to plan lifting; do not reuse the old fixed-coordinate bridge helper script.
Keep the new left far branch clear of the helper and retained on the thumb.
A successful release must clear both old-loop branches from the thumb while
preserving the new bight and other required supports. The remaining right-thumb
pickup must then be re-evaluated from the changed topology; the left-first order
is experimental and cannot be called Cup & Saucer until the resulting figure
is verified.

### Old-loop side contact, wrist lowering and support from below

The first helper lifts did not release the old left thumb loop. In
`cup_old_return_lift15` the right middle surface rose 12.74 mm before reaching
the opposing-hand clearance boundary, while material 175 rose only about
4.27 mm. `cup_old_return_lift_side` lost contact. Curling DIP in
`cup_old_return_hook` seated material 174 against the finger but did not make
an underside support. `cup_old_return_hook_lift` again lost contact (2.747 mm
final gap). These are rejected lifting attempts, not completed transfers.

The contact normals explain this failure: the original material-175 contact
normal had world Z component 0.309, the curled material-174 contact 0.220,
and the post-lift nearest contact -0.611. The finger moved past the side of the
line until the line lay below it. Merely increasing lift height is not a fix.
`scripts/control_old_loop.py` records these joint-only experiments; its collision
barrier uses `digit_clearances`. Three new regression cases bring the full suite
to 39 passing tests. The rope observations are never mutated by the controller.

An offline scan found that lowering the right wrist could place the middle PIP
under the old return line. `cup_old_wrist_drop30` physically continues the exact
38.0667 s helper checkpoint, lowering the wrist 30 mm over two seconds and
holding two seconds. Its independent audit retains the new left far strand,
old left loop and original supports, with minimum nonlocal separation 1.967 mm.
Peak stretch is 1.5231, still a failure. The old line moves down roughly 9 mm,
so static targets calculated before lowering the wrist cannot be reused.

`scripts/plan_old_loop_support.py` now evaluates the actual saved strand point,
contact normal, capsule axial seating, new-strand avoidance, and interpolated
hand clearance. The post-drop material-174 candidate has a 0.321 mm endpoint
gap and upward normal component 0.767, with static direct-path new-strand
clearance 2.806 mm and hand clearance 0.983 mm. It falls short of the nominal
0.85 normal target and is only an experimental support candidate. Its physical
approach is being evaluated as `cup_old_pip_support`. No release is certified.
Route-only continuations now retain the source transfer plan's material indices
instead of relabeling strands after every arm movement.


The physical PIP approach completed at 47.0667 s. Its selected old return
strand remains 2.13 mm above the PIP surface after holding, so the approach
alone did not capture it. The new left bight and all original supports remain.
`cup_old_pip_wrist_lift` then moves the right wrist by (0, -10, +20) mm over
three seconds, with a two-second hold. Unlike the earlier side-contact lifts,
it really carries the old return strand: final material 180 is 13.60 mm higher
than at the start, with 0.0114 mm contact gap and upward normal component 0.814.
However, the new left far bight slips off during the hold (final-second worst
gap 10.01 mm), so this is NOT a valid transfer continuation. The old left loop
remains on its thumb. Other original supports and nonlocal separation survive.

A new synchronized trial, `cup_old_pip_lift_thumb_feedback`, starts again from
the exact 47.0667 s PIP-approach state. The right wrist follows the same offset
while left-thumb joint feedback retains the new far bight and avoids the near
strand. `control_thumb_pickup.py --right-wrist-offset` supplies this motion
without modifying rope state. The seven thumb/helper tests and F-rule lint pass.
`audit_old_loop_support.py` records the closest old material, contact normal,
axial coordinate and material displacement; it explicitly does not certify
release or topology. A new rotated-capsule test checks the world-up normal sign.

#### Correction: fixed material intervals misclassified a sliding bight

The preceding claim that `cup_old_pip_wrist_lift` lost the new left bight was
based on a stale material boundary and is not supported by the current geometry.
The left-index support center moves from material 108 to 122 as rope slides;
its original plan boundary was 107. At 52.0667 s material 110 still contacts the
left thumb, but a fixed [107..67] far interval incorrectly calls it the near
strand. This also explains why fixed-material feedback chased the wrong branch.
Do not use that feedback experiment as the preferred continuation.

`audit_sliding_bight.py` independently follows the original support patches
sequentially, retaining cyclic orientation, reporting extra patches and failing
closed on contact loss, ambiguous equal-distance matches or jumps larger than
8 material indices per saved frame. It preserves the old fixed-material gap
alongside the current spatial strand gap rather than replacing the evidence.
Across 47.0667–52.0667 s, the actual far bight's worst gap is 0.0248 mm, minimum
open-arc sweep 170.74 degrees, and near-strand minimum clearance 17.02 mm. The
final far contact is material 110, while index/little/thumb support centers are
122/65/150. Thus this trial both retains the new bight and lifts the old line.
It still does NOT release the old thumb loop or pass the strain gate.

The right thumb has an additional patch at material 195–196. This is explicitly
reported, not ignored or merged into the original right-thumb support. Its
physical effect on the next lift still needs inspection. The preferred next
state is the unmodified `cup_old_pip_wrist_lift/checkpoint.npz` at 52.0667 s.
`cup_old_pip_second_lift` continues it with another (0, -10, +20) mm wrist shift;
static checks show at least 4.67 mm opposing-hand clearance and 7.09 mm helper
to current near/far strand clearance. The full physical rollout is running.

The second rigid wrist lift fails: the continuously tracked original right-thumb
patch disappears at 53.2667 s, leaving only its unrelated material-195 graze.
By 57.0667 s neither the right thumb nor the helper remains in contact. The
tracker deliberately reports failure rather than substituting the new patch.
This is a real support loss, unlike the earlier fixed-material misclassification.
`cup_old_second_lift_retained` adds right-thumb gap/seating feedback and helper
feedback but still loses the intended helper contact while the right thumb
picks up return materials 179–184. It is not an accepted continuation.

`cup_old_second_lift_surface_hold` tests a different coordination from the good
52.0667 s state. The middle finger keeps its joint pose while the wrist lifts;
the right thumb compensates using joint IK to keep its original body surface
point approximately stationary in world space. This is a target on a robot
link, not an attachment or constraint on rope particles. The optional mode is
`control_old_loop.py --retain-right-thumb --hold-right-thumb-surface --fixed-helper`.
Its physical rollout is pending. The most recent full suite passed 44 tests;
an additional parameterized case covers the new surface-target mode.

#### Hold-only control: the 52 s state is not stationary

`cup_old_lift_hold_baseline` holds all robot joints completely fixed after
52.0667 s. The original right-thumb support still disappears at 53.2333 s,
and there are no right-thumb contacts at 54.0667 s. This shows that the similar
losses in the second lift, thumb compensation and pivot trials cannot simply be
attributed to those added motions. The prior two-second hold was too short to
establish passive stability. At 52.0667 s the maximum particle speed is still
0.216 m/s, and active rest-length control changes total natural length from
0.9447 m to 0.8393 m over the next two held seconds.

`cup_old_thumb_pivot` rotates the wrist about the original thumb contact surface
(axis world -Y, 0.2 rad); geometric support-point error is under 1 micrometer and
the helper lifts 8.88 mm, but the dynamic old support still fails at 53.3 s.
`cup_old_second_lift_surface_hold` fails at 53.1333 s; its attempted joint-only
world-surface compensation cannot maintain the desired surface position under
its contact and joint constraints. Neither is an accepted transfer.

The next control experiment is `cup_old_lift_passive_hold`: restore the exact
52.0667 s checkpoint, leave every particle position, velocity and spring rest
length untouched, then disable active take-up. `continue_cup.py
--fixed-rest-lengths` records this parameter change as a time-stamped motion
physics event, while the exported configuration records the final setting.
This is passive rope dynamics, not fixed particles or hand attachments. The
full prefix used active control until the recorded event; do not interpret the
final configuration as applying retroactively to its earlier frames.

The passive-length control also fails to retain the original right-thumb loop:
its sliding-support track ends at 54.0333 s, despite rest lengths remaining
bit-for-bit identical to the 52.0667 s checkpoint. The helper contact survives
briefly, but maximum particle speed at 54.0667 s is 1.469 m/s, so this cannot be
called settled. `cup_old_lift_passive_settle` holds the same passive configuration
until 59.0667 s; this branch is diagnostic only and must not replace the best
active-control preparation.

The right-thumb seating measurement identifies an earlier problem. Before the
first lift (`cup_old_pip_support`, 47.0667 s), its old contact is at distal axial
coordinate -12.66 mm, near the back of the distal link. After the first lift
(52.0667 s), the old contact has moved to +8.03 mm, near the rounded tip. It is
therefore too late to begin retention only during the second lift.

`cup_old_first_lift_surface_hold` now retries the FIRST wrist lift from the exact
47.0667 s state, using body-surface compensation on the right thumb from the
start and a fixed middle-finger pose. Its commanded wrist displacement is
(0, -10, +20) mm over three seconds; no rope state is reset. This trial needs
an extended stationary hold beyond the previously observed failure time before
it can be used for a release attempt. No old-loop release, right new bight,
Cup & Saucer, boat or Star has been certified. Physics strain still fails.

### Return to the near-thumb bridge required for the Cup transfer

`cup_old_first_lift_surface_hold` also fails: although the right-thumb body
surface is held within sub-micrometer error through 50.5 s, the original support
is lost by 51 s and the helper later loses its return strand. This branch must
not be extended as a completed pickup. All 45 tests pass; targeted helper and
sliding-support tests also pass after the planning utility generalization.

Rechecking the traditional Cup & Saucer action changes the next plan. The
lower original **near thumb string** is the bridge between the two thumbs;
it is the strand that must be lifted over the new thumb loop. The developed
helper approaches around material 175–180 instead select the old return branch
toward the opposite index finger. Lifting that return branch is not evidence
of the specified near-string transfer. Do not continue optimizing that easier
contact as a substitute for the actual Cup operation.

The stable preparation to revisit is `cup_old_wrist_drop30` at 42.0667 s (all
original supports and the new left far contact retained, though strain still
fails). Its near-thumb bridge lies around materials 150–168, before the left
old thumb contact patch. `plan_old_loop_support.py` now supports both hands,
middle/ring helpers, PIP/DIP support and optional static wrist-offset planning.
All candidate reports retain actual gaps, normals and path clearances; none are
promoted merely because a solver returned a pose.

Bridge planning evidence, all rejected as executable targets:
- `old_thumb_bridge_dip_scan.json`: right middle, materials 160/164/168; best
  material-168 gap about 5.04 mm, with a colliding direct path.
- `old_thumb_bridge_left_middle_scan.json`: left middle remains roughly 29 mm
  away or colliding at the nominal target.
- `old_thumb_bridge_ring_scan.json`: right ring is also too far or colliding.
- `old_thumb_bridge_shift_scan.json`: right wrist (-8,+8,0) mm does not resolve it.
- `old_thumb_bridge_forward_scan.json`: right wrist (0,+25,-10) mm brings
  material 168 within 2.17 mm, but its normal is still mostly sideways (Z 0.408)
  and the direct path intersects a finger. This is not a support-from-below pose.

The next meaningful direction is coordinated wrist orientation / thumb
presentation to expose the correct near-thumb bridge, followed by physical
preparation and replanning from the changed rope. Do not revert to lifting the
old return branch solely because it is reachable. No full Cup, boat, return or
Star has been achieved; the physical strain criterion is still unchanged and
failing. The public demo and GitHub repository were not updated in these trials.

### Joint wrist/hand planning for the actual free near-thumb span

The next continuation turn revalidated the prior work and found no live trials.
The previous goal turn made progress (controllers, physical trials and a changed
strand diagnosis), but did not complete any release.

Two geometric problems were hiding in the bridge searches. First, at 42.0667 s
right ring MPR = -0.4 produces a 2.246 mm capsule overlap between ring MCP and
little MCP. `cup_bridge_ring_clear` changes only ring MPR to -0.17 over one
second and holds two seconds. It retains the left new bight and original
supports; final right-hand minimum finger gap is 3.145 mm and opposing-hand
gap 19.144 mm. Final left far contact sweep exceeds 184 degrees. Peak strain
is still 1.3622, but final strain falls to about 1.10. This does not repair the
historical overlap in the prefix; a final from-start replay must address that.

Second, the old left thumb's live contact patch at 42 s is 163–171, whereas its
recorded original patch was 169–176. Thus former bridge targets 164 and 168
were already on the thumb surface. The free bridge is bounded by the live
right-thumb and left-thumb patches; selecting a supported endpoint cannot
produce an underside helper pose with hand clearance. `free_bridge_span` now
explicitly excludes both patches and rejects missing, overlapping or reversed
supports. Its six regression cases pass, including cyclic material seams.

`scripts/plan_bridge_transfer.py` jointly optimizes seven right-arm joints plus
right thumb and middle joints, keeping a measured right-thumb body surface point
near its original world position and avoiding new strands / all hand capsules.
It never changes rope observations. The source is `cup_bridge_ring_clear` at
45.0667 s. The selected material-158, seed-1 candidate in
`near_bridge_free_span_scan.json` has gap 0.118 mm, upward normal component 0.847,
thumb surface error 0.037 mm, and at least 3.145 mm finger clearance. Its entire
linear joint path has new-strand clearance 1.996 mm and opposing-hand clearance
3.662 mm. The nominal wrist displacement is (-25.27,+2.23,-7.63) mm, plus wrist
rotation; this is a robot command, not a rope guide.

`cup_near_bridge_approach` physically replays that coordinated path over four
seconds, then holds three seconds, from the exact 45.0667 s state. The trial is
running. Validate actual original right-thumb retention, correct bridge contact,
left new-bight retention, other supports and strain before any lifting/release.

The first coordinated bridge approach failed in physical replay. Both bridge
and left-bight tracking lose the original right-thumb patch at 48.9333 s. The
middle remains about 5.1 mm beneath the live bridge just before loss. Static
endpoint contact with material 158 was not enough: the original point moved as
the rope responded. A saved draft preview is available, not a completed figure.

The new `audit_old_loop_support.py --track-bridge` follows both original thumb
patches sequentially, excludes their actual contact particles, and measures
only the free near-thumb span. It records all additional patches and stops on
lost/ambiguous support; it writes `near_bridge_support_audit.json` separately
from the fixed-material diagnostic.

The hold-only control `cup_bridge_ring_clear_hold` is materially better than
the earlier preparations. It keeps ALL joints fixed from 45.0667 to 50.0667 s.
The original right thumb, index and little tracks remain continuous with no
additional patches on those tracked supports. The moving-boundary left far
bight has worst gap 0.0090 mm and minimum sweep 176.24 degrees; the unwanted
near strand remains at least 12.75 mm away. The independent physics/timing audit
finds peak stretch 1.1513 and minimum nonlocal separation 1.970 mm over these
five seconds. This is a verified stable preparation interval, not a full-prefix
physics pass or a completed transfer.

The failed bridge route preserved a single thumb surface point but rotated the
thumb contact link by 0.412 rad (23.6 degrees). The revised planner also retains
that link's orientation. `near_bridge_thumb_pose_scan.json` material-158 seed-1
has endpoint rotation error 0.0226 rad (1.30 degrees), and its direct path retains
rope clearance 1.996 mm and finger clearance 3.145 mm. Maximum intermediate
surface-point error is 2.48 mm. `cup_near_bridge_pose` is now physically testing
that route from the same exact 45.0667 s state with a three-second final hold.

### Continuation: preserved thumb pose and live bridge retargeting

Recovered the completed `cup_near_bridge_pose` trial (45.0667–52.0667 s).
Both original thumb patches remain continuously tracked; the left new far bight
and its supporting index/little contacts also remain. Moving-boundary far gap
is at most 0.00285 mm, minimum sweep 172.17 degrees, and near gap at least
12.75 mm. Physics diagnostics for this interval report peak stretch 1.12242
and minimum nonlocal separation 1.96627 mm. The helper still misses the live
free bridge by 9.446 mm at the end, despite a nominal endpoint contact with the
old static sample. There is no pickup/release certification.

Replanned from the actual 52.0667 s rope. `near_bridge_pose_retarget_scan.json`
material 160 seed 0 has nominal gap 0.1005 mm, upward normal Z 0.9431,
axial coordinate -2.562 mm, thumb surface error 0.0134 mm and orientation
error 0.00814 rad. The full 81-sample path retains at least 3.145 mm finger
clearance and 2.000 mm clearance from the recorded new-strand samples.
`cup_near_bridge_retarget` is the physical three-second approach plus two-second
hold; it must be audited against the moving strands before any lift.

The full existing test suite passed: 51 tests in 22.78 s. These tests do not
certify the physical figure. Public demo and published repository are unchanged.

The retargeted approach fails: the original right-thumb contact is lost at
53.5000 s, before the helper carries the bridge. A stationary control from
52.0667 s (`cup_near_bridge_pose_hold`) loses it at exactly the same time.
Thus the 52 s state was transient, not a settled preparation. Original contact
moves from DIP axial -12.24 mm at 49.0667 s to +5.67 mm at 52.0667 s and
+20.05 mm immediately before release. `cup_near_bridge_thumb_seat` uses the
existing joint-only thumb retention feedback; it also loses support by 54 s.
No result is promoted. The earlier 45–50 s fixed-pose control remains valid.

A fresh sliding audit begun at 52 s using original plan centers incorrectly
fails immediately because the right-little patch has already slid nine material
indices. Rerunning sequentially from the verified 45.0667 s state reaches the
actual 53.5 s failure. Audits must warm-track original supports, not reset their
centers at arbitrary late times.

A simple right-thumb DIP curl to 0.7 rad from 45 s creates 3.287 mm overlap
with its own index. The accidentally launched replay was interrupted before
completion and `cup_bridge_thumb_hook` is rejected. This is geometric rejection,
not a candidate transfer. New planner options support bounded tip curl or a
rigid distal-thumb shape. The latter fixes PIP/DIP angles while planning arm,
proximal thumb and middle joints, so a preserved DIP pose also preserves the
MCP/PIP collision surfaces instead of merely one surface point and link pose.

`plan_bridge_transfer.py --rigid-thumb` removes thumb PIP/DIP from the solve;
`plan_thumb_pose_path.py` then refines intermediate robot configurations so the
MCP/PIP/DIP link poses remain fixed, with the middle following the desired
approach. It enforces joint bounds and finger/new-strand clearance, records
sampled pose errors, and refuses to emit a route when they fail. The method
changes robot joints only; no rope coordinates, velocities, rest lengths or
contact attachments are written. A regression test confirms immutable rope
observations, fixed PIP/DIP angles, unchanged unrelated joints and bounded outputs.

The material-158 seed-2 proposal from `near_bridge_rigid_strong_scan.json`,
refined in 12 waypoints, has maximum distal-thumb link translation drift
0.0136 mm and rotation drift 0.0000686 rad. Sampled minimum finger clearance is
3.145 mm and new-strand clearance 1.994 mm. The final static helper gap is
0.488 mm with upward normal 0.805. `cup_near_bridge_rigid` is replaying the
five-second route followed by a five-second stationary hold. The new optional
`continue_cup.py --check-self-collision` explicitly rejects same-hand finger
penetration before each phase, in addition to the existing opposing-hand guard.
Seven bridge-planning tests pass, including the new refinement regression;
modified scripts pass syntax compilation and Ruff F checks.

The rigid-distal-thumb approach passes the interval checks through 54.8667 s,
including the five-second hold. (The first route's 5/12-second waypoints rounded
to 4.8 seconds in the fixed 30 fps runner; subsequent generated routes explicitly
quantize waypoint durations to frame boundaries.) Both original thumb supports
remain continuous, and the left new far bight remains with worst gap 0.00285 mm,
minimum moving-boundary sweep 170.13 degrees and unwanted-near gap 11.28 mm.
Peak stretch is 1.15072 and nonlocal separation is at least 1.96017 mm in this
interval. The helper is now beneath the correct free bridge, final gap 0.420 mm
and normal Z 0.897; it has not yet carried or released the loop.

The same robot-only path refiner now accepts `--helper-lift X Y Z`: it selects
an actual middle-finger body surface next to the free bridge and displaces that
body surface while preserving the distal thumb geometry. The rope is an
observation only. `near_bridge_lift6_path.json` requests a 6 mm upward move,
with geometric surface displacement 5.99996 mm, thumb translation drift below
0.0157 mm, rotation below 0.000065 rad, finger gap at least 2.556 mm and
new-strand gap at least 1.490 mm. `cup_near_bridge_lift6` is the independent
Newton replay, three seconds of lift followed by three seconds hold.

The 6 mm lift does make real contact with the correct bridge: gap is 0.016 mm
at 56 s, -0.017 mm at 57 s and -0.004 mm at 58 s, with an upward normal.
It does not retain that contact. By 60.8667 s the gap is 4.60 mm and DIP axial
coordinate 12.61 mm, beyond the 8 mm cylindrical half-length. The selected
bridge material has risen 10.46 mm, but the helper slid off its tip. The left
new far bight also loses its <1.5 mm contact threshold near the end (worst
2.120 mm). Original thumbs and other supports remain, peak stretch is 1.1245,
and nonlocal separation is 1.9735 mm. No old-loop release or completed Cup.

The next probe starts again from the stable 54.8667 s checkpoint, not from the
failed lift. `near_bridge_deep_hook_path.json` advances the helper surface
(-4,+4.5,+2) mm and curls middle DIP to 0.5 rad over four seconds, then holds
three seconds. The geometry preserves the distal thumb within 0.0215 mm and
0.000083 rad, with finger clearance 1.291 mm and new-strand clearance 1.477 mm.
`cup_near_bridge_deep_hook` is the physical replay. `--helper-curl` is a robot
joint target; successful geometric interpolation is not evidence of a capture.
The path writer also checks requested surface displacement and curl errors;
it never silently emits an infeasible motion as a successful command.

The deep-hook replay retains the correct bridge: during its last second the
maximum gap is 0.01994 mm, minimum upward normal 0.8717, and final DIP axial
coordinate 3.637 mm (on the cylinder, no longer the fingertip cap). However,
it loses the left new far bight and eventually the right-little support at
61.6 s. This is a failed combined transfer, despite the real helper capture.
The coupled arm solve moved the right-little DIP link (-7.40,+18.15,-10.70) mm
while preserving the thumb, explaining why thumb preservation alone was not
enough to retain the figure.

A `--fixed-wrist` helper mode now permits only right-middle joint changes.
All original supporting fingers and both arms stay fixed. The original 6 mm
seating plus 0.5 rad curl request is infeasible under this restriction; the
path writer correctly rejects its 5.55 mm surface miss and 0.145 rad curl miss.
A smaller (+1,+2,0) mm seating with 0.3 rad curl is feasible: actual displacement
(+0.773,+2.359,-0.043) mm, DIP 0.28669 rad, maximum target error 0.427 mm,
minimum finger clearance 0.9975 mm and new-strand clearance 1.4815 mm.
`cup_near_bridge_finger_curl` replays this from 54.8667 s with four seconds of
motion and three seconds hold. Earlier failed states are not used as its source.

The finger-only seating trial retains all original supports and the left new
far bight through 61.8667 s. Far gap stays below 0.0218 mm, the moving-boundary
sweep stays above 169.63 degrees, peak stretch is 1.16183 and minimum nonlocal
separation is 1.97027 mm. The helper remains below the bridge but its final gap
is 0.565 mm (last-second maximum 1.181 mm), so this is a retained preparation,
not a sustained helper carry yet. The complete suite now passes 55 tests,
including fixed-wrist and surface/curl refinement invariants.

A subsequent fixed-wrist 6 mm lift is geometrically infeasible with the requested
curl and is rejected (2.273 mm surface error, 0.137 rad curl error). A smaller
(+1,0,+3) mm move with DIP 0.22 rad meets the unchanged geometry checks: actual
(+0.888,+0.165,+3.008) mm, DIP 0.2123 rad. `cup_near_bridge_finger_lift3`
physically replays that three-second motion and a three-second hold from the
retained 61.8667 s checkpoint. All other joints remain fixed.

The smaller finger-only lift succeeds as a local contact/retention operation.
From 61.8667 to 67.8667 s, all original support tracks and the left new bight
remain. During the full three-second hold, the helper's maximum bridge gap is
0.0436 mm and minimum upward normal 0.9299; final contact is material 163 on
DIP, axial 9.803 mm. The robot surface rose about 3 mm; the final contacting
material itself rose only 0.545 mm due to sliding, so these must not be conflated.
Peak stretch is 1.12149 and minimum nonlocal separation is 1.97481 mm.
The bridge is now physically supported while the other bights remain; this
still does NOT release the old thumb loop or complete Cup & Saucer.
A labelled draft preview is saved at
`output/cup_near_bridge_finger_lift3/portrait/final.png`.

A direct larger middle MCP extension is not safe: a -0.1 rad command raises
its surface about 8.94 mm but intersects the near-index strand (materials
120–143, worst gap -2.864 mm). Extensions of -0.4 rad also hit the opposite hand.
The next operation must route around the near strand, not sweep through it.
A negative-Y side-lift is infeasible and rejected by the unmodified target-error
and curl-error checks. All physical trials are complete; no public/GitHub update.
The positive-Y alternative is also rejected: 5.029 mm surface miss, 0.279 rad
curl miss and minimum finger clearance 0.432 mm. No route or physical replay
was emitted for either side-lift. The accepted continuation state remains
`cup_near_bridge_finger_lift3/checkpoint.npz` at 67.8667 s, with a contact on
the fingertip cap that must be handled carefully before larger motion.

Final verification: full suite 55/55 passed; after tightening the fixed-wrist
API to freeze unrelated joints to the reference even when a different desired
configuration is supplied, all 10 bridge tests passed again. Ruff F checks,
syntax compilation and git diff whitespace checks passed. Goal remains
incomplete: old-loop release, right-side new bight, Cup, boat, return and Star,
as well as the earlier full-prefix strain/collision repairs, remain outstanding.

### Friction and hand-separation factorial trials (user proposal)

The user explicitly proposed increasing friction and overall hand separation.
Added `Simulation.set_contact_friction` and a continuation `--friction` option.
The exact checkpoint is restored first; both Newton shape material friction
and soft-contact friction are changed at that time, recorded as a physics event.
Newton's XPBD kernel averages these two values; changing just one would not
apply the requested coefficient. Rope-rope friction remains 0.25. A regression
confirms positions, velocities, masses, rest lengths and time are unchanged.
Full test suite: 56 passed. `plan_hand_spread.py` commands only arm joints,
retains wrist orientations and all finger angles, and checks IK and clearance.

Four six-second trials start from the SAME 67.8667 s checkpoint:
- `cup_compare_base`: mu 1.2, no spread.
- `cup_compare_mu2`: mu 2.0, no spread.
- `cup_compare_spread20`: mu 1.2, +20 mm wrist separation over 3 s +3 s hold.
- `cup_compare_mu2_spread20`: mu 2.0, same +20 mm route.
Wrist-origin distance changes from 147.488 to 167.488 mm along the existing
inter-wrist direction. No rope placement reset, anchor or new projection.

Baseline and high-friction-only preserve original contacts and the left new
bight. Spread-only loses right-little support at 70.6333 s, then an original
thumb contact at 72.8 s. The combined trial retains those contacts and the new
bight, supporting the proposed friction increase for this separation maneuver.
However its peak stretch is 1.19440 (close to the unchanged 1.2 gate), and the
helper contact migrates to DIP axial 14.984 mm, normal Z 0.562 rather than the
baseline's 9.832 mm / 0.945. The left new-bight gap is at most 0.0589 mm and
nonlocal separation at least 1.9726 mm, but this is not completed transfer.

Larger separation does NOT automatically clear the next lift. A frozen-rope
MCP-extension scan of -0.1 rad intersects the unwanted near strand by 4.007 mm
in baseline, 3.448 mm in high-friction-only, and 7.241 mm with both adjustments.
The near strand already grazes the middle in the last two cases. This is a
geometric diagnostic, not a proposed executable path. Next probes use +10 mm
separation and mu 1.6 / 2.0 to reduce this displacement/force tradeoff.

`audit_sliding_bight.py --track-start 45.066667 --start 67.866667` now warms
material contact tracking over the common prefix but reports only the changed
interval. It fails if warm tracking loses a support; it does not re-label late
contacts to avoid a continuity failure. Full-prefix earlier physics failures
remain separate from these local interval checks.

The +10 mm follow-up trials are now complete, again with three seconds of
arm-only separation and three seconds hold from the same checkpoint. Wrist
distance is 157.488 mm. At mu 1.6, right-little support is lost at 72.8333 s;
the new far-bight gap already reaches 2.382 mm before that loss. At mu 2.0,
all tracked original supports, the left new far bight and the correct helper
bridge remain through 73.8667 s. Its far gap is at most 0.0218 mm, helper
last-second gap at most 0.00344 mm, peak stretch 1.19345 and minimum nonlocal
separation 1.96887 mm. Helper final axial position is 14.058 mm, normal Z 0.601:
it still migrates toward the tip. A labelled draft was rendered and inspected
at `output/cup_compare_mu2_spread10/portrait/final.png`.

| Friction | Added wrist distance | Retained through 6 s | Peak stretch ratio |
| --- | --- | --- | --- |
| 1.2 | 0 mm | Yes | 1.11790 |
| 2.0 | 0 mm | Yes | 1.15540 |
| 1.2 | 20 mm | No | 1.16303 |
| 2.0 | 20 mm | Yes | 1.19440 |
| 1.6 | 10 mm | No | 1.13896 |
| 2.0 | 10 mm | Yes | 1.19345 |

Here “retained” means these tracked contacts and the left new bight in this
local interval, NOT completed transfer or a full-prefix physics pass. The
fixed-rope middle-MCP extension check for mu 2.0 / +10 mm is still obstructed:
minimum unwanted-strand gap is -7.223 mm at -0.1 rad. It does not provide a
clear route for the next lift, so no such motion is executed. The +10 mm
trial has little stretch margin below 1.2 and is not promoted as an easier
handoff merely because it holds the bight.

Conclusions from these six deterministic local trials: higher friction helps
retain the loop during this separation maneuver; larger separation alone can
lose it. Neither change solves the blocked lift. Keep the mu 2.0/no-spread
checkpoint as a higher-friction candidate with more strain margin, alongside
the accepted original state, and coordinate subsequent separation with the
pickup route rather than impose a large global gap. These friction values
are simulation settings, not measured Revo3 material coefficients. Original
production defaults and published output remain unchanged. Machine-readable
results and frozen-rope clearance scans are saved in
`output/friction_spread_comparison.json`. No release/Cup/boat/Star completion
is claimed and no GitHub update was made.

### Wider initial layouts (2026-10-25)

The user authorized increasing initial rope length and hand spacing together,
then continuing the figure transitions. Position 1 originally has 300 mm wrist
separation, whereas the later Opening A pose closes to 150 mm. These are
different from the 147.488 mm assisted-transfer checkpoint discussed above.
`Motion(..., half_span=...)` and `run_single_player.py --half-span` now grow the
initial loop with the hand geometry; no finger mesh is scaled. The second
pickup retains the recorded initial span and already accepts
`--final-half-span` for Opening A extension. Initial material and centerline
lengths are exported in motion metadata. A regression verifies that increasing
the initial span from 300 to 400 mm adds approximately 200 mm of centerline
without changing finger angles.

The 400 mm / mu 2 trial fails right-arm IK by 9.7 mm before pickup. At 350 mm,
initial material length grows from about 0.734 to 0.830 m. The old pickup route
intersects opposing fingers; that run was stopped. New `--pickup-x`,
`--pickup-depth` and `--check-clearance` options permit independent geometric
preflight. A -4 mm X offset with zero insertion depth clears opposing fingers
by at least 2.98 mm, but the physical pickup only grazes the strand and loses it.
`wide35_shallow_stage1` completes 10 seconds with peak stretch 2.03119 and fails
right pickup retention. A separate rest-length-cap-3 trial also fails retention,
with peak stretch 3.11116. Neither is an accepted Opening A. Robot-only scans of
deeper pickups with folded idle fingers and alternative material points fail
clearance or reachability; those scans were not executed physically.

`initialize_wide_opening.py` also tests independent Opening A placements to
isolate downstream workspace from the failed first pickup. These explicitly
start new experiments; they do NOT continue the source dynamics or demonstrate
Position 1 → Opening A. Contact neighborhoods are translated with their hand,
and intervening string spans are interpolated only before Newton starts.
An explicit initial seed is saved in metadata for deterministic checkpoint
reconstruction. No runtime particle targets, attachments or resets are used.

The resampled 240 mm fixture retains six surface contacts but catches an extra
left-thumb branch during settling (peak stretch 2.07393). The 200 mm / 20 mm
take-up fixture has the same ambiguous extra contact (peak 2.14508). Planning
rejects these multiple support patches instead of silently choosing one. The
per-material support diagnostic now includes simulated MPR triangle surfaces:
right-index contact at the knuckle must not be misreported as a missing support
merely because its capsule gap exceeds 1.5 mm. The pointwise distance regression
checks this against the existing single-point signed-surface implementation.

An optional `--preserve-sampling` keeps the seed's original dense wrap samples
and spring count; checkpoint reconstruction retains this choice, and initial
edges exceeding the 16 mm contact-query bound are rejected. This changes the
discretization experiment, not the final acceptance thresholds. At 200 mm the
straight-index placement loses the left index (peak 1.68954); the placement from
the prepared/index-curled source loses left thumb and index (peak 2.25901).
Neither is suitable for a release attempt. These failures remain exported in
their `initialization_audit.json` reports. The full suite passes 58 tests.

A further 350 mm Position 1 trial uses the original mu 1.2 during initial seating
to separate spacing/length changes from the high-friction seating change. It is
stored in `output/wide35_mu12_stage1`; later transfer friction can still be
changed only after restoring its actual dynamic checkpoint.

The mu 1.2 trial's unmodified pickup also fails geometric preflight. Bounded
idle-finger posture optimization and an 8 mm insertion depth produce a clear
path (minimum opposing gap 1.162 mm). Only middle/ring MPR/MCP angles are
modified during the first second. `wide35_idle_stage1` physically retains the
right index bight through the two-second hold. Its full original palmar arc
has a maximum hold gap of -0.106 mm. A narrow initial target sample alone is
misleading after material slides, so the full original arc is used for this
check. Prefix peak stretch is still 2.22279, not a physics pass.

The continuous `wide35_opening20` branch reduces wrist separation to 300 mm
during the second pickup, then extends to 200 mm. Both cross-hand material
pickups and one left-index threading crossing (12.4932 s) pass the existing
validator. Joint limits, arm speed, approximate self/opposing-finger clearances
and rope self-contact also pass. However, right-little support is lost at
18.4333 s; final actual-model gap is 56.833 mm. Right-index support moves to the
MPR mesh, so the capsule-only retention check additionally reports its loss,
but mesh contact does not rescue the genuinely lost little-finger support.
The report remains failed, and Cup planning rejects ambiguous support patches.

Two retries restore the exact 17.2 s checkpoint, including velocities and rest
lengths. `--extension-friction 2` changes both Newton friction channels only at
the final extension, with an explicit metadata event. `wide35_opening20_mu2`
still loses right-little support (56.959 mm final gap; peak stretch 2.34116).
Adding `--right-little-curl .8` in `wide35_opening20_hook` also fails retention
(56.814 mm final gap). Its proposed path satisfies the existing opposing and
self-proxy clearance tolerances, but that does not establish physical capture.
No new acceptance thresholds were introduced. Cup, boat, return and Star are
still uncompleted; the existing published demo is unchanged.

Reproduce the wider continuous draft (expected failed final validation):

```bash
.venv/bin/python scripts/run_single_player.py \
  --half-span .175 --friction 1.2 --knuckles mesh \
  --idle-posture examples/wide_opening_idle_posture.json \
  --pickup-depth .008 --check-clearance --output output/wide35_idle_stage1
.venv/bin/python scripts/continue_opening_a.py \
  --source output/wide35_idle_stage1 --output output/wide35_opening20 \
  --pickup-half-span .15 --final-half-span .10 --check-clearance
.venv/bin/python scripts/validate_opening_a.py --output output/wide35_opening20
```

The labelled draft preview is `output/wide35_opening20/portrait/final.png`.
Each extension branch has an `opening_support_audit.json` with actual mesh
contacts, final patches and stretch. Full tests passed 58/58 after the geometry
and checkpoint changes; the five affected Opening A/friction tests passed again
after adding phase-specific control options. F-rule lint and whitespace checks
are clean. No simulations remain running and no GitHub update was made.


### Wider Opening A extension and resolution diagnostics (2026-10-26)

The 350 mm initial / 200 mm final layout still fails; it must not replace the
published demonstration. Tracking the original right-little contact shows it
sliding from PIP (17.2 s) to the DIP tip cap (18.4 s), then disappearing.
Raising the right wrist by 20 mm at mu 2 retains that little-finger contact but
loses the left index. Distal-only little extension and a separately abducted,
back-extended little posture likewise move the failure to the left index.
Two dorsal-extension proposals at unchanged MPR failed same-hand preflight;
their accidentally launched processes were immediately terminated and marked
aborted. The continuation now enforces the same-hand proxy tolerances as the
independent validator before each phase; initial pickup preflight does too.

`wide35_little_distal` and `wide35_little_outward` also exceed the 16 mm segment
search bound (17.516 and 17.647 mm). Their apparent retention is not evidence
of valid dynamics. Saved `rejection.json` files record this. The worst local
segments reach their 2x rest-length cap; this motivated a controlled payout
experiment, not a claim that payout alone fixes capture.

`wide35_distal_cap4` restores the exact 17.2 s checkpoint and increases only
the local payout cap at extension (with mu 2 and the distal little pose). It
fails: a segment exceeds the search bound at 18.0333 s, peak length 23.036 mm,
and several supports are subsequently lost. The continuation now stops at
the first recorded out-of-bound segment and writes `aborted.json`. Acceptance
limits have not changed. New checkpoints save both length-controller mode
and local payout cap along with the dynamic state; a regression verifies
state and controller restoration. Legacy continuation metadata reconstructs
the cap applicable to the checkpoint, not a future event's final value.

Retaining left-index MCP flexion at 0.3 or 0.6 rad does not solve the larger
extension. Both lose the right little around 18.2--18.4 s. The 0.6 rad trial
also briefly loses/reacquires left-index contact, which must not be described
as continuous retention. `audit_opening_support.py` records all six contacts
against the capsules and enabled MPR meshes, including gaps, material IDs,
first missing contact, and interval/full strain. It is explicitly a contact
diagnostic, not a topology certificate. Adding a 40 mm right-wrist lift to
the 0.3 rad index trial still loses the right little and reaches a 16.066 mm
edge, just outside the unchanged collision bound.

The advanced `cup_thumb_duck11` branch folds only the left thumb PIP/DIP to
1.1/1.3 rad from `cup_compare_mu2_spread10`, then holds for three seconds.
Sliding material tracking confirms the new far bight is lost (final gap
19.325 mm), while the original loop remains on the thumb. This is not a
successful old-loop release. It should not be continued into little-loop
release or called Cup & Saucer.

Resolution experiments expose initial-state sensitivity. With 1.5 mm samples
and local cap 4, the settled loop grows to 1.403 m, and the original approach
misses arm IK by 7.8 mm; no pickup is replayed. At cap 2 the palmar strand is
still lower than the 3 mm baseline. Doubling spring stiffness to 1600 N/m
(to preserve approximate axial rigidity at half the sample spacing) does
not restore that original target height. A geometric scan with shallower
approach offsets either fails IK or intersects the opposing hand; those
paths are not simulated. New `--spacing` and `--stretch-stiffness` options
make these experiments explicit and reproducible. No runtime resampling or
rope-point relocation is introduced.

The 60-test suite passed after adding the controller checkpoint regression.
Full Opening A/Cup/boat/Star completion remains unverified, and no GitHub
update has been made for these experimental branches.


A 2 mm / 1200 N/m initialization with active pre-roll also lowers the palmar
strand and rejects the original approach (5.2 mm arm IK miss). Fixed-rest-length
pre-roll at the same resolution retains a reachable target near z=1.268 m and
the full first-pickup path passes both opposing and own-finger preflight.
However, its two-second pre-roll still has final peak strain 1.40267 and
maximum particle speed 0.30779 m/s, so it is not a settled initialization.
The complete 10-second diagnostic is exported in `wide35_2mm_fixed_settle`;
its peak strain is 1.83921 and maximum edge length 7.026 mm. The improved edge
bound does not make the strain failure acceptable. Pre-roll states, velocities
and rest lengths are now exported separately in `settling.npz`, with an
explicit `settling_audit.json`; switching length control on for the motion
preserves all positions and velocities.


The dense fixed-pre-roll first pickup does retain the full original left
palmar material arc during 8--10 s: worst gap -0.0763 mm. This remains only a
local pickup result; its strain fails and the right-little contact has just
1.214 mm final clearance. At 96 substeps / 32 iterations the fixed-length
2 mm pre-roll peak strain improves to 1.12971, but the left-thumb support is
lost (97.135 mm gap) and final maximum rope speed remains 0.58105 m/s. The
palmar-target detector correctly refuses to launch a pickup. Do not bypass
that missing-strand gate. No full-resolution first-pickup trajectory was run.

Next useful initialization experiment: reduce initial pre-tension or change
the left-thumb bearing pose before simulating the initial loop, keeping the
350 mm hand spacing and longer rope. Compare all four original supports,
settling velocity, strain and initial material routing before planning a
pickup. Raising solver resolution alone has not supplied a stable source.
All physical and audit processes from this batch have finished. Lint,
compilation and whitespace checks pass after the last CLI changes.

### Longer material budget, lower take-up load and wider Opening A seed (2026-10-26)

The 350 mm Position 1 experiments now expose initial material length, spring
damping, settling duration and operating tension as explicit parameters. Adding
30 mm of material retains all four supports for two seconds, but the left thumb
slips off at 3.7667 s during a six-second fixed-length hold. Increasing spring
damping from 0.01 to 0.1 delays the loss to 4.1 s; it does not solve it. The
contact track moves along the thumb DIP toward its tip before disappearing.
Both-thumb posture changes instead lose the right little support and are rejected.

At 0.025 N nominal tension and zero extra take-up, the left-thumb-only bearing
posture in `examples/position_one_left_thumb_bearing.json` retains all four
supports for six seconds. Local length control has peak stretch 1.04293,
maximum edge 8.016 mm and minimum nonlocal separation 2.000 mm. Uniform global
length control retains the same four supports with peak stretch 1.04316,
maximum edge 2.093 mm and minimum separation 3.879 mm. The latter has 2.650 mm
position excursion in the final half second and peak instantaneous speed
0.0994 m/s; it is not certified settled. The new `audit_position_one.py` records
these stationarity failures instead of accepting contact alone. The pickup
selector now restricts candidates to the actual thumb-to-little palmar material
arc; the previous world-coordinate filter could select another strand after
support loss. A geometric first-pickup scan still rejects all six tested offsets
for IK error or opposing-hand overlap. None were physically replayed.

A separate experiment starts directly in a wider Opening A. This is explicitly
an independent initialization, not a continuous Position 1 success. It uses the
six unambiguous support patches from `opening_mpr_mesh_shallow`, widens wrists
from 150 to 200 mm, samples at 2 mm with 1200 N/m springs, and runs at 96 substeps,
32 iterations, friction 1.2, nominal tension 0.025 N and no extra take-up. The
initial rest length is 1.36419 m. `opening20_low_tension_fixture` contains the
first three seconds; `opening20_low_tension_hold6` continues the exact state to
six seconds using `scripts/continue_hold.py`. The copied history is identical,
and the first continuation positions, velocities and rest lengths match the
source checkpoint bitwise. All six final supports still have one contiguous
contact patch. Full six-second peak stretch is 1.09542, maximum edge 10.350 mm,
minimum nonlocal separation 1.934 mm. All particles have positive mass and no
anchors. The last half second still has 2.848 mm excursion and 0.1647 m/s peak
instantaneous speed. This is a viable contact experiment, not a settled or
completed figure claim.

The new tension parameter defaults to the historical 0.08 N. Legacy checkpoints
missing that field load with this historical value only; a different current
configuration still fails the compatibility check. The 62-test suite passed
after these changes. Sixteen affected tests, including two new tests for
metadata-based transfer audit windows, subsequently passed. Transfer validation
now reads the saved Opening A boundary instead of assuming 21 s, and uses the
enabled MPR collision meshes for other-finger support checks. The prior strain,
contact and separation thresholds remain in force; full-history strain is now
reported and checked separately. Cup continuation checks the same established
finger-proxy tolerances as Opening A and stops at the 16 mm segment search bound.

A left-thumb approach has been planned from the six-second independent seed;
`opening20_cup_left_trial` is its continuous contact replay. Its result is pending.
No Cup & Saucer, boat or Star has yet passed validation. The published demo and
GitHub repository have not been updated with these experiments.

The frozen-geometry thumb route did not remain clear in dynamics. At 9 s in
`opening20_cup_left_trial`, the left thumb contacts the near strand while the
far strand is still 11.19 mm away; the run was stopped and kept as a rejection.
An exact 7.5 s prefix is preserved in `opening20_cup_left_approach1`, but later
bilateral inspection showed the right thumb already touching its own unwanted
near strand at this boundary. This prefix is diagnostic, not an accepted source.
A left-only feedback continuation was stopped after this was discovered.

Joint feedback can now follow a time-varying nominal thumb route while retaining
only the original loops during approach. It can use an explicit bilateral
material reference and stop at observed near-strand contact. The controller
still only changes joint commands; observations and rope state are immutable,
with bounded joint speed. The new four-way scope/speed/immutability test covers
one or both active thumbs and either retained-strand mode. The full test suite
passed 66/66; F-rule lint, compilation and whitespace checks also passed.

`opening20_cup_bilateral_approach1` starts afresh from the clear 6 s boundary.
It preserves all six supports and keeps peak stretch below 1.096, but stops at
7.3 s when the right near-string gap falls to 1.155 mm. It is not a pickup.
A high thumb sweep with a 1.275 m minimum tip height has no path in the current
finger-only configuration; a higher 1.2907 m endpoint misses IK by 11.2 mm.
The next experiment lowers the right wrist by 20 mm over two seconds, retaining
its orientation and finger angles, to lower the two remote string endpoints
before left-thumb overpass. Its kinematic interpolation passes both opposing
and existing same-hand proxy-clearance tolerances. `opening20_prepare_right_lower20`
is the corresponding continuous contact rollout; this does not modify any rope
positions, velocities or attachments. Physical result pending.

### Upper thumb approach with coordinated wrist height (2026-10-26)

The 20 mm right-wrist lowering completes without losing any of the six supports.
`opening20_prepare_right_lower20` has full-history peak stretch 1.13858,
maximum edge 10.366 mm, and minimum nonlocal separation 1.988 mm during the
preparation. Both near-strand thumb gaps remain above 1.5 mm throughout that
preparation. Its labelled draft preview is in `portrait/final.png`.
An additional 40 mm lowering also completes with six retained supports in
`opening20_prepare_right_lower60`; full peak stretch is 1.16106 and maximum
edge 10.472 mm. Larger height difference does not by itself produce a static
collision-free finger-only sweep; it is a separate preparation experiment.

The thumb planner now optionally excludes paths passing below a vertical curtain
through the measured near strand. This is a tip-path planning constraint only,
not a rope edit or a topology certificate. Its segment interpolation preserves
multiple branches and horizontal segments, with a regression test. Fixed high-tip
and curtain-constrained RRT proposals still fail in the tested configurations.
A bounded joint scan finds an upper route that permits transient contact with
the near strand. The selected left-thumb goal is CMP 0.7, CMR 1.2, MCP 1.329,
PIP 0.5, DIP -0.1 rad, with the arms and other digits unchanged. This must be
judged after physical replay, not by the frozen-rope candidate's gap.

`opening20_left_overpass_contact` continues the 20 mm preparation from 9 to 13 s,
using a three-second sweep and one-second hold. All six original supports remain.
Full peak stretch is 1.15434, maximum edge 10.379 mm, and minimum nonlocal
separation in the new interval is 1.990 mm. The tip stays above the near-strand
curtain (at least 23.981 mm in the saved 9--12 s probe), while the near strand
contacts the thumb's middle segment from 9.8 s onward. At 13 s the far, near and
old-loop material all contact the left thumb; the right near strand is clear by
7.942 mm. This is explicitly NOT a valid new-bight result, because the unwanted
near contact remains. The contact-supported overpass is being continued through
reach, lower, scoop and lift in `opening20_left_scoop_trial`, using material 282
on the original far strand. That replay is pending; no old loops or little-finger
loops have been intentionally released yet.

`continue_cup.py` now supports an explicit prepared-state `--skip-approach` and
custom hold labels, so an approach hold is not labelled as a completed pickup.
An optional near-contact stop is useful for strictly clear preparation runs;
contact-permitting overpass experiments retain the original final acceptance
criteria. Transfer validation can assess one active side while still requiring
the inactive thumb's old loop and near-strand clearance. It additionally checks
index/little retention across the whole transfer interval, not just the final
second. The eight affected transfer/planning tests and F-rule lint pass after
these additions. No acceptance thresholds have been loosened.

A fresh independent 200 mm Opening A seed is also being compared using global
rather than per-edge rest-length control (`opening20_global_tension_fixture`).
This starts from the same initial placement and nominal 0.025 N tension, not a
runtime resampling or state reset of the ongoing scoop branch. Its result is
pending. This test targets the strong particle clustering observed with local
length control. No GitHub update has been made.

The original Cup & Saucer instructions were rechecked during the scoop run:
https://llx.com/Neil/string/playground/cup_and_saucer.html explicitly calls for
returning with the far index string after passing over the near strings and
under the far string. The current lift-to-a-local-target alone does not return
the thumb to its original side. `opening20_left_return_reference.json` therefore
contains a subsequent three-second return to the original left-thumb angles,
keeping both wrists and all other digits unchanged. Its interpolation from the
predicted lift endpoint passes the established own-finger clearance tolerances.
It has not yet been physically replayed. This completes the intended pickup
motion before judging near-string clearance; it does not change the acceptance
criteria. Both new bights and old-loop removal remain unverified.

### Complete return stroke and length-control comparison (2026-10-26)

The local-length scoop completed approach, lower, scoop and lift at 21 s. The
left far-material gap is 1.771 mm at that boundary and near material still
contacts the thumb. This is not an accepted pickup. The run was stopped before
its redundant intermediate hold, and `opening20_left_return_trial` resumes the
exact `state_0630.npz` lift boundary for a three-second return and one-second
hold. No rope state is edited. Its physical result is pending.

The independent global-length fixture finishes all three seconds with six
continuous support contacts. Full peak stretch is 1.10215, maximum edge 2.600 mm,
and minimum nonlocal separation 1.934 mm. All particles have positive mass and
there are no anchors. Its last-half-second maximum excursion is 2.260 mm; it is
not declared stationary. `opening20_global_hold6` continues unchanged joints
for three more seconds for comparison with the existing local-length fixture.
Full regression suite: 67 passed in 29.24 s.

The explicit return completes at 24 s and holds to 25 s. Near-string clearance
recovers to 3.55 mm, but the far string is 27.97 mm away; the old loop remains.
This rejects the pickup despite finishing the human motion sequence. All copied
frames through 21 s match the source trajectory exactly. Full validation pending.
The recorded reach had pushed the far string ahead of the thumb: at 15 s its
nearby material is x=0.4050 m, beyond the thumb tip x=0.3994 m. Consequently the
old target was no longer beyond/below the moving strand. A new branch from the
exact 15 s checkpoint extends the thumb to x=0.4330 m, z=1.2799 m before
remeasuring the target (`opening20_left_farther_reach`); no pickup accepted.

`opening20_global_hold6` retains all six supports throughout its extra three
seconds. Full peak stretch is 1.10215, maximum new-interval edge 2.572 mm, minimum
nonlocal separation 1.991 mm. Initial continuation state and copied prefix match
bitwise. Last-half-second excursion is 0.553 mm but maximum instantaneous speed
is 0.1660 m/s, so this is not certified stationary.

At 16.6 s the extended-reach tip is x=0.43304 m, z=1.27990 m, while nearby
far material 278 is x=0.42922 m, z=1.24190 m. The tip is now beyond the far
strand in X. A continuous lower stroke aims 12 mm beyond and 18 mm below that
measured point, followed by another measurement (`opening20_left_retarget_lower`).
This is a geometric candidate only; near material still contacts the thumb.

Sliding support audits now use an exact contact-membership query instead of
computing every far-away signed mesh distance. The threshold and meshes are
unchanged; AABB lower bounds only prune points that cannot satisfy the contact
query, and all interior candidates are retained. Nine real-pose comparisons at
6, 13 and 21 s return identical material-index sets; seven relevant tests pass.
This changes audit cost only, not simulation dynamics. Updated sliding audits
confirm no unexpected support patches in the tracked supports, but the rejected
return has no far bight: its final far gap is 27.971 mm and angular sweep 117.15 degrees.

The 25 s return validation finishes: only `left_new_far_bight` fails; both old
loops, both near clearances, all index/little supports throughout the transfer,
full strain, separation and dynamic-rope checks pass. Minimum nonlocal separation
is 1.988 mm. This is a clean failed pickup, not a completed figure.

The new lower stroke ends at 18.2 s with all six supports retained and full peak
stretch 1.15511. Far material has moved down to z=1.21533 m (near x=0.39710 m),
while the tip is still at z=1.22390 m. Thus fixed-position lowering again moved
the supported string. The next command changes only the left thumb DIP to 1.55 rad,
keeping proximal thumb links fixed, to curl the distal tip underneath without
pushing the bearing PIP down. `opening20_left_tip_curl` is pending. The generated
`opening20_retarget_scoop_route.json` was not replayed because its starting
tip remained above the measured far strand.

Contact-membership optimization also applies to support-arc construction. The
additional six-support comparison on the global fixture matches all material
sets exactly; 13 affected geometry/tracking/planning tests and F-rule lint pass.
The global six-second preview is saved in `opening20_global_hold6/portrait`.

The distal-only curl finishes at 19.3 s. Its tip is (0.42041,0.07990,1.20930)m;
nearby far material282 is (0.39046,0.07803,1.22492)m, so the tip is now 15.63 mm
below that strand. A four-DOF thumb IK proposal keeps DIP fixed at 1.55 rad and
aims 8 mm behind and 16 mm below the observed far point (residual 0.00041 mm).
Both the under-string draw and return pass same-hand/opposing-hand clearance
checks. `opening20_left_curled_return` continues with 1 s draw, 2 s return and 1 s
hold; physical result pending. The other hand and arm joints remain fixed.

At 21 s, the curled draw/return has far contact on the DIP side at local axial
coordinate -2.303 mm (cylindrical half length 9 mm), with an open-arc sweep of
187.38 degrees. Near clearance is only 0.365 mm, so this is not an accepted
pickup. The exact `probe_0630.npz` is saved. By 22 s, continuing to the original
pose loses far contact (28.05 mm gap); the baseline completes at 23.3 s.

`opening20_left_curled_return_mu2` resumes the exact 21 s state and changes
friction from 1.2 to 2.0, then follows the same remaining joint polynomial and
one-second hold. Its continuation endpoint is still22.3 s and source pose at
21 s was checked against that polynomial to 1e-8 rad. This preserves the
interrupted joint phase as well as rope state; result pending. For a different
route or feedback from this interior-phase checkpoint, do not silently reuse
the old 20.3 s command as a new interpolation origin: an explicit measured
21 s boundary is required to avoid a command discontinuity, and the inherited
source command prefix must remain available for physical replay.

### Curled pickup and friction-only result (2026-10-26)

Both curled-return branches are complete at 23.3 s and rejected. The original
1.2-friction branch fails new-far-bight retention and inactive-right near
clearance (1.003 mm minimum in the final hold). Its full peak stretch is
1.15511 and full transfer minimum separation is 1.978 mm; all original supports
and both old loops remain. The far gap reaches 26.741 mm during the final hold.

The 2.0-friction continuation has an identical rope prefix through 21 s and
identical joint commands at every exported frame (maximum difference zero).
It also loses the new bight (24.262 mm worst hold gap), reduces inactive-right
near clearance to 0.536 mm, and increases full peak stretch to 1.24784, which
fails the existing 1.2 limit. Its new-interval minimum separation is 1.990 mm
and maximum edge 11.288 mm; all six original supports remain. This higher
friction branch is not an improved accepted transfer. Exact comparison is in
`output/opening20_curled_return_friction_comparison.json`, with rejection notes
in both outputs. All simulations and audit processes from this experiment ended.

The useful retained probe is `opening20_left_curled_return/probe_0630.npz`: at
21 s the far strand contacts the DIP cylinder side, but the left near gap is
only 0.365 mm. It is not yet a correct bight. Next investigate holding/seating
that pose while clearing the near strand instead of opening the thumb all the
way to its original pose. Any new feedback must begin at the measured 21 s
joint state (with an explicit boundary and preserved original command prefix),
so that a truncated 20.3 s waypoint does not cause a first-step joint jump.
Keep the inactive thumb/old-loop clearance in the audit; the completed full
return showed that the other hand also needs coordinated control.

The independent 200 mm Opening A fixture with 1.364 m initial rest length
remains a useful baseline; its global-length six-second hold retains all six
supports. It is not evidence of continuous Position 1 formation or of a finished
Cup & Saucer. No Cup, boat, old-loop release or Star has passed validation,
and the active objective is incomplete. No public repository update was made.

### Feedback from the seated 21 s contact probe (2026-10-26)

The preceding goal turn made progress: completed physical pickup/return and
friction-only comparisons, retained an exact DIP-side contact checkpoint, and
rejected the failed completed returns. No simulation or audit was still live
when this turn revalidated process state. The full figure objective remains open.

`Motion.resume_at` now creates an explicit measured joint boundary for an
interior-phase checkpoint while preserving the original clipped quintic curve
in `motion.interrupted_segments`. Sampling the historical prefix is unchanged;
future feedback starts from the actual checkpoint, not a stale earlier waypoint.
The prefix polynomial is serialized with trajectory motion metadata, which must
accompany the command knots for exact replay. Regression tests cover JSON
round-trip prefix equality, first-step speed, branching again inside the saved
prefix, and rejection of inconsistent checkpoint joints. Thirteen targeted tests
and F-rule lint pass. On the real 21 s probe, boundary error is zero and the
first proposed feedback step peaks at 0.75724 rad/s under a 0.8 rad/s limit.
No rope state is changed by this command-boundary correction.

`opening20_left_seat_feedback` starts from the exact `probe_0630.npz`, using
that same pose as its preference, not the original fully opened thumb posture.
It controls only the left thumb for two seconds, with contact weight 10,
near clearance target 2.5 mm, DIP seating limit 4 mm, prediction 0.25 frames,
pose weight 0.001 and speed limit 0.8 rad/s. Friction remains 1.2. Physical
result pending; this is not a correct-bight or full-figure claim.

The full suite passes 69 tests. The two-second adaptive-length feedback ends at
23 s with far contact within 0.484 mm and left near clearance 1.190 mm; near
clearance is still below the 2 mm acceptance threshold. The contact slides
toward the DIP tip. `opening20_left_seat_hold` tests a 1.5-second passive hold.
The 21 s joint/rope/rest-length prefix is bitwise unchanged. Replaying commands
with the saved prefix-polynomial metadata at exact frame/fps times matches all
recorded joints within 1e-8 rad. Display timestamps are rounded to six decimals
and must not be treated as full-precision integration times in this replay test.

`opening20_left_seat_fixed_feedback` repeats the two-second feedback from the
same 21 s state with further rest-length updates disabled after loading the
checkpoint. Existing material lengths, positions and velocities are unchanged.
The controller parameters and friction 1.2 are identical. The physics event is
recorded in metadata. This tests whether continuing material-length redistribution
contributes to bight migration; result pending.

The adaptive feedback's passive hold completes at24.5 s. The far strand stays
within0.892 mm during its final second, but left near clearance falls to0.108 mm
and right near clearance to1.630 mm. This remains an unaccepted bight. Freezing
rest lengths during feedback also fails the clearance requirement: final-second
near minima are0.711 mm left and0.190 mm right, while far contact stays within
0.654 mm. All restored rest lengths are bitwise unchanged after21 s in that
branch, confirming that the comparison did not silently rescale the material.

To reposition the left near strand independently of its newly held far strand,
we now command the right thumb's original-loop bearing point. The left-near
closest point is on the PIP side; its outward normal is approximately
(0.815,0.302,-0.495). A10 mm right-bearing displacement along that normal has
1.992 mm IK residual, at least2.231 mm frozen right-near clearance along its
path, and positive same-/opposing-hand clearance margins. This is a proposal,
not evidence that friction will carry the loop. Attempts to move the left index
bearing instead fail own-finger clearance and are not executed.

`control_thumb_pickup.py --scripted-support-joints` now allows explicitly named
non-feedback finger joints to follow the nominal route while the selected thumb
uses contact feedback. It rejects arm joints, active feedback joints, missing
routes, and prescribed trajectories exceeding the same speed limit. It cannot
modify rope state. `opening20_partner_thumb_feedback` starts again at the exact
21 s probe, moves only right-thumb support joints over2 s, and runs left-thumb
feedback for3 s with the previous parameters. This retains adaptive length and
friction1.2; the fixed-length comparison did not improve clearance. Physical
result pending. Seven affected feedback/resume tests and F-rule lint pass.

The first partner-thumb replay was stopped after the saved22.5 s boundary:
resampling its smooth nominal curve into per-frame quintics amplified actual
peak command speed to1.35786 rad/s, above the0.8 limit. It is marked superseded
in `opening20_partner_thumb_feedback/rejection.json`. `scripted_support_target`
now applies the speed bound to each actual frame command, accounting for the
quintic peak slope; nominal tracking may lag, which is explicitly logged. The
new regression samples the resampled trajectory densely and verifies both the
speed limit and final target arrival while unrelated joints stay unchanged.
Eight feedback/resume tests and F-rule lint pass after the correction.
`opening20_partner_thumb_bounded_feedback` restarts the same physical21 s state
with that corrected bound; its result is pending.

The full suite passes70 tests after the first speed-bound fix. Per-joint lag
also distorted the supporting thumb's planned posture: at22.5 s its bearing
point had moved only1.53 mm along the requested direction. The rate limiter now
uses a common fraction across all named support joints, retaining their planned
ratios while lagging. The updated dense regression verifies this relationship
as well as speed. `opening20_partner_thumb_coordinated_feedback` compares that
variant from the identical21 s state; it is still running.

The preceding per-joint-bounded physical branch completes at24 s, with left
far contact0.488 mm and near clearance1.776 mm at that instant. Its right near
strand entered contact during the run, despite the frozen-rope preflight. At
23.5 s the commanded old-loop bearing moved8.353 mm along the intended normal,
while the original bearing material moved11.226 mm and near material300 moved
only0.685 mm. Thus the remote thumb did move the rope, but a prescribed pose
alone does not preserve its own near clearance.

`--assist-other-thumb` now closes feedback on that supporting thumb as well.
It retains only the assisting thumb's old loop and near clearance; it does not
request or seat an unpicked far strand. Per-side pose preferences allow the
supporting thumb to follow the proposed bearing motion while the pickup thumb
prefers its seated pose. A regression moves the assistant's unused far strand
by10 m and verifies that the feedback result is unchanged. Nine targeted tests
and F-rule lint pass. `opening20_bilateral_support_feedback` starts the same
21 s state with both thumbs controlled, left pickup only, right pose weight0.02,
and all other controller parameters unchanged; result pending.

The coordinated scripted support and bilateral-feedback comparisons both finish
at 24 s and remain rejected. During their final second, coordinated support has
left near clearance 1.253 mm and right near contact; bilateral feedback improves
right clearance to 1.600 mm but left clearance falls to 0.882 mm. Their far-strand
contact stays within 0.975 and 0.882 mm respectively. Each output has a contact
window audit and rejection reason; neither is an accepted pickup.

`opening20_bilateral_spread15_feedback` tests 15 mm additional outward wrist
translation on each side over three seconds, starting from the exact 21 s probe.
It uses bilateral feedback with left pickup only, 4 mm near-clearance targets,
right pose weight 0.002, friction 1.2 and the existing adaptive length controller.
All source rope positions, velocities and material lengths are restored unchanged.
At 22 s left/right near gaps are 2.664/3.713 mm and left far gap is 0.838 mm;
completion and passive retention remain pending.

A frozen-rope multi-start pose search found alternative curled left-thumb poses
with roughly 2.9 mm near clearance, but these are only geometric candidates in
`output/opening20_static_seated_thumb_search.json`. No pose has been substituted
into the physical state. The spread comparison takes priority before replaying
another posture variant.

`continue_cup.py` now uses the same `Motion.resume_at` boundary reconstruction
as feedback, so route/hold branches from interior-phase checkpoints also retain
the original prefix curve. The full suite passes 71 tests after the assisting-
thumb regression; the continuation call-site change reuses that tested helper.

The spread experiment stops before the next step after23.1 s: opposing hands
remain128.9 mm apart, but the left thumb DIP approaches the left index MCP.
The saved pose is within the existing self-clearance tolerance by78.5 microns;
the rejected step would cross it. No colliding step is simulated. The output's
`rejection.json` records this distinction.

The feedback loss now includes thumb-to-all-finger capsule clearance. Its step
backtracking enforces the same fixed self-clearance margins as preflight instead
of allowing another0.1 mm loss relative to each preceding frame. A regression
uses the recorded close thumb/index posture and local rope observations, checking
12 consecutive feedback steps and densely sampling each interpolated command.
Ten targeted tests pass after reconstructing the original idle-finger posture
in that fixture. F-rule lint, Python compilation and whitespace checks pass.

`opening20_spread_clearance_recovery` resumes the exact23.1 s physical state
with the corrected feedback, adding the remaining2.4275 mm outward travel per
wrist over one second. At23.5 s far contact is0.858 mm, left near2.946 mm and
right near5.573 mm. This is still in motion, not passive-retention evidence.
The saved rope state is never edited or reset.

The independent `opening20_left_seated_return_replay` instead commands the
static search's seed6 thumb posture from the original21 s state over two seconds,
with speed-limited support interpolation and right old-loop/near feedback. At
22.5 s the intended far and unwanted near strands both contact the left thumb.
It is an exploratory deeper-seat posture, not an accepted new bight.

The corrected feedback reaches25.1 s with wrist-center distance230.868 mm and
material rest length1.580429 m. Its source positions, joints and rest-length
history through23.1 s are bitwise identical; replaying all joint commands at
frame/fps times has zero error. All masses remain positive and no anchors exist.
All six original supports persist, full peak stretch remains1.155113, and the
new interval's maximum edge is10.358 mm. The full code suite passes72 tests.

`opening20_spread_passive_hold` then freezes every joint for1.5 s, continuing
Newton and the existing length controller to26.6 s. The left far strand remains
within0.792 mm throughout the hold; original loops remain supported. However,
left near clearance reaches1.921 mm (below the unchanged2 mm gate), so this
cannot yet be called an accepted pickup. Sliding support tracking completes with
no extra index/little/right-thumb contact patches; its final far angular sweep
is172.43 degrees, a diagnostic rather than a topology proof.

The deeper curled-posture alternative also finishes at24 s. It holds both the
wanted far and unwanted near strands against the left thumb (roughly214 and
216 degree projected sweeps), so the frozen-rope endpoint clearance did not
predict physical separation. It is explicitly rejected; all six original
supports nevertheless remained present in that experiment.

Right-thumb overpass planning on the25.1 s state finds routes targeting far
material621 and625. The625 proposal loses near clearance after the passive
hold, reaching1.214 mm when checked against the26.6 s rope, so no executable
route is written. A prematurely started dependent runner exits on its missing
route before advancing any physical frames; `opening23_right_approach_feedback`
is marked aborted. Planning is repeated against the actual held state before
any right-thumb replay.

The complete26.6 s transfer audit finishes: every existing gate passes except
left near clearance (1.921 mm versus2 mm). Full peak stretch is1.155113, minimum
nonlocal centerline separation1.978199 mm, all index/little supports persist
throughout, and the fixed-joint hold completes. Joint deviations during that
hold are only2.22e-16 rad from floating-point interpolation, recorded explicitly
rather than claimed bitwise-identical. This remains a failed acceptance result.

Replanning against26.6 s yields a two-waypoint right-thumb upper approach toward
far material625. Densely sampled frozen-state near gaps are1.686/2.204 mm,
right self margin0.754 mm, left self margin2.140 mm, and opposing-hand gap at
least119.504 mm. Tip height stays at least20.430 mm above the near-string
curtain. Durations2.3+1.0 s keep nominal quintic speed below0.8 rad/s.

`opening23_held_right_approach_feedback` runs a five-second continuous experiment
from that exact held state. Left feedback retains the far/old material; right
feedback retains only its old loop and follows the approach preference with
pose weight0.05. Both near targets are4.5 mm; no right far contact is requested
before the under-string motion. At27 s left far gap0.897 mm, left near2.714 mm,
right near4.262 mm and both original loops remain contacted. This tests the next
coordinated motion, not a declaration that the prior failed gate now passed.
All planned-vs-actual geometry and retention still require evaluation at its end.

During the coordinated right approach at29.5 s, left near clearance improves
to3.319 mm while the left far strand remains within0.734 mm. Right near clearance
is4.969 mm and its old loop remains contacted. The right far strand is still
19.302 mm away, consistent with an upper approach rather than a pickup. This
is a useful continuous two-hand preparation; no Cup/boat/Star figure is claimed.

The five-second coordinated upper approach completes at31.6 s. Right tip error
from its planned upper waypoint is2.614 mm, right far gap15.509 mm, left far
gap0.624 mm, left near3.265 mm and right near4.387 mm. Both old loops remain
contacted and hand self margins remain positive. The subsequent1.2 s fixed-joint
hold (`opening23_right_approach_hold`, ending32.8 s) has left far gap at most
0.879 mm and near gap at least3.001 mm; sliding support tracks remain continuous
with no extra tracked support patches. Full validation is running.

`plan_thumb_transfer.py --forward-offset` now exposes the world-X tip offset
from observed far material while preserving its original -12 mm default. The
first right forward target is +12 mm X and +18 mm Z from far material622.
Single-seed IK gets stuck with15.6 mm error. A16-seed static search finds precise
endpoints with3 mm near clearance, but all nine valid endpoints' direct joint
paths intersect the near strand (about11--12 mm). None is directly replayed.
A +45 mm height single-seed proposal also fails IK. A bounded800-attempt path
search checks whether a curved joint route can avoid the near strand; result
pending. Static endpoint success is not treated as a valid physical path.

The coordinated alternative `opening23_right_raise40_feedback` resumes the exact
32.8 s state and lifts only the right wrist40 mm over3 s, with bilateral thumb
feedback and a further1 s observation interval. The arm path has opposing gap
at least120.510 mm, unchanged positive own-hand margins, and conservative actual
resampled speed bound0.11953 rad/s. Left feedback retains its new/old strands;
right retains only its old loop. The goal is to create vertical clearance for
right overpass, not to reposition any rope particles. Physical result pending.

The32.8 s full transfer audit passes all existing left-side pickup gates without
changing thresholds: left far retention0.879 mm, left near minimum3.001 mm,
right near minimum4.542 mm, both old loops and all other supports retained,
full peak stretch1.155113, nonlocal separation1.978199 mm, positive masses,
zero anchors, and the completed hold. This is the first accepted left pickup
checkpoint in this widened chain. It is still an independently initialized
Opening A experiment and is not a completed Cup, boat, or Star certificate.

The right-wrist lift fails and is stopped early at saved34.0 s. Right old-loop
support first disappears at33.3 s (22.350 mm gap); by33.5 s the gap is76.175 mm
and the unwanted right near strand contacts the thumb. The rejected physical
history is preserved. Before the lift, old material415 rested on the right
DIP cap at axial+10.143 mm, beyond its9 mm cylindrical half-length. This explains
why contact retention at rest did not imply a secure bearing during wrist motion.
The800-attempt curved joint search also found no clear path; no route was emitted.

`--seat-old-support` now applies the same distal seating objective to the
assisting thumb's retained old loop. It requires an assisting side and an explicit
seat depth; it never asks that side to pick up the far strand. The regression
also verifies unused far-material invariance with old-loop seating enabled.
Eleven affected tests pass. `opening23_seat_right_old_loop` restores the exact
accepted32.8 s state, keeps wrists fixed and uses0.4 rad/s joint limits for a
2 s seating experiment. Left far/old support remains active; right old-loop
seating targets axial4 mm. Physical retention and axial progress remain pending.

The complete suite passes73 tests after adding assisting-old-loop seating. The
accepted32.8 s left-pickup checkpoint has a labelled draft preview at
`output/opening23_right_approach_hold/portrait/final.png`; this image does not
claim the right pickup or full figure is complete. The latest seating experiment
is still running. No experimental output has replaced the published demo.

Old-loop seating from the tip-loaded32.8 s posture also fails: by33.5 s the
right old gap is68.103 mm while the unwanted near strand contacts the thumb.
The experiment completes at34.8 s and is rejected. Comparing sources identifies
the transition: at26.6 s the old loop was on right PIP axial+6.432 mm (inside
its12 mm cylindrical half-length); the upper-approach optimizer curled right
DIP from0.2 to1.2131 rad, migrating that old bearing onto the rounded DIP cap.
Trying to reseat after this migration has not proved reliable.

`opening23_right_extended_overpass` therefore branches from the earlier exact
26.6 s state. It preserves the prior old-loop bearing opportunity by using a
mostly extended distal right thumb: CMP0.7, CMR1.2, MCP1.329, PIP0.5, DIP-0.1.
The command takes3 s with a further1 s observation, while left pickup feedback
continues. This is a contact-supported overpass, analogous to the earlier left
approach: temporary right near-strand contact is expected, and it is not an
accepted final pickup. Tip-over-near margin is at least46.144 mm in the frozen
preflight, hand/self margins remain positive, and nominal peak speed is0.2964
rad/s. The existing final pickup/near-clearance/retention gates are unchanged.
All right contacts are logged even though only the left thumb uses feedback.
Physical old-loop retention and subsequent reach remain to be assessed.

The extended-right-thumb overpass completes at30.6 s. Right old material410
remains on the PIP cylindrical portion at axial-9.077 mm, while right far
material617 and near material602 also contact the PIP/MCP region. This is still
an intermediate three-strand contact state, not an accepted right pickup.
Left far contact and near clearance remain preserved. Original-support audit
is running for the26.6--30.6 s interval.

`opening23_right_curl_under` now continues that exact state, commanding right
PIP1.0 and DIP1.55 rad over4.2 s with1 s additional observation. The frozen
endpoint tip is(0.399606,-0.117257,1.203415)m; the far strand at the same Y is
(0.371079,-0.117257,1.219477)m, placing the proposed tip28.527 mm beyond and
16.063 mm below it. Sampled hand/self margins remain positive (right minimum
0.118 mm), and the tip retains56.556 mm over-near-curtain margin along the route.
Nominal peak speed0.73661 rad/s is below0.8, and actual support commands keep
the per-frame bound. Only right thumb joints are scripted; left thumb feedback
continues to retain its new/old strands. No rope state or acceptance gate changes.
The moving-strand result still needs measurement before planning the return.

The extended-overpass actual-mesh support audit completes: all six original
supports remain present throughout26.6--30.6 s, interval peak stretch1.052666,
full peak1.155113, and maximum interval edge10.358 mm. Right far/near/old contacts
remain distinct recorded material patches; this is not mislabeled as a completed
right pickup. At32 s of the continuing curl, both old loops and left new support
remain, while right near contact persists. The observed scripted-joint lag is
0.0263 rad; the actual per-frame speed remains bounded.

The primary instructions were rechecked at
https://llx.com/Neil/string/playground/cup_and_saucer.html and
https://www.alysion.org/figures/introduc.htm : the intended sequence remains
far-index pickup on thumbs, removal of the lower original thumb loops, then
little-loop release. Cup-to-boat is a posture change; Star starts again from
Cup. The current right curl/return is still the pickup stage, not old-loop release.

The right curl completes at35.8 s. Its actual tip remains16.029 mm below the far
strand crossing at the same Y, close to the frozen prediction. All six original
supports remain present through30.6--35.8 s; interval peak stretch1.044832,
full peak1.155113 and maximum edge10.360 mm. Material rest length is1.611716 m.
Right near contact remains, so this is still an intermediate state.

A bounded four-joint fit (right DIP fixed at1.55) targets8 mm behind and16 mm
below the measured far material617. The reachable endpoint has1.465 mm error,
right self margin0.754 mm and opposing gap145.423 mm along the sampled path.
Trying an extended index produces the same fit; no index change is needed or
executed. `opening23_right_under_draw` resumes35.8 s and commands only right
thumb joints over2.2 s plus1 s observation, while left feedback continues.
Final right target is CMP0, CMR0.77808, MCP0.58993, PIP1.5533, DIP1.55. This
is robot motion only. New-bight retention, unintended near contact and subsequent
return must be assessed from the resulting dynamic state.
A compact path/status index is saved in `output/opening23_transfer_milestones.json`.

The right under-draw completes at 39.0 s. All six original supports remain
present throughout 35.8–39.0 s on the actual simulated capsule/knuckle geometry;
interval peak stretch is 1.044569 and full-history peak remains 1.155113.
The right target strand contacts DIP at axial -0.656 mm, while its unwanted
near strand also contacts DIP/PIP. Thus the draw is not an accepted right pickup.
The left target remains within 0.792 mm and its near clearance is 2.336 mm.

`opening23_both_pickup_feedback` resumes the exact 39 s state with both thumbs
retaining their far and original loops, 4.5 mm near-clearance preference and
4 mm distal seating preference. The right nominal pose returns only one quarter
of the way toward its 26.6 s pose over 2 s, followed by 1 s observation. Commands
remain bounded to 0.8 rad/s; no rope state or acceptance threshold is changed.
At 40 s right near clearance has become positive (0.104 mm), still well below
the 2 mm acceptance gate. Final retention and clearance remain pending.
A frozen-state exploratory fit omitted a penetration penalty and returned an
8 mm old-loop overlap; its candidates are diagnostic only and were not replayed.
### Final comparison before publication (2026-10-26)

The 39–42 s bilateral-feedback experiment retains both old loops but loses
accepted right far contact (2.331 mm final gap). Holding its final joints to
43.5 s does not recover the pickup: the final-second far gap reaches 6.847 mm.

Two branches resume the exact 40 s checkpoint with contact weight 40 and a
0.4 rad/s joint-speed bound. The fixed-wrist branch ends at 43 s: right far gap
stays below 0.834 mm, but the unwanted near strand still touches the thumb.
Spreading each wrist outward by 15 mm over 3 s, then continuing feedback for
1 s, ends at 44 s. Final-second right far gap stays below 0.548 mm; right near
clearance improves to a minimum of 1.545 mm, still below the existing 2 mm gate.
Left far gap stays below 0.464 mm and left near clearance above 2.280 mm. Both
old loops retain contact. These are contact diagnostics, not a full transfer
certificate. No experiment or audit process remained active at publication.

The publication adds portable recorded-command recipes for the accepted
32.8 s left-pickup milestone and the unaccepted 44 s bilateral attempt. They
embed the initial independent Opening A seed and preserve interrupted command
polynomials. A fresh 0.1 s CPU replay of each recipe exactly matches the original
first four frames in joint angles, rope positions and rest lengths. Sampling
their commands at every original frame time (frame index divided by FPS) has
zero joint error. The full physical recipes have not been rerun from scratch
for this packaging change; the left validation file is explicitly historical.

## November 8 edge contact and hold experiments

Continuing the 44 s bilateral attempt exposed a collision-resolution gap.
The rope particles touched the finger capsules, but the straight edges between
them cut into the capsules. The deepest edge at 44 s was 0.747 mm inside the
left little PIP capsule; its length was 8.16 mm. The left and right thumb PIP
edges penetrated by 0.706 and 0.688 mm. A particle-only audit reported about
0.02 mm at these contacts.

`BimanualSimulation` reads `Config.segment_hand_contacts` to enable additional
interior edge/capsule constraints in `SegmentRopeSolverXPBD`. Each XPBD iteration distributes the
normal and Coulomb friction corrections to the edge endpoints by barycentric
weight. Relative velocity includes the kinematic body's translation and
rotation. Endpoint contacts still use Newton's particle solver. This is a
discrete capsule test; swept collisions and edge/mesh contacts are not covered.
The default remains off so the published recipes keep their original physics.

The static comparison starts with the same saved positions, velocities,
rest lengths and robot pose. Switching the model at 44 s is recorded as a
physics event. After 1.2 s, the worst sampled capsule penetration falls to
0.034 mm. Holding another 2 s keeps both requested bights and the old loops in
contact. Full validation at 47.2 s fails only the right near-strand clearance:
1.559 mm against the existing 2 mm threshold. Its full-history stretch maximum
is 1.168 and its minimum nonlocal rope-edge separation is 1.968 mm. The
particle-only static continuation reached 1.237 stretch and lost the right
bight. Neither branch establishes Cup & Saucer.

Reproduce the new-contact branch from the bundled 44 s attempt:

```bash
.venv/bin/python scripts/replay_transfer.py examples/transfers/right_pickup_attempt.json \
  --output output/opening23_both_pickup_spread
.venv/bin/python scripts/continue_hold.py \
  --source output/opening23_both_pickup_spread \
  --output output/bilateral_segment_contact_hold_20261108 \
  --seconds 1.2 --segment-hand-contacts --label 'Hold new thumb bights'
.venv/bin/python scripts/continue_hold.py \
  --source output/bilateral_segment_contact_hold_20261108 \
  --output output/bilateral_segment_contact_settle_20261108 \
  --seconds 2 --label 'Hold new thumb bights'
.venv/bin/python scripts/validate_transfer.py \
  --output output/bilateral_segment_contact_settle_20261108
.venv/bin/python scripts/audit_finger_contacts.py \
  --source output/bilateral_segment_contact_hold_20261108 --time 44 45.2
```

The validator is expected to exit 1 for this rejected trial. The continuation
commands were run from the existing 44 s checkpoint; the full initial recipe
has not been rerun with this patch. Runtime physics events are supported by
these checkpoint continuations, not by the initial-placement recipe runner.

Other branches tested greater wrist separation, a raised right wrist, right
index flexion, fixed rest lengths and increased longitudinal spring damping.
They did not pass the pickup gates. With edge contacts enabled, moving both
wrists outward by 3 mm also reduced left near-strand clearance below 2 mm.
These comparisons motivate keeping the left hand fixed during the next right
pickup adjustment.

The right-only 6 mm wrist translation and 8 mm axial seating target also failed
the final near-strand margin. A frozen-rope fit suggested a thumb adjustment
smaller than one degree; dynamic replay reached 2.660 mm clearance at 48 s,
then fell to 1.623 mm during its static hold. This is another case where the
frozen geometry does not predict the settled string position.
The [saved diagnostics](bilateral_contacts_20261108.json) include the completed
run names, checkpoint hashes, parameters and final-second measurements.
The subsequent right-index MPR change from 0.35 to 0.48 rad also fails: at
50.2 s, the final-second right near clearance reaches only 1.034 mm. Both target
bights remain within the contact threshold, but the near-strand condition is
still unsatisfied.

The episode runner now calls the same transfer evaluator as the standalone
validator. It checks the inactive thumb, other supports throughout transfer,
joint limits, and a full second of unchanged joints with the hold phase.
Incomplete episodes cannot pass. Logs are written during execution, and the
result bundle includes the commands and material plan needed for continuation.
The validator loads each NPZ array once instead of decompressing it per frame.

Verification: 87 non-integration tests passed with the contact change; an
additional checkpoint test passed afterward. The new tests cover an interior
edge hit missed by particle spheres, unilateral response, rotation, friction
against a moving body, XPBD integration and exact checkpoint continuation.
F-rule lint passes. The existing single-hand integration failure on GitHub
remains unresolved; these results do not claim a green full CI run.
