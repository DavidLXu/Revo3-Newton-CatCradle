# Timing the released loop

The [accepted release preparation](alternate-cup.md#post-release-swings-and-hand-spacing)
ends at 35.5 s with the right thumb raised and 240 mm lateral wrist spacing.
Both little fingers remain clear, and four original thumb/index material tracks
survive the hold. The next operation is to bring the free loop over the other
strings before exchanging the thumb loops. Cup, boat and Star are not yet
complete.

The latest stable intermediate clears a distal little-finger snag and holds
at 47.866667 s with peak stretch 1.16251. The left little MCP still touches the
rope. A subsequent left-thumb reach only produces intermittent proximal
contact; it has not captured a new bight. The 35.5 s preparation above remains
the latest verified state with both little fingers clear.

## A joint path can change the hand spacing

The first turning-point trial moved both wrists 80 mm forward, waited for the
loop to turn, then commanded 80 mm backward. Both endpoint poses had the right
relative hand transform. Between them, however, interpolating the arm joint
angles displaced the relative wrist position by 19.870 mm at 36.533333 s and
rotated it by up to 3.210°. Rope stretch reached 1.23279 at 36.733333 s, above
the unchanged 1.2 gate. That trial is rejected.

`rejected_phase_swing_joint_path.json` preserves its joint commands after the
35.5 s checkpoint. Replay it through `continue_cup.py --route-only`, with all
three collision guards and a final `--hold-duration 0.1`, to reach the recorded
36.9 s end. This is a failed comparison, not an accepted continuation.

`common_wrist_translation` now solves IK at intermediate Cartesian positions.
Each subsegment retains the existing quintic joint interpolation with zero
endpoint speed. Eleven poses within each segment check relative wrist drift;
the planner rejects sampled translation drift above 1 mm, rotation drift above
0.2°, or an analytical quintic joint speed above 3 rad/s. These are sampled
kinematic checks, not continuous collision certification. The nine-segment,
0.6 s reversal reduces the planned translation drift to 0.830 mm and rotation
drift to 0.126°, with peak joint speed 2.805 rad/s. A regression uses the real
robot model to reject the old direct path and accept this segmented path.

The first segmented physical comparison used 0.6 s strokes and inspected only
the instantaneous velocity at each 0.1 s observation boundary. It completed
two strokes, missed the second turning point and timed out. Its final hold at
40.3 s passes the release diagnostics and four original material tracks, with
peak stretch still 1.19185. Its highest fixed free-band position is 316.7 mm
below the thumb bridge. It did not throw the loop over the strings. The current
controller therefore uses per-frame position estimates for its timing cue.
These comparisons are recorded in [the numerical report](free_loop_control_20261108.json).

Shortening the strokes to ±40 mm in 0.3 s and using the position-based timing
cue increased the free band's swing. Its highest point was still 253.3 mm
below the thumb bridge. On stroke eight, the right thumb lost its original
material contact at 39.733333 s. Peak stretch was 1.19337, but contact loss
rejects this branch; the full release audit was not run. The controller had
already integrated to the end of that batch at 39.9 s.

## Observe, move, then check retention

`control_loop_swing.py` proposes common-wrist translations and advances Newton
through the existing continuation tool. It waits for an observed turning point
before reversing. It uses simulated rope coordinates for this development
experiment; it is not a perception policy or a hardware controller.

The timing cue follows the same fixed free-band material used by
`inspect_loop_throw.py`. It estimates velocity from three successive recorded
positions, reducing the effect of high-frequency motion in instantaneous
particle velocities. It examines every saved frame during the observation
holds, even when the next decision is made after a 0.1 s batch. Each decision
records both the observed turning time and the later reversal time.

Every batch checks the four original support tracks and the 1.2 strain gate.
The continuation tool now also accepts `--max-stretch-ratio`; the controller
sets it to 1.2 so a stretch failure stops at its first recorded frame and writes
`stretch_stop.json` with an exact checkpoint. Earlier trials above checked
stretch only after each batch.
Failures stop the controller; a batch may already contain later frames after
the first failed sample, so its final state must not be used as a valid restart.
Finger/palm preflight, central-body envelopes and complete-edge robot-hull
guards remain enabled. No rope coordinates, velocities, rest lengths or
friction parameters are altered by the controller.

```bash
# First reproduce and audit output/raised_thumb_spread using alternate-cup.md.
.venv/bin/python scripts/control_loop_swing.py \
  --source output/raised_thumb_spread --output output/phase_swing \
  --amplitude 0.04 --stroke-duration 0.3 --cartesian-steps 9 \
  --probe-step 0.1 --apex-min-offset 0.015 --strokes 9
```

`swing_control.json` records settings, measurements, decisions and rejection
reasons. `joint_commands.json` preserves the executed motion. The continuation
tool's optional `--checkpoint-every-frame` saves each complete dynamic state,
so a later thumb-capture trial can resume the exact moment of interest rather
than reconstructing velocities from a rendered frame. A checkpoint from this
path was checked against its corresponding exported physical frame.

An observed free-band height above the thumb bridge only identifies a possible
capture time. It does not establish the necessary threading, a thumb-loop
exchange or a completed Cup. The usual full-history release and material
audits are still required after a trial, followed by figure-specific checks.

## Returning the hands upright

A geometric alternative is to lower the thumb bridge relative to the index
supports by returning the whole figure upright. This could make the next loop
transfer easier, but changing the wrist orientation does not itself establish
the required crossing. The first three 240 mm spacing comparisons all failed:

| Wrist path from 35.5 s | Result |
| --- | --- |
| Advance 50 mm in 0.6 s, then rotate 120° toward upright | Stretch reaches 1.21727 at 35.8 s, before rotation. All four original material tracks survive the later 41.566667 s hold, but the earlier stretch failure invalidates the trajectory. |
| Advance 50 mm in 1.8 s before that rotation | First stretch failure at 35.7 s; peak 1.32530 at 35.866667 s. Interrupted rather than completing a known-invalid branch. |
| Rotate directly, without advancing | The frame-level gate stops at 36.066667 s with stretch 1.20200. |

The slower-translation replay also verifies the new stretch guard: its time,
joint, rope and rest-length arrays exactly match the unguarded prefix through
the first failure at 35.7 s. Its saved checkpoint matches that physical frame.
The accepted 240 mm hold should not be treated as evidence that it can safely
enter the following motion.

Starting instead from the 200 mm preparation at 33.1 s allows the full 120°
upright return and a 1.2 s hold. At 38.566667 s, full-history peak stretch is
1.11409, all four original material tracks remain continuous without extra
final patches, and the other physical checks pass. However, the free strand
touches the left little MCP during the hold. The both-little-clear diagnostic
therefore fails. `attempt_upright_return.json` reproduces this intermediate
state; it is not a verified free-loop start.

Moving the left little MPR from −0.08729 to +0.174 rad over one second, followed
by another 1.2 s hold, retains the four original tracks but does not clear the
free strand. `attempt_upright_little_clear.json` records this comparison.

A further 40° left-wrist roll in eight 0.2 s segments keeps its measured index
reference within 0.321 mm in the sampled joint path. This is a kinematic
reference, not a rope constraint. The subsequent 1.2 s hold ends at 43.566667 s
with all four original material tracks still present, but the free strand now
also touches the left little PIP. The clearance check still fails, so this
branch is not used for the next transfer. Further whole-figure rotation starts
from the earlier 38.566667 s upright state instead. Its remaining little
contact must still clear before the release stage can be considered complete.

Reproduce the upright and knuckle-clearance comparisons from the accepted
`output/raised_thumb` state described in `alternate-cup.md` (before widening):

```bash
.venv/bin/python scripts/continue_cup.py --source output/raised_thumb \
  --output output/upright_return --side both --route-only \
  --approach-path examples/transfers/attempt_upright_return.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame \
  --hold-duration 1.2 --hold-label 'Hold upright before widening the hands'
.venv/bin/python scripts/continue_cup.py --source output/upright_return \
  --output output/upright_little_clear --side both --route-only \
  --approach-path examples/transfers/attempt_upright_little_clear.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame \
  --hold-duration 1.2 --hold-label 'Hold upright with the left little knuckle withdrawn'
.venv/bin/python scripts/continue_cup.py --source output/upright_little_clear \
  --output output/upright_left_roll --side both --route-only \
  --approach-path examples/transfers/attempt_upright_left_roll.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame \
  --hold-duration 1.2 --hold-label 'Hold the rolled left hand clear of the hanging loop'
.venv/bin/python scripts/audit_loop_release.py --source output/upright_left_roll \
  --start 33.1 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/upright_left_roll --start 6
```

## A target-strand contact on the right thumb

From the earlier upright state, a common translation moves the wrists 240 mm
up and 100 mm back over 3.6 s. A further 58.125° rotation about world Y takes
3.1 s, followed by a 1.2 s hold. This reaches 46.466667 s without losing the
four original material tracks. The physical checks pass, with full-history
peak stretch 1.19968, just below the unchanged 1.2 gate, and minimum sampled
rope separation 1.934 mm.

The right thumb has a new contact patch on particles 643–654, which belong to
its target free strand. The left thumb instead has an additional patch on
particles 336–349 of its old near strand. The left little also remains in
contact, so the release diagnostic still fails. This is an intermediate
contact result: neither the new thumb bight nor a completed Cup is established.
The highest fixed free-band point remains 137.0 mm below the original thumb
bridge; that world-height measurement alone cannot decide the figure's topology.

```bash
.venv/bin/python scripts/continue_cup.py --source output/upright_return \
  --output output/raised_gravity_turn --side both --route-only \
  --approach-path examples/transfers/attempt_raised_gravity_turn.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame \
  --hold-duration 1.2 --hold-label 'Hold the raised figure after the gravity-assisted turn'
.venv/bin/python scripts/audit_loop_release.py --source output/raised_gravity_turn \
  --start 33.1 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/raised_gravity_turn --start 6
```

## Coordinating the left thumb with the wrist turn

Withdrawing the left thumb after the raised turn is too late: reducing MCP
from 1.1 toward 0.9 rad stops at 46.866667 s with stretch 1.20577. The worst
edge is 209, against the left little **DIP**, not its PIP. The PIP capsule is
about 14.9 mm from the rope in that stopped frame.

Restoring the exact 42.166667 s checkpoint, before rotation, allows that
0.8 s thumb adjustment and a 0.2 s observation. All four original material
tracks remain present. Holding MCP at 0.9 rad during the subsequent wrist turn
still contacts the forbidden near strand: the guard stops at 44.4 s with a
0.864 mm gap, below the 1.5 mm requirement.

Counter-rotating left thumb MCP from 0.9 to 0.35 rad throughout the 3.1 s wrist
turn avoids that near-strand contact. The moving prefix ends at 46.266667 s
with peak stretch 1.16251. However, the unchanged pose then jams the free strand
against the left little DIP: at 46.966667 s, stretch reaches 1.25222 on edge
209. The original four support tracks survive, and the right thumb has an
additional target-strand patch, but the failed hold rejects this continuation.

These branches resume complete checkpoints, including rope velocities. In
particular, `state_1388.npz` is the end of the valid moving prefix; the later
failed `checkpoint.npz` must not be used as a valid starting state. This prefix
is not a stable hold or a completed figure. A near-avoiding search for direct
left-thumb pickup from it found no feasible path among the 18 sampled target
positions.

```bash
.venv/bin/python scripts/continue_cup.py --source output/raised_gravity_turn \
  --checkpoint output/raised_gravity_turn/state_1265.npz \
  --output output/early_thumb_clear --side both --route-only \
  --approach-path examples/transfers/attempt_early_thumb_clear.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --hold-duration 0.2
.venv/bin/python scripts/continue_cup.py --source output/early_thumb_clear \
  --output output/coordinated_thumb_turn --side both --route-only \
  --approach-path examples/transfers/attempt_coordinated_thumb_turn.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact \
  --hold-duration 1.2
# The second command is expected to stop during its hold with a strain error.
```

## Clearing the distal little-finger snag

From the 46.266667 s moving checkpoint, a 0.4 s adjustment moves left little
MPR to 0.174, PIP to 1.3 and DIP to 0.6 rad, while leaving MCP at 1.583 rad.
This extends the distal links away from the rope instead of curling them
farther into it. The ensuing 1.2 s hold completes at 47.866667 s. Full-history
peak stretch stays at 1.16251, sampled minimum rope separation is 1.934 mm,
and all four original material tracks remain continuous. The distal capsule
is 24.34 mm clear of the rope in the final frame.

The complete physical and retention audits pass except for left-little
release: the free strand still touches its MCP. The right thumb retains its
new free-strand patch at particles 647–658; it also has a separate contact
patch at 389–406 on old material. This is a stable intermediate state with
unresolved little-finger contact, not a completed Cup or a verified pair of
new thumb loops.

```bash
.venv/bin/python scripts/continue_cup.py --source output/coordinated_thumb_turn \
  --checkpoint output/coordinated_thumb_turn/state_1388.npz \
  --output output/little_unhook --side both --route-only \
  --approach-path examples/transfers/attempt_little_unhook.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact \
  --hold-duration 1.2
.venv/bin/python scripts/audit_loop_release.py --source output/little_unhook \
  --start 33.1 --released both
.venv/bin/python scripts/audit_retained_supports.py --source output/little_unhook --start 6
# Release audit exits nonzero because the left little MCP remains in contact.
```

## A proximal touch is not a new thumb loop

From that stable intermediate, `feedback_target` proposes a 0.4 s left-thumb
adjustment using the current free strand and original thumb patch. It uses
speed 1.5 rad/s, near-strand margin 3 mm, no velocity prediction and the
current pose as its nominal target. `continue_cup.py` executes the resulting
joint route with the same runtime guards. A 1.2 s hold ends at 49.466667 s with
an extra left-thumb patch on free particles 174–184 and the four original
material tracks intact.

This contact is on the **proximal MCP surface**: particle 180 has axial
coordinate +13.13 mm in a capsule with half-length 14 mm. The left thumb
extends in the negative axial direction. The free strand is still 17.51 mm
from its PIP capsule and 44.42 mm from its DIP capsule. Counting this as a
distal pickup would misidentify a touch at the thumb base.

The next feedback proposal requests no joint change because its instantaneous
contact objective is already satisfied. An additional 1.6 s at that pose
exposes the limitation: left target gap ranges from 0.002 to 9.005 mm, so the
new contact is intermittent. The right target gap stays below 0.033 mm, and
the left forbidden near strand remains at least 2.855 mm clear. Full-history
physical checks and the four original material tracks pass through 51.066667 s;
left-little release still fails. No thumb-loop exchange has been attempted
from this unverified capture.

A further fixed-wrist search aimed the left thumb tip at five lower free-loop
particles, 130–170. With near-strand avoidance enabled, all five local IK solves
missed their target by 24.8–45.8 mm. This search did not find a reachable tip
path; it does not prove that one cannot exist. The next planning problem is
to bring the free strand to the distal thumb while preserving the original
supports, rather than optimizing only the minimum thumb-to-strand distance.

```bash
.venv/bin/python scripts/continue_cup.py --source output/little_unhook \
  --output output/left_free_contact --side both --route-only \
  --approach-path examples/transfers/attempt_left_free_contact.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact \
  --hold-duration 1.2
.venv/bin/python scripts/continue_cup.py --source output/left_free_contact \
  --output output/left_contact_observation --side both --route-only \
  --approach-path examples/transfers/observe_left_free_contact.json \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact \
  --hold-duration 1.2
# The second route keeps identical joints for 0.4 s, then holds for 1.2 s.
```

The controller changes pass all 106 non-integration tests locally and in
[CI for 9ec1610](https://github.com/DavidLXu/Revo3-Newton-CatCradle/actions/runs/42428070305).
That run still fails the previously recorded single-hand integration check:
the specified-strand lift is 23.876 mm against a 25 mm requirement. Its threshold
has not been changed; the full CI run is not green.
