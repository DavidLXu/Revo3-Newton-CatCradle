# Timing the released loop

The [accepted release preparation](alternate-cup.md#post-release-swings-and-hand-spacing)
ends at 35.5 s with the right thumb raised and 240 mm lateral wrist spacing.
Both little fingers remain clear, and four original thumb/index material tracks
survive the hold. The next operation is to bring the free loop over the other
strings before exchanging the thumb loops. Cup, boat and Star are not yet
complete.

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
The narrower 200 mm preparation at 33.1 s has more strain margin and is the
next source to test; the accepted 240 mm hold should not be treated as evidence
that it can safely enter the following motion.
