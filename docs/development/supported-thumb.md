# Coordinating the wrist, index and thumb

Cup & Saucer, boat and Star remain incomplete. These experiments continue the
[47.866667 s intermediate state](free-loop-control.md#clearing-the-distal-little-finger-snag).
It retains four original thumb/index material tracks and a right-thumb contact
on the free strand, but the left little MCP still supports that strand.

The left thumb's earlier contact was proximal and intermittent. The new goal
is to bring its **tip** to the observed free strand while retaining the old
thumb loop. No rope particles, velocities, masses or rest lengths are changed.
The rope remains passive, with friction 1.2 and no anchors.

## Planning from each observed state

An endpoint IK solution can reach the free strand, but direct interpolation
to it penetrates the frozen near strand by 11.58 mm and displaces the chosen
index reference by 13.42 mm. The endpoint alone is not an executable transfer.

`scripts/plan_supported_thumb.py` instead proposes one bounded step. It moves
the left arm, thumb and index joints, aims the thumb tip at a selected material
point, and penalizes separation from the tracked original thumb patch. An
index surface reference discourages unnecessary support movement. Both are
soft planning objectives; neither is a rope attachment or retention proof.

The planner samples the joint path for finger/palm clearance and left-thumb
near-strand separation. The optional right-thumb MCP target can provide slack
while the left hand moves. Its frozen near-strand gap is reported, but that
gap is not a preflight gate: the physical near strand moves during the left
hand's action. **Both actual near strands must be guarded during Newton replay.**
The original 1.5 mm near-strand and 1.2 strain gates remain unchanged.

The source must have current, complete original-support tracking, matching
checkpoint/export times and a passing recorded strain history. The planner
also rejects saved failure reports and a current near-strand gap below 1.5 mm.
Matching timestamps alone do not prove completion: a guard can reject the
exact endpoint of a phase. These checks do not replace the full physical or
figure-specific audits.

## Executed comparisons

All guarded continuations use the same collision-enabled model and exact
source checkpoints. The tip errors below are measured against the moving
material point 170 plus `[5, 0, -5]` mm, not against its earlier frozen position.

| Continuation | Result |
| --- | --- |
| First 0.5 s left-arm/thumb/index step, lighter retention cost | Stops at 48.3 s: stretch 1.20846 on edge 611 near the right index. Four original tracks remain, but strain invalidates the branch. |
| Same step with right thumb MCP 0.35 → 0.55 rad | Completes a 0.2 s observation at 48.566667 s. Four tracks survive; full-history peak stretch 1.17292. Tip error falls from 105.39 to 68.76 mm. |
| Second step with the lighter retention cost | Left old-thumb track disappears at 48.866667 s, before a new bight forms. Right near-strand contact stops the run at 48.9 s. Rejected. |
| Second step with stronger old-patch and index-reference costs | Keeps four original tracks but reaches stretch 1.20157 on edge 614 at 48.933333 s. Rejected. |
| Stronger-retention step with right thumb MCP 0.55 → 0.70 rad | Completes a 0.2 s observation at 49.266667 s; all four tracks survive and peak stretch remains 1.17292. Tip error is 55.75 mm. This is a moving intermediate, not a completed pickup. |
| Hold that two-step pose | Right near gap falls to 1.391 mm at 49.433333 s; the guard stops the hold. The motion endpoint is not a stable preparation. |
| One 0.3 s right-thumb correction, then hold | Keeps clear during the adjustment but stops at 50.0 s when right near gap falls to 1.420 mm. One correction does not provide lasting clearance. |
| Right-thumb feedback at 30 Hz with one-frame prediction | Stops at 50.0 s with right near gap 0.405 mm. This controller lacks the continuation tool's runtime body/hull/strain guards; no full physical acceptance is claimed. |
| Right hand 20 mm inward and 25 mm down, starting at the exact 49.066667 s motion boundary | Stops at its first waypoint, 49.166667 s, with right near gap 0.004 mm. Translating the whole hand does not resolve this branch's near-strand interference. |

The stronger costs preserve the old loop in this comparison. They do not solve
the complete transfer: the left little still carries the free strand as the
hand moves, and the right thumb needs continuing clearance management. The
new tip has not captured the free strand, and no old-loop exchange is claimed.

## Reproduction

First reproduce `output/little_unhook` using the linked preparation note and
run its original-support audit. Generate and execute the first proposal:

```bash
.venv/bin/python scripts/plan_supported_thumb.py \
  --source output/little_unhook --output output/supported_step1.json \
  --retention-weight 50 --index-weight 30 --right-thumb-mcp 0.55
.venv/bin/python scripts/continue_cup.py --source output/little_unhook \
  --output output/supported_step1 --side both --route-only \
  --approach-path output/supported_step1.json --hold-duration 0.2 \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact
.venv/bin/python scripts/audit_retained_supports.py --source output/supported_step1 --start 6
.venv/bin/python scripts/plan_supported_thumb.py \
  --source output/supported_step1 --output output/supported_step2.json \
  --right-thumb-mcp 0.70
.venv/bin/python scripts/continue_cup.py --source output/supported_step1 \
  --output output/supported_step2 --side both --route-only \
  --approach-path output/supported_step2.json --hold-duration 0.2 \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame --stop-on-near-contact
.venv/bin/python scripts/audit_retained_supports.py --source output/supported_step2 --start 6
```

The public planner produces both recorded relief targets exactly. Its default
retention/index costs are 500/100; the first comparison uses 50/30. The saved
`attempt_supported_thumb_relief.json` and
`attempt_supported_thumb_retention_relief.json` recipes can replace generation.
The `rejected_supported_thumb_*` recipes preserve the failed comparisons.
Run the near-clearance recipe from the second step, with a 1.2 s hold, to
reproduce its later rejection. Do not continue from a failed run's final state.

The inward/downward comparison uses `rejected_supported_thumb_inward_down.json`
with the second step's `--checkpoint output/supported_step2/state_1472.npz`
and a planned 1.2 s hold. Its source is the end of the moving step, before the
0.2 s observation. The right-thumb feedback comparison uses
`control_thumb_pickup.py` from the second step's final checkpoint with that
same checkpoint as `--nominal`, `--active-side right --duration 1.2 --speed 1.5
--margin .006 --predict 1 --stop-on-near-contact`.

## Turning the little knuckle away

Two independent branches start again at 47.866667 s. Both rotate the left wrist
about a geometric reference on its original thumb support; this reference
guides the robot pose and never pins the rope. A 20° world-X roll in four
0.2 s segments ends its 1.2 s hold at 49.866667 s. A 30° world-Y pitch in six
0.2 s segments ends at 50.266667 s.

Both branches pass the full physical checks and preserve the four original
material tracks. Peak stretch remains 1.16251 and minimum sampled nonlocal
rope separation remains 1.934 mm. Neither releases the left little MCP.
The frozen geometry predicted clearance, but Newton's rope follows the moving
knuckle. This is another reason that static endpoint distances cannot establish
release. The recipes are `attempt_thumb_pivot_release.json` and
`attempt_thumb_pitch_release.json`; each starts from `output/little_unhook`
with a 1.2 s hold and the same guards as above.

Continuing the 30° pitch with left little PIP/DIP at 1.6/1.59 rad over 1.4 s,
then holding for 1.2 s, ends at 52.866667 s. The original tracks and physical
checks pass, but the little finger now contacts the rope at **both MCP and
DIP**. Further curling does not release the strand. The recipe is
`attempt_little_full_curl.json`, starting from the completed pitch branch.

Two further independent continuations start at 47.866667 s:

- `attempt_little_unfold.json` extends and abducts the left little finger to
  MPR/MCP/PIP/DIP = −0.5/−0.07/0/0 rad over 1.8 s, then holds for 1.2 s.
  At 50.866667 s the strand is still on MCP, closer to the finger base.
  It also contacts the left middle and ring fingers.
- `attempt_support_axis_release.json` rotates the left wrist 40° about the
  line between its original thumb and index surface references, in eight
  0.2 s segments, then holds for 1.2 s. Sampled reference drift is below
  0.240 mm; no rope is attached to those references. At 50.666667 s the
  left little MCP still carries the strand.

Both retain all four original material tracks and pass the other physical
release diagnostics, with full-history peak stretch 1.16251. They do not pass
the left-little release check. The contact map also shows that this target
span touches the left middle finger: little-finger clearance alone would not
establish a freely hanging span.

## Preparing a middle-finger helper

The first proposed middle-finger curl crosses the opposite index fingertip
by 9.26 mm in the sampled path. The continuation tool rejects it before
advancing Newton. Folding PIP before MCP avoids that collision; MPR and DIP
then reach the same final joint target. `attempt_middle_helper_staged.json`
reproduces this 2.633333 s sequence from the 47.866667 s source, followed by
a 1.2 s hold. At 51.7 s all four original tracks and the physical diagnostics
pass, with peak stretch 1.16251. The left little still contacts its strand.
The middle finger still touches the existing span near particles 263–281;
this preparation has not captured a new bight.

## Stopped states and validation

`continue_cup.py --stop-on-near-contact` now writes `near_stop.json` with the
measured gaps, phase and commanded phase endpoint, plus the exact rejected
frame checkpoint. Replaying the short inward/downward failure produces arrays
identical to the previous run for every checkpoint field, including rope
positions, velocities, body state and rest lengths. The logging change does
not change its physics.

[The numerical report](supported_thumb_20261108.json) includes source checkpoint
hashes, outcomes, support tracking and completed audits. The planner's source
checks have five regression tests, including rejection at a phase endpoint.
All 111 non-integration tests pass; the previously recorded single-hand
integration failure remains outside these experiments. Full Cup, boat and
Star validation is still pending.
