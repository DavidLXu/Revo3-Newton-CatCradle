# Clamp before releasing the little loops

Both intended near-index strands can remain within the existing **0.1 mm
proximity tolerance** during held preparations. That is not evidence of
compressive contact: re-auditing the 26 s preparation with nonpositive gaps
finds no persistent edge touching both surfaces on either hand. The historical
"pinch" results below use the 0.1 mm geometric threshold unless stated otherwise.

Both little loops can subsequently be released while retaining the four
original thumb/index supports, but the intended near-strand proximity is lost
during the final hold. The complete Cup, boat and Star sequence remains
unfinished. The full-history peak stretch ratio is **1.19927**, close to the
unchanged 1.2 rejection threshold.

The [Dressing a Pelt instructions](https://www.alysion.org/figures/introkids1.htm)
press the near index strings before releasing the little loops. Earlier
experiments added the clamp after release. This branch restores the reference
order, starting from the collision-enabled 60° forward-tilted Opening A at
9.6 s. The initial Opening A remains an independent placement.

The rope uses the same 689 dynamic particles, 1 mm radius, friction 1.2 and
fixed 1.363453388 m rest length after 6 s. Only robot joints change during these
continuations. Finger, palm, arm/body and strain guards stay enabled.

## Recorded comparisons

| Trial | Recorded result |
| --- | --- |
| Left fingertip closure from 9.6 s, planned over 2.4 s | Contact stops closure at 11.033333 s. A 1.2 s hold ends at 12.233333 s. All six supports and physical checks pass; near edges 311–312 remain paired through the final second. Peak stretch 1.14073. This is a separate trial. |
| Right side closure from 9.6 s, planned over 0.633333 s | Stops at 10.133333 s with stretch 1.54332 on near edge 595. Old thumb material also enters the grip. Rejected. |
| Right fingertip closure from 9.6 s, planned over 2.4 s | Stops at 11.866667 s with stretch 1.20407 on edge 617. No paired target sample or edge has formed. Rejected. |
| The same right side-closure target, planned over 2.4 s | Contact stops closure at 11.533333 s. A 1.2 s hold ends at 12.733333 s. All six supports and physical checks pass; near edge 596 remains paired through the final second. Peak stretch 1.14247. |
| Add the left clamp to that right-hand hold | Both near strands contact opposing surfaces at 14.566667 s, with all six supports retained. The subsequent hold fails at 15.533333 s: right near edge 599 reaches stretch 1.24339. The final state is rejected. |
| Resume the exact 14.566667 s checkpoint and move each wrist inward 5 mm | Motion lasts 0.6 s, followed by a 1.2 s hold ending at 16.366667 s. All six supports and physical checks pass. The intended pinch gaps are slightly open; persistent bilateral pinch fails. Peak stretch 1.17059. |
| Close those gaps by index joints alone | Stops at 16.533333 s with stretch 1.21047 on edge 599. Rejected. The planned right contact point has 0.395 mm tangential motion as well as 0.376 mm normal motion. |
| Close along the surface normal while counter-moving the wrists | Both-hand contact stops closure at 16.6 s. The 1.2 s hold ends at 17.8 s. All six support tracks and physical checks pass. Left near edges 306–307 and right near edges 598–599 remain paired throughout the final second. Peak stretch 1.19927. |

The slow right trial stops before the planned endpoint. Slower motion changes
both dynamics and displacement between the 30 Hz contact checks; the result
does not establish that the full endpoint is safe.

The final closure coordinates thumb-base and index joints with wrist IK.
Thumb PIP/DIP angles remain unchanged. The sampled planned path preserves each
thumb's world pose within 0.000480 mm position error, with maximum commanded
joint speed 0.01379 rad/s. The index-carried surface reference approaches along
the nearest thumb normal. This reference is a planning target, not a rope
attachment. Newton still determines the material motion and contact.

The final wrist separation is 190.046 mm. No old-thumb or whole-old-loop point
samples are paired in the final contact observation. These sampled results do
not exclude all unsampled contact. The last twelve stretch measurements vary
from 1.18231 to 1.19062 and end at 1.18370; stationary joints do not imply that
the rope has settled.

## Turning and little-finger withdrawal

A further common 20° downward turn from the 17.8 s clamp keeps all finger
angles fixed. The 1 s motion and 1.2 s hold end at 20 s. All six original
material tracks, physical checks and bilateral pinch checks pass. Near edges
306–307 on the left and 598–599 on the right persist through the final second.
The final-frame stretch ratio falls to 1.14570; the earlier full-history peak
remains 1.19927. Neither old thumb strand has a paired point or edge sample in
the final observation. Both little fingers still contact their original loops.
This is the clean source for the two subsequent comparisons.

Abducting and folding the little fingers from that pose takes 1.8 s, followed
by a 1.2 s hold to 23 s. The original six tracks and intended pinches remain,
and physical checks pass, but both little loops remain caught. More critically,
the right old thumb strand enters the grip at 22.633333 s: point 408 and edges
407–408 have paired contact samples. The near strand is still held as well.
This is an unwanted additional grip, despite passing the near-pinch audit.

The folded branch then tries a further 40° common turn and 80 mm advance.
Unmodified wrist IK switches arm configurations mid-path; its sampled relative
wrist drift reaches 12.004 mm and peak joint speed 3.665 rad/s, so that proposal
is not executed. Guiding the shoulder joints toward a reachable endpoint while
solving the wrist pose at each waypoint reduces these values to 0.0821 mm and
0.797 rad/s. The accepted geometry keeps the fingers unchanged and does not
move or constrain the rope.

Newton nevertheless rejects this folded-finger branch at 24.5 s: right near
edge 599 reaches stretch 1.20038. The old right strand is still caught, and the
left intended strand no longer has paired edge samples. The failed state is
not a restart. Geometry planning and preservation of the four original
supports did not make this a successful release.

A separate branch resumes the clean 20 s checkpoint, leaving the little
fingers extended. It turns both wrists another 60°, advances them 100 mm and
lowers them 40 mm over 4.8 s, then holds for 1.2 s. Shoulder guidance limits
sampled relative wrist drift to 0.208 mm, orientation drift to 0.0709° and
peak joint speed to 0.833 rad/s. At 26 s all six original tracks, physical
checks and bilateral pinch checks pass. Left edges 306–307 and right edges
597–598 persist through the final second. Both little loops remain retained.

This downward-facing pose is the newest clean preparation. Final-frame stretch
is 1.03744, and the nearest right old-thumb sample has 0.980 mm clearance from
the index. Neither old loop has paired point or edge samples in the final
observation. The earlier 1.19927 history peak still applies. The recipe is
`attempt_clamped_unfolded_turn60.json`, starting from the clean 20 s state,
not from the rejected folded branch.

These comparisons use `attempt_clamped_turn20.json`,
`attempt_clamped_little_fold.json` and `rejected_clamped_folded_turn40.json`,
respectively. Keep the same continuation guards and 1.2 s hold duration as
below; no near-contact trigger is needed when the hands are already clamped.
Audit both near-strand contact and unwanted old-strand contact after replay.

## Release, secondary contacts and loss of the near grip

Folding the little fingers from the clean 26 s downward pose clears both little
loops by 27.366667 s, with about 50 mm capsule clearance and both intended
near strands still inside the proximity threshold. Continuing the full fold
and hold fails at 28.033333 s: edge 624 reaches stretch 1.23180. The released
far strand is caught between the right index and middle MCP capsules. The
original thumb strands are not in the pinch.

An exact continuation from `state_0821.npz` at 27.366667 s changes only the right
middle MPR from 0.4 to −0.2 rad over 0.4 s, then holds for 1.2 s. It ends at
28.966667 s. All four original material tracks and the physical bilateral
little-release audit pass. Minimum little clearances during the final second
are 59.604 mm left and 27.656 mm right. Final-frame stretch is 1.00908.

The near-strand audit nevertheless fails: left proximity is first absent at
28.3 s, and right proximity at 28.966667 s. The final left thumb gap is 0.624 mm.
The right middle finger also contacts another part of the free arc near
particle 678. This is a measured release with retained original loops, not a
ready-to-throw bilateral pinch.

A subsequent right-middle MCP fold to 1.4 rad ends at 31.366667 s after another
1.2 s hold. The four original tracks and bilateral release audit still pass.
Only right near edge 597 persists; the left near strand has moved farther away.
The right middle now contacts old-loop material near 386 instead of the free
arc. Normal-only reclamping proposals from this pose, including a wider joint
search and an index-PIP variant, fail their geometric target tolerances and
are not simulated.

A separate continuous-motion comparison starts from the exact 27.766667 s
checkpoint (`state_0833.npz`) after the first middle withdrawal, while both
intended strands still have paired samples. A 60 mm forward wrist translation
over 0.4 s, followed by one observation frame, ends at 28.2 s with both intended
strands still within the proximity threshold. A 60 mm return over the same
duration ends at 28.633333 s with the left proximity lost. These are dynamic
prefixes, not completed stationary pinch holds. The free band's highest measured
position remains 281.1 mm below the original thumb bridge. The loop has swung;
it has not gone over the bridge. The route helper's generic turning-point
caption does not imply that these two fixed-duration strokes used an apex trigger.

The additional recipes are `rejected_downward_clamped_fold.json`,
`attempt_release_middle_clear.json`, `attempt_release_middle_fold.json`,
`attempt_continuous_forward_stroke.json` and
`attempt_continuous_return_stroke.json`. Use the exact prefixes specified above;
the failed fold's final state is not a valid source. The continuous strokes
use a hold duration of `0.03333333333333333` s, not the 1.2 s preparation hold.

## Proximity and nonpositive surface gaps

The original stop test accepts up to 0.1 mm positive gap on each capsule
surface. It can therefore stop before opposed compressive contact. The segment
contact adapter applies its normal and friction corrections only when the
segment penetrates the combined collision radius. Proximity, sampled surface
contact and measured grip forces are distinct quantities. Other collision
geometry and contact at solver substeps are outside the capsule snapshot test.

`continue_cup.py --hold-on-near-pinch both --pinch-max-gap 0` now requires
nonpositive gaps to both surfaces at the same material sample before stopping
closure. The original default stays at 0.0001 m for historical reproduction.
The value can be tightened within `[0, 0.0001]`; it cannot weaken the existing
proximity gate. The trigger records the threshold used.

The matching audit is:

```bash
.venv/bin/python scripts/audit_near_pinch.py --source output/preloaded \
  --side both --sampling edges --max-gap 0
```

A tightened audit writes `near_pinch_strict_edge_audit.json` (or
`near_pinch_strict_audit.json` for point samples), preserving the original
proximity report. Nonpositive gaps are still a sampled geometric condition,
not a contact-force measurement or force-closure certificate.

The first tighter closure resumes the clean 26 s preparation and moves the
planned index surface references about 81 µm left and 66 µm right toward the
thumbs. It completes a 1.2 s hold at 28.4 s with six supports and the physical
checks retained, but does not trigger the zero-gap condition. Neither hand has
a persistent edge passing the zero-gap audit. This result motivates further
small joint closures; it is not a validated pressure grip.

The next closure uses the measured 28.4 s state and a −0.15 mm target gap for
the index-carried planning references. Its zero-gap event stops motion at
28.866667 s, followed by a hold to 30.066667 s. Six original supports and the
physical preparation checks pass. The zero-gap hold audit still fails on both
hands, while the historical proximity audit passes. The worst per-frame best
common-edge gaps are 0.206 µm left and 4.364 µm right; those best samples can
change material and are not a persistent-contact certificate.

The two closures use `attempt_zero_gap_closure.json` and
`attempt_zero_gap_second_closure.json`. Use `--hold-on-near-pinch both
--pinch-max-gap 0` alongside the existing collision, strain and original-support
guards. The second recipe is also suitable for a separately recorded comparison
that executes the entire target without the near-contact stop. Those are
different physical trajectories and must retain separate outputs.

Executing the full second target without stopping at contact fails at
29.466667 s: left near edge 309 reaches stretch 1.21608. All six original
material tracks remain, but the physical strain and completed-hold checks
fail. This rules out that full closure as a usable preparation; its failed
final state is not a restart.

Regression checks cover separated surfaces that pass the old proximity test
but fail zero-gap contact, compressed opposing surfaces, interior edge samples
and invalid thresholds. There are 137 passing non-integration tests; the two
integration tests are excluded from that result.

## Check the current phase

`audit_loop_release.py --released none` requires both little contacts to remain
through the transfer, along with the existing strain, separation,
robot-clearance and completed-hold checks. It writes `pre_release_audit.json`.
After intended release, use the appropriate `left`, `right` or `both` mode.
The swing controller explicitly requires a current report with both little
loops released; passing a retention audit cannot authorize a free-loop swing.

`audit_retained_supports.py --include-little` follows the original little
material as well. Each little support is the endpoint of the opposite hand's
original far-index arc. Its report is `six_material_support_tracking.json`;
the four-support report used by continuation stays separate. Runtime
`--stop-on-support-loss` still guards the four thumb/index tracks; little
retention is audited after replay. Contact tracks are not a threading certificate.

The edge-pinch audit requires opposing surfaces at the same sampled fraction
of a near-strand edge. A persistent edge can have different contact fractions
over time. In the final right grip, the particle-only audit misses contact;
common interior-edge samples pass. Neither diagnostic establishes force closure.

## Reproduce

First reproduce the collision-enabled Opening A and the 9.6 s `forward_tilt`
preparation using the [collision correction and tilt commands](alternate-cup.md).
Then execute the following continuations, auditing the source before each one:

| Source | Recipe in `examples/transfers/` | Near-contact stop | Output |
| --- | --- | --- | --- |
| `output/forward_tilt` | `attempt_pre_release_right_clamp.json` | `right` | `output/early_right` |
| `output/early_right` | `attempt_pre_release_second_clamp.json` | `left` | `output/early_bilateral` |
| `output/early_bilateral`, exact `state_0437.npz` | `attempt_pre_release_span_relief.json` | none | `output/early_relief` |
| `output/early_relief` | `attempt_pre_release_normal_refine.json` | `both` | `output/early_normal` |

For example, the first continuation is:

```bash
.venv/bin/python scripts/audit_retained_supports.py --source output/forward_tilt --start 6
.venv/bin/python scripts/continue_cup.py --source output/forward_tilt \
  --output output/early_right --side both --route-only \
  --approach-path examples/transfers/attempt_pre_release_right_clamp.json \
  --hold-on-near-pinch right --hold-duration 1.2 --stop-on-support-loss \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame
.venv/bin/python scripts/audit_retained_supports.py --source output/early_right --start 6
.venv/bin/python scripts/audit_retained_supports.py --source output/early_right --start 6 --include-little
.venv/bin/python scripts/audit_loop_release.py --source output/early_right --start 9.6 --released none
.venv/bin/python scripts/audit_near_pinch.py --source output/early_right --side right --sampling edges
```

Use the same guards and hold duration for the other rows. The second row is
expected to fail during its final hold. The third row must explicitly specify
`--checkpoint output/early_bilateral/state_0437.npz`, the valid bilateral-contact
frame, rather than using that failed final state. Preserve its rope positions,
velocities and spring rest lengths. Audit both hands after the final row.
A replay with different contact timing needs its own verified prefix; do not
assume the historical checkpoint number is valid for different parameters.

The [machine-readable report](pre_release_clamp_20261108.json) records source
and result hashes, stops, support and pinch evidence for the recorded trials.
Recipes named `rejected_` are failure reproductions. A simultaneous-closing
proposal passed geometry planning but was not simulated. Subsequent release
and loop-throw work must keep these distinctions.
