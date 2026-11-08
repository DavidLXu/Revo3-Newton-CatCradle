# Clamp before releasing the little loops

Both hands now maintain opposing thumb/index contact on the intended near-index
strands through a 1.2 s stationary-joint hold. All six original material support
tracks survive. The full-history peak stretch ratio is **1.19927**, close to the
unchanged 1.2 rejection threshold: this is a measured preparation, with little
strain margin. It does not establish a settled rope, force closure, little-loop
release or a completed Cup, boat or Star.

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
and result hashes, stops, support and pinch evidence for all eight trials.
Recipes named `rejected_` are failure reproductions. A simultaneous-closing
proposal passed geometry planning but was not simulated. Subsequent release
and loop-throw work must keep these distinctions.
