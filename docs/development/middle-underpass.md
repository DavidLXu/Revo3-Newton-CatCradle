# Passing a middle finger around the free strand

Cup & Saucer, boat and Star remain incomplete. These trials continue the
[middle-helper experiment](supported-thumb.md#preparing-a-middle-finger-helper).
They test passage around a strand, before attempting to seat it on the finger.
They do not establish the hanging-loop throw or the later thumb-loop exchange
required by the [alternative Cup method](https://llx.com/Neil/string/playground/cup_and_saucer.html).

The source is the helper-hook trial's exact `state_1736.npz` at 57.866667 s.
The four original thumb/index material tracks are intact. Rope edge 205 touches
the middle DIP at an axial coordinate of 19.407 mm, beyond its 8 mm cylindrical
half-length: this is contact against the fingertip cap. The left little MCP
still supports the free strand. Nothing in this source certifies a new bight.

## Joint-only proposals and physical observations

The proposals aim the middle DIP's distal cap center at successive lateral and
forward waypoints. Each new proposal uses the latest simulated strand position.
The lateral direction is perpendicular to both the local strand tangent and
the DIP axis. Thumb/index surface references discourage movement of the
original supports; a weaker reference discourages movement at the middle MCP.
These are soft planning costs, never attachments to the rope.

The robot executes the resulting joint commands with the same Newton settings:
friction 1.2, 689 positive-mass particles, 1 mm radius, no anchors, and fixed
material length 1.363453388 m after 6 s. Every executed step enables the existing
finger, palm, body, hull, strain and near-strand guards. The 1.2 stretch and
1.5 mm near-strand limits are unchanged. Each moving step lasts 0.6 s, generally
with a 0.1 s observation. The stronger-reference positive-side comparison has
a 1.2 s hold. A short observation is not a stationary-hold certificate.

The first route takes two nominal 8 mm steps to the negative lateral side,
then three bounded forward steps:

| End time | Action | Observed result |
| --- | --- | --- |
| 58.566667 s | First lateral step | Side offset −8.175 mm; target-span gap 10.568 mm. |
| 59.266667 s | Second lateral step | Side offset −21.459 mm; target-span gap 26.977 mm. |
| 59.966667 s | First forward step | Axial distance to material point 206 improves from 31.839 to 21.980 mm. |
| 60.666667 s | Second forward step | Axial distance is still 21.810 mm: the strand follows the motion. |
| 61.366667 s | Third forward step | Axial distance 20.976 mm; target-span gap grows to 30.932 mm. No passage. |

The four original material tracks survive all five steps. The complete-history
release audit at 61.366667 s passes strain, sampled rope separation, dynamic
particle, joint, finger/palm, body and hull-centerline checks. It fails
`left_little_clear` and `completed_hold`. The following planned 1.2 s hold stops
at 61.533333 s: the left near-strand gap falls to 0.010 mm. This route is not a
stable preparation and must not be continued from that rejected state.

A comparison starts at 60.666667 s and increases the middle reference weight
from 30 to 150. Frozen reference drift drops from 3.793 to 0.850 mm, but the
physical run stops at 61.166667 s with left near gap 0.032 mm. Holding that
reference more tightly does not solve the transfer. Two proposals using only
middle PIP/DIP fail their position checks before simulation; they are recorded
as planning results, not physical attempts.

The opposite-side route starts again at 57.866667 s. Its first nominal 6 mm
lateral step ends at 58.566667 s with side offset +6.335 mm and target-span
gap 2.426 mm. Material point 206 moves only about 2 mm during this step.
A second 6 mm lateral step ends at 59.266667 s with side offset +12.390 mm
and target-span gap 6.781 mm. Both steps retain the four original material
tracks; the minimum sampled separation in the second step is 1.995 mm.
The following 8 mm bounded forward step ends at 59.966667 s. Its target gap
is 2.592 mm, but the side offset shrinks to 6.404 mm and the cap remains
12.226 mm behind material point 206. The strand has moved with the action;
the original material tracks still survive.

Repeating that forward step from 59.266667 s with middle-reference weight 150
reduces maximum frozen middle-reference drift from 1.929 to 0.778 mm. This
branch completes a 1.2 s hold at 61.066667 s. All four original material tracks
and the full physical diagnostics pass; the release audit fails only
`left_little_clear`. Peak history stretch remains 1.16251 and minimum sampled
nonlocal separation remains 1.934 mm. Its final target gap is 2.332 mm, and
the nearest target edge's axial coordinate is still 20.120 mm: it has not
reached the 8 mm shaft interval or formed a secured bight.

A matched control starts at the same 59.266667 s state and holds every joint
for 1.8 s. It retains the four original tracks and completes the runtime guards;
only the moving comparison has a separate complete-history release audit.
At 61.066667 s, material point 206 has moved −0.932 mm in world Y in the fixed
control, versus +10.573 mm in the stronger-reference forward trial. Thus that
Y displacement is chiefly associated with the action, rather than simply
settling. The control also moves −5.977 mm in world X, and ends with a target
speed of 10.370 mm/s: the rope is not stationary merely because the joints are.
The moving trial's target speed is 18.125 mm/s after its hold.

Keeping one middle-MCP surface reference nearly fixed does not keep the strand
fixed. The wrist and finger orientations still change, and the free span also
touches other surfaces. Further planning needs to address those contacts and
the observed velocity; repeating the same point pursuit has not produced
passage. Exact source hashes, results and unexecuted proposals are in the
[numerical report](middle_underpass_20261108.json).

## Reproduction and contact inspection

Reproduce the helper-hook source using the linked preparation note, saving
every frame. Run the first positive-side step from its contact checkpoint:

```bash
.venv/bin/python scripts/continue_cup.py --source output/middle_helper_hook \
  --checkpoint output/middle_helper_hook/state_1736.npz \
  --output output/underpass_plus1 --side both --route-only \
  --approach-path examples/transfers/attempt_middle_underpass_plus1.json \
  --hold-duration 0.1 --check-self-collision --check-robot-hulls \
  --check-body-envelope --max-stretch-ratio 1.2 \
  --checkpoint-every-frame --stop-on-near-contact
.venv/bin/python scripts/audit_retained_supports.py \
  --source output/underpass_plus1 --start 6
.venv/bin/python scripts/audit_finger_contacts.py \
  --source output/underpass_plus1 --material-range 180 220
```

The contact audit now reports the nearest edge's axial coordinate and the
closest capsule region: shaft or end cap. The gap still determines whether
contact exists. `--material-range START END` selects
edges `START <= i < END` without renumbering them. This separates the target
span from an existing contact elsewhere on the same finger. It inspects actual
rope edges, rather than only particle-to-finger distances. A shaft contact by
itself is still insufficient to certify threading or a secured bight.

The numerical report maps each saved recipe to its exact source and hold time.
Negative-side recipes `lateral1`, `lateral2`, `advance1`, `advance2`, `advance3`
form a chain; `advance3_stable` is an independent rejected branch from
`advance2`. The subsequent negative-route failure uses `--hold-only
--hold-duration 1.2` from `advance3`. Replayed physics must be audited anew.
On the positive side, `plus1` → `plus2` is the lateral sequence. Both
`plus_advance1` and `plus_stable` start from `plus2`; use 0.1 s and 1.2 s holds,
respectively. The fixed-pose control uses `--hold-only --hold-duration 1.8`
from `plus2` and does not use a route recipe.

All 118 non-integration tests pass. Four contact-location regressions cover
shaft/end-cap distinction under a rigid transform and material-range isolation.
The [CI run for the preceding commit](https://github.com/DavidLXu/Revo3-Newton-CatCradle/actions/runs/42439666339)
still fails the earlier single-hand integration check: 24.576 mm lift versus
its unchanged 25 mm requirement. That failure is not resolved by these trials.
