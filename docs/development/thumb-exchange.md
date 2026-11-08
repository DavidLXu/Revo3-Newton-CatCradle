# Coordinated thumb pickup and the remaining right loop

Moving both hands together produces a useful transfer state at 67.366667 s.
Both new far strands remain at the thumbs throughout the final stationary
second. The left thumb clears its complete old loop and its near strand;
the right thumb still carries both unwanted strands. Cup & Saucer, boat and
Star are not complete.

This continues the [orbit and middle-clearance experiments](orbit-transfer.md).
The [numerical record](thumb_exchange_20261109.json) includes the failed
alternatives, source hashes, joint recipes and audit results. All reported
results come from Newton replays; proposed finger positions are not substituted
for rope motion.

## What changed in the motion

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
the two right-side clearance conditions. These observations establish retained
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
near clearance and the complete old-loop clearance. It always reports
`completed_figure: false`: these are necessary conditions rather than a figure
recognizer. The opened-clamp negative control fails the capture and release
conditions as expected.

Once both new far contacts have passed a stationary hold,
`--stop-on-far-loss` tracks their material through the next route. It requires
a matching physical audit and held far-material report, together with the
original-support guard. A new patch elsewhere on the thumb cannot silently
replace a lost material track. The guard stops the experiment; it applies no
force or constraint to the rope.

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
expected to fail the two right-side clearance checks. Do not ignore any other
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
