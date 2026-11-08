# Distinguishing the two strands at the right grip

The narrow side-clamp trial retained the four original supports, but gripped
the wrong strand. Right-thumb material 413–414 lay between the thumb and index;
the intended near-index material did not. Withdrawing the right ring finger
then overstretched edge 416 to 1.25424. That branch is rejected. Passing generic
contact and strain checks before the withdrawal did not establish the required
near-strand clamp.

The new comparison starts at the verified 240 mm lateral wrist spacing. A
left closure completes a 1.2 s hold at 38.233333 s, with all four original
material tracks retained and both little fingers clear. Edges 308 and 309
have opposing thumb/index contact samples throughout the final second.
Full-history peak stretch is 1.19185 and minimum sampled nonlocal separation
is 1.934 mm. Cup, boat and Star remain incomplete.

## Counter-moving the wrist

The right proposal changes thumb-base and index joints while counter-moving
the wrist to preserve the thumb's world pose. PIP and DIP angles stay fixed,
so this also preserves the other thumb capsule poses at the IK waypoints.
The sampled joint path has at most 0.213 mm thumb-position drift, 0.001122 rad
orientation drift and 1.172 rad/s joint speed. The original local thumb
material stays at least 1.670 mm from the index in the frozen-geometry check.
These measurements are planning checks; they do not constrain the rope.

The first replay stops at 39.966667 s because the support tracker compares
patch midpoints. Its right-index patch grows from 587–611 to 568–611, moving
the midpoint from 599 to 589. Material 599 still contacts the index. This is
a false loss report caused by patch growth, not evidence of a released loop.

The tracker now prefers a patch containing its previous representative
material sample. If that patch's midpoint moves beyond the existing
eight-particle bound, it keeps the still-contacting sample as its representative.
When the sample is absent, the original midpoint bound and ambiguity rejection
still apply. Other patches remain explicit in the report. A merged patch can
include an unwanted strand; this correction does not certify topology.

Re-auditing the stopped prefix retains all four tracks. Its strain, separation,
hand/body clearance and both-little-clear checks pass; the required stationary
hold has not yet completed. Continuation resumes the exact 39.833333 s
waypoint checkpoint, including rope velocities, before finishing the remaining
four 0.2 s command segments.

That continuation exposes a physical failure at 40.6 s: old material 403–405
enters the right grip before the intended near strand. Edge 404 reaches
stretch 1.31616 and the strain guard stops the run. Keeping the thumb pose
fixed and clearing the old strand's initial position did not prevent that
strand from moving into the closure. The failed checkpoint is not a restart.

An earlier waypoint at 40.433333 s can hold for 1.2 s without that overclosure.
At 41.633333 s all four original tracks remain and the physical deformation
and robot-clearance checks pass. The left near-strand pinch persists on edges
309–310. The right target still has a 3.369 mm thumb gap, and the right little
MCP touches the free strand near edge 653. This is a retained preparation,
not a completed bilateral clamp or a clear start for the loop throw.

Whole-old-loop avoidance proposals and a separate distal-thumb-only closure
fail their geometric checks and are not simulated. The latter cannot reach
the intended material while maintaining its old-strand clearances in the
sampled joint paths. Further work needs to separate those strands before
closing the grip.

The original right-thumb loss at 37.766667 s in the earlier
[near-clamp trial](near-strand-clamp.md) still fails the corrected tracker.
The patch-growth fix does not turn that old failure into a retained support.

## Contact between particle samples

`audit_near_pinch.py --sampling edges` evaluates 21 common fractions on each
eligible near-strand edge. Both opposing surfaces must meet the same fraction;
contacts at different places on one edge do not count as a pinch. A persistent
edge may have different contact fractions over time. This is sampled contact
geometry, not persistent contact of one exact material point or a force-closure
certificate. The point-based runtime trigger is unchanged.

```bash
.venv/bin/python scripts/audit_near_pinch.py --source output/near_clamp \
  --side both --sampling edges
```

Negative samples do not exclude unsampled contact. Knuckle meshes are outside
this capsule diagnostic; run the physical audit and original-support tracker
as well. The tests cover an interior contact missed by both endpoints, separate
contacts on one edge, the closing edge, and retained material during patch
growth and merging.

## Reproduce the wider preparation

First reproduce and audit `output/raised_thumb_spread` at 35.5 s using the
[raised-thumb spacing commands](alternate-cup.md#post-release-swings-and-hand-spacing).
Then run:

```bash
.venv/bin/python scripts/continue_cup.py --source output/raised_thumb_spread \
  --output output/wide_left_clamp --side both --route-only \
  --approach-path examples/transfers/attempt_wide_left_clamp.json \
  --hold-on-near-pinch left --hold-duration 1.2 --stop-on-support-loss \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame
.venv/bin/python scripts/audit_retained_supports.py --source output/wide_left_clamp --start 6
.venv/bin/python scripts/audit_loop_release.py --source output/wide_left_clamp --start 35.5 --released both
.venv/bin/python scripts/audit_near_pinch.py --source output/wide_left_clamp --side left --sampling edges
.venv/bin/python scripts/continue_cup.py --source output/wide_left_clamp \
  --output output/wide_right_preclamp --side both --route-only \
  --approach-path examples/transfers/attempt_wide_right_preclamp.json \
  --hold-duration 1.2 --stop-on-support-loss \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame
.venv/bin/python scripts/audit_retained_supports.py --source output/wide_right_preclamp --start 6
.venv/bin/python scripts/audit_loop_release.py --source output/wide_right_preclamp --start 38.233333333333334 --released both
.venv/bin/python scripts/audit_near_pinch.py --source output/wide_right_preclamp --side both --sampling edges
```

The last two audits are expected to reject the right-little contact and missing
right pinch. The held result reported here resumed the exact 40.433333 s
checkpoint before the failed final closure. The shorter recipe above commands
the same eleven complete waypoints and then holds; fresh replays require fresh
audits. `rejected_wide_right_full_clamp.json` adds the twelfth segment and
reproduces the overstretched closure from the same wide-left source.

For the narrow comparison, start at the 35.766667 s left hold described in the
near-clamp notes. `attempt_side_clamp_right.json` uses the right pinch trigger
and a 1.2 s hold; `attempt_side_clamp_indices.json` then uses a 1.2 s hold
without the trigger. `rejected_side_clamp_ring_release.json` continues that
wrong-strand grip and is expected to hit the strain gate. Keep all collision,
strain and support-retention guards enabled.

[The numerical report](side_clamp_20261108.json) records checkpoint and command
hashes, accepted and failed checks, and material identities. No failed final
state is offered as an accepted continuation.
