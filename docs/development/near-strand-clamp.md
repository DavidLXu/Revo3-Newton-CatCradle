# Clamping the near index strand

The left thumb and index now maintain opposing contact with the same near-strand
material through a 1.2-second hold. Right-hand clamping remains unresolved;
the attempted closure loses the original right-thumb loop. Cup & Saucer, boat
and Star are still incomplete.

The [Dressing a Pelt instructions](https://www.alysion.org/figures/introkids1.htm)
describe pressing the near index strings against the indices before releasing
the little loops and throwing the hanging loop. The present experiment adds
that clamp to the already released preparation. It therefore tests a missing
contact condition, not the complete reference sequence. The resulting figure
in that reference is an inverted Cup; the required later transfers and figure
orientation still need validation.

Near-string contact is intentional during this preparation. The standard
far-string pickup's `--stop-on-near-contact` rule is incompatible with it.
Omitting that rule does not relax the strain, collision, support-retention or
same-material pinch checks, and does not establish a successful thumb exchange.

## Left clamp and rejected right closure

The source is the [raised-right-thumb preparation](alternate-cup.md) at 33.1 s,
before widening the wrists. It has more strain margin than the later 240 mm
spacing. The rope remains passive after 6 s: 689 positive-mass particles,
1 mm radius, friction 1.2, fixed material length 1.363453388 m, and no anchors.
All following actions change robot joints only.

The left proposal moves a thumb surface toward the index-carried near strand.
Planning predicts carried contact; Newton alone determines where the rope
actually goes. Continuing to the geometric endpoint overcloses the fingers:
edge 312 reaches stretch 1.29141 at 34.633333 s. That final state is rejected.

The exact prefix checkpoint at 34.566667 s is different: same-material opposing
contact has already formed, peak strain during the new approach is 1.02802,
and sampled nonlocal separation is at least 1.981 mm. Holding that measured
pose, with its actual rope velocities, completes at 35.766667 s. All four
original thumb/index material tracks remain continuous; both little fingers
are clear; the complete-history physical diagnostics pass. History peak
stretch is 1.11409 and minimum sampled separation is 1.934 mm.

Material point 312 meets both capsule surfaces throughout the final second.
Its larger surface gap stays below 0.090 mm and the opposing-normal dot product
stays between −0.8464 and −0.8327. This is persistent sampled pinch geometry
under Newton dynamics, not a force-closure or threading certificate.

The next right-hand proposal reaches its geometric target but fails physically.
At 37.766667 s, the original thumb patch disappears; contact with a new near
strand must not replace that original material track. The subsequent hold
also fails the bilateral pinch check. Generic finger contact and strain audits
alone would miss this failure.

A branch stops the right thumb earlier, at the valid 37.6 s checkpoint, and
uses only the index to close the remaining gap. The new runtime material guard
stops it at 37.833333 s because the right thumb has no rope contact. Stopping
the thumb this late does not arrest the slipping loop. An earlier 37.3 s
index-only proposal fails geometric reach checks and is not simulated.

A separate full approach limits right-thumb PIP flexion to 0.8 rad and uses
more DIP flexion (1.7002 rad). This reduces the index MCP target from 1.3112
to 0.6638 rad and passes the geometric checks, but loses all right-thumb
contact at 37.6 s. It is also rejected. The next preparation must address
the old loop's support and tension before continuing closure; selecting a
different geometric endpoint alone has not kept that loop on the thumb.

## Runtime control and audit

`continue_cup.py --hold-on-near-pinch left|right|both` interrupts a supplied
route when the requested hands have opposing thumb/index capsule contacts on
the same near-strand material sample. It clips the original quintic at the
measured joint pose and enters the requested hold. It never resets rope
positions, velocities or rest lengths. `pinch_trigger.json` records the event;
its existence does not certify a stable grip.

The sampled gate requires each surface gap to lie between −0.3 and +0.1 mm
and the normal dot product to be at most −0.8. Knuckle meshes and edge-only
contacts are outside this pinch diagnostic. `audit_near_pinch.py` additionally
requires stationary joints for a full final second and at least one persistent
material sample per requested hand. Run the physical audit and original-patch
tracker separately.

`--stop-on-support-loss` starts from the source's four tracked contact patches
at the exact checkpoint and applies the existing eight-particle continuity
bound every output frame. A failure writes `support_stop.json` and ends the
run. This guard is appropriate for preparation; deliberately exchanging an
old thumb loop later will require a different material-support criterion.

Reproduce the source using the raised-thumb instructions, then:

```bash
.venv/bin/python scripts/audit_retained_supports.py --source output/raised_thumb --start 6
.venv/bin/python scripts/continue_cup.py --source output/raised_thumb \
  --output output/near_clamp_left --side both --route-only \
  --approach-path examples/transfers/attempt_near_clamp_left.json \
  --hold-on-near-pinch left --hold-duration 1.2 --stop-on-support-loss \
  --check-self-collision --check-robot-hulls --check-body-envelope \
  --max-stretch-ratio 1.2 --checkpoint-every-frame
.venv/bin/python scripts/audit_retained_supports.py --source output/near_clamp_left --start 6
.venv/bin/python scripts/audit_loop_release.py --source output/near_clamp_left --start 33.1 --released both
.venv/bin/python scripts/audit_near_pinch.py --source output/near_clamp_left --side left
```

The measured left result above came from the exact prefix and hold, before
adding the automatic trigger. The new trigger was exercised separately on an
already clamped state: it stops at 35.8 s and preserves the executed command
prefix. That two-frame smoke run is not another one-second hold result.
Replayed physics must pass the audits again.

`attempt_near_clamp_right.json` records the rejected right approach from the
35.766667 s left hold. `attempt_near_clamp_index_finish.json` records its
rejected index-only branch from `state_1128.npz` at 37.6 s. Their numerical
results, the rejected `attempt_near_clamp_right_distal.json` comparison, and
source hashes are in [the experiment report](near_clamp_20261108.json).
