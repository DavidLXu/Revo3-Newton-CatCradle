# Design and data notes

## Motion data

`data/forward_motion.json` contains named left-hand URDF joint positions in **radians**, and per-command durations in **seconds**. The loader verifies exact joint-name coverage, finite values, positive durations and joint limits, then reorders by name. There is no SDK connection or hardware control.

The example was normalized from a supplied forward command recording, not measured feedback. The conversion used `targets_deg + target_offset_deg`, then SDK-to-URDF reordering. The sign is an inference supported by limits, not a confirmed property of the recording application. Of 483 original values, no offset produced 93 limit violations, subtraction 124, and addition one. That last middle-PIP value was clamped by approximately 0.08065° to the model limit before export. The source calibration, device identifier and unused inverse recording are excluded.

The first target is held for 0.5 s because its preceding pose is unknown. Subsequent targets use `u³(10−15u+6u²)` interpolation. All 23 command entries, including the repeated step in the original sequence, remain in order. The duration stays 11.5 s; there is no claimed frame alignment with the four-second reference clip.

## Local pickup correction

Angles below are absolute URDF degrees, ordered little-finger MPR / MCP / PIP / DIP:

| Time (s) | Angles |
| --- | --- |
| 4.00 | Original pose |
| 4.13 | −17 / 85 / 90 / 90 |
| 4.35 | −17 / 85 / 90 / 90 |
| 4.65 | −17 / 45 / 70 / 30 |
| 5.00 | −20 / 40 / 40 / 0.4 |
| 5.50 | Original pose |

The thumb traverses its original 4.0–4.5 s poses during 4.0–4.3 s using a quintic time mapping, holds the final pose until 4.5 s, then rejoins the source. This delivers the diagonal strand while the little finger is tucked below it. The correction acts on joints only; it never reads or overwrites rope positions. `--original-joints` disables both changes.

## Physics

Units are meters, seconds and kilograms. Defaults are 30 output frames/s, 96 substeps/frame, 32 XPBD iterations, gravity −9.81 m/s², 1 mm rope-particle radius, approximately 3 mm spacing, friction 0.65, stretch stiffness 800 N/m, damping 0.01 N·s/m and bend-spring stiffness 0.002 N/m. Line density is assumed to be 4 g/m, not measured.

The initial route is thumb → index → middle → ring → little → thumb. Surface arcs sit around the outside of four fingers and the palm side of the middle finger, near DIP metal bands. All samples become dynamic after initialization. The displayed visual meshes and fitted collision capsules are distinct approximations.

Active length control adjusts spring rest lengths using rope strain alone. Its nominal tension is 0.08 N, total take-up rate is limited to 0.30 m/s, and payout is immediate. The target includes another 1 cm of shortening. Total rest length is bounded to 0.20–0.70 m; each edge also has bounds relative to its initial length. This distributed approximation can inject or remove energy. It is not a model of a real reel, nor a promise of passive stability. `--fixed-length` disables it.

## Newton adapter

The pinned Newton version does not expose the needed particle-pair filter. `RopeSolverXPBD` creates a subclass-local copy of `SolverXPBD.step` with one contact-kernel binding replaced. Adjacent particles up to two topological hops, including the loop seam, do not collide; remote parts of the rope still do. This supports one ordered, all-dynamic closed loop. It does not mutate Newton's global solver or vendored source. Upgrade Newton only after checking this adapter against the new kernel signature and running all tests.

## Output and validation

The CLI exports per-frame joint positions, link transforms, rope positions and rest lengths. NPZ additionally stores particle masses, initial geometry and diagnostic wrap indices. Wrap indices are not attachment constraints. The JSON includes motion provenance, contact mode and a validation flag. The viewer marks invalid deformation results visibly.

The acceptance limit is an edge stretch ratio of 1.2 and a bounded scene, not a material calibration. Tests also check positive rope masses, initial seating, friction/gravity response, no hand influence at a distance, local finger separation and the specified strand's contact/lift. The default fixture has 180 particles; material indices used by the pickup regression are specific to this fixture and must be re-identified if the initial geometry or discretization changes.
