# Two-hand transfer experiments

The current difficulty is retaining the intended strand while the thumb clears
its neighboring strand. Increasing hand spacing helps provide room, but a loop
can still slide off a rounded fingertip during the next motion. These tools
preserve the actual rope state between attempts so that contact at one frame
cannot stand in for a completed transfer.

## Recorded results, 2026-10-26

| Experiment | Result |
| --- | --- |
| Original Position 1 → Opening A draft | Pickup and support checks pass; strict stretch and self-contact checks fail. See [Opening A](opening-a.md). |
| Wider, independently placed Opening A | Initial wrist spacing 200 mm, initial material rest length 1.36419 m. All six supports survive a six-second run; the rope is not certified stationary. |
| Left-thumb pickup, 32.8 s | Passes the existing left-pickup validator after a 1.2 s hold. Both original thumb loops and index/little supports remain. |
| Right-thumb attempt, 44 s | Both target strands remain near the thumbs, but right near-string clearance falls to 1.545 mm during the final second, below the 2 mm gate. Unaccepted. |
| Cup & Saucer, boat and Star | Full continuous transitions have not been completed. |

The [left-pickup report](validation/transfers-left-pickup.json) records a maximum
target-strand gap of 0.879 mm, minimum near-string clearance of 3.001 mm,
full-history peak stretch of 1.1551 and minimum nonlocal centerline separation
of 1.978 mm. These are the original experiment's results on Apple Silicon CPU.
The right old loop later slipped during a wrist-lift experiment, so passing the
static hold does not establish a secure bearing for every subsequent motion.

## Run a portable experiment

The [November 8 alternative Cup experiments](development/alternate-cup.md)
test releasing the little loops before exchanging the thumb loops. A left-only
release passes the contact and deformation checks. A 24.6 s bilateral attempt
retains the rope supports but intersects the opposite palm and is rejected.
The page includes a compact initial recipe, exact commands and the expanded
audit. The loop throw and thumb exchange remain unfinished.

Run the normal setup and `bash scripts/setup_revotron.sh` first. The recipes
embed the initial rope centerline, ordered joint commands, clipped command
polynomials and material-strand identities. They require no earlier `output/`
directory or checkpoint from the development machine.

```bash
# Quick import, recipe and physics check. Use a fresh output directory each time.
.venv/bin/python scripts/replay_transfer.py examples/transfers/left_pickup.json \
  --duration 0.1 --output output/transfer_smoke

# Recompute the complete recorded left-pickup command sequence.
.venv/bin/python scripts/replay_transfer.py examples/transfers/left_pickup.json \
  --output output/transfer_left
.venv/bin/python scripts/validate_transfer.py --output output/transfer_left --side left

# Recompute the unfinished right-pickup attempt for further development.
.venv/bin/python scripts/replay_transfer.py examples/transfers/right_pickup_attempt.json \
  --output output/transfer_right_attempt
.venv/bin/python scripts/validate_transfer.py \
  --output output/transfer_right_attempt --side both
```

The last validator is expected to fail. The right attempt also lacks a completed
static-hold phase. Never copy the historical validation report into a fresh run
as its result; validate the generated trajectory. Full runs are expensive on
CPU: these recipes use 96 substeps and 32 solver iterations per output frame.

The replay integrates Newton from a fresh initial placement with zero initial
rope velocity. It follows the joint commands recorded during feedback experiments;
it does not recompute that feedback or load recorded rope frames. Numerical
differences can change later contact, so the recorded milestone is a reference
to compare against, not a guarantee on another solver or device. Runtime physics
parameter events are rejected by this replay entry point. The bundled recipes
use constant settings and do not require such events.

For a preview after generating a trajectory:

```bash
.venv/bin/python scripts/render_revotron.py --simulation-dir output/transfer_left \
  --single-player --preview --draft --label 'Left thumb pickup experiment'
bash scripts/serve.sh
```

The renderer keeps its existing Opening A validation path, so experimental
transfer previews use `--draft` even when the separate left-pickup validator
passes. Images appear under the simulation directory's `portrait/` folder.

## Continue an experiment

An output directory contains `checkpoint.npz`, `trajectory.json`,
`trajectory.npz`, `joint_commands.json` and `transfer_plan.json`. The checkpoint
preserves positions, velocities, rest lengths, rigid proxy state and length
controller settings. Keep command metadata with the commands: an interrupted
quintic segment needs its original polynomial to reproduce the earlier motion.

```bash
# Hold the final pose while rope physics continues.
.venv/bin/python scripts/continue_hold.py --source output/transfer_left \
  --output output/transfer_left_hold --seconds 1.2 --label 'Hold new thumb bights'

# Example feedback experiment: hold both target and old thumb strands while
# seeking clearance from the near strands. This is a trial, not a solved route.
.venv/bin/python scripts/control_thumb_pickup.py \
  --source output/transfer_right_attempt \
  --checkpoint output/transfer_right_attempt/checkpoint.npz \
  --nominal output/transfer_right_attempt/checkpoint.npz \
  --output output/transfer_feedback --duration 2 --active-side both \
  --speed .4 --margin .0045 --predict .25 --contact-weight 40 \
  --pose-weight .0002 --seat-depth .004
```

`--speed` bounds the quintic joint interpolation, not just the difference
between exported poses. `--nominal-route` accepts a JSON list of objects with
`duration`, `phase` and a full `joints` vector in the robot's exported joint order.
Scripted non-feedback finger joints can follow that route while thumb feedback
continues. Clearance checks reject a colliding command before integration.

### Episode interface for the next bimanual solver

The [RoboSkills workflow pattern](https://github.com/openrobokit/roboskills)
keeps a task definition, policy actions and evaluation artifacts separate. The
same boundary is now available here through `BimanualSkillEpisode`: `reset`
constructs the recipe's initial scene, `observe` returns contact-level
measurements, `step` advances to an output frame, `score` applies the existing
physical gates, and `finish` writes the trajectory plus an auditable episode
bundle. The observation intentionally omits particle coordinates and joint
vectors so a future policy can use the same interface without privileged state.

```bash
.venv/bin/python scripts/run_skill_episode.py \
  examples/transfers/right_pickup_attempt.json \
  --output output/skill_right_attempt --active-side both --step 2
```

The command is a reproducible bimanual baseline. Its expected failure is useful:
the right near-string clearance remains the next gate to solve before releasing
the original loops and moving on to Cup, boat and Star. Results are stored in
`episode.json`, `log.jsonl`, `result.json` and `summary.json` beside the normal
Newton files. The standalone task manifest is
`examples/skills/revotron_bimanual_transfer.json`.

## Tool map

Scripts remain in `scripts/` to preserve command lines used by previous runs.
Use `--help` for each tool's parameters.

| Work | Entry points |
| --- | --- |
| Initial placement and widened hands | `run_single_player.py`, `initialize_wide_opening.py`; posture examples in `examples/` |
| Replay or continue dynamics | `replay_transfer.py`, `continue_hold.py`, `continue_opening_a.py`, `continue_cup.py` |
| Move fingers using observed rope | `control_thumb_pickup.py`, `control_old_loop.py` |
| Propose robot joint paths | `plan_thumb_transfer.py`, `plan_thumb_pose_path.py`, `plan_hand_spread.py`, `plan_bridge_transfer.py`, `plan_old_loop_support.py` |
| Search candidate motions and resolution | `search_preparation.py`, `search_thumb_transfer.py`, `search_thumb_sequence.py`, `probe_resolution.py` |
| Accept or reject trajectories | `validate_opening_a.py`, `validate_transfer.py`, `validate_bimanual.py` |
| Diagnose contacts and timing | `audit_position_one.py`, `audit_opening_support.py`, `audit_old_loop_support.py`, `audit_sliding_bight.py`, `audit_knuckle_clearance.py`, `audit_transfer_timing.py` |

Planning and search tools use geometric objectives to propose commands. Their
scores do not certify a captured bight, a released loop, or a completed figure.
The older old-loop helper tools are exploratory and currently target the left
old thumb loop with the right middle finger. Inspect their side assumptions
before adapting them to the opposite hand.

The shared implementation is in `revo3_cat_cradle/`: `transfers.py` tracks
material arcs and plans finger motion; `mesh_geometry.py` measures support on
enabled URDF knuckle meshes; `bimanual_checks.py` handles capsule clearance;
`segment_contacts.py` supplies the Newton segment self-contact kernel.

## What changed in the physics tools

MPR collision meshes close the gap below the MCP capsules. Support audits use
the same enabled geometry, including the concave knuckle surfaces. Self-contact
excludes only a local centerline neighborhood measured in meters; compressed
particle spacing no longer makes index distance a sufficient locality rule.
The 16 mm segment search bound remains enforced.

Hand–rope friction updates set both Newton friction channels. Initial material
length, wrist separation, settling duration and nominal active tension are
explicit parameters. The bundled recipes use friction 1.2 and a nominal 0.025 N
active tension target with zero additional take-up. These are simulation settings,
not measured material properties. Rest lengths can still change dynamically;
this is an ideal active-length model.

Every rope particle has positive mass. Robot joints are kinematic, and the rope
responds through contact and friction. Continuations do not attach, reset,
resample or project rope particles onto fingers. Initial placement is separate
from runtime integration. The two bundled recipes begin with an independently
placed Opening A and therefore do not prove a successful Position 1 transition.

## Rejected approaches and remaining work

The November 8 continuation adds an optional rope-edge contact model for the
finger capsules. At the 44 s boundary, an 8.16 mm rope edge cut 0.747 mm into
the little-finger capsule even though particle penetration was only 0.020 mm.
After enabling edge contacts and holding the same joints for 1.2 s,
the worst sampled edge penetration was 0.034 mm. The correction is solved inside
each XPBD iteration with contact friction; all particles remain dynamic.

A 3.2 s static continuation with this model retained both new bights, both old
thumb loops and the index/little supports. It passed the strain, self-contact
and completed-hold checks. Right near-strand clearance still reached only
1.559 mm in the final second, so the bilateral pickup remains unaccepted.
The full-history peak stretch was 1.168, versus 1.237 for the corresponding
particle-only hold. These are single deterministic CPU runs from one checkpoint.
See [the contact experiment](development/transfers.md#november-8-edge-contact-and-hold-experiments)
for commands and limits.

Higher friction alone did not solve the transfer: one comparison exceeded the
1.2 stretch limit. A full thumb return lost the new bight, while a partial return
provided a usable contact state. Raising the right wrist from a pose with the
old loop on the rounded DIP cap lost that loop. Keeping the thumb extended for
the approach preserved the old loop deeper on PIP, but later near-string
separation is still unresolved.

For the latest 44 s attempt, spreading each wrist outward by 15 mm improved
right near-string clearance over the fixed-wrist comparison, but the final-second
minimum was still 1.545 mm. The next required result is a bilateral pickup with
near-string clearance and a completed hold, followed by removal of the lower
original thumb loops and release of the little-finger loops. Boat and Star
transitions depend on that Cup & Saucer state.

[The chronological experiment journal](development/transfers.md) keeps older
candidate names and measurements, including rejected branches. Its `output/`
paths identify local runs; those large generated files are excluded from Git.
Use the recipes above for a portable starting point. Procedure references are
[Cup & Saucer](https://llx.com/Neil/string/playground/cup_and_saucer.html) and
[Alysion's figure instructions](https://www.alysion.org/figures/introduc.htm).

## Checks

```bash
.venv/bin/python -m pytest -m 'not integration' -q
uvx ruff==0.14.0 check --select F revo3_cat_cradle scripts tests
git diff --check
```

The tests cover state continuity, friction updates, clipped command replay,
material tracking, collision geometry and joint-rate limits. They do not replace
the full trajectory validators or establish real-hardware transfer.
