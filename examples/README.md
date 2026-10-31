# Experiment inputs

Run the repository setup and `scripts/setup_revotron.sh` before using these
inputs. All angles are radians and all distances are meters.

| Input | Use and recorded status |
| --- | --- |
| `transfers/left_pickup.json` | Self-contained initial placement and recorded joint sequence. The original run passed the left-pickup checks; see the [report](../docs/validation/transfers-left-pickup.json). |
| `transfers/right_pickup_attempt.json` | Same initial fixture with a later right-thumb attempt. Right near-string clearance failed. |
| `position_one_left_thumb_bearing.json` | `run_single_player.py --initial-posture`: left thumb adjustment that retained the initial four supports in the low-tension experiment. A complete Opening A is not established. |
| `position_one_thumb_bearing.json` | Rejected two-thumb posture comparison; lost the right-little support. Kept as an experimental input. |
| `wide_opening_idle_posture.json` | `run_single_player.py --idle-posture`: middle/ring posture proposal for a 350 mm Position 1. The full sequence failed retention and strain checks. |
| `skills/revotron_bimanual_transfer.json` | Task manifest for the episode runner; it intentionally points to the recorded right-pickup attempt so the failed clearance gate remains visible. |

Transfer recipes use `schema_version: 1`, an explicit `joint_names` ordering,
`commands` with absolute times, and `motion` metadata. Both command data and
metadata are required because interior checkpoint branches preserve clipped
quintic segments. `transfer_plan` contains material indices in the initialized
rope discretization. Changing sampling invalidates those indices.

The recipes contain the initial geometry only; they do not contain per-frame
rope positions, videos, device calibration, or robot meshes. The replay entry
point refuses to overwrite an existing output directory. See the
[experiment guide](../docs/experiments.md) for commands and acceptance limits.

Following the small, explicit workflow pattern in
[openrobokit/roboskills](https://github.com/openrobokit/roboskills), run the task
manifest's recipe through the episode lifecycle wrapper:

```bash
.venv/bin/python scripts/run_skill_episode.py \
  examples/transfers/right_pickup_attempt.json \
  --output output/skill_right_attempt --active-side both
```

The output contains `episode.json`, policy-facing `log.jsonl`, `result.json`,
and `summary.json` alongside the normal Newton trajectory. The episode exposes
contact gaps and strain to a policy, while the score keeps the rope particles,
joint vectors and material identities private. The bundled manifest uses the
unaccepted right-pickup attempt on purpose: it is a reproducible starting point
for the next bimanual route, not a claim that Cup & Saucer is solved.
