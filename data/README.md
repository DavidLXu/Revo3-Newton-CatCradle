# Normalized motion example

`forward_motion.json` is a simulation fixture with 23 commands over 11.5 seconds.
Each frame contains `duration_s` and `positions`, ordered by `joint_names`.
Angles are URDF radians; there is no runtime zero-offset conversion.

These are command targets, not measurements. The first command's approach is
unknown, so the first target is held. The little-finger pickup and thumb timing
correction live in `motion.py`, not in this baseline fixture. Raw calibration,
device identifiers and the unused reverse recording are not included.

See [normalization details](../docs/design.md#motion-data) before interpreting
these values as a model of the physical hand. The example is provided under
the repository's Apache-2.0 license.
