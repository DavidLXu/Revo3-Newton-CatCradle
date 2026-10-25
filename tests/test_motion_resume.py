"""An interior checkpoint must not restart the next command at an older pose."""

import json

import numpy as np
import pytest

from revo3_cat_cradle.single_player import Motion


def motion_from(commands, metadata=None):
    motion = Motion.__new__(Motion)
    motion.times = commands["times"].copy()
    motion.targets = [np.asarray(q) for q in commands["targets"]]
    motion.phases = commands["phases"].copy()
    motion.metadata = metadata or {}
    return motion


def commands_of(motion):
    return dict(
        times=motion.times,
        targets=[q.tolist() for q in motion.targets],
        phases=motion.phases,
    )


def test_interior_resume_preserves_prefix_and_first_step_speed_after_serialization():
    source = motion_from(
        dict(times=[0, 2, 4], targets=[[0, 0], [1, -1], [2, 0]], phases=["a", "b", "c"])
    )
    start, dt, speed = 1.3, 1 / 30, 1.5
    measured = source.sample(start)[0]
    resumed = motion_from(commands_of(source))
    resumed.resume_at(commands_of(source), source.metadata, start, measured)
    next_pose = measured + np.array([1, -1]) * speed * dt / 1.875
    resumed.append(start + dt, next_pose, "Feedback")
    restored = motion_from(
        json.loads(json.dumps(commands_of(resumed))),
        json.loads(json.dumps(resumed.metadata)),
    )
    for t in np.linspace(0, start, 101):
        np.testing.assert_allclose(
            restored.sample(t)[0], source.sample(t)[0], atol=1e-14
        )
    np.testing.assert_allclose(restored.sample(start + 1e-8)[0], measured, atol=1e-12)
    times = np.linspace(start, start + dt, 201)
    samples = np.array([restored.sample(t)[0] for t in times])
    assert np.max(abs(np.diff(samples, axis=0) / np.diff(times)[:, None])) <= speed
    np.testing.assert_allclose(samples[-1], next_pose)

    # Branching again inside the preserved prefix keeps its original polynomial.
    earlier = motion_from(commands_of(restored))
    earlier.resume_at(
        commands_of(restored), restored.metadata, 0.8, source.sample(0.8)[0]
    )
    earlier.append(1, source.sample(0.8)[0], "Hold")
    for t in np.linspace(0, 0.8, 31):
        np.testing.assert_allclose(
            earlier.sample(t)[0], source.sample(t)[0], atol=1e-14
        )


def test_resume_rejects_inconsistent_checkpoint_instead_of_hiding_a_pose_jump():
    commands = dict(times=[0, 2], targets=[[0], [1]], phases=["a", "b"])
    motion = motion_from(commands)
    with pytest.raises(ValueError, match="Checkpoint joint pose"):
        motion.resume_at(commands, {}, 1, np.array([0.9]))


def test_scripted_support_rate_bounds_the_resampled_quintic_not_just_nominal_motion():
    from scripts.control_thumb_pickup import scripted_support_target

    nominal = motion_from(
        dict(
            times=[0, 2],
            targets=[[0, 0, 7], [0.773, 0.2, 7]],
            phases=["Start", "Support"],
        )
    )
    actual = motion_from(dict(times=[0], targets=[[0, 0, 7]], phases=["Start"]))
    speed, fps = 0.8, 30
    for frame in range(1, 91):
        q = scripted_support_target(
            actual.targets[-1], nominal.sample(frame / fps)[0], [0, 1], 1 / fps, speed
        )
        actual.append(frame / fps, q, "Rate-limited support")
    times = np.linspace(0, 3, 9001)
    samples = np.array([actual.sample(t)[0] for t in times])
    assert np.max(abs(np.diff(samples[:, 0]) / np.diff(times))) <= speed + 1e-10
    np.testing.assert_allclose(samples[:, 1], samples[:, 0] * 0.2 / 0.773, atol=1e-14)
    np.testing.assert_array_equal(np.array(actual.targets)[:, 2], np.full(91, 7))
    np.testing.assert_allclose(samples[:, 2], 7, rtol=0, atol=1e-14)
    np.testing.assert_allclose(samples[-1], nominal.targets[-1])
