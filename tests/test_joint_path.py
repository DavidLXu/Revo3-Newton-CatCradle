"""Continuous robot commands keep momentum through spatial planning knots."""

import json

import numpy as np
import pytest

from revo3_cat_cradle.joint_path import (
    make_joint_path, evaluate_joint_path, joint_path_bounds, validate_joint_path,
)
from revo3_cat_cradle.single_player import Motion


def empty_motion():
    motion = Motion.__new__(Motion)
    motion.times, motion.targets, motion.phases = [0.0], [np.array([0.0, 0.0])], ["Start"]
    motion.metadata = {}
    return motion


def commands(motion):
    return dict(times=motion.times, targets=[q.tolist() for q in motion.targets], phases=motion.phases)


def test_continuous_path_has_no_stop_at_an_internal_knot_and_resumes_exactly():
    knots = np.linspace(0, 1, 5)
    path = make_joint_path(knots, np.column_stack([knots, knots**2]))
    source = empty_motion()
    source.append(2, np.array([1., 1.]), "Continuous", joint_path=path)
    dt = 1e-5
    left = (source.sample(1)[0] - source.sample(1 - dt)[0]) / dt
    right = (source.sample(1 + dt)[0] - source.sample(1)[0]) / dt
    np.testing.assert_allclose(left, right, atol=2e-5)
    np.testing.assert_allclose(right, [.9375, .9375], atol=2e-5)
    np.testing.assert_allclose((source.sample(dt)[0] - source.sample(0)[0]) / dt, 0, atol=1e-8)

    restored = empty_motion()
    start = 1.31
    restored.resume_at(json.loads(json.dumps(commands(source))),
                       json.loads(json.dumps(source.metadata)), start, source.sample(start)[0])
    restored.append(3, restored.targets[-1], "Hold")
    for t in np.linspace(0, start, 101):
        np.testing.assert_allclose(restored.sample(t)[0], source.sample(t)[0], atol=1e-14)
    np.testing.assert_allclose(restored.sample(2)[0], source.sample(start)[0])
    # Branch again inside the preserved spline, retaining the original time scale.
    earlier = empty_motion()
    earlier.resume_at(commands(restored), restored.metadata, .73, source.sample(.73)[0])
    earlier.append(1, earlier.targets[-1], "Hold")
    for t in np.linspace(0, .73, 101):
        np.testing.assert_allclose(earlier.sample(t)[0], source.sample(t)[0], atol=1e-14)


def test_analytic_bounds_catch_between_knot_overshoot_and_bound_time_speed():
    knots = np.linspace(0, 1, 4)
    values = np.column_stack([np.array([0., 1., 1., 0.]), knots**3])
    path = make_joint_path(knots, values)
    low, high, speed = joint_path_bounds(path, .6)
    assert high[0] > 1.1  # Checking only the IK knots would miss this overshoot.
    times = np.linspace(0, .6, 6001)
    u = times / .6
    u = u**3 * (10 + u * (-15 + 6 * u))
    q = np.array([evaluate_joint_path(path, v) for v in u])
    assert np.all(q >= low - 1e-12) and np.all(q <= high + 1e-12)
    assert abs(np.diff(q, axis=0) / np.diff(times)[:, None]).max() <= speed


def test_serialized_discontinuity_and_wrong_endpoint_are_rejected():
    path = make_joint_path([0, .5, 1], [[0, 0], [.5, .25], [1, 1]])
    with pytest.raises(ValueError, match="endpoints"):
        validate_joint_path(path, [0, 0], [1, .9], 1)
    path["coefficients"][3][1][0] += .01
    with pytest.raises(ValueError, match="C2"):
        validate_joint_path(path, [0, 0], [1, 1], 1)


def test_speed_bound_uses_slow_endpoint_envelope_without_missing_middle_peak():
    from scipy.interpolate import BSpline
    from math import factorial

    breaks = np.linspace(0, 1, 25)
    knots = np.r_[np.zeros(3), breaks, np.ones(3)]
    # A steep endpoint change, then a straight second coordinate. The old
    # global product incorrectly pairs the endpoint slope with mid-motion speed.
    targets = np.zeros((len(knots) - 4, 2))
    targets[1:, 0] = .1
    targets[:, 1] = [np.mean(knots[i + 1:i + 4]) for i in range(len(targets))]
    spline = BSpline(knots, targets, 3)
    path = dict(knots=breaks.tolist(), coefficients=np.stack([
        spline(breaks[:-1], nu=k) / factorial(k) for k in (3, 2, 1, 0)
    ]).tolist())
    for duration in (.4, 2):
        _, _, bound = validate_joint_path(path, targets[0], targets[-1], duration)
        t = np.linspace(0, 1, 100001)
        u = t**3 * (10 + t * (-15 + 6 * t))
        velocity = spline(u, nu=1) * (30 * t**2 * (1 - t)**2 / duration)[:, None]
        assert np.max(abs(velocity)) <= bound
        assert bound >= 1.875 / duration
        assert bound < 7.2 * 1.875 / duration / 2


def test_newton_checkpoint_continuation_preserves_continuous_path_and_rope(tmp_path):
    from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation, ROBOT_URDF
    from revo3_cat_cradle.single_player import initial_loop
    from revo3_cat_cradle.sim import Config

    if not ROBOT_URDF.exists():
        pytest.skip("Optional RevoTron assets missing")
    robot = RevoTron()
    motion = Motion(robot, upright=True)
    initial = motion.targets[0].copy()
    target = initial.copy()
    target[robot.indices["right_ring_MCP_joint"]] += .02
    path = make_joint_path([0, .5, 1], [initial, (initial + target) / 2, target])
    motion.append(.2, target, "Continuous", joint_path=path)
    config = Config(substeps=2, iterations=2, adaptive_length=False)
    sim = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    sim.advance(1 / 30)
    sim.save_checkpoint(tmp_path / "state.npz")
    restored_motion = Motion(robot, upright=True)
    restored_motion.times = motion.times.copy()
    restored_motion.targets = [q.copy() for q in motion.targets]
    restored_motion.phases = motion.phases.copy()
    restored_motion.metadata = json.loads(json.dumps(motion.metadata))
    restored = BimanualSimulation(robot, restored_motion, initial_loop(robot, restored_motion), config)
    restored.load_checkpoint(tmp_path / "state.npz")
    sim.advance(2 / 30)
    restored.advance(2 / 30)
    np.testing.assert_array_equal(restored.q, sim.q)
    np.testing.assert_array_equal(sim.q, motion.sample(2 / 30)[0])
    for field in ("particle_q", "particle_qd"):
        np.testing.assert_array_equal(getattr(restored.state, field).numpy(),
                                      getattr(sim.state, field).numpy())
    np.testing.assert_array_equal(restored.rest_lengths, sim.rest_lengths)
