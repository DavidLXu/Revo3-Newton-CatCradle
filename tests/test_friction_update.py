"""Parameter experiments must preserve the loaded physical state."""

import numpy as np
import pytest
from revo3_cat_cradle.sim import Simulation, Config


def test_legacy_checkpoint_uses_historical_tension_target(tmp_path):
    import json

    sim = Simulation(Config(substeps=1, iterations=1))
    path = tmp_path / "legacy.npz"
    sim.save_checkpoint(path)
    with np.load(path) as saved:
        content = {key: saved[key].copy() for key in saved.files}
    config = json.loads(str(content["config"]))
    del config["adaptive_tension_n"]
    del config["segment_hand_contacts"]
    content["config"] = json.dumps(config)
    np.savez_compressed(path, **content)
    sim.load_checkpoint(path)
    assert sim.config.adaptive_tension_n == 0.08
    sim.config.adaptive_tension_n = 0.025
    with pytest.raises(ValueError, match="physics configuration"):
        sim.load_checkpoint(path)


def test_bimanual_checkpoint_restores_payout_controller_and_dynamic_state(tmp_path):
    from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation
    from revo3_cat_cradle.single_player import Motion, initial_loop

    robot = RevoTron()
    motion = Motion(robot, upright=True)
    sim = BimanualSimulation(
        robot,
        motion,
        initial_loop(robot, motion),
        Config(substeps=1, iterations=1),
        length_control="local",
        rest_length_max_scale=3.0,
    )
    sim.advance(1 / 30)
    before = {
        name: getattr(sim.state, name).numpy().copy()
        for name in ("particle_q", "particle_qd", "body_q", "body_qd")
    }
    lengths = sim.rest_lengths.copy()
    checkpoint = tmp_path / "state.npz"
    sim.save_checkpoint(checkpoint)
    sim.rest_length_max_scale = 2.0
    sim.length_control = "global"
    sim.advance(2 / 30)
    sim.load_checkpoint(checkpoint)
    assert sim.rest_length_max_scale == 3.0
    assert sim.length_control == "local"
    assert sim.t == 1 / 30
    np.testing.assert_array_equal(sim.rest_lengths, lengths)
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(sim.state, name).numpy(), value)


def test_contact_friction_updates_both_newton_materials_without_resetting_rope():
    sim = Simulation(Config())
    sim.advance(1 / 30)
    before = {
        name: getattr(sim.state, name).numpy().copy()
        for name in ("particle_q", "particle_qd", "body_q", "body_qd")
    }
    lengths = sim.rest_lengths.copy()
    masses = sim.model.particle_mass.numpy().copy()
    springs = sim.model.spring_rest_length.numpy().copy()
    particle_mu = sim.model.particle_mu
    time = sim.t
    sim.set_contact_friction(2.0)
    assert sim.model.soft_contact_mu == 2.0
    assert sim.config.friction == 2.0
    np.testing.assert_array_equal(sim.model.shape_material_mu.numpy(), 2.0)
    assert sim.model.particle_mu == particle_mu
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(sim.state, name).numpy(), value)
    np.testing.assert_array_equal(sim.rest_lengths, lengths)
    np.testing.assert_array_equal(sim.model.particle_mass.numpy(), masses)
    np.testing.assert_array_equal(sim.model.spring_rest_length.numpy(), springs)
    assert sim.t == time
    for invalid in (-1.0, np.nan, np.inf):
        with pytest.raises(ValueError):
            sim.set_contact_friction(invalid)
        assert sim.config.friction == 2.0


def test_damping_change_preserves_state_and_checkpoint_replays_it(tmp_path):
    sim = Simulation(Config(substeps=8, iterations=4))
    sim.advance(1 / 30)
    before = {
        name: getattr(sim.state, name).numpy().copy()
        for name in ("particle_q", "particle_qd", "body_q", "body_qd")
    }
    lengths = sim.rest_lengths.copy()
    damping = sim.model.spring_damping.numpy().copy()
    sim.set_stretch_damping(.08)
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(sim.state, name).numpy(), value)
    np.testing.assert_array_equal(sim.rest_lengths, lengths)
    np.testing.assert_array_equal(
        sim.model.spring_damping.numpy()[sim.model.particle_count:],
        damping[sim.model.particle_count:],
    )
    for invalid in (-1, np.inf, np.nan):
        with pytest.raises(ValueError):
            sim.set_stretch_damping(invalid)
    path = tmp_path / "damped.npz"
    sim.save_checkpoint(path)
    restored = Simulation(Config(substeps=8, iterations=4, stretch_damping=.08))
    restored.load_checkpoint(path)
    sim.advance(2 / 30)
    restored.advance(2 / 30)
    np.testing.assert_array_equal(
        restored.state.particle_q.numpy(), sim.state.particle_q.numpy()
    )
    np.testing.assert_array_equal(
        restored.state.particle_qd.numpy(), sim.state.particle_qd.numpy()
    )


def test_bimanual_segment_contact_switch_preserves_state_and_resumes(tmp_path):
    from dataclasses import replace
    from revo3_cat_cradle.bimanual import RevoTron, BimanualSimulation
    from revo3_cat_cradle.single_player import Motion, initial_loop

    robot = RevoTron()
    motion = Motion(robot, upright=True)
    config = Config(substeps=4, iterations=4)
    sim = BimanualSimulation(robot, motion, initial_loop(robot, motion), config)
    sim.advance(1 / 30)
    before = {name: getattr(sim.state, name).numpy().copy() for name in (
        "particle_q", "particle_qd", "body_q", "body_qd",
    )}
    lengths = sim.rest_lengths.copy()
    sim.set_segment_hand_contacts(True)
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(sim.state, name).numpy(), value)
    np.testing.assert_array_equal(sim.rest_lengths, lengths)
    path = tmp_path / "segments.npz"
    sim.save_checkpoint(path)
    restored = BimanualSimulation(
        robot, motion, initial_loop(robot, motion), replace(config),
    )
    restored.load_checkpoint(path)
    assert restored.solver.segment_hand_contacts
    sim.advance(2 / 30)
    restored.advance(2 / 30)
    np.testing.assert_array_equal(restored.state.particle_q.numpy(), sim.state.particle_q.numpy())
    np.testing.assert_array_equal(restored.state.particle_qd.numpy(), sim.state.particle_qd.numpy())
