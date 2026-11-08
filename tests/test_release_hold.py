import numpy as np

from scripts.audit_loop_release import completed_stationary_hold


def test_stationary_hold_uses_recorded_motion_without_a_caption():
    times = np.arange(61) / 30
    joints = np.zeros((len(times), 2))
    joints[:30, 0] = np.linspace(0.2, 0, 30)
    joints[31:, 1] = 2e-16
    assert completed_stationary_hold(times, joints, 2)


def test_equal_endpoints_do_not_hide_an_intervening_motion():
    times = np.arange(31) / 30
    joints = np.zeros((31, 2))
    joints[15, 0] = 0.01
    assert not completed_stationary_hold(times, joints, 1)


def test_short_or_interrupted_observation_is_not_a_completed_hold():
    short = np.arange(25) / 30
    assert not completed_stationary_hold(short, np.zeros((25, 2)), short[-1])
    times = np.arange(31) / 30
    assert not completed_stationary_hold(times, np.zeros((31, 2)), 1.2)
    assert not completed_stationary_hold(times, np.full((31, 2), np.nan), 1)
