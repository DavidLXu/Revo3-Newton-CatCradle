"""A transfer audit must follow its physical boundary, not a legacy timestamp."""

import numpy as np
import pytest
from scripts.validate_transfer import transfer_window


def test_transfer_audit_is_invariant_to_initial_hold_duration():
    for start in (3, 6, 21):
        times = np.arange(start * 30 + 91) / 30
        original, moving, last = transfer_window(
            times, {"transfer_reference_time_s": start}
        )
        assert times[original] == start
        np.testing.assert_allclose(times[moving] - start, np.arange(1, 91) / 30)
        np.testing.assert_allclose(times[last] - start, np.arange(60, 91) / 30)


def test_transfer_audit_rejects_missing_boundary_and_empty_continuation():
    with pytest.raises(ValueError, match="reference"):
        transfer_window(np.arange(60) / 30, {"transfer_reference_time_s": 3})
    with pytest.raises(ValueError, match="history"):
        transfer_window(np.arange(91) / 30, {"transfer_reference_time_s": 3})
