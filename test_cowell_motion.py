import pytest
import numpy as np
from orbit import cowell_motion

test_samples = [
    # four normal testing
    {
        "x": np.array([1000000.0, 0.0, 2000000.0, 0.0, 3000.0, 0.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([0., 3000., 0., -35.65204, 0., -71.30407])
    },
    {
        "x": np.array([1000000.0, 0.0, 2000000.0, 0.0, 3000.0, 0.0]),
        "add_drag": True,
        "add_J2": False,
        "expected_output": np.array([0., 3000., 0., -35.65204, -2.97e-07, -71.30407])
    },
    {
        "x": np.array([1000000.0, 0.0, 2000000.0, 0.0, 3000.0, 0.0]),
        "add_drag": False,
        "add_J2": True,
        "expected_output": None # or raise error for "Position vector must be at least 100km above Earth's surface."
    },
    {
        "x": np.array([1000000.0, 0.0, 2000000.0, 0.0, 3000.0, 0.0]),
        "add_drag": True,
        "add_J2": True,
        "expected_output": None # or raise error for "Position vector must be at least 100km above Earth's surface."
    },
    # zero/near-zero position/velocity vector
    {
        "x": np.array([0.0, 0.0, 0.0, 1000.0, 2000.0, 3000.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([1000., 2000., 3000., np.nan, np.nan, np.nan])
    },
    {
        "x": np.array([1e-3, 0.0, 0.0, 1000.0, 2000.0, 3000.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([1000., 2000., 3000., -3.98601877e20, 0., 0.])
    },
    {
        "x": np.array([1000000.0, 2000000.0, 3000000.0, 0.0, 0.0, 0.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([0., 0., 0., -7.60935, -15.21869, -22.82804])
    },
    {
        "x": np.array([1000000.0, 2000000.0, 3000000.0, 1e-3, 0.0, 0.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([1e-3, 0., 0., -7.60935, -15.21869, -22.82804])
    },
    # large position
    {
        "x": np.array([1e13, 1000000.0, 2000000.0, 3000.0, 4000.0, 5000.0]),
        "add_drag": False,
        "add_J2": False,
        "expected_output": np.array([3000., 4000., 5000., -3.98602e-12, -3.98602e-19, -7.97204e-19])
    }
]

@pytest.mark.parametrize("sample", test_samples)
def test_cowell_motion_cases(sample):
    x = sample["x"]
    add_drag = sample["add_drag"]
    add_J2 = sample["add_J2"]
    expected_output = sample["expected_output"]

    xdot = cowell_motion(x, add_drag, add_J2)

    if expected_output is None:
        assert xdot == expected_output
    np.testing.assert_array_almost_equal(xdot, expected_output)