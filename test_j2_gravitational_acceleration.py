import pytest
import numpy as np
from environment import j2_acceleration_m_s2

def test_only_xaxis_j2_acceleration_m_s2():
    """Tests the function when only given a position vector in the x-axis"""
    test_result = j2_acceleration_m_s2((8000000, 0, 0))
    assert test_result[0] == pytest.approx(-0.0064288812525067655)
    assert test_result[1] == pytest.approx(0)
    assert test_result[2] == pytest.approx(0)
    
def test_only_yaxis_j2_acceleration_m_s2():
    """Tests the function when only given a position vector in the y-axis"""
    test_result = j2_acceleration_m_s2((0, 9000000, 0))
    assert test_result[0] == pytest.approx(0)
    assert test_result[1] == pytest.approx(-0.004013518916364534)
    assert test_result[2] == pytest.approx(0)

def test_all_axes_j2_acceleration_m_s2():
    """Ensures that the function works properly when given a position vector with a measurement in all axes"""
    test_result = j2_acceleration_m_s2((7000000, 100, 50))
    assert test_result[0] == pytest.approx(-0.010967387586626055)
    assert test_result[1] == pytest.approx(-0.000000156676965)
    assert test_result[2] == pytest.approx(-0.000000235015448)

def test_symmetry_j2_acceleration_m_s2():
    """This test ensures that the symmetry of the equation is followed through"""

    test_result_xaxis = j2_acceleration_m_s2((7000000, 1000, 50))
    test_result_yaxis = j2_acceleration_m_s2((1000,7000000, 50))

    assert test_result_xaxis[0] == test_result_yaxis[1]
    assert test_result_xaxis[1] == test_result_yaxis[0]
    assert test_result_xaxis[2] == test_result_yaxis[2]

def test_below_100km_j2_acceleration_m_s2():
    """This unit test checks if an error is raised when the position vector is less that 100km above the Earth's surface"""

    with pytest.raises(ValueError, match = "Position vector must be at least 100km above Earth's surface."):
        j2_acceleration_m_s2((7000,0,0))

def test_at_100km_j2_acceleration_m_s2():
    """This test ensures that the function runs and gives numerical data if the position vector is exactly 100km above the Earth's surface"""

    test_result = j2_acceleration_m_s2((0,6478136.3,0))
    assert(len(test_result)) == 3
    for result in test_result:
        assert np.isfinite(result) == True
