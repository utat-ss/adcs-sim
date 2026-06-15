import pytest
import numpy as np

from hardware.sensors.sensors import VirtualGNSS
from pathlib import Path


GNSS_CFG = Path("hardware/sensors/icd/gnss/orion_b16.json")


def test_loads_config():
    gnss = VirtualGNSS(GNSS_CFG)
    
    assert gnss.model == "Orion B16"
    assert "GPS" in gnss.compatible_satellites
    assert gnss.LLA_cov_matrix_meters.shape == (3, 3)

def test_rejects_invalid_latitude():
    gnss = VirtualGNSS(GNSS_CFG)

    with pytest.raises(ValueError):
        gnss.measure([-91, 0, 1])
    with pytest.raises(ValueError):
        gnss.measure([91, 0, 1])

def test_rejects_invalid_longitude():
    gnss = VirtualGNSS(GNSS_CFG)

    with pytest.raises(ValueError):
        gnss.measure([0, -181, 1])
    with pytest.raises(ValueError):
        gnss.measure([0, 181, 1])

def test_rejects_invalid_altitude():
    gnss = VirtualGNSS(GNSS_CFG)

    with pytest.raises(ValueError):
        gnss.measure([0, 0, 0])

def test_produces_accurate_position_measurements():
    gnss = VirtualGNSS(GNSS_CFG)
    # TODO
    pass

def test_jamming():
    # TODO: VirtualGNSS jamming behaviour is not implemented yet
    pass