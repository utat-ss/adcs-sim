import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from hardware.sensors.sensors import VirtualGNSS


GNSS_CFG = Path("hardware/sensors/icd/gnss/orion_b16.json")


def test_loads_config():
    gnss = VirtualGNSS(GNSS_CFG)
    
    assert gnss.model == "Orion B16"
    assert "GPS" in gnss.compatible_satellites
    assert gnss.LLA_cov_matrix_meters.shape == (3, 3)

def test_rejects_invalid_latitude():
    pass

def test_rejects_invalid_longitude():
    pass

def test_rejects_invalid_altitude():
    pass

def test_produces_accurate_position_measurements():
    pass