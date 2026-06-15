import pytest
import math
import numpy as np

from hardware.sensors.sensors import VirtualGNSS
from pathlib import Path


GNSS_CFG = Path("hardware/sensors/icd/gnss/orion_b16.json")


def GC_distance(lat_1, long_1, lat_2, long_2, center_dist):
    """
    Compute distance along a great circle between two given pairs of coordinates (geocentric latitude). 

    The average GC_distance between true input coords and a sufficient set of output LLA coords using measure() should be ~2.13 m based on the 2.0 CEP accuracy of the Orion B16 Receiver.
    """
    delta_lat = (lat_2 - lat_1) * math.pi / 180
    delta_long = (long_2 - long_1) * math.pi / 180

    hav_theta = VirtualGNSS.haversine(delta_lat) + math.cos(lat_1 * math.pi / 180) * math.cos(lat_2 * math.pi / 180) * VirtualGNSS.haversine(delta_long)

    return 2 * math.asin(hav_theta**0.5) * center_dist
    
def Euc_distance(lat_1, long_1, center_dist_1, lat_2, long_2, center_dist_2):
    """
    Compute Euclidean distance between two different pairs of coordinates (geocentric latitude).
    """
    theta_1_rad = math.pi / 2 - lat_1 * math.pi / 180
    long_1_rad = long_1 * math.pi / 180
    theta_2_rad = math.pi / 2 - lat_2 * math.pi / 180
    long_2_rad = long_2 * math.pi / 180

    x1 = center_dist_1 * math.sin(theta_1_rad) * math.cos(long_1_rad)
    x2 = center_dist_2 * math.sin(theta_2_rad) * math.cos(long_2_rad)

    y1 = center_dist_1 * math.sin(theta_1_rad) * math.sin(long_1_rad)
    y2 = center_dist_2 * math.sin(theta_2_rad) * math.sin(long_2_rad)

    z1 = center_dist_1 * math.cos(theta_1_rad)
    z2 = center_dist_2 * math.cos(theta_2_rad)

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


def test_loads_config():
    gnss = VirtualGNSS(GNSS_CFG)
    
    assert gnss.model == "Orion B16"
    assert "GPS" in gnss.compatible_satellites
    assert gnss.LLA_cov_matrix_meters.shape == (3, 3)

def test_rejects_invalid_latitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    with pytest.raises(ValueError):
        gnss.measure([-91, 0, 10000])
    with pytest.raises(ValueError):
        gnss.measure([91, 0, 10000])

def test_rejects_invalid_longitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    # From the docstring, the longitude uses the range -180 < longitude <= 180, so -180 is invalid.
    with pytest.raises(ValueError):
        gnss.measure([0, -180, 10000])
    with pytest.raises(ValueError):
        gnss.measure([0, -181, 10000])
    with pytest.raises(ValueError):
        gnss.measure([0, 181, 10000])

def test_rejects_invalid_altitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    with pytest.raises(ValueError):
        gnss.measure([0, 0, 0])

def test_produces_valid_position_outputs():
    gnss = VirtualGNSS(GNSS_CFG)

    # Check the lat and long boundaries
    coords = [[89.999, 0, 10000], [-89.999, 0, 10000], [0, 179.999, 10000], [0, -179.999, 10000]]
    for i in range(len(coords)):
        measured_coords = gnss.measure(coords[i])

        assert len(measured_coords) == 3
        assert -90 <= measured_coords[0] <= 90
        assert -180 < measured_coords[1] <= 180
        assert measured_coords[2] > 0


def test_produces_accurate_position_measurements():
    gnss = VirtualGNSS(GNSS_CFG)

    # There could be a better comprehensive list of coords or maybe randomize it instead
    coords = [[89.999, 0, 10000], [-89.999, 0, 10000], [0, 179.999, 10000], [0, -179.999, 10000], [-89.999, 179.999, 10000], [89.999, 179.999, 10000], [-89.999, -179.999, 10000], [89.999, -179.999, 10000], [10, 10, 6871000], [-10, -10, 6871000]]
    total_error = 0

    cep50 = 2.0
    std = cep50 / math.sqrt(-2 * math.log(1 - 0.50))
    cep997 = std * math.sqrt(-2 * math.log(1 - 0.997))

    for i in range(len(coords)):
        measured_coords = gnss.measure(coords[i])
        error = GC_distance(coords[i][0], coords[i][1], measured_coords[0], measured_coords[1], coords[i][2])
        total_error += error

        assert error < cep997
    
    assert total_error / len(coords) < 2.13