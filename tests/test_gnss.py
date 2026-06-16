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
    coords = [-90, -180, 6871000]
    cases = 0
    total_error = 0

    while coords[0] < 90:
        coords[1] = -180
        coords[0] += 10
        while coords[1] < 180:
            measured_coords = gnss.measure(coords)
            error = GC_distance(coords[0], coords[1], measured_coords[0], measured_coords[1], coords[2])
            total_error += error
            cases += 1
            coords[1] += 10
    
    # The average GC_distance between true input coords and a sufficient set of output LLA coords using measure() should be ~2.13 m based on the 2.0 CEP accuracy of the Orion B16 Receiver.
    assert total_error / cases < 2.13


def test_measurement_noise_is_statistically_consistent():
    np.random.seed(0)

    gnss = VirtualGNSS(GNSS_CFG)
    coords = [-90, -180, 6871000]
    std = np.sqrt(gnss.LLA_cov_matrix_meters[0][0])

    cases = 0
    within_1_std = 0
    within_2_std = 0
    within_3_std = 0

    while coords[0] < 90:
        coords[1] = -180
        coords[0] += 10
        while coords[1] < 180:
            measured_coords = gnss.measure(coords)
            error = math.radians(measured_coords[0] - coords[0]) * coords[2]
            error = abs(error)

            if error < 1 * std:
                within_1_std += 1
            if error < 2 * std:
                within_2_std += 1
            if error < 3 * std:
                within_3_std += 1

            cases += 1
            coords[1] += 10

    assert within_1_std / cases == pytest.approx(0.68, abs=0.1)
    assert within_2_std / cases == pytest.approx(0.95, abs=0.1)
    assert within_3_std / cases == pytest.approx(0.997, abs=0.1)