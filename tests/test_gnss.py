import math
from pathlib import Path

import numpy as np
import pytest

from hardware.sensors.sensors import VirtualGNSS
from hardware.utils import great_circle_distance_geocentric_lla


GNSS_CFG = Path("hardware/sensors/icd/gnss/orion_b16.json")


def test_loads_config():
    gnss = VirtualGNSS(GNSS_CFG)
    
    assert gnss.model == "Orion B16"
    assert "GPS" in gnss.compatible_satellites
    assert gnss.LLA_cov_matrix_meters.shape == (3, 3)


def test_rejects_invalid_latitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    for _ in range(100):
        with pytest.raises(ValueError):
            gnss.measure([-91, 0, 10000])
        with pytest.raises(ValueError):
            gnss.measure([91, 0, 10000])


def test_rejects_invalid_longitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    for _ in range(100):
        with pytest.raises(ValueError):
            gnss.measure([0, -181, 10000])
        with pytest.raises(ValueError):
            gnss.measure([0, 181, 10000])


def test_rejects_invalid_altitude_inputs():
    gnss = VirtualGNSS(GNSS_CFG)

    for _ in range(100):
        with pytest.raises(ValueError):
            gnss.measure([0, 0, 0])


def test_produces_valid_position_outputs():
    gnss = VirtualGNSS(GNSS_CFG)
    # Check the lat and long boundaries
    coords = [[89.999, 0, 10000], [-89.999, 0, 10000], [0, 179.999, 10000], [0, -179.999, 10000]]
    
    for _ in range(100):
        for i in range(len(coords)):
            measured_coords = gnss.measure(coords[i])

            assert len(measured_coords) == 3
            assert -90 <= measured_coords[0] <= 90
            assert -180 < measured_coords[1] <= 180
            assert measured_coords[2] > 0


def test_produces_accurate_position_measurements():
    gnss = VirtualGNSS(GNSS_CFG)
    coords_geodetic = [-90, -180, 6871000]
    cases = 0
    total_error = 0

    while coords_geodetic[0] < 90:
        coords_geodetic[1] = -180
        coords_geodetic[0] += 10
        while coords_geodetic[1] < 180:
            measured_coords = gnss.measure(coords_geodetic)

            coords_geocentric = gnss.from_geodetic_LLA(coords_geodetic)
            measured_coords_geocentric = gnss.from_geodetic_LLA(measured_coords)

            error = great_circle_distance_geocentric_lla(
                coords_geocentric[0],
                coords_geocentric[1],
                measured_coords_geocentric[0],
                measured_coords_geocentric[1],
                coords_geocentric[2],
            )
            total_error += error
            cases += 1
            coords_geodetic[1] += 10
    
    # The average great circle distance between true input coords and a sufficient set of output LLA coords using measure() should be ~2.13 m based on the 2.0 CEP accuracy of the Orion B16 Receiver.
    sigma = math.sqrt(gnss.LLA_cov_matrix_meters[0][0])
    expected_mean_error = sigma * math.sqrt(math.pi / 2)

    assert total_error / cases < expected_mean_error + 0.1


def test_measurement_noise_is_statistically_consistent():
    np.random.seed(0)

    gnss = VirtualGNSS(GNSS_CFG)
    coords_geodetic = [-90, -180, 6871000]
    std = np.sqrt(gnss.LLA_cov_matrix_meters[0][0])

    cases = 0
    within_1_std = 0
    within_2_std = 0
    within_3_std = 0

    while coords_geodetic[0] < 90:
        coords_geodetic[1] = -180
        coords_geodetic[0] += 10
        while coords_geodetic[1] < 180:
            measured_coords = gnss.measure(coords_geodetic)

            coords_geocentric = gnss.from_geodetic_LLA(coords_geodetic)
            measured_coords_geocentric = gnss.from_geodetic_LLA(measured_coords)

            error = math.radians(
                measured_coords_geocentric[0] - coords_geocentric[0]
            ) * coords_geocentric[2]
            error = abs(error)

            if error < 1 * std:
                within_1_std += 1
            if error < 2 * std:
                within_2_std += 1
            if error < 3 * std:
                within_3_std += 1

            cases += 1
            coords_geodetic[1] += 10

    assert within_1_std / cases == pytest.approx(0.68, abs=0.1)
    assert within_2_std / cases == pytest.approx(0.95, abs=0.1)
    assert within_3_std / cases == pytest.approx(0.997, abs=0.1)