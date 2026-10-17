import math
from pathlib import Path

import numpy as np
import pytest

from hardware.sensors.sensors import VirtualIMU


# WARNING: These unit tests are written before the implementation of the VirtualIMU. 
# Certain assumptions are made and placeholders are used, so the tests need to be 
# updated once the functions are implemented.


# This part requires adjustment once the IMU config file is implemented.
# TODO: Update the IMU config file path once the IMU config file is implemented.
IMU_CFG = Path("hardware/sensors/icd/imu.json")

@pytest.mark.skip(reason="Waiting for IMU config file implementation")
def test_loads_config():
    imu = VirtualIMU(IMU_CFG)

    # TODO: Input the correct model name once the IMU config file is implemented.
    # assert imu.model == "Generic IMU"

    assert len(imu.gyro_lims_rad_s) == 2
    assert imu.gyro_lims_rad_s[0] < imu.gyro_lims_rad_s[1]

    assert isinstance(imu.bias_instability_deg_hr, float)
    assert imu.bias_instability_deg_hr >= 0

    assert isinstance(imu.angle_random_walk_deg_sqrthr, float)
    assert imu.angle_random_walk_deg_sqrthr >= 0

    assert isinstance(imu.rate_random_walk_rad_s2_sqrthz, float)
    assert imu.rate_random_walk_rad_s2_sqrthz >= 0

    assert isinstance(imu.rate_hz, float)
    assert imu.rate_hz > 0


@pytest.mark.skip(reason="Waiting for IMU measure() implementation")
def test_rejects_invalid_input_dimensions():
    imu = VirtualIMU(IMU_CFG)

    with pytest.raises(ValueError):
        imu.measure([0, 0])

    with pytest.raises(ValueError):
        imu.measure([0, 0, 0, 0])


@pytest.mark.skip(reason="Waiting for IMU measure() implementation")
def test_handles_rates_exceeding_specified_limits():
    imu = VirtualIMU(IMU_CFG)
    
    # TODO: Confirm whether the IMU should return max/min values or raise
    # a ValueError when body rates exceed the specified limits.
    for _ in range(100):
        with pytest.raises(ValueError):
            imu.measure([imu.gyro_lims_rad_s[1] + 0.1, 0, 0])
        with pytest.raises(ValueError):
            imu.measure([0, imu.gyro_lims_rad_s[1] + 0.1, 0])
        with pytest.raises(ValueError):
            imu.measure([0, 0, imu.gyro_lims_rad_s[1] + 0.1])


@pytest.mark.skip(reason="Waiting for IMU implementation")
def test_produces_accurate_body_rates_in_sensor_frame():
    pass


@pytest.mark.skip(reason="Waiting for IMU implementation")
def test_sensor_noise_and_bias_characteristics_conserved_over_time():
    pass


