import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
import numpy as np
from hardware.sensors.sensors import VirtualFSS

@pytest.fixture

def mock_fss():
    fss = VirtualFSS.__new__(VirtualFSS)

    fss.model = "Test FSS"
    fss.fov_deg = 45.0
    fss.rate_hz = 10.0
    fss.cov_deg2 = np.zeros((2,2), dtype = float)

    return fss

def test_sun_vector_fov(mock_fss):
    sun_vector = np.array([1.0, 0.0, 1.0])

    output = mock_fss._angle_computation(sun_vector)

    assert output["sun_present"] == True
    assert output["alpha_deg"] == pytest.approx(45.0)
    assert output["beta_deg"] == pytest.approx(0.0)


def test_sun_vector_out_of_fov(mock_fss):
    sun_vector = np.array([2.0, 0.0, 1.0])

    output = mock_fss._angle_computation(sun_vector)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(63.43, abs = 0.5)
    assert output["beta_deg"] == pytest.approx(0.0, abs = 0.5)

def test_eclipse_vector(mock_fss):
    sun_vector = np.array([0.0, 0.0, 0.0])

    output = mock_fss._angle_computation(sun_vector)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(0.0)
    assert output["beta_deg"] == pytest.approx(0.0)

def test_statistically_consistent_noise(mock_fss):
    sun_vector = np.array([0.0, 0.0, 1.0])
    num_samples = 1000
    alpha_samples = []
    beta_samples = []

    for _ in range(num_samples):
        output = mock_fss._angle_computation(sun_vector)
        alpha_samples.append(output["alpha_deg"])
        beta_samples.append(output["beta_deg"])

    calculated_stdev_alpha = np.std(alpha_samples)
    calculated_stdev_beta = np.std(beta_samples)

    true_stdev_alpha = np.sqrt(mock_fss.cov_deg2[0,0])
    true_stdev_beta = np.sqrt(mock_fss.cov_deg2[1,1])

    assert calculated_stdev_alpha == pytest.approx(true_stdev_alpha, abs = 0.05)
    assert calculated_stdev_beta == pytest.approx(true_stdev_beta, abs = 0.05)

def test_fss_digitization_precision():
    """
    Verify that FSS measurements adhere to the sensor's digitization precision.
    """
    # TODO: Once quantization logic is added to sensors.py, 
    # define a mock bit-precision and assert that angles match discrete step steps.
    pytest.skip("Skipping until quantization step size details are finalized.")