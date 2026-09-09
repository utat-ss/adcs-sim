import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest
import numpy as np
from hardware.sensors.sensors import VirtualFSS

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])
IDENTITY_ROTMAT = np.identity(3)

@pytest.fixture

def mock_fss():
    fss = VirtualFSS.__new__(VirtualFSS)

    fss.model = "Test FSS"
    fss.fov_deg = 45.0
    fss.rate_hz = 10.0
    fss.cov_deg2 = np.zeros((2,2), dtype = float)

    return fss


def test_sun_vector_fov(mock_fss):
    sun_vector = np.array([1.0, 1.0, 0.0])

    output = mock_fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == True
    assert output["alpha_deg"] == pytest.approx(45.0)
    assert output["beta_deg"] == pytest.approx(0.0)


def test_sun_vector_out_of_fov(mock_fss):
    sun_vector = np.array([1.0, 2.0, 0.0]) # alpha = 63.43 deg, beta = 0.0 deg

    output = mock_fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(0.0)
    assert output["beta_deg"] == pytest.approx(0.0)


def test_eclipse_vector(mock_fss):
    sun_vector = np.array([1.0, 0.0, 0.0])

    output = mock_fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=0.5, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(0.0)
    assert output["beta_deg"] == pytest.approx(0.0)

def test_eclipse_vector_never_reports_sun_even_when_geometrically_in_fov(mock_fss):
    sun_vector = np.array([1.0, 0.0, 0.0])
 
    for visibility in [0.0, 0.1, 0.5, 0.89]:
        output = mock_fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=visibility, offset_rotmat=IDENTITY_ROTMAT)
        assert output["sun_present"] == False


def test_statistically_consistent_noise(mock_fss):
    mock_fss.cov_deg2 = np.array([[0.01, 0.0], [0.0, 0.01]])
    sun_vector = np.array([1.0, 0.0, 0.0])

    num_samples = 1000
    alpha_samples = []
    beta_samples = []

    for _ in range(num_samples):
        output = mock_fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)
        alpha_samples.append(output["alpha_deg"])
        beta_samples.append(output["beta_deg"])

    calculated_stdev_alpha = np.std(alpha_samples)
    calculated_stdev_beta = np.std(beta_samples)

    true_stdev_alpha = np.sqrt(mock_fss.cov_deg2[0,0])
    true_stdev_beta = np.sqrt(mock_fss.cov_deg2[1,1])

    assert calculated_stdev_alpha == pytest.approx(true_stdev_alpha, abs = 0.05)
    assert calculated_stdev_beta == pytest.approx(true_stdev_beta, abs = 0.05)

    within_1_std_alpha = np.mean(np.abs(alpha_samples) <= true_stdev_alpha)
    within_1_std_beta = np.mean(np.abs(beta_samples) <= true_stdev_beta)
    assert within_1_std_alpha == pytest.approx(0.68, abs=0.05)
    assert within_1_std_beta == pytest.approx(0.68, abs=0.05)

def test_fss_digitization_precision():
    """
    Verify that FSS measurements adhere to the sensor's digitization precision.
    """
    # TODO: Once quantization logic is added to sensors.py, 
    # define a mock bit-precision and assert that angles match discrete step steps.
    pytest.skip("Skipping until quantization step size details are finalized.")