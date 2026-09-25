from pathlib import Path

import pytest
import numpy as np
from hardware.sensors.sensors import VirtualFSS


FSS_CFG = Path("hardware/sensors/icd/fss/fss_15.json")

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])
IDENTITY_ROTMAT = np.identity(3)

def test_loads_config():
    """
    Verify that FSS parameters taken from the config file used are correct.
    """
    fss = VirtualFSS(FSS_CFG)

    assert fss.model == "FSS-15"
    assert fss.fov_deg == pytest.approx(60.0)
    assert fss.rate_hz == pytest.approx(8.0)
    assert fss.cov_deg2.shape == (2, 2)
    assert np.allclose(fss.cov_deg2, np.array([[0.2, 0.0], [0.0, 0.2]]))


def test_sun_vector_fov():
    """
    A sun vector within the FOV (60 deg) should be detected and the angles should be calculated correctly.
    """
    fss = VirtualFSS(FSS_CFG)

    fss.cov_deg2 = np.zeros((2, 2), dtype=float) # No noise for this test
    sun_vector = np.array([1.0, 0.5, 0.0]) # alpha = 26.565 deg, beta = 0.0 deg

    output = fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == True
    assert output["alpha_deg"] == pytest.approx(26.565, abs=0.01)
    assert output["beta_deg"] == pytest.approx(0.0)


def test_sun_vector_out_of_fov():
    """
    A sun vector outside the FOV (60 deg) should not be detected and the angles should be set to 0.0.
    """
    fss = VirtualFSS(FSS_CFG)

    fss.cov_deg2 = np.zeros((2, 2), dtype=float) # No noise for this test
    sun_vector = np.array([1.0, 2.0, 0.0]) # alpha = 63.43 deg, beta = 0.0 deg

    output = fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(0.0)
    assert output["beta_deg"] == pytest.approx(0.0)


def test_eclipse_vector():
    """
    A sun vector that is geometrically in the FOV but is eclipsed should not be detected and the angles should be set to 0.0.
    """
    fss = VirtualFSS(FSS_CFG)

    sun_vector = np.array([1.0, 0.0, 0.0])

    output = fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=0.5, offset_rotmat=IDENTITY_ROTMAT)

    assert output["sun_present"] == False
    assert output["alpha_deg"] == pytest.approx(0.0)
    assert output["beta_deg"] == pytest.approx(0.0)

def test_eclipse_vector_never_reports_sun_even_when_geometrically_in_fov():
    """
    Go through a range of sun visibility values and verify that the FSS never reports the sun as present when the sun is eclipsed.
    """
    fss = VirtualFSS(FSS_CFG)

    sun_vector = np.array([1.0, 0.0, 0.0])
 
    for visibility in [0.0, 0.1, 0.5, 0.89]:
        output = fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=visibility, offset_rotmat=IDENTITY_ROTMAT)
        assert output["sun_present"] == False


def test_statistically_consistent_noise():
    """
    Verify that the FSS introduces noise that is statistically consistent with its covariance matrix.
    """
    fss = VirtualFSS(FSS_CFG)

    sun_vector = np.array([1.0, 0.0, 0.0]) # Sun vector along the +X axis, which is in the center of the FOV

    num_samples = 1000
    alpha_samples = []
    beta_samples = []

    for _ in range(num_samples):
        output = fss.measure(IDENTITY_QUAT, sun_vector, sun_visibility=1.0, offset_rotmat=IDENTITY_ROTMAT)
        alpha_samples.append(output["alpha_deg"])
        beta_samples.append(output["beta_deg"])

    # Calculate the standard deviation and compare it to what is expected from cov_deg2
    calculated_stdev_alpha = np.std(alpha_samples)
    calculated_stdev_beta = np.std(beta_samples)

    true_stdev_alpha = np.sqrt(fss.cov_deg2[0,0])
    true_stdev_beta = np.sqrt(fss.cov_deg2[1,1])

    assert calculated_stdev_alpha == pytest.approx(true_stdev_alpha, abs = 0.05)
    assert calculated_stdev_beta == pytest.approx(true_stdev_beta, abs = 0.05)

    # Check that approximately 68% of the samples fall within 1 standard deviation of the mean
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