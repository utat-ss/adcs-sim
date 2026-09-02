import pytest
import numpy as np
from hardware.adcs import ADCS

#pytest fixtures
@pytest.fixture
def valid_moi():
    """Simple physically valid moment of inertia matrix."""
    return np.diag([2.0, 3.0, 4.0]) 

@pytest.fixture
def adcs(valid_moi):
    return ADCS("test-adcs", valid_moi) # Create an ADCS instance with the valid moment of inertia


# Tests for Initialization
def test_import_adcs():
    # Test if ADCS class can be imported successfully
    assert ADCS is not None

def test_adcs_initialization(adcs, valid_moi):
    # Test if ADCS instance is created successfully
    assert adcs.id == "test-adcs"
    assert np.array_equal(adcs.moi, valid_moi)

# Tests for _find_config_path method of ADCS class
def test_find_config_path_existing_model(tmp_path, monkeypatch):
    icd_file = tmp_path / "test_model.json" # temporary file to simulate the ICD file
    icd_file.write_text("{}")

    monkeypatch.setattr("hardware.adcs.SENSOR_ICD_DIR", tmp_path)

    adcs = ADCS("test", np.eye(3))

    result = adcs._find_config_path("Test-Model")
    assert result == icd_file


def test_find_config_path_not_existing_model():
    adcs  = ADCS("test", np.eye(3))

    with pytest.raises(FileNotFoundError):
        adcs._find_config_path("does-not-exist-model")


# Tests for _validate_moi method of ADCS class
def test_validate_moi_valid(adcs):
    valid_moi = np.diag([2.0, 3.0, 4.0])
    result, error_code = adcs._validate_moi(valid_moi)

    assert result is True
    assert error_code is 0

def test_validate_moi_non_symetric():
    adcs = ADCS("test", np.eye(3))

    non_symmetric_moi = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    result, error_code = adcs._validate_moi(non_symmetric_moi)

    assert result is False
    assert error_code == -1

@pytest.mark.parametrize(
    "moi_negative_eigenvalue",
    [
        pytest.param(
            np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]]),
            id="negative-eigenvalue-1",
        ),
        pytest.param(
            np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]]),
            id="negative-eigenvalue-2",
        ),
        pytest.param(
            np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
            id="negative-eigenvalue-3",
        ),
    ],
)
def test_validate_moi_negative_eigenvalue(moi_negative_eigenvalue):
    adcs = ADCS("test", np.eye(3))

    result, error_code = adcs._validate_moi(moi_negative_eigenvalue)

    assert result is False
    assert error_code == -2

@pytest.mark.parametrize(
    "moi_triangle_inequality",
    [
        pytest.param(
            np.array([[1, 0, 0], [0, 2, 0], [0, 0, 4]]),
            id="triangle-inequality-1",
        ),
        pytest.param(
            np.array([[1, 0, 0], [0, 4, 0], [0, 0, 1]]),
            id="triangle-inequality-2",
        ),
        pytest.param(
            np.array([[4, 0, 0], [0, 1, 0], [0, 0, 2]]),
            id="triangle-inequality-3",
        ),
    ],
)
def test_validate_moi_triangle_inequality(moi_triangle_inequality):
    adcs = ADCS("test", np.eye(3))

    result, error_code = adcs._validate_moi(moi_triangle_inequality)

    assert result is False
    assert error_code == -3
