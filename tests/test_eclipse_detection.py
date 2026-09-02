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
def test_validate_moi_non_symetric():
    adcs = ADCS("test", np.eye(3))

    non_symmetric_moi = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    result, error_code = adcs._validate_moi(non_symmetric_moi)

    assert result is False
    assert error_code == -1
