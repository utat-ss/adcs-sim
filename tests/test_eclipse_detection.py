import pytest
import numpy as np
from hardware.adcs import ADCS

# Tests for _find_config_path method of ADCS class
def test_import_adcs():
    # Test if ADCS class can be imported successfully
    print("Testing import of ADCS class...")
    assert ADCS is not None
    
"""
def test_find_config_path_existing_model(tmp_path, monkeypatch):
    icd_file = tmp_path / "test_model.json" # temporary file to simulate the ICD file
    icd_file.write_text("{}")

    monkeypatch.setattr(
        "your_module.SENSOR_ICD_DIR",
        tmp_path,
    )

    adcs = ADCS("test", np.eye(3))

    result = adcs._find_config_path("Test-Model")
    assert result == icd_file


def test_find_config_path_not_existing_model():
    adcs  = ADCS("test", np.eye(3))

    with pytest.raises(FileNotFoundError):
        adcs._find_config_path("does-not-exist-model")
"""
