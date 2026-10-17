import pytest
import numpy as np
from hardware.adcs import ADCS

# command: python -m uv run pytest tests/test_eclipse_detection.py -v 

# pytest fixtures
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

    adcs = ADCS("test", np.eye(3)) # np.eye(3) is a placeholder for the moment of inertia matrix

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

# Tests for eval_fine_eclipse method of ADCS class
@pytest.mark.parametrize(
    "sun_vec, body_vec",
    [
        (np.array([1.0, 0.0]), np.array([1.0, 0.0, 0.0])),         # 2D sun vector
        (np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0])), # 4D body vector
        (np.array([[1.0, 0.0, 0.0]]), np.array([1.0, 0.0, 0.0])),   # 2D matrix shape (1, 3)
        ([], [1.0, 0.0, 0.0]),                                       # Empty input
    ],
    ids=["2d-sun", "4d-body", "matrix-sun", "empty-sun"]
)
def test_eval_eclipse_fine_invalid_shape(sun_vec, body_vec): #input validation
    adcs = ADCS("test", np.eye(3))
    with pytest.raises(ValueError, match="must both be 3D vectors"):
        adcs.eval_eclipse_fine(sun_vec, body_vec, body_radius=1e6)


@pytest.mark.parametrize(
    "earth_vector_for_sun_visible",
    [
        pytest.param(
            np.array([3.84e7, 3.84e7, 0]),
            id="earth-offset-quadrant-I-xyplane",
        ),
        pytest.param(
            np.array([0, 3.84e7, 0]),
            id="earth-offset-y-axis",
        ),
        pytest.param(
            np.array([0, 0, 3.84e7]),
            id="earth-offset-z-axis",
        ),
        pytest.param(
            np.array([-3.84e7, 3.84e7, 0]),
            id="earth-offset-quadrant-II-xyplane",
        ),
        pytest.param(
            np.array([3.84e7, -3.84e7, 0]),
            id="earth-offset-quadrant-IV-xyplane",
        ),
    ],
)
def test_sun_eval_eclipse_fine_fully_visible(earth_vector_for_sun_visible):
    adcs = ADCS("test", np.eye(3)) 

    AU_m = 149.6e9  # Astronomical Unit in meters

    # below in metres
    result = adcs.eval_eclipse_fine(
        np.array([AU_m, 0, 0]), # sun vector, Distance of satelite from sun: 1 AU_m
        earth_vector_for_sun_visible, # earth vector 
        1e6
    )

    assert result == pytest.approx(1.0) # Sun 100% visible

@pytest.mark.parametrize(
    "sun_vector_notVisible, earth_vector_notVisible",
    [
        pytest.param(
            np.array([149.6e9, 0, 0]),
            np.array([2.0e8, 0, 0]),
            id="eclipse-along-x-axis",
        ),
        pytest.param(
            np.array([0, 149.6e9, 0]),
            np.array([0, 2.0e8, 0]),
            id="eclipse-along-y-axis",
        ),
        pytest.param(
            np.array([0, 0, 149.6e9]),
            np.array([0, 0, 2.0e8]),
            id="eclipse-along-z-axis",
        ),
        pytest.param(
            np.array([149.6e9, 149.6e9, 149.6e9]),
            np.array([2.0e8, 2.0e8, 2.0e8]),
            id="eclipse-diagonal-xyz",
        ),
    ],
)
def test_sun_eval_eclipse_fine_not_visible(sun_vector_notVisible, earth_vector_notVisible):
    adcs = ADCS("test", np.eye(3))

    result = adcs.eval_eclipse_fine(
        sun_vector_notVisible,
        earth_vector_notVisible,
        1e6,
    )

    assert result == pytest.approx(0.0)

def test_sun_eval_eclipse_fine_half_visibility():
    """Body is centered on the Sun but smaller in angular diameter."""
    AU_m = 149.6e9  # Astronomical Unit in meters
    SUN_RADIUS_m = 6.9634e8  # Sun radius in meters

    adcs = ADCS("test", np.eye(3))
    sun_vector = np.array([AU_m, 0.0, 0.0])
    body_vector = np.array([AU_m * 0.5, 0.0, 0.0])

    # angular radius sun ≈ SUN_RADIUS / AU_m
    # Choose body_radius so body angular diameter is half the sun's angular diameter
    body_radius = (SUN_RADIUS_m / 2.0) * 0.5

    result = adcs.eval_eclipse_fine(sun_vector, body_vector, body_radius)
    
    # Expected visibility is approx 1 - (theta_body / theta_sun) ≈ 0.5 (Sun 50% visible)
    assert 0.0 < result < 1.0
    assert result == pytest.approx(0.5, rel=1e-2)

def test_partial_eclipse_less_than_half_visible():
    """Body overlaps center of sun disk but does not fully engulf it.
    Result should be less than 50% visible."""
    adcs = ADCS("test", np.eye(3))
    AU_m = 149.6e9 
    sun_vector = np.array([AU_m, 0.0, 0.0])
    
    # Slight angular offset on y-axis
    dist_body = 1.0e8
    # Angular radius of Sun ≈ 6.9634e8 / 1.496e11 ≈ 4.65e-3 rad
    # Place body edge just past the center of the Sun
    offset_y = dist_body * 1.0e-3
    body_vector = np.array([dist_body, offset_y, 0.0])
    body_radius = dist_body * 4.0e-3

    result = adcs.eval_eclipse_fine(sun_vector, body_vector, body_radius)
    
    # Partial eclipse covering > 50% of the diameter
    assert 0.0 < result < 0.5

def test_partial_eclipse_more_than_half_visible():
    """Body overlaps outer edge of sun disk without covering the center.
    Result should be more than 50% visible."""
    adcs = ADCS("test", np.eye(3))
    AU_m = 149.6e9
    sun_vector = np.array([AU_m, 0.0, 0.0])

    dist_body = 1.0e8
    # Angular radius of Sun ≈ 4.65e-3 rad
    # Place body center at 5.5 mrad with radius 2.0 mrad:
    # Inner edge reaches 5.5 - 2.0 = 3.5 mrad (inside the 4.65 mrad disk, but outside center)
    offset_y = dist_body * 5.5e-3
    body_vector = np.array([dist_body, offset_y, 0.0])
    body_radius = dist_body * 2.0e-3

    result = adcs.eval_eclipse_fine(sun_vector, body_vector, body_radius)

    assert 0.5 < result < 1.0