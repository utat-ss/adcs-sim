import pytest
import numpy as np
from environment import aerodynamic_drag_perturbation_m_s2

test_samples = [
    # three standard cases
    {
        "velocity_m_s": np.array([10.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": 1.0,
        "drag_coeff": 2.0,
        "area_m_2": 4.0,
        "mass_kg": 2.0,
        "expected_vector": np.array([-200.0, 0.0, 0.0]),
        "expected_exception": None,
        "expected_message": None,
    },
    {
        "velocity_m_s": np.array([3.0, 4.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": 1.0,
        "drag_coeff": 2.0,
        "area_m_2": 4.0,
        "mass_kg": 2.0,
        "expected_vector": np.array([-30.0, -40.0, 0.0]),
        "expected_exception": None,
        "expected_message": None,
    },
    {
        "velocity_m_s": np.array([5.0, 5.0, 5.0]),
        "velocity_atm_m_s": np.array([1.0, 1.0, 1.0]),
        "air_kg_m3": 2.0,
        "drag_coeff": 1.5,
        "area_m_2": 3.0,
        "mass_kg": 10.0,
        "expected_vector": np.array([-12.470765814495716, -12.470765814495716, -12.470765814495716]),
        "expected_exception": None,
        "expected_message": None,
    },
    # zero relative velocity
    {
        "velocity_m_s": np.array([7500.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([7500.0, 0.0, 0.0]),
        "air_kg_m3": 1.2,
        "drag_coeff": 2.2,
        "area_m_2": 10.0,
        "mass_kg": 500.0,
        "expected_vector": np.array([0.0, 0.0, 0.0]),
        "expected_exception": None,
        "expected_message": None,
    },
    # raise the four types of errors
    # mass error
    {
        "velocity_m_s": np.array([10.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": 1.0,
        "drag_coeff": 2.0,
        "area_m_2": 4.0,
        "mass_kg": 0.0,
        "expected_vector": None,
        "expected_exception": ValueError,
        "expected_message": "Mass must be positive.",
    },
    # area error
    {
        "velocity_m_s": np.array([10.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": 1.0,
        "drag_coeff": 2.0,
        "area_m_2": -1.0,
        "mass_kg": 2.0,
        "expected_vector": None,
        "expected_exception": ValueError,
        "expected_message": "Cross-sectional area must be non-negative.",
    },
    # density error
    {
        "velocity_m_s": np.array([10.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": -1.0,
        "drag_coeff": 2.0,
        "area_m_2": 4.0,
        "mass_kg": 2.0,
        "expected_vector": None,
        "expected_exception": ValueError,
        "expected_message": "Air density must be non-negative.",
    },
    # drag coefficient error
    {
        "velocity_m_s": np.array([10.0, 0.0, 0.0]),
        "velocity_atm_m_s": np.array([0.0, 0.0, 0.0]),
        "air_kg_m3": 1.0,
        "drag_coeff": -1.0,
        "area_m_2": 4.0,
        "mass_kg": 2.0,
        "expected_vector": None,
        "expected_exception": ValueError,
        "expected_message": "Drag coefficient must be non-negative.",
    }
]

@pytest.mark.parametrize("sample", test_samples)
def test_aerodynamic_drag_perturbation_m_s2(sample):
    velocity_m_s = sample["velocity_m_s"]
    velocity_atm_m_s = sample["velocity_atm_m_s"]
    air_kg_m3 = sample["air_kg_m3"]
    drag_coeff = sample["drag_coeff"]
    area_m_2 = sample["area_m_2"]
    mass_kg = sample["mass_kg"]
    expected_vector = sample["expected_vector"]
    expected_exception = sample["expected_exception"]
    expected_message = sample["expected_message"]

    if expected_exception is not None:
        with pytest.raises(expected_exception, match=expected_message):
            aerodynamic_drag_perturbation_m_s2(
                velocity_m_s,
                velocity_atm_m_s,
                air_kg_m3,
                drag_coeff,
                area_m_2,
                mass_kg,
            )
    else:
        vector = aerodynamic_drag_perturbation_m_s2(
            velocity_m_s,
            velocity_atm_m_s,
            air_kg_m3,
            drag_coeff,
            area_m_2,
            mass_kg
        )
        np.testing.assert_array_almost_equal(vector, expected_vector)