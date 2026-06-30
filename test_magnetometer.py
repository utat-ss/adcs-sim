'''
Develop units tests for the MTM sensor model.
'''

import numpy as np
import pytest

from hardware.sensors.sensors import VirtualMTM

def test_accurate_measurement_in_sensor_frame():
    mtm = VirtualMTM()

    mtm.lims_ut = [-800.0, 800.0]
    mtm.bias_ut = 0.0
    mtm.soft_iron = np.array([[1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]])

    # test measured field and set expected field
    true_field = np.array([23.7, -15.2, 42.1])
    measured_field = mtm.measure(true_field)

    expected_field = true_field
    
    # used assert_array_almost_equal in case of floating point precision issues
    np.testing.assert_array_almost_equal(measured_field, expected_field, decimal=5)

def test_sensor_saturation_limit():
    mtm = VirtualMTM()

    mtm.lims_ut = [-800.0, 800.0]
    mtm.bias_ut = 0.0
    mtm.soft_iron = np.array([[1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]])
    
    
    # test measured field and set expected field
    true_field = np.array([822.5, -880.9, 15.0])
    measured_field = mtm.measure(true_field)

    expected_field = np.array([800.0, -800.0, 15.0])

    # used assert_array_almost_equal in case of floating point precision issues
    np.testing.assert_array_almost_equal(measured_field, expected_field, decimal=5)

def test_biases_and_distortions():

    # define tests
    test_cases = [
        # three cases with various soft_iron matrices
        # soft_iron is identity matirx
        {
            "bias": 1.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, -20.0, 30.0])
        },
        
        {
            "bias": 1.0,
            "soft_iron": np.array([[1.5, 0.0, 0.0],
                                   [0.0, 0.8, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, -20.0, 30.0])
        },

        {
            "bias": 1.0,
            "soft_iron": np.array([[1.5, 0.5, 0.0],
                                   [0.0, 0.8, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, -20.0, 30.0])
        },

        # three cases with various true_fields
        # test zero field to check for ZeroDivisionError
        {
            "bias": 1.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([0.0, 0.0, 0.0])
        },

        # test numerial overflow with large values
        {
            "bias": 1.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([1e12, -1e12, 1e12])
        },

        # check calculations with exact boundaries
        {
            "bias": 1.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([-800.0, 800.0, 800.0])
        },

        # three cases with various biases
        # bias cancels true_field
        {
            "bias": -10.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, 10.0, 10.0])
        },

        # bias is zero
        {
            "bias": 0.0,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, -20.0, 30.0])
        },

        # bias is over the limit of +/-800
        {
            "bias": 1000,
            "soft_iron": np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]),
            "true_field": np.array([10.0, -20.0, 30.0])
        }
    ]

    # test measured field and set expected field
    for case in test_cases:
        mtm = VirtualMTM()
        mtm.lims_ut = (-800.0, 800.0)
        mtm.bias_ut = case["bias"]
        mtm.soft_iron = case["soft_iron"]

        measured_field = mtm.measure(case["true_field"])

        expected_field = (case["true_field"] @ mtm.soft_iron) + np.full(3, mtm.bias_ut)
        expected_field = np.clip(expected_field, -800.0, 800.0)
        np.testing.assert_array_almost_equal(measured_field, expected_field, decimal=5)

def test_noise_statistical_consistency():
    mtm = VirtualMTM()

    mtm.lims_ut = [-800.0, 800.0]
    mtm.bias_ut = 0.0
    mtm.soft_iron = np.array([[1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]])
    
    std = 0.015

    true_field = np.array([0.0, 0.0, 0.0])

    # set the number of test cases to run
    cases = 1000

    # initlize counters for the number of measurements within 1, 2, and 3 standard deviations
    within_1_std = 0
    within_2_std = 0
    within_3_std = 0 

    # running the test cases
    for _ in range (cases):
        measured_field = mtm.measure(true_field)
        error = abs(measured_field[0] - true_field[0])

        if error < 1 * std:
            within_1_std += 1
        if error < 2 * std:
            within_2_std += 1
        if error < 3 * std:
            within_3_std += 1
    
    within_1_std_percentage = within_1_std / cases
    within_2_std_percentage = within_2_std / cases
    within_3_std_percentage = within_3_std / cases

    # used assert to check if the percentages are within the expected ranges +/- 5%
    assert within_1_std_percentage == pytest.approx(0.68, abs=0.05)
    assert within_2_std_percentage == pytest.approx(0.95, abs=0.05)
    assert within_3_std_percentage == pytest.approx(0.997, abs=0.05)


def test_digitization_precision():
    mtm = VirtualMTM()
    # not sure what the precision is, so this is a placeholder
    precision = 0.1
    mtm.precision = precision
    mtm.bias_ut = 0.0
    mtm.soft_iron = np.array([[1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]])

    # Test inputs that should round down
    assert mtm.measure(np.array([1.11, 0.0, 0.0]))[0] == 1.1
    assert mtm.measure(np.array([1.24, 0.0, 0.0]))[0] == 1.2

    # Test inputs that should round up
    assert mtm.measure(np.array([1.26, 0.0, 0.0]))[0] == 1.3
    assert mtm.measure(np.array([1.39, 0.0, 0.0]))[0] == 1.4
    assert mtm.measure(np.array([1.45, 0.0, 0.0]))[0] == 1.5

    # Test inputs that does not require rounding
    assert mtm.measure(np.array([1.60, 0.0, 0.0]))[0] == 1.6