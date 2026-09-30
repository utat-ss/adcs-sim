# ------------------------------------------------------------------------------
#  File: test_quest.py
# Author: Mahat Joshi
# GitHub: MahatPhy
# Date: September 29, 2026

# Description: Unit tests for normalize_quat and QUEST_algorithm.py

# Style Note: The Dashes are something I carried forward from my time 
# programming with Cpp as a visual ruler for the line length. I understand
# that it may not be best practice for Python Code. But I don't think it goes
# against the PEP 8 standard. The Black formatter would also do this. But,
# eh... habit.
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Importing the modules we're testing
# ------------------------------------------------------------------------------


import numpy as np
import pytest

from utils.quaternion_math import normalize_quat
from state_estimation.QUEST_algorithm import QUEST


# ------------------------------------------------------------------------------
# Testing the normalize_quat
# ------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "invalid_input",
    [
        [1.0, 2.0, 3.0],  # Too few elements (shape 3,)
        [1.0, 2.0, 3.0, 4.0, 5.0],  # Too many elements (shape 5,)
        [[1.0, 2.0], [3.0, 4.0]],  # 2D array (shape 2, 2)
        [],  # Empty array
    ],
)
def test_normalize_quat_invalid_shape(invalid_input):
    """Ensure non-(4,) shapes raise a ValueError."""
    with pytest.raises(ValueError, match="Quaternion must have shape"):
        normalize_quat(invalid_input)


def test_normalize_quat_zero_norm():
    """Ensure a zero-norm quaternion raises a ValueError."""
    with pytest.raises(ValueError, match="Quaternion has zero norm."):
        normalize_quat([0.0, 0.0, 0.0, 0.0])


# ------------------------------------------------------------------------------
# REMOVE BEFORE FLIGHT: (COMMENT) 
# Are we accounting for values exceptionally
# close to 0?
# Because I don't know how much in terms of noise do the sensors give
# ------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "near_zero_vals",
    [
        [1.0e-15, 0.0, 0.0, 0.0],
        [1.0e-18, 1.0e-17, 0.0, 0.0],
        # Successfully normalizes because no truncation threshold is present
        # Can be a problem if sensors are doing erronious stuff ??? Maybe
        [0.0, 0.0, 1e-20, 1e-20],
        # Will definitely fail (as of 29 September) because no near 0 handling
    ],
)
def test_normalize_near_zero_handling(near_zero_vals):
    """Test handling of quaternion norms near zero."""
    result = normalize_quat(near_zero_vals)
    assert np.isclose(np.linalg.norm(result), 1.0)


# ------------------------------------------------------------------------------
# Testing QUEST
# ------------------------------------------------------------------------------


@pytest.mark.parametrize("Identity", [
    np.eye(3),  
])
def test_quest_nominal_alignment(Identity):
    """Test whether identity Quarternion is returned"""
    Identity = np.array(Identity)
    result = QUEST(Identity,Identity)
    expected = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(result,expected,atol=1e-16)


# ------------------------------------------------------------------------------
# REMOVE BEFORE FLIGHT: (COMMENT) 
# So, we don't consider cases where insufficient arguments are given to QUEST
# right?
# ------------------------------------------------------------------------------


def test_insufficient_arguments():
    """Test whether the QUEST function can handle less than 2 args"""
    Val = np.array([3.0,4.7,1.3395])
    with pytest.raises(TypeError):
        QUEST(Val)



