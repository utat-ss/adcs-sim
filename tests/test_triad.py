import numpy as np
import pytest

from triad import triad

# 90° rotation about Z
R90_Z = np.array(
    [[0.0, -1.0, 0.0],
     [1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0]])

# Test 1: Check that the triad function recovers a known rotation matrix when given two reference vectors and their rotated counterparts.
def test_recovers_known_rotation():
    w1 = np.array([1.0, 0.0, 0.0])
    w2 = np.array([0.0, 1.0, 0.0])

    R = triad(w1, w2, R90_Z @ w1, R90_Z @ w2)

    np.testing.assert_allclose(R, R90_Z)

# Test 2: Check that the triad function returns a valid rotation matrix (orthogonal and determinant = 1).
def test_returns_valid_rotation_matrix():
    w1 = np.array([1.0, 2.0, 3.0])
    w2 = np.array([-2.0, 1.0, 4.0])

    R = triad(w1, w2, R90_Z @ w1, R90_Z @ w2)

    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12) #Orthogonalwhen R^T @ R = I
    assert np.linalg.det(R) == pytest.approx(1.0) # determinant of a rotation matrix is 1

# Test 3: Check that the triad function raises a ValueError when given parallel reference vectors.
def test_rejects_parallel_reference_vectors():
    w1 = np.array([1.0, 0.0, 0.0])
    #w2 = np.array([0.0, 1.0, 0.0])
    v1 = np.array([0.0, 1.0, 0.0])
    v2 = np.array([0.0, 0.0, 1.0])

    with pytest.raises(ValueError):
        triad(w1, 2.0 * w1, v1, v2)  # parallel

    with pytest.raises(ValueError):
        triad(w1, -2.0 * w1, v1, v2)  # anti-parallel

# Test 4: Check that the triad function raises a ValueError when any input vector is the zero vector.
def test_rejects_zero_vectors():
    w1 = np.array([1.0, 0.0, 0.0])
    w2 = np.array([0.0, 1.0, 0.0])
    v1 = np.array([0.0, 1.0, 0.0])
    v2 = np.array([0.0, 0.0, 1.0])
    zero = np.array([0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        triad(zero, w2, v1, v2)  # zero reference vector
    with pytest.raises(ValueError):
        triad(w1, zero, v1, v2)  # zero second reference vector
    with pytest.raises(ValueError):
        triad(w1, w2, zero, v2)  # zero rotated vector
    with pytest.raises(ValueError):
        triad(w1, w2, v1, zero)  # zero second rotated vector
        
# Test 5: Check that the triad function raises a ValueError when any input vector is not 3-dimensional.
def test_rejects_vectors_not_in_3d():
    w1 = np.array([1.0, 0.0, 0.0])
    w2 = np.array([0.0, 1.0, 0.0])
    v1 = np.array([0.0, 1.0, 0.0])
    v2 = np.array([0.0, 0.0, 1.0])
    two_d = np.array([1.0, 0.0])
    four_d = np.array([1.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        triad(two_d, w2, v1, v2)  # 2D reference vector
    with pytest.raises(ValueError):
        triad(w1, four_d, v1, v2)  # 4D second reference vector
    with pytest.raises(ValueError):
        triad(w1, w2, two_d, v2)  # 2D rotated vector
    with pytest.raises(ValueError):
        triad(w1, w2, v1, four_d)  # 4D second rotated vector