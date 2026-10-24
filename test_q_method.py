import pytest
import numpy as np
from q-method import q_method

CASES = [
    # identity
    {"V": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), 
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
     "q_expected": np.array([0.0, 0.0, 0.0, 1.0])},

     {"V": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
      "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
      "q_expected": np.array([0.0, 0.0, 0.0, 1.0])},

    # 90 degrees about y
    {"V": np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
     "W": np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]),
     "q_expected": np.array([0.0, np.sin(np.pi/4), 0.0, np.cos(np.pi/4)])},

     {"V": np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]]),
      "W": np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
      "q_expected": np.array([0.0, np.sin(np.pi/4), 0.0, np.cos(np.pi/4)])},

    # 60 degrees about z
    {"V": np.array([[0.5, -np.sqrt(3)/2, 0.0], [np.sqrt(3)/2, 0.5, 0.0]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([0.0, 0.0, np.sin(np.pi/6), np.cos(np.pi/6)])},

    # 90 degrees about x
    {"V": np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]),
     "W": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
     "q_expected": np.array([np.sin(np.pi/4), 0.0, 0.0, np.cos(np.pi/4)])},

    {"V": np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]),
     "W": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
     "q_expected": np.array([-np.sin(np.pi/4), 0.0, 0.0, np.cos(np.pi/4)])},

    # 180 degrees about z
    {"V": np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([0.0, 0.0, 1.0, 0.0])},

    # -120 degrees about the (1,1,1) diagonal
    {"V": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
     "q_expected": np.array([0.5, 0.5, 0.5, -0.5])},

    # 180 degrees about the (1,1,1) diagonal
    {"V": np.array([[-1/3, 2/3, 2/3], [2/3, -1/3, 2/3]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([1.0, 1.0, 1.0, 0.0]) / np.sqrt(3)},

    # 90 degrees about x with a nearly parallel pair (1 degree apart)
    {"V": np.array([[0.0, 1.0, 0.0], [np.sin(np.deg2rad(1.0)), np.cos(np.deg2rad(1.0)), 0.0]]),
     "W": np.array([[0.0, 0.0, 1.0], [np.sin(np.deg2rad(1.0)), 0.0, np.cos(np.deg2rad(1.0))]]),
     "q_expected": np.array([np.sin(np.pi/4), 0.0, 0.0, np.cos(np.pi/4)])},

    # arbitrary axis q = [1,2,3,4]/sqrt(30)
    {"V": np.array([[4.0, -20.0, 22.0], [28.0, 10.0, 4.0]]) / 30.0,
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([1.0, 2.0, 3.0, 4.0]) / np.sqrt(30)},

    # inconsistent data, 10 and 20 degrees
    {"V": np.array([[np.cos(np.deg2rad(10)), -np.sin(np.deg2rad(10)), 0.0], [np.sin(np.deg2rad(20)), np.cos(np.deg2rad(20)), 0.0]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([0.0, 0.0, np.sin(np.deg2rad(15)/2), np.cos(np.deg2rad(15)/2)])},

    # inconsistent data, 5 and 25 degrees
    {"V": np.array([[np.cos(np.deg2rad(5)), -np.sin(np.deg2rad(5)), 0.0], [np.sin(np.deg2rad(25)), np.cos(np.deg2rad(25)), 0.0]]),
     "W": np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
     "q_expected": np.array([0.0, 0.0, np.sin(np.deg2rad(15)/2), np.cos(np.deg2rad(15)/2)])},

    # parallel: z
    {"V": np.array([[0.0, 0.0, 1.0]]), 
     "W": np.array([[0.0, 0.0, 1.0]]),
     "zero_idx": [0, 1]},

    # parallel: z flips to -z
    {"V": np.array([[0.0, 0.0, -1.0]]), 
     "W": np.array([[0.0, 0.0, 1.0]]),
     "zero_idx": [2, 3]},

    # parallel: x
    {"V": np.array([[1.0, 0.0, 0.0]]), 
     "W": np.array([[1.0, 0.0, 0.0]]),
     "zero_idx": [1, 2]},

    # parallel: x flips to -x
    {"V": np.array([[1.0, 0.0, 0.0]]), 
     "W": np.array([[-1.0, 0.0, 0.0]]),
     "zero_idx": [0, 3]},
]

@pytest.mark.parametrize("case", CASES)
def test_mathematical_correctness(case, tol=1e-6):
    q = q_method(case["V"].T, case["W"].T)
    assert np.isclose(np.linalg.norm(q), 1.0, atol=tol, rtol=0)

    if "q_expected" in case:
        q_expected = case["q_expected"]
        q = q / np.linalg.norm(q) * np.sign(np.dot(q, q_expected))
        assert np.allclose(q, q_expected, atol=tol, rtol=0)
    else:
        assert np.allclose(q[case["zero_idx"]], 0.0, atol=tol, rtol=0)

def test_kinematical_correctness(tol = 1e-6): # test if quaternion is unitary
    # perpendicular vectors
    V = np.array([[1.0, 0.0, 0.0]])
    W = np.array([[0.0, 1.0, 0.0]])
    q_est = q_method(V.T, W.T)

    assert np.isclose(np.linalg.norm(q_est), 1.0, atol=tol)

    # noisy perpendicular vectors
    V = np.array([[1.0, 0.05, -0.02]])
    W = np.array([[0.03, 1.0, 0.04]])
    V = V / np.linalg.norm(V)
    W = W / np.linalg.norm(W)
    q_est = q_method(V.T, W.T)

    assert np.isclose(np.linalg.norm(q_est), 1.0, atol=tol)

    # multiple vectors
    V = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    W = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    q_est = q_method(V.T, W.T)

    assert np.isclose(np.linalg.norm(q_est), 1.0, atol=tol)

def test_error_handling():
    # test for handling different dimensions between V and W
    V = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    W = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError):
        q_method(V.T, W.T)

    # test for handling non-3D vector inputs
    V = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
    W = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
    with pytest.raises(ValueError):
        q_method(V.T, W.T)

    # test for zero vectors
    V = np.array([[0.0, 0.0, 0.0]])
    W = np.array([[0.0, 0.0, 0.0]])
    with pytest.raises(ValueError):
        q_method(V.T, W.T)