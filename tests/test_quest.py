"""Tests and assumptions for the QUEST attitude estimator.

Input contract:
- V and W must be NumPy arrays with matching shape (3, m), where m >= 2. 
  The algorithm given doesn't check for this, so it is implied hopefully.
- Every observation vector must be finite and non-zero.
- Each input set must contain at least two linearly independent vectors;
  parallel or antiparallel-only observations do not determine attitude.
- Now another thing that I'm confused about is how to cross test if this will
  do the exact same thing as davenport's Q method or SVD aided EKF. Now, I would
  write those functions myself, but then how do I test the validity of those functions.
- Array-like lists are not accepted. Invalid input types raise TypeError;
  invalid shapes or observation data raise ValueError.

Algorithm and convention:
- QUEST returns a scalar-first quaternion [q0, q1, q2, q3].
- Tests interpret that quaternion using the passive direction cosine matrix
  convention and construct body observations as W = A @ V.
- The implementation has no explicit observation weights or normalization.
  Recovery tests therefore use unit vectors; non-unit observations are not
  rejected and their magnitudes affect the solution.

Coverage and known limits:
- Recovery cases are noise-free and use deterministic inputs or locally
  seeded random generators. Noisy-data accuracy is not tested here.
- The quaternion result is expected to be finite and unit norm.
- Exact 180-degree recovery is currently expected to fail; the strict xfail
  captures the known numerical limitation. A separate degenerate-geometry
  test checks only that this case returns a finite, unit quaternion, not that
  the recovered attitude is correct.
"""

import pytest
import numpy as np
from state_estimation.QUEST_algorithm import QUEST
from utils.quaternion_math import (
    normalize_quat,
    quat_from_rotvec,
)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Convert a scalar-first QUEST quaternion [w, x, y, z] to its passive DCM."""
    w, x, y, z = normalize_quat(q)
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y + w * z), 2 * (x * z - w * y)],
            [2 * (x * y - w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z + w * x)],
            [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )  # wild goose array


def get_quat_angle_deg(q: np.ndarray) -> float:
    """Extract the rotation angle in degrees from a scalar-first quaternion [w, x, y, z]."""
    w, _, _, _ = normalize_quat(q)
    w_clipped = np.clip(np.abs(w), 0.0, 1.0)
    return 2.0 * np.degrees(np.arccos(w_clipped))


def scalar_first_quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Convert a rotation vector to QUEST's scalar-first quaternion convention."""
    q = quat_from_rotvec(rotvec)
    return q[[3, 0, 1, 2]]


@pytest.mark.parametrize("m", [2, 3, 5, 20])
def test_random_round_trip(m):
    """Test 1: Random round trip for various observation counts m."""
    rng = np.random.default_rng(100 + m)
    # Can we recover a_true after performing a rotation
    rotvec_true = rng.standard_normal(3)
    q_true = scalar_first_quat_from_rotvec(rotvec_true)
    a_true = quat_to_rotmat(q_true)

    # Generate random reference vectors V and corresponding body vectors W
    v = rng.standard_normal((3, m))
    v /= np.linalg.norm(v, axis=0, keepdims=True)  # getting norms of the rvg (vector)
    w = a_true @ v  # multiplying the quarternion matrix with the random vectors

    q_rec = QUEST(v, w)
    a_rec = quat_to_rotmat(q_rec)

    np.testing.assert_allclose(
        a_rec, a_true, atol=1e-9
    )  # objects equal upto desired tolerance


def test_identity_rotation():
    """Test 2: V = W gives identity rotation matrix A almost equal to I."""
    rng = np.random.default_rng(300)
    m = 5
    v = rng.standard_normal((3, m))  # 5 vectors
    v /= np.linalg.norm(v, axis=0, keepdims=True)  # normalized
    w = v.copy()  # Now, I may have to check whether this does a deep/shallow copy

    q_rec = QUEST(v, w)  # QUEST upon itself should give a quarternion
    a_rec = quat_to_rotmat(q_rec)  # corresponding to the identity vector when rotated

    np.testing.assert_allclose(a_rec, np.eye(3), atol=1e-9)


@pytest.mark.parametrize(
    "axis, angle_deg",
    [
        (np.array([1.0, 0.0, 0.0]), 90.0),  # 90 degs about x
        (np.array([0.0, 1.0, 0.0]), 90.0),  # 90 degs about y
        (np.array([0.0, 0.0, 1.0]), 90.0),  # 90 degs about z
        (
            np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0),
            30.0,
        ),  # 30 degs about [1,1,1]/sqrt(3)
    ],
)
def test_known_axis_angle_cases(axis, angle_deg):
    """Test 3: Known axis-angle cases checked against expected quaternions."""
    rotvec = axis * np.radians(angle_deg)
    q_expected = scalar_first_quat_from_rotvec(rotvec)
    a_true = quat_to_rotmat(q_expected)

    rng = np.random.default_rng(42)
    v = rng.standard_normal((3, 4))
    v /= np.linalg.norm(v, axis=0, keepdims=True)
    w = a_true @ v

    q_rec = QUEST(v, w)
    a_rec = quat_to_rotmat(q_rec)
    # still checking recovery cases against known angles which is a property of quest
    # I don't really know where to find better valiadation conditions for this particular property of QUEST
    # because this does the effective same job as test 1
    np.testing.assert_allclose(a_rec, a_true, atol=1e-9)


def test_minimal_case():
    """Test 4: Minimal case with m = 2 non-parallel vectors recovers A exactly."""
    # Again to test the mathematical accuracy of this thing.
    v = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])

    rotvec = np.array([0.0, 0.0, np.radians(45.0)])
    q_true = scalar_first_quat_from_rotvec(rotvec)
    a_true = quat_to_rotmat(q_true)
    w = a_true @ v

    q_rec = QUEST(v, w)
    a_rec = quat_to_rotmat(q_rec)

    np.testing.assert_allclose(a_rec, a_true, atol=1e-9)


@pytest.mark.parametrize("angle_deg", [150.0, 170.0])
def test_angle_sweep(angle_deg):
    """Test 5: Check recovery for rotations approaching the known 180-degree limit."""
    axis = np.array([0.0, 1.0, 0.0])
    rotvec = axis * np.radians(angle_deg)
    q_true = scalar_first_quat_from_rotvec(rotvec)
    a_true = quat_to_rotmat(q_true)

    rng = np.random.default_rng(200 + int(angle_deg))
    v = rng.standard_normal((3, 5))
    v /= np.linalg.norm(v, axis=0, keepdims=True)
    w = a_true @ v

    q_rec = QUEST(v, w)
    angle_rec = get_quat_angle_deg(q_rec)

    np.testing.assert_allclose(angle_rec, angle_deg, atol=1e-5)


@pytest.mark.xfail(
    reason="QUEST is numerically unstable at exactly 180 degrees.",
    strict=True,
)
def test_180_degree_rotation():
    """Test 5 (Edge Case): Exact 180 degrees rotation recovery is currently unstable."""
    axis = np.array([1.0, 0.0, 0.0])
    rotvec = axis * np.radians(180.0)
    q_true = scalar_first_quat_from_rotvec(rotvec)
    a_true = quat_to_rotmat(q_true)

    v = np.eye(3)
    w = a_true @ v

    q_rec = QUEST(v, w)
    a_rec = quat_to_rotmat(q_rec)

    np.testing.assert_allclose(a_rec, a_true, atol=1e-9)


def test_quest_quaternion_is_unit_norm():
    """Test 6: QUEST returns a finite, unit scalar-first quaternion."""
    rotvec = np.array([0.2, -0.4, 0.7])
    q_true = scalar_first_quat_from_rotvec(rotvec)
    a_true = quat_to_rotmat(q_true)
    v = np.eye(3)
    w = a_true @ v

    q_rec = QUEST(v, w)

    assert q_rec.shape == (4,)
    assert np.all(np.isfinite(q_rec))
    np.testing.assert_allclose(np.linalg.norm(q_rec), 1.0, atol=1e-12)


def test_rejects_malformed_inputs():
    """Test 7: Testing whether QUEST Algorithm Rejects Malformed Inputs"""
    cases = [
        (np.ones(3), np.ones(3), ValueError, r"shape \(3, m\)"),
        (np.ones((2, 3)), np.ones((2, 3)), ValueError, r"shape \(3, m\)"),
        (np.ones((4, 3)), np.ones((4, 3)), ValueError, r"shape \(3, m\)"),
        (np.ones((3, 2)), np.ones((3, 3)), ValueError, "same shape"),
        (None, np.ones((3, 2)), TypeError, "NumPy arrays"),
        (np.ones((3, 2)), "vectors", TypeError, "NumPy arrays"),
    ]
    for V, W, exception, message in cases:
        with pytest.raises(exception, match=message):
            QUEST(V, W)


def test_rejects_too_few_observations():
    """Test 8: Testing Whether QUEST Rejects vectors less than 2"""
    for observation_count in (0, 1):
        V = np.ones((3, observation_count))
        W = np.ones((3, observation_count))

        with pytest.raises(ValueError, match="At least two observation vectors"):
            QUEST(V, W)


def test_rejects_nonfinite_or_zero_length_vectors():
    """Test 9: Testing if QUEST rejects vectors with dim 0 or nan"""
    for side, invalid_value, message in [
        ("V", np.nan, "finite"),
        ("W", np.nan, "finite"),
        ("V", np.inf, "finite"),
        ("W", np.inf, "finite"),
        ("V", 0.0, "non-zero length"),
        ("W", 0.0, "non-zero length"),
    ]:
        V = np.eye(3)[:, :2].copy()
        W = V.copy()
        observations = V if side == "V" else W
        if invalid_value == 0.0:
            observations[:, 0] = 0.0
        else:
            observations[0, 0] = invalid_value

        with pytest.raises(ValueError, match=message):
            QUEST(V, W)


def test_degenerate_geometry_has_defined_behavior():
    collinear_cases = [
        (np.array([[1.0, 1.0], [0.0, 0.0], [0.0, 0.0]]), np.eye(3)[:, :2]),
        (np.array([[1.0, -1.0], [0.0, 0.0], [0.0, 0.0]]), np.eye(3)[:, :2]),
        (np.eye(3)[:, :2], np.array([[1.0, 1.0], [0.0, 0.0], [0.0, 0.0]])),
        (np.eye(3)[:, :2], np.array([[1.0, -1.0], [0.0, 0.0], [0.0, 0.0]])),
    ]
    # Basically if two vectors are aligned parallel to each other.
    for V, W in collinear_cases:
        with pytest.raises(ValueError, match="non-collinear"):
            QUEST(V, W)

    V = np.eye(3)
    W = np.diag([1.0, -1.0, -1.0])
    q = QUEST(V, W)
    assert np.all(np.isfinite(q))
    np.testing.assert_allclose(np.linalg.norm(q), 1.0, atol=1e-12)
