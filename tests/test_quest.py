import importlib.util
import pathlib

import numpy as np
import pytest

# ------------------------------------------------------------------------------
# Had to load this in to use the QUEST method due to continous failures in
# running due to NameError: name 'QUEST' is not defined
# ------------------------------------------------------------------------------

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_from_state_estimation(filename: str, function_name: str):
    """Load function_name from state_estimation/<filename> by file path."""
    path = REPO_ROOT / "state_estimation" / filename
    spec = importlib.util.spec_from_file_location(path.stem.replace("-", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, function_name)


QUEST = load_from_state_estimation("QUEST_algorithm.py", "QUEST")
q_method = load_from_state_estimation("q-method.py", "q_method")

# ------------------------------------------------------------------------------
# Helper functions.
# Normalizing the quaternion, getting the quaternion from rotation vec and
# to rotation matrix, the respective angles and new test cases regarding noise
# were added so I needed a little AI help to decipher how to implement their
# functions as well. Most of the logic of the quaternion functions came from the
# old implementation of the quaternion functions.
# Additional details in test_quest.md. which I need to write.
# ------------------------------------------------------------------------------


def normalize_quat(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion q = [x, y, z, w]."""
    q = np.asarray(q, dtype=float)
    if q.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero norm.")
    return q / norm


def quat_from_rotvec(rotvec_rad: np.ndarray) -> np.ndarray:
    """
    Convert rotation vector to quaternion [x, y, z, w].
    Direction is the rotation axis; magnitude is the angle in radians.
    """
    rotvec_rad = np.asarray(rotvec_rad, dtype=float)
    if rotvec_rad.shape != (3,):
        raise ValueError(
            f"Rotation vector must have shape (3,), got {rotvec_rad.shape}"
        )
    angle = np.linalg.norm(rotvec_rad)
    if angle < 1e-15:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis = rotvec_rad / angle
    half_angle = 0.5 * angle
    xyz = axis * np.sin(half_angle)
    w = np.cos(half_angle)
    return np.array([xyz[0], xyz[1], xyz[2], w], dtype=float)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Convert a scalar-last quaternion [x, y, z, w] to its passive DCM (W = A @ V)."""
    x, y, z, w = normalize_quat(q)
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y + w * z), 2 * (x * z - w * y)],
            [2 * (x * y - w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z + w * x)],
            [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x**2 + y**2)],
        ]
    )


def get_quat_angle_deg(q: np.ndarray) -> float:
    """Rotation angle in degrees of a scalar-last quaternion [x, y, z, w]."""
    w = normalize_quat(q)[3]
    return 2.0 * np.degrees(np.arccos(np.clip(abs(w), 0.0, 1.0)))


def angle_between_deg(A1: np.ndarray, A2: np.ndarray) -> float:
    """Angle in degrees of the rotation that takes DCM A1 to DCM A2."""
    c = (np.trace(A1.T @ A2) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def make_case(seed: int, m: int, noise: float = 0.0):
    """Random true attitude A, m unit reference vectors V, and body vectors W."""
    rng = np.random.default_rng(seed)
    A = quat_to_rotmat(quat_from_rotvec(rng.standard_normal(3)))
    V = rng.standard_normal((3, m))
    V /= np.linalg.norm(V, axis=0, keepdims=True)
    W = A @ V + noise * rng.standard_normal((3, m))
    W /= np.linalg.norm(W, axis=0, keepdims=True)
    return A, V, W


# ------------------------------------------------------------------------------
# Testing the helpers themselves
# Test 0 to 7. I'm testing the validity of QUEST and my own helpers first to see
# if they match the mathematical properties of the QUATERNIONS and the QUEST
# algorithm
# ------------------------------------------------------------------------------


def test_helpers_match_hand_derived_rotation():
    """Test 0: 90 deg about z gives the hand-derived passive Direction
    Cosine Matrix and quaternion."""
    q = quat_from_rotvec(np.array([0.0, 0.0, np.pi / 2]))
    s = np.sqrt(0.5)
    np.testing.assert_allclose(q, [0.0, 0.0, s, s], atol=1e-12)

    A_expected = np.array(
        [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(quat_to_rotmat(q), A_expected, atol=1e-12)
    np.testing.assert_allclose(get_quat_angle_deg(q), 90.0, atol=1e-12)


@pytest.mark.parametrize("m", [2, 3, 5, 20])
def test_random_round_trip(m):
    """Test 1: Random round trip for various observation counts m."""
    A_true, V, W = make_case(100 + m, m)
    np.testing.assert_allclose(quat_to_rotmat(QUEST(V, W)), A_true, atol=1e-9)


def test_identity_rotation():
    """Test 2: V = W gives the identity attitude."""
    _, V, _ = make_case(
        300, 5
    )  # I love this little underscore trick to give us V and discard A,W
    W = V.copy()
    q_rec = QUEST(V, W)
    np.testing.assert_allclose(quat_to_rotmat(q_rec), np.eye(3), atol=1e-9)
    np.testing.assert_allclose(abs(q_rec[3]), 1.0, atol=1e-9)


S45 = np.sin(
    np.radians(45.0)
)  # Shortcuts I learnt when i was writing the speedup code for spacenet
S15 = np.sin(np.radians(15.0))  # woohoo
C15 = np.cos(np.radians(15.0))


@pytest.mark.parametrize(
    "rotvec, q_expected",
    [
        (np.radians(90.0) * np.array([1.0, 0.0, 0.0]), [S45, 0.0, 0.0, S45]),
        (np.radians(90.0) * np.array([0.0, 1.0, 0.0]), [0.0, S45, 0.0, S45]),
        (np.radians(90.0) * np.array([0.0, 0.0, 1.0]), [0.0, 0.0, S45, S45]),
        (
            np.radians(30.0) * np.ones(3) / np.sqrt(3.0),
            [S15 / np.sqrt(3.0), S15 / np.sqrt(3.0), S15 / np.sqrt(3.0), C15],
        ),
    ],
)
def test_known_axis_angle_cases(rotvec, q_expected):
    """Test 3: Known axis-angle cases match hard-coded quaternions (up to sign).
    Look a the above parameterizations to check the rotvec and expected"""
    A_true = quat_to_rotmat(quat_from_rotvec(rotvec))
    rng = np.random.default_rng(42)
    V = rng.standard_normal((3, 4))
    V /= np.linalg.norm(V, axis=0, keepdims=True)  # random unit vectors
    W = A_true @ V

    q_rec = QUEST(V, W)  # ask QUEST to find the rotation
    q_rec = q_rec if q_rec[3] >= 0 else -q_rec  # q and -q are the same attitude
    np.testing.assert_allclose(q_rec, q_expected, atol=1e-9)


def test_minimal_case():  # smallest possible test for running quest
    """Test 4: m = 2 perpendicular vectors recover a 45 deg z rotation exactly."""
    V = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
    A_true = quat_to_rotmat(quat_from_rotvec(np.array([0.0, 0.0, np.radians(45.0)])))
    W = A_true @ V
    np.testing.assert_allclose(quat_to_rotmat(QUEST(V, W)), A_true, atol=1e-9)


# ------------------------------------------------------------------------------
# Test 5 was a failing test last time, I think this has been solved in this
# implementation except the 180 degree case which is a rare occurance but can
# occur in tumbling scenarios or fast movements or in the test benches.
# To be taken into account. Sensor drift can also cause the max angle of 180 to
# be seen (That's my hypothesis - Mahat/ aka Yuvi)
# ------------------------------------------------------------------------------


@pytest.mark.parametrize("angle_deg", [90.0, 150.0, 170.0, 179.0])
def test_angle_sweep(angle_deg):
    """Test 5: Recovery for rotations approaching the 180 deg limit."""
    A_true = quat_to_rotmat(
        quat_from_rotvec(np.radians(angle_deg) * np.array([0.0, 1.0, 0.0]))
    )
    rng = np.random.default_rng(200 + int(angle_deg))
    V = rng.standard_normal((3, 5))
    V /= np.linalg.norm(V, axis=0, keepdims=True)
    W = A_true @ V

    q_rec = QUEST(V, W)
    np.testing.assert_allclose(get_quat_angle_deg(q_rec), angle_deg, atol=1e-5)
    np.testing.assert_allclose(quat_to_rotmat(q_rec), A_true, atol=1e-6)


@pytest.mark.xfail(
    reason="QUEST's Gibbs vector is infinite at exactly 180 degrees.",
    raises=(AssertionError, np.linalg.LinAlgError),
    strict=True,
)
def test_180_degree_rotation():
    """Test 6: Exact 180 deg recovery is a known QUEST limitation."""
    A_true = quat_to_rotmat(quat_from_rotvec(np.array([np.pi, 0.0, 0.0])))
    V = np.eye(3)
    W = A_true @ V
    np.testing.assert_allclose(quat_to_rotmat(QUEST(V, W)), A_true, atol=1e-9)


def test_quest_quaternion_is_unit_norm():
    """Test 7: QUEST returns a finite, unit, length-4 quaternion."""
    A_true = quat_to_rotmat(quat_from_rotvec(np.array([0.2, -0.4, 0.7])))
    V = np.eye(3)
    q_rec = QUEST(V, A_true @ V)

    assert q_rec.shape == (4,)
    assert np.all(np.isfinite(q_rec))
    np.testing.assert_allclose(np.linalg.norm(q_rec), 1.0, atol=1e-12)


# ------------------------------------------------------------------------------
# Current status of Test 0 to 7 as of Novemeber 20 8:50 pm
# Multiple tests failing, I can't tell if I'm wrong or not I'll pause for a
# minute...anyways
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Noisy data
# ------------------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(5))
def test_matches_q_method(seed):
    """Test 8: QUEST and Davenport's q-method agree on noisy data."""
    _, V, W = make_case(seed, 6, noise=1e-3)
    np.testing.assert_allclose(
        quat_to_rotmat(QUEST(V, W)), quat_to_rotmat(q_method(V, W)), atol=1e-8
    )


def test_noisy_error_is_small():
    """Test 9: Small sensor noise gives a small attitude error."""
    A_true, V, W = make_case(7, 20, noise=1e-3)
    assert angle_between_deg(quat_to_rotmat(QUEST(V, W)), A_true) < 0.1


def test_column_order_does_not_matter():
    """Test 10: Reordering observations (both V and W together) changes nothing."""
    _, V, W = make_case(8, 6, noise=1e-3)
    p = np.random.default_rng(0).permutation(V.shape[1])
    np.testing.assert_allclose(
        quat_to_rotmat(QUEST(V[:, p], W[:, p])),
        quat_to_rotmat(QUEST(V, W)),
        atol=1e-10,
    )


def test_uniform_scaling_does_not_change_attitude():
    """Test 11: Scaling every vector by the same factor changes nothing."""
    _, V, W = make_case(9, 5, noise=1e-3)
    np.testing.assert_allclose(
        quat_to_rotmat(QUEST(3.0 * V, 3.0 * W)),
        quat_to_rotmat(QUEST(V, W)),
        atol=1e-10,
    )


def test_swapping_inputs_gives_inverse_rotation():
    """Test 12: QUEST(W, V) is the inverse (transpose) of QUEST(V, W)."""
    _, V, W = make_case(10, 5)
    np.testing.assert_allclose(
        quat_to_rotmat(QUEST(W, V)), quat_to_rotmat(QUEST(V, W)).T, atol=1e-9
    )


def test_does_not_modify_inputs():
    """Test 13: QUEST leaves V and W unchanged."""
    _, V, W = make_case(11, 5)
    V0, W0 = V.copy(), W.copy()
    QUEST(V, W)
    np.testing.assert_array_equal(V, V0)
    np.testing.assert_array_equal(W, W0)


# ------------------------------------------------------------------------------
# Now the input validation
# ------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "V, W, exception",
    [
        (np.ones(3), np.ones(3), ValueError),  # 1-D vector, not (3, m)
        (np.ones((2, 3)), np.ones((2, 3)), ValueError),  # 2 rows, not 3
        (np.ones((4, 3)), np.ones((4, 3)), ValueError),  # 4 rows, not 3
        (np.ones((3, 2)), np.ones((3, 3)), ValueError),  # shapes differ
        (None, np.ones((3, 2)), TypeError),  # V isn't an array
        (np.ones((3, 2)), "vectors", TypeError),  # W is a string
        ([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]], np.eye(3)[:, :2], TypeError),  # list
    ],
)
def test_rejects_malformed_inputs(V, W, exception):
    """Test 14: Malformed inputs raise the documented exception type."""
    with pytest.raises(exception):
        QUEST(V, W)


@pytest.mark.parametrize("m", [0, 1])
def test_rejects_too_few_observations(m):
    """Test 15: Fewer than 2 observations can't determine attitude."""
    V = np.eye(3)[:, :m]
    with pytest.raises(ValueError):
        QUEST(V, V.copy())


@pytest.mark.parametrize("side", ["V", "W"])
@pytest.mark.parametrize("bad_value", [np.nan, np.inf, 0.0])
def test_rejects_nonfinite_or_zero_vectors(side, bad_value):
    """Test 16: NaN, inf, or zero-length observation vectors are rejected."""
    V = np.eye(3)[:, :2].copy()
    W = V.copy()
    target = V if side == "V" else W
    if bad_value == 0.0:
        target[:, 0] = 0.0  # whole vector is zero
    else:
        target[0, 0] = bad_value  # one bad component
    with pytest.raises(ValueError):
        QUEST(V, W)


@pytest.mark.parametrize(
    "V, W",
    [
        (
            np.array([[1.0, 1.0], [0.0, 0.0], [0.0, 0.0]]),
            np.eye(3)[:, :2],
        ),  # V parallel
        (
            np.array([[1.0, -1.0], [0.0, 0.0], [0.0, 0.0]]),
            np.eye(3)[:, :2],
        ),  # V antiparallel
        (
            np.eye(3)[:, :2],
            np.array([[1.0, 1.0], [0.0, 0.0], [0.0, 0.0]]),
        ),  # W parallel
        (
            np.eye(3)[:, :2],
            np.array([[1.0, -1.0], [0.0, 0.0], [0.0, 0.0]]),
        ),  # W antiparallel
    ],
)
def test_rejects_collinear_observations(V, W):
    """Test 17: Only parallel/antiparallel vectors can't determine attitude."""
    with pytest.raises(ValueError):
        QUEST(V, W)
