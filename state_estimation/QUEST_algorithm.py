import numpy as np
from utils.quaternion_math import normalize_quat


def QUEST(V: np.ndarray, W: np.ndarray):
    """
    Estimate spacecraft attitude using Shuster's QUEST algorithm.

    Each column of V and W represents one observation vector.
    The i-th column of V and the i-th column of W must correspond
    to the same physical observation.

    :param V: np.ndarray
        Reference observation vectors expressed in the chosen
        reference frame. Expected shape is (3, m), where m is
        the number of observations.
    :param W: np.ndarray
        Corresponding measured observation vectors expressed in
        the spacecraft body frame. Expected shape is (3, m).
    :return: Quaternions indicating current attitude
    :raises TypeError: If either input is not a NumPy array.
    :raises ValueError: If the inputs have invalid dimensions, contain
        non-finite or zero-length vectors, or do not provide enough
        non-collinear observations.
    """
    if not isinstance(V, np.ndarray) or not isinstance(W, np.ndarray):
        raise TypeError("V and W must be NumPy arrays")
    if V.ndim != 2 or W.ndim != 2 or V.shape[0] != 3 or W.shape[0] != 3:
        raise ValueError("V and W must have shape (3, m)")
    if V.shape != W.shape:
        raise ValueError("V and W must have the same shape")
    if V.shape[1] < 2:
        raise ValueError("At least two observation vectors are required")
    if not np.all(np.isfinite(V)) or not np.all(np.isfinite(W)):
        raise ValueError("V and W must contain only finite values")
    if np.any(np.linalg.norm(V, axis=0) == 0) or np.any(np.linalg.norm(W, axis=0) == 0):
        raise ValueError("Observation vectors must have non-zero length")
    if np.linalg.matrix_rank(V) < 2 or np.linalg.matrix_rank(W) < 2:
        raise ValueError("V and W must each contain at least two non-collinear vectors")

    B = W @ V.T
    sigma = np.trace(B)
    H = B + B.T
    z = np.array([
        B[1, 2] - B[2, 1],
        B[2, 0] - B[0, 2],
        B[0, 1] - B[1, 0],
    ])
    kappa = 0.5 * (np.trace(H) ** 2 - np.trace(H @ H))
    a = sigma ** 2 - kappa
    b = sigma ** 2 + z.T @ z
    c = np.linalg.det(H) + z.T @ H @ z
    d = z.T @ H @ H @ z
    coefficients = [1, 0, -a-b, -c, a * b + c * sigma - d]
    try:
        eigval = np.roots(coefficients).real
    except np.linalg.LinAlgError as exc:
        raise ValueError("QUEST could not determine a finite eigenvalue") from exc
    if not np.all(np.isfinite(eigval)):
        raise ValueError("QUEST could not determine a finite eigenvalue")
    eigval_max = np.max(eigval)
    M = (eigval_max + sigma) * np.identity(3) - H
    try:
        p = np.linalg.solve(M, z)
    except np.linalg.LinAlgError as exc:
        raise ValueError(
            "QUEST attitude solution is singular for these observations"
        ) from exc
    if not np.all(np.isfinite(p)):
        raise ValueError("QUEST attitude solution contains non-finite values")
    q = np.concatenate(([1.0], p))
    q = normalize_quat(q)
    if not np.all(np.isfinite(q)) or not np.isclose(np.linalg.norm(q), 1.0):
        raise ValueError("QUEST produced an invalid quaternion")
    return q
