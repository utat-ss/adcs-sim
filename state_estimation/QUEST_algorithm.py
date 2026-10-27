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
    """
    if V.shape != W.shape:
        raise ValueError("In correct length, V and W should have same number of element")
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
    eigval = np.roots([1, 0, -a-b, -c, a * b + c * sigma - d]).real
    eigval_max = np.max(eigval)
    M = (eigval_max + sigma) * np.identity(3) - H
    p = np.linalg.solve(M, z)
    q = np.concatenate(([1.0], p))
    q = normalize_quat(q)
    return q

