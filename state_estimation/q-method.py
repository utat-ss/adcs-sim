import numpy as np
from utils.quaternion_math import normalize_quat


def q_method(V: np.ndarray, W: np.ndarray):
    """
    Estimate spacecraft attitude from reference and measured
    observation vectors using Davenport's q-method.

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
        raise ValueError("In correct length, r and b should have same number of element")
    B = W @ V.T
    sigma = np.trace(B)
    H = B + B.T
    z = np.array([
        B[1, 2] - B[2, 1],
        B[2, 0] - B[0, 2],
        B[0, 1] - B[1, 0],
    ])
    K = np.block([
        [sigma, z],
        [z.T, H - sigma * np.identity(H.shape[0])]
    ])
    eigvals, eigvecs = np.linalg.eigh(K)
    q = eigvecs[:, np.argmax(eigvals)]
    q = normalize_quat(q)
    return q
