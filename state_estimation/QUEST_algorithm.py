import numpy as np


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
    :return: Unit attitude quaternion [x, y, z, w]. Reference vectors are
        transformed to body coordinates by transform_vect_coord_system(V[:, i], q),
        equivalently W = quat_to_rotmat(q).T @ V for exact observations.
    """
    if V.shape != W.shape:
        raise ValueError("Incorrect length, V and W should have same number of element")
    for i in range(len(V)):
        if (V[i] != np.nan and W[i] != np.nan) or (V[i] != [0,0,0] and W[i] != [0,0,0]):
            for j in range(len(V[i])):
                if V[i][j] == np.nan and W[i][j] == np.nan:
                    raise ValueError("element in W and V should not be np.nan")
        else:
            raise ValueError("W and V should not be np.nan or 0")
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
    q = np.concatenate((p, [1.0]))
    q = q / np.linalg.norm(q)
    return q
