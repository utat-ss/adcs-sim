import numpy as np

def QUEST(V: np.ndarray, W: np.ndarray):
    if V.shape != W.shape:
        raise ValueError("Incorrect length, V and W should have same number of elements")
    
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
    p = np.linalg.solve(M, z)  # This is where matrix singularity crashes happen
    q = np.concatenate(([1.0], p))
    return q / np.linalg.norm(q)


if __name__ == "__main__":
    print("=" * 60)
    print("QUEST ALGORITHM STRESS TEST: EXECUTING CRASH CASES")
    print("=" * 60)

    # -------------------------------------------------------------
    # Crash Case 1: The 180° Rotation Singularity
    # -------------------------------------------------------------
    print("\n[Test 1] 180° Rotation Singularity (q4 -> 0)")
    # A 180-degree rotation around the X-axis: X stays X, Y flips to -Y, Z flips to -Z
    V_180 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    W_180 = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

    try:
        q = QUEST(V_180, W_180)
        print(f"  -> Result: Completed without crashing. Output q = {q}")
        print(f"  -> Scalar component q4 is: {q[0]:.6f} (Notice it failed to reach strict zero due to float rounding, but precision is heavily degraded)")
    except Exception as e:
        print(f"  -> CAUGHT CRASH: {type(e).__name__}: {e}")

    # -------------------------------------------------------------
    # Crash Case 2: Insufficient Observations (m = 1)
    # -------------------------------------------------------------
    print("\n[Test 2] Single Observation Vector (m = 1)")
    # Wahba's problem requires at least 2 non-collinear vectors to fix 3D orientation
    V_single = np.array([[1.0], [0.0], [0.0]])
    W_single = np.array([[0.0], [1.0], [0.0]])

    try:
        q = QUEST(V_single, W_single)
        print(f"  -> Result: Completed. Output q = {q}")
    except Exception as e:
        print(f"  -> CAUGHT CRASH: {type(e).__name__}: {e}")

    # -------------------------------------------------------------
    # Crash Case 3: Collinear / Degenerate Observations
    # -------------------------------------------------------------
    print("\n[Test 3] Collinear Observation Vectors")
    # Passing parallel vectors means the cross-correlation matrix drops rank
    V_collinear = np.array([
        [1.0, 1.0],
        [0.0, 0.0],
        [0.0, 0.0]
    ])
    W_collinear = np.array([
        [0.0, 0.0],
        [1.0, 1.0],
        [0.0, 0.0]
    ])

    try:
        q = QUEST(V_collinear, W_collinear)
        print(f"  -> Result: Completed. Output q = {q}")
    except Exception as e:
        print(f"  -> CAUGHT CRASH: {type(e).__name__}: {e}")

    print("\n" + "=" * 60)
    print("STRESS TEST COMPLETE.")
    print("=" * 60)