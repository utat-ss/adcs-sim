import numpy as np

def triad(
    w1_ref: np.ndarray,
    w2_ref: np.ndarray,
    v1_body: np.ndarray,
    v2_body: np.ndarray,
    tol: float = 1e-8,
) -> np.ndarray:
    """
        Compute the Direction Cosine Matrix (DCM) from reference to body frame. 
    
        w1_ref (np.ndarray): a known primary vector in the reference frame (preferably the most precise vector from reference frame).
        w2_ref (np.ndarray): a known secondary vector in the reference frame.
        v1_body (np.ndarray): a primary observation in the body frame (preferably the most precise vector from body frame).
        v2_body (np.ndarray): a secondary observation in the body frame.
        
        Returns:
        r (np.ndarray): the direction cosine matrix (DCM) from reference to body frame
        """
    w1_hat = w1_ref / np.linalg.norm(w1_ref)
    v1_hat = v1_body / np.linalg.norm(v1_body)

    w2 = np.cross(w1_ref, w2_ref)
    v2 = np.cross(v1_body, v2_body)
    w2_norm = np.linalg.norm(w2)
    v2_norm = np.linalg.norm(v2)

    if w2_norm < tol or v2_norm < tol:
        raise ValueError ("Primary and secondary vectors are parallel or anti-parallel.")

    w2_hat = w2 / w2_norm
    v2_hat = v2 / v2_norm

    w3_hat = np.cross(w1_hat, w2_hat)
    v3_hat = np.cross(v1_hat, v2_hat)

    body_frame = np.column_stack((v1_hat, v2_hat, v3_hat))
    reference_frame = np.column_stack((w1_hat, w2_hat, w3_hat))

    return body_frame @ reference_frame.T