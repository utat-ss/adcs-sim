import math
import numpy as np

def normalize_quat(q: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion q = [x, y, z, w].
    """
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")

    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("Quaternion has zero norm.")

    return q / norm

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Determine quaterion conjugate q' = [-x, -y, -z, w] from quaternion q = [x, y, z, w].
    """
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")
    
    q_prime = np.zeros(4)

    q_prime[0] = -q[0]
    q_prime[1] = -q[1]
    q_prime[2] = -q[2]
    q_prime[3] = q[3]

    return q_prime

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton product q = q1 ⊗ q2.

    Both quaternions use [x, y, z, w].
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=float)

def rotate_vect_by_quat(vect: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Rotate 3D vector vect by quaternion q [x, y, z, w]. 
    Equivalent to vect_rot = q * vect * q'.
    """
    if vect.shape != (3,):
        raise ValueError("vect should be a 3D vector.")
    
    if q.shape != (4,):
        raise ValueError("Quaternion q must have shape (4,).")
    
    quat_vect = np.append(vect, [0])
    
    q_inv = quat_conjugate(q)
    
    rot_quat_vect = quat_multiply(quat_multiply(q, quat_vect), q_inv)
    return rot_quat_vect[0:3]

def transform_vect_coord_system(vect: np.ndarray, q: np.ndarray) -> np.ndarray:
    """
    Transform 3D vector vect between coordinate systems related by the quaternion q [x, y, z, w]. 
    Equivalent to vect_trans = q' * vect * q.
    """
    if vect.shape != (3,):
        raise ValueError("vect should be a 3D vector.")
    
    if q.shape != (4,):
        raise ValueError("Quaternion q must have shape (4,).")
    
    quat_vect = np.append(vect, [0])
    
    q_inv = quat_conjugate(q)
    
    trans_quat_vect = quat_multiply(quat_multiply(q_inv, quat_vect), q)
    return trans_quat_vect[0:3]

def quat_from_rotvec(rotvec_rad: np.ndarray) -> np.ndarray:
    """
    Convert rotation vector to quaternion [x, y, z, w].

    rotvec_rad direction is the rotation axis.
    rotvec_rad magnitude is the rotation angle in radians.
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

def rotvec_from_quat(quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x, y, z, w] to rotation vector.
    """
    quat = np.asarray(quat, dtype=float)

    quat = quat / np.linalg.norm(quat)

    if quat.shape != (4,):
        raise ValueError(
            f"Quaternion quat must have shape (4,), got {quat.shape}"
        )
    
    angle = math.acos(quat[3]) * 2
    axis = quat[:3]
    normed_axis = axis / np.linalg.norm(axis)

    if angle < 1e-15:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    return normed_axis * angle