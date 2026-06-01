# Module for handling state estimation/navigation of the spacecraft orbit and attitude states

import numpy as np

def triad(v1: np.ndarray, v2: np.ndarray):
    """
    Tri-Axial Attitude Determination (TRIAD) algorithm for batch attitude estimation.

    Argument:
    v1:     (np.ndarray) (3x1) First measured vector used to constrain pitch and yaw.
            NOTE: In TRIAD, this is treated as the high accuracy vector, so order inputs accordingly.
    v2:     (np.ndarray) (3x1) Second measured vector used to constrain roll.
            Assumed to be lower accuracy.

    Returns:
    c_bi:   (np.ndarray) (3x3) Direction Cosine Matrix describing the estimated
            spacecraft attitude.
    """
    c_bi = np.array([[1., 0., 0.],
                     [0., 1., 0.],
                     [0., 0., 1.]])
    return c_bi

def quest():
    """
    Quaternion Estimator (QUEST) algorithm for batch attitude estimation.
    """
    q_b = np.array([0., 0., 0., 1.])
    return q_b

def q_method():
    """
    Davenport's q-method for batch attitude estimation.
    """
    pass

def orbit_ekf():
    """
    Extended Kalman Filter for use in orbit estimation.
    """
    pass

def attitude_ekf():
    """
    Extended Kalman Filter for use in attitude estimation.
    """
    pass
