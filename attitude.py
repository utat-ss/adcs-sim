# Module for handling attitude dynamics, kinematics, and relevant functions
# Full, nonlinear kinematics and dynamics

# kinematics are the quaternion equations
# dynamics are the rotational equations

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from test import Jinv


# TODO: Doc strings explaining what the functions do, what their inputs are, and what their outputs are. Refer to virtual-sensors branch for some examples.
def attitude_kinematics(epsilon, eta, omega):
    '''
    Compute quarternion kinematics

    Arguments:
        epsilon (ndarray): Vector component of the quarternion
        eta (float): Scalar component of the quaternion
        omega (ndarray): Body of angular velocity vector [rad/s]
    
    Returns:
        tuple: 
            epsilon_dot (ndarray): Quaternion vector derivative
            eta_dot (float) = Quaternion scalar derivative

    '''
    epsilon_cross = skew(epsilon)

    epsilon_dot = (
        0.5 * 
        (eta * np.eye(3) + epsilon_cross)
        @ omega
    )

    eta_dot = -0.5 * (epsilon @ omega)

    return epsilon_dot, eta_dot

def attitude_dynamics(omega, J, Jinv, torque_ext):
    '''
    Compute attitude dynamics
    
    Arguments:
        omega (ndarray): Body of angular velocity vector [rad/s]
        J (ndarray): Inertia matrix
        Jinv (ndarray): Inverse of the inertia matrix
        torque_ext (ndarray): External torque on the cubesat [Nm]

    Returns:
        omega_dot (ndarray): Time derivative of the body angular velocity vector [rad/s^2]
    
    '''
    omega_dot = (
        Jinv
        @ (
            torque_ext.flatten()
            -np.cross(omega, J @ omega)
        )
    )

    return omega_dot

# TODO: This should live with utils. Leave here until code gets plumbed together.
def skew(b):
    return np.array([
        [0, -b[2], b[1]],
        [b[2], 0, -b[0]], 
        [-b[1], b[0], 0]
    ])

def dynamics(t, x, J, torque_ext):
    '''
    Convert current states into their time derivatives for the ODE solver
    
    Arguments:
        t (float): current simulation time
        x (ndarray): full state vector of the spacecraft
        J (ndarray): inertia matrix
        torque_external(ndarray): External torque on the cubesat [Nm]

    Returns:
        xdot (ndarray): time derivative of full state vector
    '''
    epsilon = x[0:3]
    eta = x[3]
    omega = x[4:7]

    epsilon_dot, eta_dot = attitude_kinematics(
        epsilon, eta, omega
    )

    omega_dot = attitude_dynamics(
        omega, J, Jinv, torque_ext
    )
    return np.concatenate([
        epsilon_dot,
        [eta_dot],
        omega_dot
    ])


def linearized_attitude_matrices(q_0, omega_0, J):
    """
    Compute A, B for the linearized reduced-quaternion attitude system.
    Linearized system:

    Arguments:
        q_0:
            Reduced quaternion vector part at the linearization point.
            Shape: (3,)

        omega_0:
            Body-frame angular velocity at the linearization point.
            Shape: (3,)

        J:
            Spacecraft inertia matrix.
            Shape: (3, 3)

    Returns:
        A:
            Linearized state matrix.
            Shape: (6, 6)

        B:
            Linearized input matrix.
            Shape: (6, 3)
    """
    J_inv = np.linalg.inv(J)

    q1, q2, q3 = q_0
    omega1, omega2, omega3 = omega_0

    A_omega_omega = J_inv@(skew(J @ omega_0)-skew(omega_0)@J)

    A_omega_q = np.zeros((3, 3))
    
    f = np.sqrt(1 - q1**2 - q2**2 - q3**2)

    dg_domega = 0.5 * np.array([
        [f, -q3, q2],
        [q3, f, -q1],
        [-q2, q1, f],
    ])
    
    df_dq1 = -q1/f
    df_dq2 = -q2/f
    df_dq3 = -q3/f
    
    dg_dq = 0.5 * np.array([
        [
            df_dq1 * omega1,
            df_dq2 * omega1 + omega3,
            df_dq3 * omega1 - omega2,
        ],
        [
            df_dq1 * omega2 - omega3,
            df_dq2 * omega2,
            df_dq3 * omega2 + omega1,
        ],
        [
            df_dq1 * omega3 + omega2,
            df_dq2 * omega3 - omega1,
            df_dq3 * omega3,
        ],
    ])

    A = np.block([
        [A_omega_omega, A_omega_q],
        [dg_domega,     dg_dq],
    ])

    B = np.vstack([
        J_inv,
        np.zeros((3, 3)),
    ])

    return A, B


# if __name__ == "__main__":

#     J = np.diag([0.02, 0.025, 0.03])

#     q_0 = np.array([0.0, 0.0, 0.0])
#     omega_0 = np.array([0.0, 0.0, 0.0])

#     A, B = linearized_attitude_matrices(q_0, omega_0, J)

#     print("A =")
#     print(A)

#     print("\nB =")
#     print(B)