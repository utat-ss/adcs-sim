# Module for handling attitude dynamics, kinematics, and relevant functions
# Full, nonlinear kinematics and dynamics

# kinematics are the quaternion equations
# dynamics are the rotational equations

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

def attitude_kinematics(epsilon, eta, omega):
    epsilon_cross = skew(epsilon)

    epsilon_dot = (
        0.5 * 
        (eta * np.eye(3) + epsilon_cross)
        @ omega
    )

    eta_dot = -0.5 * (epsilon @ omega)

    return epsilon_dot, eta_dot

def attitude_dynamics(omega, J, Jinv, torque_ext):
    omega_dot = (
        Jinv
        @ (
            torque_ext.flatten()
            -np.cross(omega, J @ omega)
        )
    )

    return omega_dot

def skew(b):
    return np.array([
        [0, -b[2], b[1]],
        [b[2], 0, -b[0]], 
        [-b[1], b[0], 0]
    ])

# Inertia Matrix
Jxx = 1
Jxy = 0
Jxz = 0
Jyx = 0
Jyy = 1
Jyz = 0
Jzx = 0
Jzy = 0
Jzz = 1

J = np.array([
    [Jxx, Jxy, Jxz],
    [Jyx, Jyy, Jyz],
    [Jzx, Jzy, Jzz]
])

Jinv = np.linalg.inv(J)

print(J)
print(Jinv)

# Initial condictions: values of the state vector at time = 0. 
# Initial tilt
epsilon0 = np.array([
    [0.1],
    [0.0],
    [0.0]
])
print(epsilon0)

# Keep unit quaternion 
# This comes from keeping the quarternion constraint : e1^2 + e2^2 + e3^3 + eta^2 = 1
# As we need to calculate the length of the vector we are using
eta0 = math.sqrt(1 - np.linalg.norm(epsilon0)**2)
print(eta0)

# Initial spin
omega0 = np.array([
    [0.01],
    [0.02],
    [0.0]
])
print(omega0)

# State vector
x0 = np.concatenate([
    epsilon0.flatten(),
    [eta0],
    omega0.flatten()
])
print(x0)

# External torque (asuming no external torque on cubesat)
torque_ext = np.array([
    [0],
    [0],
    [0]
])

# Simulation time (currently running for 2 minutes)
t_start = 0
t_end = 60*90
total_time = [t_start, t_end]


# ODE for the simulation 
# in MATLAB, it was: [t, X] = ode45(@(t, x) dynamics(t, x, J, torque_ext), total_time, x0);
def dynamics(t, x, J, torque_ext):
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


sol = solve_ivp(
    lambda t, x: dynamics(t, x, J, torque_ext),
    total_time,
    x0,
    method='RK45'
)
t = sol.t 
X = sol.y.T # sol.y has shape (states, time_points)

print(sol.success)
print(sol.message)
print(X[:5,:])
epsilon_values = X[:, 0:3]
eta_values = X[:, 3]
omega_values = X[:, 4:7]

# Results

print("Time eps_x eps_y eps_z eta omg_x omg_y omg_z")

# Figure 1 : Quaternion Components
plt.figure()
plt.plot(t, epsilon_values[:,0], label = 'eps_x')
plt.plot(t, epsilon_values[:,1], label = 'eps_y')
plt.plot(t, epsilon_values[:,2], label = 'eps_z')
plt.plot(t, eta_values, label='eta')
plt.legend()

# Figure 2 : Angular Velocity Components
plt.figure()
plt.plot(t, omega_values[:,0], label = 'omg_x')
plt.plot(t, omega_values[:,1], label = 'omg_y')
plt.plot(t, omega_values[:,2], label = 'omg_z')
plt.legend()

# Figure 3 : Quaternion Norm
qnorm = np.sqrt(
    epsilon_values[:,0]**2 +
    epsilon_values[:,1]**2 +
    epsilon_values[:,2]**2 +
    eta_values**2
)

plt.figure()
plt.plot(t, qnorm)
plt.show()
