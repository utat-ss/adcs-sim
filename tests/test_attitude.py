import numpy as np
import pytest
from scipy.integrate import sovle_ivp

from attitude import(
    skew,
    attitude_kinematics,
    attitude_dynamics,
    dynamics
)

'''
8 unit tests to check the individual functions skew(), attitude_kinematics(), attitude_dynamics(), and attitude()

3 property (physics/integration) tests to check:
    1. quaternion norm preservation
    2. angular momentum conservation (torque = 0)
    3. angular momentum change matches torque
'''

# Unit tests
# Test 1: Checking if skew produces the correct matrix
def test_skew_known_vector(): #test the skew function under the condition that the vector is known
    # Arrange
    b = np.array([1, 2, 3])

    expected = np.array([
        [0, -3, 2],
        [3, 0, -1], 
        [-2, 1, 0]
    ])
    # Act
    result = skew(b)
    
    # Assert
    np.resting.assert_allclose(result, expected) #check if two values equal within a small tolerance

# Test 2: Checking if the matrix is skew-symmetric
def test_skew_is_skew_symmetric():
    # Arrange
    b = np.array([4, -2, 7])

    # Act
    S = skew(b)

    # Assert
    np.testing.assert_allclose(S.T, -S) # if the matrix is skew symmetric, S^T = -S

# Test 3: Checking if skew(a)b == a x b 
def test_skew_equivalent_to_cross_product():
    # Arrange
    a = np.array([1, 2, 3])
    b = np.array([4, 5, 6])

    # Act
    left  = skew(a) @ b
    right = np.cross(a,b)

    # Assert
    np.testing.assert_allclose(left, right)

# Test 4: Checking if attitude_kinematics works with zero angular velocity
def test_attitude_kinematics_zero_angular_velocity():
    # Arrange
    epsilon = np.array([0.1, 0.2, 0.3])
    eta = np.sqrt(1 - np.linalg.norm(epsilon)**2)
    omega = np.zeros(3)

    # Act
    epsilon_dot, eta_dot = attitude_kinematics(
        epsilon, 
        eta, 
        omega
    )

    # Assert
    np.testing.assert_allclose(epsilon_dot, np.zeros(3))
    np.testing.assert_allclose(eta_dot, 0.0)

# Test 5: Checking if attitude_kinematics works with rotation along only one axis
def test_attitude_kinematics_identity_quaternion():
    # Arrange
    epsilon = np.zeros(3)
    eta = 1.0
    omega = np.zeros(3)

    # Act
    epsilon_dot, eta_dot = attitude_kinematics(
        epsilon, eta, omega
    )

    # Assert
    np.testing.assert_allclose(epsilon_dot, np.array([0.5, 0.0, 0.0]))
    np.testing.assert_allclose(eta_dot, 0.0)

# Test 6: Checking if attitude_dynamics works with zero torque and zero angular velocity
def test_attitude_dunamics_zero_input():
    # Arrange
    omega = np.zeros(3)
    J = np.eye(3)
    Jinv = np.linalg.inv(J)
    torque = np.zeros((3,1))

    # Act
    omega_dot = attitude_dynamics(omega, J, Jinv, torque)

    #Assert
    np.testing.assert_allclose(omega_dot, np.zeros(3))

# Test 7: Checking if attitude_dynamics returns omega_dot = tau when J = I
def test_attitude_dynamics_identity_inertia():
    # Arrange
    omega = np.array([1.0, 2.0, 3.0])
    J = np.eye(3)
    Jinv = np.linalg.inv(J)
    torque = np.array([[0.1], [0.2], [0.3]])

    # Act
    omega_dot = attitude_dynamics(omega, J, Jinv, torque)
    
    # Assert
    np.testing.assert_allclose(omega_dot, torque.flatten())

# Test 8: Checking if dynamics unpacks the state vector correctly
def test_dynamics_structure_and_slicing():
    # Arrange
    epsilon = np.array([0.1, 0.2, 0.3])
    eta = 0.9
    omega = np.array([1.0, 2.0, 3.0])

    x = np.concatenate([epsilon, [eta], omega])

    J = np.eye(3)
    torque = np.zeros((3, 1))
    t = 0.0

    # Act
    xdot = dynamics(t, x, J, torque)

    # Assert structure
    assert xdot.shape == (7,)
    
    epsilon_dot = xdot[0:3]
    eta_dot = xdot[3]
    omega_dot = xdot[4:7]

    assert epsilon_dot.shape == (3,)
    assert omega_dot.shape == (3,)

# Integration tests
# Test 1: Checking if quaternion remains unit length (||q|| = 1)
def test_quaternion_norm_preservation():
    # Arrange
    epsilon0 = np.array([0.1, 0.0, 0.0])
    eta0 = np.sqrt(1 - np.linalg.norm(epsilon0)**2)
    omega0 = np.array([0.01, 0.02, 0.0])

    x0 = np.concatenate([epsilon0, [eta0], omega0])

    J = np.eye(3)
    torque = np.zeros((3,1))

    t_span = (0,100)

    # Act
    sol = solve_ivp(
        lambda t, x: dynamics(t, x, J, torque)
        t_span,
        x0,
        method = "RK45"
    )

    X = sol.y.T
    
    epsilon = X[:, 0:3]
    eta = X[:,3]

    q_norm = np.sqrt(
        np.sum(epsilon**2, axis=1) + eta**2
    )

    # Assert
    np.testing.assert_allclose(q_norm, 1.0, atol=1e-2)

# Test 2: Checking if angular momentum is conserved (torque = 0)
def test_angular_momentum_conservation():
    # Arrange
    epsilon0 = np.array([0.1, 0.2, 0.3])
    eta0 = np.sqrt(1 - np.linalg.norm(epsilon0)**2)
    omega0 = np.array([1.0, 2.0, 3.0])

    x0 = np.concatenate([epsilon0, [eta0], omega0])

    J = np.diag([2.0, 3.0, 4.0])
    torque = np.zeros((3,1))

    # Act
    sol = solve_ivp(
        lambda t, x: dynamics(t, x, J, torque)
        (0, 50),
        x0,
        method = "RK45"
    )

    X = sol.y.T
    omega = X[:, 4:7]

    H = (J @ omega.T).T
    H0 = H[0]

    error = np.linalg.norm(H - H0, axis=1)

    # Assert
    np.testing.assert_allclose(error, 0.0, atol=1e-2)

# Test 3: Checking if rate of change of angular momentum matches torque
def test_torque_matches_angular_momentum_rate():
    # Arrange
    epsilon0 = np.array([0.1, 0.2, 0.3])
    eta0 = np.sqrt(1 - np.linalg.norm(epsilon0)**2)
    omega0 = np.array([0.5, -0.3, 0.2])

    x0 = np.concatenate([epsilon0, [eta0], omega0])

    J = np.diag([2.0, 3.0, 4.0])
    torque = np.array([[0.1], [0.0], [0.0]])

    # Act
    sol = solve_ivp(
        lambda t, x: dynamics(t, x, J, torque)
        (0, 20),
        x0,
        method = "RK45"
    )

    t = sol.t
    X = sol.y.T

    omega = X[:, 4:7]
    H = (J @ omega.T).T

    dH_dt = np.gradient(H, t, axis=0)

    torque_array = np.repeat(torque.T, len(t), axis = 0)
    error = np.linalg.norm(dH_dt - torque_array, axis = 1)

    # Assert
    np.testing.assert_allclose(error, 0.0, atol=5e-2)
