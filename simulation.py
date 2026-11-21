# Handle the simulation runtime

from collections.abc import Callable
from pathlib import Path
from dataclasses import dataclass
import numpy as np

@dataclass
class SimConfig:
    """
    Simulation configuration dataclass
    """
    epoch:  str     # Reference date for simulation time
    t0:     float   # Simulation start time [JD]
    tf:     float   # Simulation end time [JD]

def load_sim_cfg(filepath: Path) -> SimConfig:
    """
    Load the simulator configuration from a file.
    
    Arguments:
    filepath:       (Path) Path to the simulator configuration file.
    
    Returns:
    SimConfig:      Simulator configuration dataclass.
    """
    raise NotImplementedError()

def run_sim(cfg):
    """
    Main runtime function for simulating the dynamics according to the providing configuration.

    Arguments:
    cfg:            (SimConfig) Simulator configuration dataclass.

    Returns:
    bool:           Simulator completion status.
    """
    raise NotImplementedError()




def custom_integrator(func: Callable, x0: np.ndarray, t0: float, tf: float, dt: float) -> list[np.ndarray]:
    """
    Custom integrator for orbital motion.

    Arguments:
    func:       (Callable) Function to integrate.
    x0:         (np.ndarray) Initial state vector consisting of the position, velocity, angular velocity, and control torques in body frame [x, y, z, v_x, v_y, v_z, w_x, w_y, w_z, u_x, u_y, u_z].
    t0:         (float) Initial time.
    tf:         (float) Final time.
    dt:         (float) Time step.

    Returns:
    states:     (list[np.ndarray]) List of state vectors at each time step.
    """
    states = []
    t = t0
    x = x0

    while t <= tf:
        states.append(x)
        x = x + RK4_step(func, x, t, dt)  # Use RK4 integration step
        t += dt

    return states

def euler_step(func: Callable, x: np.ndarray, t: float, dt: float) -> np.ndarray:
    """
    Perform a single Euler integration step.

    Arguments:
    func:       (Callable) Function to integrate.
    x:          (np.ndarray) Current state vector.
    t:          (float) Current time.
    dt:         (float) Time step.

    Returns:
    np.ndarray: New state vector after the Euler step.
    """
    xdot = func(t, x)
    return x + xdot * dt

def RK4_step(func: Callable, x: np.ndarray, t: float, dt: float) -> np.ndarray:
    """
    Perform a single Runge-Kutta 4th order (RK4) integration step.

    Arguments:
    func:       (Callable) Function to integrate.
    x:          (np.ndarray) Current state vector.
    t:          (float) Current time.
    dt:         (float) Time step.

    Returns:
    np.ndarray: New state vector after the RK4 step.
    """
    k1 = func(t, x)
    k2 = func(t + dt / 2, x + k1 * dt / 2)
    k3 = func(t + dt / 2, x + k2 * dt / 2)
    k4 = func(t + dt, x + k3 * dt)

    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)