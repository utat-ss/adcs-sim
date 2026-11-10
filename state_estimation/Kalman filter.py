
import numpy as np
from abc import ABC, abstractmethod
from pathlib import Path


class Estimator(ABC):
    def __init__(self, cfg_file: Path):
        self.cfg_file = cfg_file

    @abstractmethod
    def predict(self, dt, u=None):
        pass

    @abstractmethod
    def update(self, measurement):
        pass

    @abstractmethod
    def get_estimate(self):
        pass

    def load_cfg(self):
        raise NotImplementedError()


class Dynamics(ABC):
    """This calss stores the mechanics for the Kalman filter"""

    @abstractmethod
    def get_matrices(self, dt):
        """Return F, B, Q"""
        pass


class Measurement(ABC):
    """This calss stores the measurements for the Kalman filter"""

    def __init__(self, z):
        self.z = np.asarray(z, dtype=float).reshape(-1)

    @abstractmethod
    def get_matrices(self):
        """Return H, R"""
        pass


class KalmanFilter(Estimator):

    def __init__(self, x_0, P_0, dynamics: Dynamics, cfg_file: Path = None):
        super().__init__(cfg_file)
        self.x = np.asarray(x_0, dtype=float).reshape(-1)
        self.P = np.asarray(P_0, dtype=float)
        self.dynamics = dynamics

    def predict(self, dt, u=None):
        """
        Calling this function to predict the prior x and P using mechanics
        """
        F, B, Q = self.dynamics.get_matrices(dt)

        prior_x = F @ self.x
        if u is not None:
            prior_x += B @ np.asarray(u, dtype=float).reshape(-1)

        prior_P = F @ self.P @ F.T + Q

        self.x = prior_x
        self.P = prior_P
        return self.x, self.P

    def update(self, measurement: Measurement):
        """
        This function updates posterior x and P variable using the new measurement
        """
        z = measurement.z
        H, R = measurement.get_matrices()

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = np.linalg.solve(S, H @ self.P).T

        self.x = self.x + K @ y

        I = np.eye(len(self.x))
        A = I - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T

        return self.x, self.P

    def get_estimate(self):
        """Call to returns the current estimation for P and x"""
        return self.x.copy(), self.P.copy()
