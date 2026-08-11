# Virtual actuator models for use as hardware stubs in simulation

from pathlib import Path
from abc import ABC, abstractmethod
import numpy as np

class VirtualActuator(ABC):
    def __init__(self, cfg_file: Path):
        # common fields
        self.cfg_file = cfg_file
        self.model: str = ""

    def load_cfg(self):
        """
        Optional shared helper for standardizez config loading (?)
        """
        raise NotImplementedError("Each sensor can override this if needed")

    @abstractmethod
    def command(input: np.ndarray):
        """
        Every actuator must implement its own command function.
        Output format is sensor-specific.
        """

class VirtualCMG(VirtualActuator):
    """
    Generic virtual Control Moment Gyroscope model for use in simulation
    """
    def __init__(self, cfg_file: Path = None, max_gimbal_rate: float = 1.0, tau: float = 0.05):
        super().__init__(cfg_file)
        self.model: str = "Generic CMG"

        self.max_gimbal_rate =  max_gimbal_rate  # Maximum gimbal rate [rad/s]
        self.tau = tau  # Gimbal time constant [seconds]
        self.gimbal_rate = 0.0 # Current true gimbal rate [rad/s]
        self.gimbal_angle = 0.0 # Current gimbal angle [rad]

    def _load_cfg(self, cfg_file: Path):
        """
        Populate CMG parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the CMG model to be used.

        Returns:
        None
        """
        # TODO: Config file format
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()
        
    def command(self, target_gimbal_rate: np.ndarray, dt: float = 0.01) -> np.ndarray:
        """
        Relates a target gimbal rate to a true gimbal rate via the dynamical equations 
        of the CMG (first-order motor response + saturation limits).

        Arguments:
        target_gimbal_rate: (np.ndarray) Commanded target gimbal rate(s) [rad/s]
        dt:                 (float) Integration timestep size [s]

        Returns:
        np.ndarray: True gimbal rate(s) achieved by the actuator [rad/s]
        """

        target_gimbal_rate = np.clip(target_gimbal_rate, -self.max_gimbal_rate, self.max_gimbal_rate)

        d_rate_dt = (target_gimbal_rate - self.gimbal_rate) / self.tau

        self.gimbal_rate += d_rate_dt * dt
        self.gimbal_rate = np.clip(self.gimbal_rate, -self.max_gimbal_rate, self.max_gimbal_rate)

        self.gimbal_angle += self.gimbal_rate * dt

        return np.atleast_1d(self.gimbal_rate)

    
class VirtualRWL(VirtualActuator):
    """
    Generic virtual Reaction Wheel model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic RWL"

    def _load_cfg(self, cfg_file: Path):
        """
        Populate RWL parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the RWL model to be used.

        Returns:
        None
        """
        # TODO: Config file format
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()
    
    def command(self, input: np.ndarray) -> np.ndarray:
        """
        """


class VirtualMTQ(VirtualActuator):
    """
    Generic magnetorquer class.
    """

    def command(self, input: np.ndarray) -> np.ndarray:
        """Input current, output magnetic field / torque vector in sensor frame"""


