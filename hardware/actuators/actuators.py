# Virtual actuator models for use as hardware stubs in simulation

from pathlib import Path
from abc import ABC, abstractmethod
import numpy as np
import json as json

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
    def __init__(self, cfg_file: Path):
        super().__init__(cfg_file)

        self.model: str = "Generic CMG"  # Model name
        self.max_gimbal_rate_rad_s: float = 0.0  # Maximum gimbal rate [rad/s]
        self.tau_s: float = 0.0  # Gimbal time constant [s]

        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate CMG parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the CMG model to be used.

        Returns:
        None
        """

        cfg_file = Path(cfg_file)
        if not cfg_file.exists():
            raise FileNotFoundError(f"CMG config file not found: {cfg_file}")
        with open(cfg_file, 'r') as f:
            cfg = json.load(f)
        self.model = cfg.get("model", self.model)
        self.max_gimbal_rate_rad_s = float(cfg["max_gimbal_rate_rad_s"])
        self.tau_s = float(cfg["tau_s"])
        
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


