# Virtual actuator models for use as hardware stubs in simulation

from pathlib import Path
from abc import ABC, abstractmethod
import numpy as np
import json

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
        
    def command(self, target_gimbal_rate: np.ndarray, state: np.ndarray, dt: float = 0.01) -> np.ndarray:
        """
        Relates a target gimbal rate to a true gimbal rate via the dynamical equations 
        of the CMG (first-order motor response + saturation limits).

        Physical Model:
        The gimbal motor is modeled as a first-order system with a time constant tau_s. 
        The true gimbal rate moves toward the target rate at a rate proportional to 
        how far off it currently is, so it approaches the target exponentially instead of
        jumping to it instantly. Both the commanded and resulting true rate are clipped to 
        max_gimbal_rate_rad_s, which is the maximum gimbal rate of the CMG.

        Arguments:
        target_gimbal_rate: (np.ndarray) Commanded target gimbal rate(s) [rad/s]
        state:              (np.ndarray) Current gimbal rate(s) and angle(s) [rad/s, rad]
        dt:                 (float) Integration timestep size [s]

        Returns:
        np.ndarray: Updated [gimbal_rate, gimbal_angle] state [rad/s, rad]
        """

        gimbal_rate, gimbal_angle = state

        target_gimbal_rate = np.clip(target_gimbal_rate, -self.max_gimbal_rate_rad_s, self.max_gimbal_rate_rad_s)

        # Compute the new gimbal rate using a first-order response model  
        d_rate_dt = (target_gimbal_rate - gimbal_rate) / self.tau_s
        new_gimbal_rate = gimbal_rate + d_rate_dt * dt
        new_gimbal_rate = np.clip(new_gimbal_rate, -self.max_gimbal_rate_rad_s, self.max_gimbal_rate_rad_s)

        # Update the gimbal angle based on the new gimbal rate
        new_gimbal_angle = gimbal_angle + new_gimbal_rate * dt

        return np.array([new_gimbal_rate, new_gimbal_angle])

    
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


