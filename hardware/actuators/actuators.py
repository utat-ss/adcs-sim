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
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic CMG"

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
        
    def command(self, input: np.ndarray) -> np.ndarray:
        """
        """

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


