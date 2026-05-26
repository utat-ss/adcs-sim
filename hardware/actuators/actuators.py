# Virtual actuator models for use as hardware stubs in simulation

from pathlib import Path

import numpy as np

class VirtualCMG:
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

class VirtualRWL:
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

