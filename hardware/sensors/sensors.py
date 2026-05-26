# Virtual sensor models for use as hardware stubs in simulation

from pathlib import Path

import numpy as np

class VirtualFSS:
    """
    Generic virtual Fine Sun Sensor model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic FSS"                         # Model name
        self.fov_deg: float = 0.0                               # Half-cone field of view [deg]
        self.rate_hz: float = 0.0                               # Data rate [Hz]
        self.cov_deg2: np.ndarray = np.array([[0, 0], [0, 0]]) # Measurement covariance (2x2) [deg^2]
        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate FSS parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the FSS model to be used.

        Returns:
        None
        """
        # TODO: Config file format
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()

class VirtualSTR:
    """
    Generic virtual star tracker model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic STR"
        self.fov_deg: float = 0.0
        self.exclusion_deg: float = 0.0
        self.rate_hz: float = 0.0
        self.cov_deg2: np.ndarray = np.array([[0, 0], [0, 0]])
        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate STR parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the STR model to be used.

        Returns:
        None
        """
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()

class VirtualIMU:
    """
    Generic virtual Inertial Measurement Unit model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic IMU"
        self.gyro_lims_rad_s: tuple[float, float] = (0.0, 0.0)
        self.bias_instability_deg_hr: float = 0.0
        self.angle_random_walk_deg_sqrthr: float = 0.0
        self.rate_random_walk_rad_s2_sqrthz: float = 0.0
        self.rate_hz: float = 0.0

    def _load_cfg(self, cfg_file: Path):
        """
        Populate IMU parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the IMU model to be used.

        Returns:
        None
        """
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()

class VirtualMTM:
    """
    Generic virtual Magnetometer model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic MTM"
        self.lims_ut: tuple[float, float] = (0.0, 0.0)
        self.bias_ut: float = 0.0
        self.soft_iron: np.ndarray = np.array([[0.0, 0.0, 0.0],
                                               [0.0, 0.0, 0.0],
                                               [0.0, 0.0, 0.0]])
        self.rate_hz: float = 0.0

    def _load_cfg(self, cfg_file: Path):
        """
        Populate Magnetometer parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the MTM model to be used.

        Returns:
        None
        """
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()

class VirtualGNSS:
    """
    Generic virtual Global Navigation Satellite System model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic GNSS"
        self.cov_m2: np.ndarray = np.array([[0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0]])

    def _load_cfg(self, cfg_file: Path):
        """
        Populate GNSS parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the GNSS model to be used.

        Returns:
        None
        """
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()

