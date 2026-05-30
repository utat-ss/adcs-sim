# Virtual sensor models for use as hardware stubs in simulation

from pathlib import Path
import json
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


from pathlib import Path
import json
import numpy as np


class VirtualSTR:
    """
    Generic virtual star tracker model for use in simulation.

    Config JSON fields:
        model: str
        fov_rad: float
        exclusion_rad: float
        rate_hz: float
        cov_rad: [float, float, float]
    """

    def __init__(self, cfg_file: Path):
        self.model: str = "Generic STR"
        self.fov_rad: float = 0.0
        self.exclusion_rad: float = 0.0
        self.rate_hz: float = 0.0

        # Diagonal measurement covariance for [roll, pitch, yaw] in rad^2
        self.cov_rad: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=float)

        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate STR parameters using a JSON configuration file.
        """
        cfg_file = Path(cfg_file)

        if not cfg_file.exists():
            raise FileNotFoundError(f"STR config file not found: {cfg_file}")

        with open(cfg_file, "r") as f:
            cfg = json.load(f)

        self.model = str(cfg.get("model", self.model))
        self.fov_rad = float(cfg["fov_rad"])
        self.exclusion_rad = float(cfg["exclusion_rad"])
        self.rate_hz = float(cfg["rate_hz"])
        self.cov_rad = np.asarray(cfg["cov_rad"], dtype=float)

    @staticmethod
    def _normalize_quat(q: np.ndarray) -> np.ndarray:
        """
        Normalize quaternion q = [x, y, z, w].
        """
        q = np.asarray(q, dtype=float)

        if q.shape != (4,):
            raise ValueError(f"Quaternion must have shape (4,), got {q.shape}")

        norm = np.linalg.norm(q)
        if norm == 0:
            raise ValueError("Quaternion has zero norm.")

        return q / norm

    @staticmethod
    def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Hamilton product q = q1 ⊗ q2.

        Both quaternions use [x, y, z, w].
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        return np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ], dtype=float)

    @staticmethod
    def _quat_from_rotvec(rotvec_rad: np.ndarray) -> np.ndarray:
        """
        Convert rotation vector to quaternion [x, y, z, w].

        rotvec_rad direction is the rotation axis.
        rotvec_rad magnitude is the rotation angle in radians.
        """
        rotvec_rad = np.asarray(rotvec_rad, dtype=float)

        if rotvec_rad.shape != (3,):
            raise ValueError(
                f"Rotation vector must have shape (3,), got {rotvec_rad.shape}"
            )

        angle = np.linalg.norm(rotvec_rad)

        if angle < 1e-15:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        axis = rotvec_rad / angle
        half_angle = 0.5 * angle

        xyz = axis * np.sin(half_angle)
        w = np.cos(half_angle)

        return np.array([xyz[0], xyz[1], xyz[2], w], dtype=float)

    def measure_attitude(self, q_true: np.ndarray) -> np.ndarray:
        """
        Return noisy measured attitude quaternion.

        Arguments:
            q_true:
                True attitude quaternion [x, y, z, w].

        Returns:
            q_meas:
                Noisy measured attitude quaternion [x, y, z, w].
        """
        q_true = self._normalize_quat(q_true)

        sigma_rad = np.sqrt(self.cov_rad)
        noise_rad = np.random.normal(loc=0.0, scale=sigma_rad, size=3)

        dq = self._quat_from_rotvec(noise_rad)

        q_meas = self._quat_multiply(q_true, dq)

        return self._normalize_quat(q_meas)

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

