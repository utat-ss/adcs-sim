# Virtual sensor models for use as hardware stubs in simulation

from pathlib import Path
import json
import numpy as np
from abc import ABC, abstractmethod

class VirtualSensor(ABC):
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
    def measure(self, *args, **kwargs):
        """
        Every sensor must implement its own measurement function.
        Output format is sensor-specific.
        """
        pass


# ----- Sensor specific subclasses that inherit from VirtualSensor are below ----

class VirtualFSS:
    """
    Generic virtual Fine Sun Sensor model for use in simulation
    """

    def __init__(self, cfg_file: Path):
        self.model: str = "Generic FSS"  # Model name
        self.fov_deg: float = 0.0  # Half-cone field of view [deg]
        self.rate_hz: float = 0.0  # Data rate [Hz]
        self.cov_deg2: np.ndarray = np.zeros((2, 2), dtype=float)  # Measurement covariance (2x2) [deg^2]
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

    def _angle_computation(self, incident_light_sensor: np.ndarray):
        """
        Compute FSS angular output from an incident light vector in sensor frame.

        Sensor-frame convention:
            +z axis points along the sensor boresight.
            +x direction gives positive alpha.
            +y direction gives positive beta.

        Angle definition:
            alpha = atan2(I_s,x, I_s,z)
            beta  = atan2(I_s,y, I_s,z)

        Arguments:
            incident_light_sensor:
                3D incident light direction vector expressed in sensor frame.

        Returns:
            dict with:
                alpha_deg: signed x-z plane angle [deg]
                beta_deg: signed y-z plane angle [deg]
                sun_present: True if light is inside FOV
        """
        incident_light_sensor = np.asarray(incident_light_sensor, dtype=float)
        if incident_light_sensor.shape != (3,):
            raise ValueError("incident_light_sensor must be a 3D vector.")

        if np.linalg.norm(incident_light_sensor) == 0:
            return {
                "alpha_deg": 0,
                "beta_deg": 0,
                "sun_present": False,
            }

        alpha_rad = np.arctan2(incident_light_sensor[0], incident_light_sensor[2])
        beta_rad = np.arctan2(incident_light_sensor[1], incident_light_sensor[2])

        alpha_deg = np.rad2deg(alpha_rad)
        beta_deg = np.rad2deg(beta_rad)

        inside_fov = (
                abs(alpha_deg) <= self.fov_deg
                and abs(beta_deg) <= self.fov_deg
                and incident_light_sensor[2] > 0.0
        )

        noise = np.random.multivariate_normal(mean=np.zeros(2), cov=self.cov_deg2)
        # TODO: self.cov_deg2 must follow the same ordering as the measurement vector:
        #       [alpha_deg, beta_deg].
        #       cov_deg2[0, 0] is Var(alpha), cov_deg2[1, 1] is Var(beta),
        #       and cov_deg2[0, 1] / cov_deg2[1, 0] is Cov(alpha, beta).
        alpha_noise = noise[0]
        beta_noise = noise[1]

        alpha_deg += alpha_noise
        beta_deg += beta_noise

        return {
            "alpha_deg": alpha_deg,
            "beta_deg": beta_deg,
            "sun_present": inside_fov,
        }

    def measure(self, sun_vec_body, R_BS, r_mount=None):
        """
        Compute FSS angular output from Sun and satellite position vectors.

        This method assumes:
            incident_light = sun_pos - r_mount (optional)

        Then the sensor mounting matrix is applied:
            I_s = R_BS @ I_b

        Arguments:

            :param sun_vec_body:
                3D Sun position vector in the body frame.

            :param R_BS:
                External mounting matrix turns from body frame to sensor frame

            :param r_mount: mounting position, usually negligible.

        Returns:
            Same dict as measure_from_sensor_vector.
        """
        if r_mount is None:
            r_mount = np.zeros(3)
        else:
            r_mount = np.asarray(r_mount, dtype=float)

        sun_vec_body = np.asarray(sun_vec_body, dtype=float)
        if sun_vec_body.shape != (3,) or r_mount.shape != (3,):
            raise ValueError("sun_pos and r_mount must both be 3D vectors.")

        R_BS = np.asarray(R_BS, dtype=float)
        if R_BS.shape != (3, 3):
            raise ValueError("R_BS must be a 3x3 matrix.")

        incident_light_body = sun_vec_body - r_mount
        incident_light_sensor = R_BS @ incident_light_body

        return self._angle_computation(incident_light_sensor)


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
        cov_diag: [float, float, float]
    """

    def __init__(self, cfg_file: Path):
        self.type: str = "STR"
        self.model: str = "Generic STR"
        self.fov_rad: float = 0.0
        self.exclusion_rad: float = 0.0
        self.rate_hz: float = 0.0

        # Measurement covariance matrix for [roll, pitch, yaw] in rad^2
        self.cov_rad2: np.ndarray = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
        self.cov_diag: np.array = np.array([0.0, 0.0, 0.0], dtype=float)
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
        self.cov_rad2 = np.asarray(cfg["cov_rad2"], dtype=float)
        self.cov_diag = np.diag(self.cov_rad2)

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

        sigma_rad = np.sqrt(np.diag(self.cov_diag))
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
