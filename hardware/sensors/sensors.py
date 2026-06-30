# Virtual sensor models for use as hardware stubs in simulation

from pathlib import Path
import json
import numpy as np
from abc import ABC, abstractmethod

import math

class VirtualSensor(ABC):
    def __init__(self, cfg_file: Path):
        # common fields
        self.cfg_file = cfg_file
        self.model: str = ""

    def load_cfg(self):
        """
        Optional shared helper for standardized config loading.
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

class VirtualFSS(VirtualSensor):
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
        cfg_file = Path(cfg_file)
        if not cfg_file.exists():
            raise FileNotFoundError(f"STR config file not found: {cfg_file}")
        with open(cfg_file, "r") as f:
            cfg = json.load(f)
        self.model = str(cfg.get("model", self.model))
        self.fov_rad = float(cfg["fov_hcone_deg"])
        self.rate_hz = float(cfg["rate_hz"])
        self.cov_deg2 = np.zeros(cfg["cov_deg2"], dtype=float)

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


class VirtualSTR(VirtualSensor):
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
        self.fov_rad = float(cfg["fov_full_cone_rad"])
        self.exclusion_rad = float(cfg["exclusion_full_cone_rad"])
        self.rate_hz = float(cfg["rate_hz"])
        self.cov_rad2 = np.asarray(cfg["cov_matrix_rad2"], dtype=float)
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

    def determine_body_tangent_vectors(self, planar_body_vector: np.ndarray, body_radius: float) -> tuple:
        """to be added"""

        normed_vec = np.linalg.norm(planar_body_vector)
        tangent_dot_body = normed_vec ** 2 - (body_radius ** 2)
        if tangent_dot_body <= 0.0:
            raise ValueError("planar_body_vector must be of a greater magnitude than body_radius.")
        
        temp = math.sqrt(tangent_dot_body) * body_radius
        
        tangent_1_coord_1 = (tangent_dot_body * planar_body_vector[0] + planar_body_vector[1] * temp) / (normed_vec ** 2)
        tangent_1_coord_2 = (tangent_dot_body * planar_body_vector[1] - planar_body_vector[0] * temp) / (normed_vec ** 2)

        tangent_2_coord_1 = (tangent_dot_body * planar_body_vector[0] - planar_body_vector[1] * temp) / (normed_vec ** 2)
        tangent_2_coord_2 = (tangent_dot_body * planar_body_vector[1] + planar_body_vector[0] * temp) / (normed_vec ** 2)

        return (np.array([tangent_1_coord_1, tangent_1_coord_2]), np.array([tangent_2_coord_1, tangent_2_coord_2]))

    def _to_planar_vector(self, sat_vector: np.ndarray, body_vector: np.ndarray) -> np.ndarray:
        """to be added"""
        
        coord_1 = np.dot(sat_vector, body_vector) / np.linalg.norm(sat_vector)
        temp = (np.linalg.norm(body_vector) ** 2) - (coord_1 ** 2)
        if temp < 0.0:
            temp = round(temp, 10)
            if temp < 0.0:
                raise ValueError("Invalid vectors entered into function.")
            
        coord_2 = math.sqrt(temp)

        return np.array([coord_1, coord_2])
    
    def _from_planar_vector(self, sat_vector: np.ndarray, body_vector_3D: np.ndarray, body_vector_2D: np.ndarray) -> np.ndarray:
        """to be added"""
        
        plane_normal = np.cross(body_vector_3D, sat_vector)
        dir_vect_2 = np.cross(sat_vector, plane_normal)

        unit_vect_1 = sat_vector / np.linalg.norm(sat_vector)
        if np.array_equal(dir_vect_2, np.zeros(3)):
            unit_vect_2 = np.zeros(3)
        else:
            unit_vect_2 = dir_vect_2 / np.linalg.norm(dir_vect_2)

        return unit_vect_1 * body_vector_2D[0] + unit_vect_2 * body_vector_2D[1]

    def angle_between_vectors(self, vector_1: np.ndarray, vector_2: np.ndarray) -> float:
        """to be added"""

        dot_p = np.dot(vector_1, vector_2)
        norm_1 = np.linalg.norm(vector_1)
        norm_2 = np.linalg.norm(vector_2)
        if norm_1 == 0.0 or norm_2 == 0.0:
            raise ValueError("Both vector_1 and vector_2 must have nonzero magnitude.")

        cos_theta = dot_p / (norm_1 * norm_2)
        vector_angle_rad = math.acos(cos_theta)
        return vector_angle_rad

    def eval_body_in_zone(self, sat_vector: np.ndarray, body_vector: np.ndarray, body_name: str) -> bool:
        """to be added"""

        if sat_vector.shape != (3,) or np.linalg.norm(sat_vector) == 0:
            raise ValueError("sat_vector must be a nonzero 3D vector.")
        
        if body_vector.shape != (3,) or np.linalg.norm(body_vector) == 0:
            raise ValueError("body_vector must be a nonzero 3D vector.")
        
        if body_name not in ["sun", "moon", "earth"]:
            raise ValueError("body_name must be one of 'sun', 'moon', or 'earth'.")

        zone_angle_rad = 0.0
        body_radius = 1.0

        if body_name == "sun":
            zone_angle_rad = self.exclusion_rad / 2
            body_radius = 695700000.0 # should be moved eventually to constants file
        elif body_name == "moon":
            zone_angle_rad = self.fov_rad / 2
            body_radius = 1737400.0
        elif body_name == "earth":
            zone_angle_rad = self.fov_rad / 2
            body_radius =  6378137.0 # equatorial radius (class may give false negative near poles)

        body_angle = self.angle_between_vectors(sat_vector, body_vector)

        body_vector_2D = self._to_planar_vector(sat_vector, body_vector)
        tangent_vectors_2D = self.determine_body_tangent_vectors(body_vector_2D, body_radius)
        tangent_vectors_3D = (self._from_planar_vector(sat_vector, body_vector, tangent_vectors_2D[0]), self._from_planar_vector(sat_vector, body_vector, tangent_vectors_2D[1]))
        tangent_angles = (self.angle_between_vectors(sat_vector, tangent_vectors_3D[0]), self.angle_between_vectors(sat_vector, tangent_vectors_3D[1]))

        if body_angle <= zone_angle_rad:
            # print("0")
            return True
        elif tangent_angles[0] <= zone_angle_rad:
            # print("1")
            return True
        elif tangent_angles[1] <= zone_angle_rad:
            # print("2")
            return True
        elif body_angle < math.pi / 2: # double check accuracy
            if tangent_vectors_2D[0][1] > 0 and tangent_vectors_2D[1][1] < 0:
                # print("3")
                return True
            elif tangent_vectors_2D[0][1] < 0 and tangent_vectors_2D[1][1] > 0:
                # print("4")
                return True

        return False

    def measure(self, q_true: np.ndarray, sun_vector: np.ndarray, moon_vector: np.ndarray, earth_vector: np.ndarray) -> np.ndarray:
        """
        Return noisy measured attitude quaternion.

        Arguments:
            q_true:
                True attitude quaternion [x, y, z, w].

        Returns:
            q_meas:
                Noisy measured attitude quaternion [x, y, z, w].
        """

        if self.eval_body_in_zone(q_true[0:3], sun_vector, "sun"):
            return np.zeros(4)
        elif self.eval_body_in_zone(q_true[0:3], moon_vector, "moon"):
            return np.zeros(4)
        elif self.eval_body_in_zone(q_true[0:3], earth_vector, "earth"):
            return np.zeros(4)
        
        # TODO: Implement eclipse functionality

        q_true = self._normalize_quat(q_true)

        noise_rad = np.random.multivariate_normal(np.zeros(3), self.cov_rad2)

        dq = self._quat_from_rotvec(noise_rad)

        q_meas = self._quat_multiply(q_true, dq)

        return self._normalize_quat(q_meas)


class VirtualIMU(VirtualSensor):
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


class VirtualMTM(VirtualSensor):
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


class VirtualGNSS(VirtualSensor):
    """
    Generic virtual Global Navigation Satellite System model for use in simulation.
    """

    def __init__(self, cfg_file: Path):
        self.type: str = "GNSS"
        self.model: str = "Generic GNSS"
        self.compatible_satellites: str = ""

        # Measurement covariance matrix for [latitude, longitude, altitude] in meters
        self.LLA_cov_matrix_meters: np.ndarray = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate GNSS parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the GNSS model to be used.

        Returns:
        None
        """
        cfg_file = Path(cfg_file)

        if not cfg_file.exists():
            raise FileNotFoundError(f"REC config file not found: {cfg_file}")

        with open(cfg_file, "r") as f:
            cfg = json.load(f)

        self.model = str(cfg.get("model", self.model))
        self.compatible_satellites = str(cfg["compatible_satellites"])
        self.LLA_cov_matrix_meters = np.asarray(cfg["LLA_cov_matrix_meters"], dtype=float)

    @staticmethod
    def from_geodetic_LLA(input_coords, semi_major = 6378137.0, semi_minor = 6356752.314245):
        """
        Convert input coordinates from geodetic LLA (geodetic latitude, longitude, altitude) to geocentric LLA
        with distance from Earth's center as the third coordinate (geocentric latitude, longitude, distance to Earth center).
        Uses values for the semi-major and semi-minor axes of Earth's ellipsoid.
        """
        conv_factor = (semi_major ** 2) / (semi_minor ** 2)
        geocentric_lat = math.atan(1/conv_factor * math.tan(input_coords[0] * math.pi / 180)) * 180 / math.pi

        ellipsoid_height = semi_major * semi_minor / (math.sqrt(semi_minor**2 + (semi_major**2 - semi_minor**2) * (math.sin(geocentric_lat * math.pi / 180))**2))
        dist_from_center = input_coords[2] + ellipsoid_height 

        return (geocentric_lat, input_coords[1], dist_from_center)
    
    @staticmethod
    def to_geodetic_LLA(input_coords, semi_major = 6378137.0, semi_minor = 6356752.314245):
        """
        Convert input coordinates to geodetic LLA (geodetic latitude, longitude, altitude) from geocentric LLA
        with distance from Earth's center as the third coordinate (geocentric latitude, longitude, distance to Earth center).
        Uses values for the semi-major and semi-minor axes of Earth's ellipsoid.
        """
        conv_factor = (semi_major ** 2) / (semi_minor ** 2)
        geodetic_lat = math.atan(conv_factor * math.tan(input_coords[0] * math.pi / 180)) * 180 / math.pi

        ellipsoid_height = semi_major * semi_minor / (math.sqrt(semi_minor**2 + (semi_major**2 - semi_minor**2) * (math.sin(input_coords[0] * math.pi / 180))**2))
        actual_alt = input_coords[2] - ellipsoid_height 

        return (geodetic_lat, input_coords[1], actual_alt)

    def measure(self, true_coords_LLA):
        """
        Compute REC noisy positon output in LLA from input LLA coordinates.

        Note 1: contains conversion from standard LLA (geodetic latitude, longitude, altitude)
        to modified LLA (geocentric latitude (meters), longitude (meters), distance to Earth center (meters)) and back to use known
        covariance values.

        Note 2: should be accurate (in providing values with the desired randomness) for all valid positions
        except within ~2 m of either pole, where values are more concentrated than desired (may be fixed in future version).

        Arguments:

            :param true_coords_LLA:
                True, standard LLA (geodetic latitude, longitude, altitude) coordinates of satellite.
                Values must be in the range -90 <= latitude <= 90, -180 <= longitude <= 180, and altitude > 0.

        Returns:
            Noisy, standard LLA (geodetic latitude, longitude, altitude) coordinates of satellite. 
            Values should be in the range -90 <= latitude <= 90, -180 < longitude <= 180, and altitude > 0.
        """
        if len(true_coords_LLA) != 3:
            raise ValueError("true_coords_LLA must be a list of length 3.")
        
        if true_coords_LLA[0] < -90.0 or true_coords_LLA[0] > 90.0:
            raise ValueError("Latitude value must be in the range -90 <= latitude <= 90.")
        
        if true_coords_LLA[1] < -180.0 or true_coords_LLA[1] > 180.0:
            raise ValueError("Longitude value must be in the range -180 <= longitude <= 180.")
        
        if true_coords_LLA[2] <= 0.0:
            raise ValueError("Altitude value must be in the range altitude > 0.")
        
        true_coords_mod = self.from_geodetic_LLA(true_coords_LLA)

        true_lat_rad = true_coords_mod[0] * math.pi / 180
        true_long_rad = true_coords_mod[1] * math.pi / 180
        true_dist = true_coords_mod[2]

        lat_dist_m = true_lat_rad * true_dist
        long_dist_m = true_long_rad * true_dist * math.cos(true_lat_rad)

        noise = np.random.multivariate_normal(np.zeros(3), self.LLA_cov_matrix_meters)

        noisy_lat_dist_m = lat_dist_m + noise[0]
        noisy_long_dist_m = long_dist_m + noise[1]
        noisy_dist = true_dist + noise[2]

        noisy_coords_mod = np.zeros(3)
        noisy_lat_rad = noisy_lat_dist_m / true_dist
        noisy_coords_mod[0] = (180 * noisy_lat_rad) / math.pi

        buff = 0.00000001
        if true_lat_rad < math.pi/2 - buff and true_lat_rad > -math.pi/2 + buff: # Handling edge case
            true_cos_val = math.cos(true_lat_rad)
            noisy_coords_mod[1] = (180 * noisy_long_dist_m) / (math.pi * true_dist * true_cos_val) 
        else:
            noisy_coords_mod[1] = 0.0
        noisy_coords_mod[2] = noisy_dist

        # Ensuring output is within intended range

        if noisy_coords_mod[0] > 90.0:
            noisy_coords_mod[0] = 180.0 - noisy_coords_mod[0]
            noisy_coords_mod[1] += 180.0

        if noisy_coords_mod[0] < -90.0:
            noisy_coords_mod[0] = -180.0 - noisy_coords_mod[0]
            noisy_coords_mod[1] += 180.0
            
        while noisy_coords_mod[1] > 180.0:
            noisy_coords_mod[1] -= 360.0
        
        while noisy_coords_mod[1] <= -180.0:
            noisy_coords_mod[1] += 360.0
            
        return self.to_geodetic_LLA(noisy_coords_mod)
