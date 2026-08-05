# Virtual sensor models for use as hardware stubs in simulation

from pathlib import Path
import json
import numpy as np
from abc import ABC, abstractmethod

import math
from ...utils import geometric_calculations as gc
from ...utils import quaternion_math as quat
from ... import constants as const

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
        self.fov_deg: float = 0.0 # Half-cone field of view [deg]
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
        self.fov_deg = float(cfg["fov_hcone_deg"])
        self.rate_hz = float(cfg["rate_hz"])
        self.cov_deg2 = np.asarray(cfg["cov_deg2"], dtype=float)

    def _angle_computation(self, incident_light_sensor: np.ndarray) -> tuple:
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
            Tuple (alpha_deg, beta_deg) where:
                alpha_deg: signed x-z plane angle [deg]
                beta_deg: signed y-z plane angle [deg]
        """
        incident_light_sensor = np.asarray(incident_light_sensor, dtype=float)
        if incident_light_sensor.shape != (3,) or np.linalg.norm(incident_light_sensor) == 0:
            raise ValueError("incident_light_sensor must be a nonzero 3D vector.")

        alpha_rad = np.arctan2(incident_light_sensor[0], incident_light_sensor[2])
        beta_rad = np.arctan2(incident_light_sensor[1], incident_light_sensor[2])

        alpha_deg = np.rad2deg(alpha_rad)
        beta_deg = np.rad2deg(beta_rad)

        noise = np.random.multivariate_normal(mean=np.zeros(2), cov=self.cov_deg2)
        alpha_noise = noise[0]
        beta_noise = noise[1]

        alpha_deg += alpha_noise
        beta_deg += beta_noise

        return (alpha_deg, beta_deg)
    
    def _angle_computation_general(self, sun_vector: np.ndarray, alpha_vector: np.ndarray, beta_vector: np.ndarray) -> tuple:
        """
        Compute FSS angular output from given sun_vector in any arbitrary 
        coordinate system (defined by alpha_vector and beta_vector).

        Note: boresight_vector = alpha_vector x beta_vector is the direction vector of the FSS boresight.

        Arguments:
            sun_vector:
                3D sun direction vector.

            alpha_vector:
                3D direction vector indicating the direction of positive alpha angles.

            beta_vector:
                3D direction vector indicating the direction of positive beta angles.

        Returns:
            Tuple (alpha_deg, beta_deg) where:
                alpha_deg: signed angle between projection of sun_vector onto beta plane and boresight_vector [deg]
                beta_deg: signed angle between projection of sun_vector onto alpha plane and boresight_vector [deg]
        """
        if round(np.dot(alpha_vector, beta_vector), 10) != 0.0:
            raise ValueError("Direction vectors for positive alpha and positive beta should be perpendicular.")
        
        boresight_vector = np.cross(alpha_vector, beta_vector)
        proj_sun_alpha = gc.planar_projection(sun_vector, beta_vector)
        proj_sun_beta = gc.planar_projection(sun_vector, alpha_vector)

        alpha_rad = 0.0
        beta_rad = 0.0
        
        if not np.allclose(proj_sun_alpha, np.zeros(3)):
            alpha_rad = gc.angle_between_vectors(boresight_vector, proj_sun_alpha)
        
        if not np.allclose(proj_sun_beta, np.zeros(3)):
            beta_rad = gc.angle_between_vectors(boresight_vector, proj_sun_beta)

        if np.dot(alpha_vector, proj_sun_alpha) < 0:
            alpha_rad *= -1

        if np.dot(beta_vector, proj_sun_beta) < 0:
            beta_rad *= -1

        alpha_deg = np.rad2deg(alpha_rad)
        beta_deg = np.rad2deg(beta_rad)

        noise = np.random.multivariate_normal(mean=np.zeros(2), cov=self.cov_deg2)
        alpha_noise = noise[0]
        beta_noise = noise[1]

        alpha_deg += alpha_noise
        beta_deg += beta_noise
        return (alpha_deg, beta_deg)

    def measure(self, attitude_quat: np.ndarray, sun_vector: np.ndarray, sun_visibility: float, 
                offset_quat: np.ndarray = np.array([0,0,0,1])) -> dict:
        """
        Compute FSS angular output from coordinate transformation quaternions as well as sun, alpha, and beta vectors.
        The sun vector provided should be in a satellite-fixed non-rotating reference frame, with attitude_quat transforming 
        between this frame and a frame rotating with the satellite (where the unit vector [1, 0, 0] 
        maps to the direction of the STR's boresight). Quaternion offset_quat then transforms between this
        coordinate system to an FSS-specific system in the same frame of reference (where the unit vectors [0, 1, 0] 
        and [0, 0, 1] map to the direction of the FSS's alpha and beta axes respectively by default).

        Arguments:

            attitude_quat: 
                Attitude quaternion [x, y, z, w] representing the satellite's current orientation.
            
            sun_vector:
                3D sun position vector.

            sun_visibility:
                Float between 0.0 (full eclipse) to 1.0 (sun fully visible) representing the 
                approximate fraction of sun visible to a satellite positioned at the origin.

            offset_quat:
                Quaternion [x, y, z, w] transforming from general rotating satellite coordinate system
                to FSS-specific system. 

        Returns:
            Dictionary containing {
                "alpha_deg" : signed angle between projection of sun_vector onto beta plane and boresight_vector [deg]
                "beta_deg" : signed angle between projection of sun_vector onto alpha plane and boresight_vector [deg]
                "sun_present" : True if sun is inside FOV and not in eclipse.
            }
        """
        attitude_quat = np.asarray(attitude_quat, dtype=float)
        sun_vector = np.asarray(sun_vector, dtype=float)
        offset_quat = np.asarray(offset_quat, dtype=float)

        if attitude_quat.shape != (4,) or np.linalg.norm(attitude_quat) == 0.0:
            raise ValueError("attitude_quat must be a quaternion with nonzero norm.")
        
        if sun_vector.shape != (3,) or np.linalg.norm(sun_vector) == 0.0:
            raise ValueError("sun_vector must be a nonzero 3D vector.")
        
        if offset_quat.shape != (4,) or np.linalg.norm(offset_quat) == 0.0:
            raise ValueError("offset_quat must be a quaternion with nonzero norm.")

        attitude_quat = quat.normalize_quat(attitude_quat)
        offset_quat = quat.normalize_quat(offset_quat)

        # Default reference axes in FSS coordinate system, which can be changed if desired:
        alpha_vector = np.array([0,1,0])
        beta_vector = np.array([0,0,1])

        no_sun_dict = {
            "alpha_deg": 0.0,
            "beta_deg": 0.0,
            "sun_present": False,
        }

        sun_tolerance = 0.15 # Use more relevant value once known

        if sun_visibility < sun_tolerance:
            return no_sun_dict

        transformation_quat = quat.quat_multiply(attitude_quat, offset_quat)

        FSS_frame_sun_vector = quat.transform_vect_coord_system(sun_vector, transformation_quat)
        
        ab_angles_deg = self._angle_computation_general(FSS_frame_sun_vector, alpha_vector, beta_vector)

        inside_fov = (
            abs(ab_angles_deg[0]) <= self.fov_deg
            and abs(ab_angles_deg[1]) <= self.fov_deg
            and np.dot(np.cross(alpha_vector, beta_vector), FSS_frame_sun_vector) >= 0
        )

        if not inside_fov:
            return no_sun_dict

        return {
            "alpha_deg": ab_angles_deg[0],
            "beta_deg": ab_angles_deg[1],
            "sun_present": True,
        }


class VirtualSTR(VirtualSensor):
    """
    Generic virtual star tracker model for use in simulation.

    Config JSON fields:
        type: str
        model: str
        fov_full_cone_rad: float
        exclusion_full_cone_rad: float
        max_sensor_update_rate_hz: float
        max_total_body_rate_rad/s: float
        cov_matrix_rad2: [[float, float, float], [float, float, float], [float, float, float]]
    """

    def __init__(self, cfg_file: Path):
        self.type: str = "STR"
        self.model: str = "Generic STR"
        self.fov_rad: float = 0.0
        self.exclusion_rad: float = 0.0
        self.max_update_rate_hz: float = 0.0
        self.max_body_rate_radhz: float = 0.0

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
        self.max_update_rate_hz = float(cfg["max_sensor_update_rate_hz"])
        self.max_body_rate_radhz = float(cfg["max_total_body_rate_rad/s"]) # With Sagitta, should be ~2.5 deg/s
        self.cov_rad2 = np.asarray(cfg["cov_matrix_rad2"], dtype=float)
        self.cov_diag = np.diag(self.cov_rad2)

    def eval_body_in_zone(self, sat_vector: np.ndarray, body_vector: np.ndarray, body_radius: float, fov_type: str = "regular") -> bool:
        """
        Determine whether a given planetary body is within a given conic field of view.
        Both vector arguments can be in any shared reference frame.

        Arguments: 
            sat_vector: 
                3D direction vector of STR boresight.

            body_vector: 
                3D stellar body position vector.

            body_radius:
                Radius of stellar body.

            fov_type:
                Type of FOV considered (either 'regular' or 'exclusion').

        Returns: Boolean value which is True when the given stellar body is in the given FOV.
        """

        if sat_vector.shape != (3,) or np.linalg.norm(sat_vector) == 0:
            raise ValueError("sat_vector must be a nonzero 3D vector.")
        
        if body_vector.shape != (3,) or np.linalg.norm(body_vector) == 0:
            raise ValueError("body_vector must be a nonzero 3D vector.")
  
        if fov_type not in ["regular", "exclusion"]:
            raise ValueError("fov_type must be either 'regular' or 'exclusion'.")

        zone_angle_rad = 0.0
        if fov_type == "regular":
            zone_angle_rad = self.fov_rad / 2
        elif fov_type == "exclusion":
            zone_angle_rad = self.exclusion_rad / 2

        # Determines the angle between the stellar body and boresight vectors:
        body_angle = gc.angle_between_vectors(sat_vector, body_vector) 

        # Returns 2D coordinates of the body vector projected onto a plane defined by the boresight and itself:
        body_vector_2D = gc.to_planar_vector(sat_vector, body_vector) 
        
        # Determines the two 2D vectors tangent to the circle representing the body considered within the defined planar coordinate system:
        tangent_vectors_2D = gc.determine_circle_tangent_vectors(body_vector_2D, body_radius) 
        
        # Converts the two 2D tangent vectors back into regular 3D Cartesian coordinates:
        tangent_vectors_3D = (gc.from_planar_vector(sat_vector, body_vector, tangent_vectors_2D[0]), gc.from_planar_vector(sat_vector, body_vector, tangent_vectors_2D[1])) 
        
        # Determines the angles between the each tangent vector and the boresight vector:
        tangent_angles = (gc.angle_between_vectors(sat_vector, tangent_vectors_3D[0]), gc.angle_between_vectors(sat_vector, tangent_vectors_3D[1])) 

        inside_zone = (
            body_angle <= zone_angle_rad 
            or tangent_angles[0] <= zone_angle_rad
            or tangent_angles[1] <= zone_angle_rad
            or (body_angle < math.pi / 2 and (tangent_vectors_2D[0][1] > 0 and tangent_vectors_2D[1][1] < 0))
            or (body_angle < math.pi / 2 and (tangent_vectors_2D[0][1] < 0 and tangent_vectors_2D[1][1] > 0))
        )

        if inside_zone:
            return True
        return False
    
    def eval_rate_exceeded(self, body_rates: np.ndarray) -> bool:
        """
        Determine whether the maximum total body rate at which the STR can make measurements has been exceeded.

        Arguments: 
            body_rates:
                Current body rates [x_rate, y_rate, z_rate] along each axis in rad/s, 
                relative to a satellite-fixed non-rotating reference frame.

        Returns: Boolean value which is True when the current body rate is greater than the known maximum.
        """
        max_rate = self.max_body_rate_radhz
        body_rate_vec_sum = np.linalg.norm(body_rates)
        
        if body_rate_vec_sum > max_rate:
            return True
        return False

    def measure(self, q_true: np.ndarray, body_rates: np.ndarray, sun_vector: np.ndarray, 
                earth_vector: np.ndarray, moon_vector: np.ndarray) -> dict:
        """
        Return noisy measured attitude quaternion. All vector arguments should be in a satellite-fixed 
        non-rotating reference frame, with q_true transforming between this frame and a frame rotating
        with the satellite (where the unit vector [1, 0, 0] maps to the direction of the STR's boresight).

        Arguments:
            q_true:
                True attitude quaternion [x, y, z, w].

            body_rates:
                Current body rates [x_rate, y_rate, z_rate] along each axis in rad/s, 
                relative to the satellite-fixed non-rotating reference frame.

            sun_vector:
                3D sun position vector.

            earth_vector:
                3D Earth position vector.

            moon_vector:
                3D moon position vector.

        Returns:
            Dictionary containing {
                "q_meas" : Noisy measured attitude quaternion [x, y, z, w]. If any stellar bodies are present, 
                or the maximum body rate is exceeded, q_meas is equal to the identity quaternion [0, 0, 0, 1].
                "rate_exceeded" : Boolean value, True if the STR's maximum allowable body rate has been exceeded.
                "sun_in_exclusion" : Boolean value, True if the sun is in the STR's exclusion zone.
                "earth_in_fov" : Boolean value, True if the Earth is in the STR's FOV.
                "moon_in_fov" : Boolean value, True if the moon is in the STR's FOV.
            }
        """
        q_true = np.asarray(q_true, dtype=float)
        body_rates = np.asarray(body_rates, dtype=float)

        if q_true.shape != (4,) or np.linalg.norm(q_true) == 0.0:
            raise ValueError("q_true must be a quaternion with nonzero norm.")
        
        if body_rates.shape != (3,):
            raise ValueError("body_rates must have shape (3,).")

        sun_vector = np.asarray(sun_vector, dtype=float)
        earth_vector = np.asarray(earth_vector, dtype=float)
        moon_vector = np.asarray(moon_vector, dtype=float)

        if sun_vector.shape != (3,) or np.linalg.norm(sun_vector) == 0.0:
            raise ValueError("sun_vector must be a nonzero 3D vector.")
        
        if earth_vector.shape != (3,) or np.linalg.norm(earth_vector) == 0.0:
            raise ValueError("earth_vector must be a nonzero 3D vector.")
        
        if moon_vector.shape != (3,) or np.linalg.norm(moon_vector) == 0.0:
            raise ValueError("moon_vector must be a nonzero 3D vector.")
        
        q_true = quat.normalize_quat(q_true)

        noise_rad = np.random.multivariate_normal(np.zeros(3), self.cov_rad2)

        dq = quat.quat_from_rotvec(noise_rad)

        q_meas = quat.quat_multiply(q_true, dq)

        return_dict = {
            "q_meas" : quat.normalize_quat(q_meas),
            "sun_in_exclusion" : False,
            "earth_in_fov" : False,
            "moon_in_fov" : False,
            "rate_exceeded": False
        }

        false_reading = np.array([0,0,0,1])
        boresight_vector = np.array([1,0,0])

        fixed_frame_boresight_vector = quat.transform_vect_coord_system(boresight_vector, quat.quat_conjugate(q_true))

        if self.eval_rate_exceeded(body_rates):
            return_dict["rate_exceeded"] = True
            return_dict["q_meas"] = false_reading
        if self.eval_body_in_zone(fixed_frame_boresight_vector, sun_vector, const.SUN_RADIUS_m, "exclusion"):
            return_dict["sun_in_exclusion"] = True
            return_dict["q_meas"] = false_reading
        if self.eval_body_in_zone(fixed_frame_boresight_vector, earth_vector, const.EARTH_RADIUS_EQ_m, "regular"): # earth equatorial radius (class may give false positive near poles)
            return_dict["earth_in_fov"] = True
            return_dict["q_meas"] = false_reading
        if self.eval_body_in_zone(fixed_frame_boresight_vector, moon_vector, const.MOON_RADIUS_m, "regular"):
            return_dict["moon_in_fov"] = True
            return_dict["q_meas"] = false_reading

        return return_dict


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
    def from_geodetic_LLA(input_coords: tuple, semi_major: float = const.EARTH_RADIUS_EQ_m, semi_minor: float = const.EARTH_RADIUS_POLAR_m):
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
    def to_geodetic_LLA(input_coords: tuple, semi_major: float = const.EARTH_RADIUS_EQ_m, semi_minor: float = const.EARTH_RADIUS_POLAR_m):
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

    def measure(self, true_coords_LLA: tuple):
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
