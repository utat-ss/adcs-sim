from pathlib import Path
import json

import numpy as np
import math

from sensors.sensors import (VirtualFSS, VirtualSTR,
VirtualIMU, VirtualMTM, VirtualGNSS, 
VirtualSensor)
from actuators.actuators import (VirtualRWL, VirtualCMG, VirtualActuator)

from ..utils import geometric_calculations as gc
from .. import constants as const

SENSOR_ICD_DIR = Path("./sensors/icd")

class ADCS:
    """
    Virtualization class for capturing ADCS subsystem information.
    """
    def __init__(self, id: str, moi: np.ndarray):
        self.id: str = id
        self.moi: np.ndarray = moi
        self.sensors: list[tuple[str, VirtualSensor, np.ndarray]] = []
        self.actuators: list[tuple[str, VirtualActuator, np.ndarray]] = []

    def _find_config_path(self, model: str) -> Path:
        """
        Searches for an ICD file associated with the specified hardware model.

        Arguments:
        model:              (str) Name of the hardware model.

        Returns:
        cfg_filepath:       (Path) Path to the hardware's ICD.

        Raises:
        FileNotFoundError:  If no ICD is available under the specified name.
        """
        cfg_filename = model.lower().replace('-', '_') + ".json"
        cfg_filepath = next(SENSOR_ICD_DIR.rglob(cfg_filename), None)
        if not cfg_filepath:
            raise FileNotFoundError(f"Could not find ICD for hardware: {model}.")
        
        return cfg_filepath

    def _validate_moi(self, moi: np.ndarray):
        """
        Ensure the specified moment of inertia matrix is physically meaningful.

        Arguments:
        moi:        (np.nddaray) (3x3) Moment of Inertia matrix.

        Returns:
        is_valid:   (bool) Validity of the Moment of Inertia matrix.
        """
        # Default condition
        is_valid = True
        error = 0 # no problem
        
        # Symmetry
        if not np.max(np.abs(moi - moi.T)) < 1e-12:
            is_valid = False
            error = -1 # non-symmetric

        # Positive-definite
        if np.any(np.less_equal(np.linalg.eig(moi), 0)):
            is_valid = False
            error = -2 # not positive-definite

        # Triangle inequality
        principals = np.linalg.diagonal(moi)
        sum_of_principals = np.sum(principals)
        for el in principals:
            if el > sum_of_principals - el:
                is_valid = False
                error = -3 # triangle inequality not satisfied
        
        # Products of inertia bounds
        if not moi[0,0] * moi[1,1] > moi[0,1]**2:
            is_valid = False
            error = -4 # non-physical mass distribution
        if not moi[0,0] * moi[2,2] > moi[0,2]**2:
            is_valid = False
            error = -4
        if not moi[1,1] * moi[2,2] > moi[1,2]**2:
            is_valid = False
            error = -4

        return is_valid, error
    
    @staticmethod
    def eval_eclipse_heuristic(sun_vector: np.ndarray, earth_vector: np.ndarray, earth_radius: float = const.EARTH_RADIUS_EQ_m) -> bool:
        """
        Determine whether the sun is being eclipsed by the Earth using a computationally light
        heuristic. Only usable when the satellite is relatively close to the Earth's surface.
        
        Arguments:

            sun_vector:
                3D sun position vector in arbitrary Cartesian coordinate system.

            earth_vector:
                Earth position vector in arbitrary Cartesian coordinate system.

            earth_radius:
                Radius of the Earth.

        Returns: Boolean value which is True when the sun is being eclipsed.
        """

        sun_vector = np.asarray(sun_vector, dtype=float)
        earth_vector = np.asarray(earth_vector, dtype=float)
        
        if sun_vector.shape != (3,) or earth_vector.shape != (3,):
            raise ValueError("sun_vector and earth_vector must both be 3D vectors.")

        # Returns magnitude of earth_vector's projection onto the plane normal to sun_vector: 
        earth_projection = gc.planar_projection(earth_vector, sun_vector)
        proj_magnitude = np.linalg.norm(earth_projection)

        sun_dot_earth = np.dot(sun_vector, earth_vector)

        if proj_magnitude < earth_radius and sun_dot_earth > 0:
            return True
        return False

    @staticmethod
    def eval_eclipse_fine(sun_vector: np.ndarray, body_vector: np.ndarray, body_radius: float) -> float:
        """
        Determine the extent to which the sun is being eclipsed by another body of a given radius
        in the satellite's reference frame. Assumes that such a body is closer than the sun
        to the satellite.

        Arguments:

            sun_vector:
                3D sun position vector in arbitrary Cartesian coordinate system.

            body_vector:
                3D eclipsing body position vector in arbitrary Cartesian coordinate system.

            body_radius:
                Radius of eclipsing body.

        Returns: Float value ranging from 0.0 (full eclipse) to 1.0 (sun fully visible) representing the 
        approximate fraction of sun visible to a satellite positioned at the origin. This fraction 
        is calculated by determining the linear distance across the sun (relative to the plane 
        defined by the origin, the sun, and the given body) covered by the body in question.
        """

        sun_vector = np.asarray(sun_vector, dtype=float)
        body_vector = np.asarray(body_vector, dtype=float)

        if sun_vector.shape != (3,) or body_vector.shape != (3,):
            raise ValueError("sun_vector and body_vector must both be 3D vectors.")

        plane_normal = np.cross(sun_vector, body_vector)
        if np.allclose(plane_normal, np.zeros(3)): # Addresses case where body is directly in front of sun
            if np.allclose(sun_vector / np.linalg.norm(sun_vector), np.array([1,0,0])):
                plane_normal = np.cross(sun_vector, np.array([0,1,0]))
            else:
                plane_normal = np.cross(sun_vector, np.array([1,0,0]))
                
        # Returns 2D coordinates of the body vector projected onto a plane defined by the sun_vector and itself:
        body_vect_planar = gc.to_planar_vector(sun_vector, body_vector)

        # Determines the two 2D vectors tangent to the circle representing the body considered within the defined planar coordinate system:
        body_tangent_vects_planar = gc.determine_circle_tangent_vectors(body_vect_planar, body_radius)

        # Provides 2D coordinates of the sun in the used planar coordinate system
        sun_vect_planar = np.array([1,0]) * np.linalg.norm(sun_vector)
        
        # Determines the two 2D vectors tangent to the circle representing the sun within the defined planar coordinate system:
        sun_tangent_vects_planar = gc.determine_circle_tangent_vectors(sun_vect_planar, const.SUN_RADIUS_m)

        # Calculate angles between the pairs of tangent vectors
        angle_between_body_tangents = gc.angle_between_vectors(body_tangent_vects_planar[0], body_tangent_vects_planar[1])
        angle_between_sun_tangents = gc.angle_between_vectors(sun_tangent_vects_planar[0], sun_tangent_vects_planar[1])

        # Determine fraction of sun visible
        if gc.eval_vector_between_vectors(sun_vect_planar, body_tangent_vects_planar[0], body_tangent_vects_planar[1]):
            if gc.eval_vector_between_vectors(sun_tangent_vects_planar[0], body_tangent_vects_planar[0], body_tangent_vects_planar[1]) and gc.eval_vector_between_vectors(sun_tangent_vects_planar[1], body_tangent_vects_planar[0], body_tangent_vects_planar[1]):
                return 0.0
            if (not gc.eval_vector_between_vectors(sun_tangent_vects_planar[0], body_tangent_vects_planar[0], body_tangent_vects_planar[1])) and (not gc.eval_vector_between_vectors(sun_tangent_vects_planar[1], body_tangent_vects_planar[0], body_tangent_vects_planar[1])):
                return (angle_between_sun_tangents - angle_between_body_tangents) / angle_between_sun_tangents
            min_angle = min(gc.angle_between_vectors(sun_vect_planar, body_tangent_vects_planar[0]), gc.angle_between_vectors(sun_vect_planar, body_tangent_vects_planar[1]))
            return (angle_between_sun_tangents / 2 - min_angle) / angle_between_sun_tangents
        
        if gc.eval_vector_between_vectors(sun_tangent_vects_planar[0], body_tangent_vects_planar[0], body_tangent_vects_planar[1]) or gc.eval_vector_between_vectors(sun_tangent_vects_planar[1], body_tangent_vects_planar[0], body_tangent_vects_planar[1]):
            if (not gc.eval_vector_between_vectors(sun_tangent_vects_planar[0], body_tangent_vects_planar[0], body_tangent_vects_planar[1])) and (not gc.eval_vector_between_vectors(sun_tangent_vects_planar[1], body_tangent_vects_planar[0], body_tangent_vects_planar[1])):
                return (angle_between_sun_tangents - angle_between_body_tangents) / angle_between_sun_tangents
            min_angle = min(gc.angle_between_vectors(sun_vect_planar, body_tangent_vects_planar[0]), gc.angle_between_vectors(sun_vect_planar, body_tangent_vects_planar[1]))
            return (angle_between_sun_tangents / 2 + min_angle) / angle_between_sun_tangents
        return 1.0

    def add_sensor(self, id: str, model: str, dcm: np.ndarray):
        cfg_file_path = self._find_config_path(model)

        with open(cfg_file_path, 'r') as file:
            data = json.load(file)
        
        sensor_type = data["type"]

        match sensor_type:
            case "STR":
                 new_sensor = VirtualSTR(cfg_file_path)
            case "FSS":
                 new_sensor = VirtualFSS(cfg_file_path)
            case "IMU":
                 new_sensor = VirtualIMU(cfg_file_path)
            case "MTM":
                 new_sensor = VirtualMTM(cfg_file_path)
            case "GNSS":
                 new_sensor = VirtualGNSS(cfg_file_path)
            case _:
                raise ValueError(f"Unknown sensor type: {sensor_type}")

        self.sensors.append((id, new_sensor, dcm))

    def add_actuator(self, id: str, model: str, dcm: np.ndarray):
        path = model + ".json"

        with open(path, 'r') as file:
            data = json.load(file)
        
        actuator_type = data["type"]

        match actuator_type:
            case "CMG":
                 new_actuator = VirtualCMG(path)
            case "RWL":
                 new_actuator = VirtualRWL(path)
            case _:
                raise ValueError(f"Unknown actuator type: {actuator_type}")

        self.actuators.append((id, new_actuator, dcm))
