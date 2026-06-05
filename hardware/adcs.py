from pathlib import Path
import json

import numpy as np

from sensors.sensors import (VirtualFSS, VirtualSTR,
<<<<<<< HEAD
VirtualSTR, VirtualIMU, VirtualMTM, VirtualGNSS, 
VirtualSensor)

from actuators.actuators import (VirtualRWL, VirtualCMG, VirtualActuator)
=======
 VirtualIMU, VirtualMTM, VirtualGNSS, 
VirtualSensor, VirtualActuator)
>>>>>>> 37ae5ec (ADD: ADCS builder safety features)

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
