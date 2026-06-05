import numpy as np
import json

from sensors.sensors import (VirtualFSS, VirtualSTR,
VirtualSTR, VirtualIMU, VirtualMTM, VirtualGNSS, 
VirtualSensor, VirtualActuator)

class ADCS:
    def __init__(self, id: str, moi: np.ndarray):
        self.id: str = id
        self.moi: np.ndarray = moi
        self.sensors: list[tuple[str, VirtualSensor, np.ndarray]] = []
        self.actuators: list[tuple[str, VirtualActuator, np.ndarray]] = []

    def add_sensor(self, id: str, model: str, dcm: np.ndarray):
        path = model + ".json"

        with open(path, 'r') as file:
            data = json.load(file)
        
        sensor_type = data["type"]

        match sensor_type:
            case "STR":
                 new_sensor = VirtualSTR(path)
            case "FSS":
                 new_sensor = VirtualFSS(path)
            case "IMU":
                 new_sensor = VirtualIMU(path)
            case "MTM":
                 new_sensor = VirtualMTM(path)
            case "GNSS":
                 new_sensor = VirtualGNSS(path)
            case _:
                raise ValueError(f"Unknown sensor type: {sensor_type}")

        self.sensors.append((id, new_sensor, dcm))

    def add_actuator(self, id: str, model: str, dcm: np.ndarray):
        path = model + ".json"

        with open(path, 'r') as file:
            data = json.load(file)
        
        actuator_type = data["type"]

        match actuator_type:
            case "STR":
                 new_actuator = VirtualSTR(id, path)
            case "FSS":
                 new_actuator = VirtualSTR(id, path)
            case "IMU":
                 new_actuator = VirtualSTR(id, path)
            case "MTM":
                 new_actuator = VirtualSTR(id, path)
            case "GNSS":
                 new_actuator = VirtualSTR(id, path)
            case _:
                raise ValueError(f"Unknown actuator type: {actuator_type}")

        self.actuators.append((id, new_actuator, dcm))