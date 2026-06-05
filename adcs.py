# Handle virtual construction of the Attitude Determination and Control Subsystem

from typing import Self

import numpy as np

class ADCS:
    def __init__(self):
        self.sensor_list = []
        self.actuator_list = []

class ADCSBuilder:
    def __init__(self):
        self.adcs = ADCS()

    def add_sensor(self, s_name: str, dcm: np.ndarray) -> Self:
        self.adcs.sensor_list.append((s_name, dcm))
        return self

    def add_actuator(self, a_name: str, dcm: np.ndarray) -> Self:
        self.adcs.actuator_list.append((a_name, dcm))
        return self

    def build():
        return ADCS()

def _find_hardware_file(filename: str):
        # Search through the project for a file associated with the hardware name
        pass

