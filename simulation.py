# Handle the simulation runtime

from pathlib import Path
from dataclasses import dataclass
from typing import Literal, Union
from orbit import *

@dataclass
class SimConfig:
    """
    Simulation configuration dataclass
    """
    epoch:  str     # Reference date for simulation time
    t0:     float   # Simulation start time [JD]
    tf:     float   # Simulation end time [JD]
    initial_state: InitialStateConfig # Contain initial state in any of the accepted formats


# At the least it should specify the time range of the simulation 
# (preferably something like UTC time or JD) and the initial conditions
# (initial attitude and orbital state vector). Ideally will enable us to 
# toggle things like J2 and aerodynamic drag and to choose an orbit propagation
# method. If you can think of other toggle-able options or selectors, 
# then by all means include them

.
def load_sim_cfg(filepath: Path) -> SimConfig:
    """
    Load the simulator configuration from a file.
    
    Arguments:
    filepath:       (Path) Path to the simulator configuration file.
    
    Returns:
    SimConfig:      Simulator configuration dataclass.
    """
    raise NotImplementedError()

def run_sim(cfg):
    """
    Main runtime function for simulating the dynamics according to the providing configuration.

    Arguments:
    cfg:            (SimConfig) Simulator configuration dataclass.

    Returns:
    bool:           Simulator completion status.
    """

    raise NotImplementedError()
