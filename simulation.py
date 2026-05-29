# Handle the simulation runtime

from pathlib import Path
from dataclasses import dataclass

@dataclass
class SimConfig:
    """
    Simulation configuration dataclass
    """
    epoch:  str     # Reference date for simulation time
    t0:     float   # Simulation start time [JD]
    tf:     float   # Simulation end time [JD]

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
