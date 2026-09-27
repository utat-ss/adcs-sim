# Handle the simulation runtime

from typing import Protocol
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path

@dataclass
class Spacecraft:
    """
    Spacecraft properties
    """
    mass_kg: float
    drag_coefficient: float
    area_m2: float
    
@dataclass
class SimulationConfig:
    """
    Configurations for simulation
    """
    t0: datetime
    tf: datetime
    x0: np.ndarray
    spacecraft: Spacecraft
    perturbations: list[PerturbationModel]
    output_steps: int = 1000

class OrbitPropagator(Protocol):

    def propagate(
        self,
        config: SimulationConfig,
        spacecraft: Spacecraft,
    ) -> tuple[np.ndarray, np.ndarray]:
        ...
    
class PerturbationModel(Protocol):

    def acceleration(
        self,
        t: float,
        state: np.ndarray,
        spacecraft: Spacecraft,
    ) -> np.ndarray:
        ...

def load_sim_cfg(filepath: Path) -> SimulationConfig:
    """
    Load the simulator configuration from a file.
    
    Arguments:
    filepath:       (Path) Path to the simulator configuration file.
    
    Returns:
    SimConfig:      Simulator configuration dataclass.
    """
    raise NotImplementedError()

def load_spacecraft_cfg(filepath: Path) -> Spacecraft:
    """
    Load the spacecraft configuration from a file.
    
    Arguments:
    filepath:       (Path) Path to the spacecraft configuration file.
    
    Returns:
    Spacecraft:     Spacecraft configuration dataclass.
    """
    raise NotImplementedError()

def run_sim(config: SimulationConfig, orbit_propagator: OrbitPropagator) -> np.ndarray:
    return orbit_propagator.propagate(
        config,
        config.spacecraft,
    )
