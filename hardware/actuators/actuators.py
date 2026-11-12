# Virtual actuator models for use as hardware stubs in simulation

import json
from pathlib import Path
from abc import ABC, abstractmethod
import numpy as np
import json

class VirtualActuator(ABC):
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
    def command(input: np.ndarray):
        """
        Every actuator must implement its own command function.
        Output format is sensor-specific.
        """

class VirtualCMG(VirtualActuator):
    """
    Generic virtual Control Moment Gyroscope model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        self.model: str = "Generic CMG"

    def _load_cfg(self, cfg_file: Path):
        """
        Populate CMG parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the CMG model to be used.

        Returns:
        None
        """
        # TODO: Config file format
        with open(cfg_file, 'r') as f:
            raise NotImplementedError()
        
    def command(self, input: np.ndarray) -> np.ndarray:
        """
        """


class VirtualRWL(VirtualActuator):
    """
    Generic virtual Reaction Wheel model for use in simulation
    """
    def __init__(self, cfg_file: Path):
        super().__init__(cfg_file)

        self.model: str = "Generic RWL"
        self.max_torque_Nm: float = 0.0
        self.max_speed_rad_s: float = 0.0
        self.moment_of_inertia_kg_m2: float = 0.0
        self.tau_s: float = 0.0

        self._load_cfg(cfg_file)

    def _load_cfg(self, cfg_file: Path):
        """
        Populate RWL parameters using a configuration file.

        Arguments:
        cfg_file:   (Path) Path to the configuration file for the RWL model to be used.

        Returns:
        None
        """
        cfg_file = Path(cfg_file)

        if not cfg_file.is_file():
            raise FileNotFoundError(
                f"Configuration file {cfg_file} does not exist."
            )

        with open(cfg_file, "r") as f:
            cfg_data = json.load(f)

        self.model = cfg_data.get("model", self.model)
        self.max_torque_Nm = float(cfg_data["max_torque_Nm"])
        self.max_omega_rad_s = float(cfg_data["max_omega_rad_s"])
        self.moment_of_inertia_kg_m2 = float(cfg_data["moment_of_inertia_kg_m2"])
        self.tau_s = float(cfg_data["tau_s"])

        #Parameter validation 
        
        if not np.isfinite(self.moment_of_inertia_kg_m2) or self.moment_of_inertia_kg_m2 <= 0:
            raise ValueError("Reaction wheel moment of inertia must be positive and finite.")
        if not np.isfinite(self.max_torque_Nm) or self.max_torque_Nm <= 0:
            raise ValueError("Reaction wheel maximum torque must be positive and finite.")
        if not np.isfinite(self.max_omega_rad_s) or self.max_omega_rad_s <= 0:
            raise ValueError("Reaction wheel maximum angular velocity must be positive and finite.")
        if not np.isfinite(self.tau_s) or self.tau_s <= 0:
            raise ValueError("Reaction wheel time constant must be positive and finite.")

    def command(self, target_torque: float, current_state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculate the response of a virtual reaction wheel to a motor
        torque command, subject to torque and rotational speed limits.

        The wheel is modeled as a rigid rotor with a constant moment
        of inertia. The motor torque produces angular acceleration
        according to Newton's second law for rotation.

        Motor torque follows a first-order dynamic response with time
        constant tau_s, approaching the commanded torque gradually.

        For a constant-inertia wheel, limiting angular speed also
        limits angular momentum to I_w * omega_max.

        The spacecraft experiences an equal and opposite reaction
        torque along the wheel's rotation axis.

        The function returns the wheel's state derivative rather than
        updating its state, allowing an external ODE solver to perform
        time integration. However, this implies that this command function 
        does not prevent the wheel from exceeding its maximum speed, since the
        time step is based on the ODE solver's integration step.

        Arguments:
            target_torque: (float)
                Commanded motor torque [N*m]

            current_state: (np.ndarray)
                Current wheel state containing angular velocity [rad/s]
                and actual motor torque [N*m]

        Returns:
            state_derivative: (np.ndarray)
                Wheel angular acceleration [rad/s^2] and motor torque
                rate of change [N*m/s], represented as a two-element array

            spacecraft_torque: (np.ndarray)
                Reaction torque applied to the spacecraft in the
                reaction wheel's local frame [N*m], represented
                as a three-element array
        """
        current_state = np.asarray(current_state)

        # Input Validation
        if current_state.shape != (2,):
            raise ValueError("Current state must contain current wheel speed and motor torque.")

        if np.ndim(target_torque) != 0:
            raise ValueError("Target torque must be a scalar.")

        current_speed, current_torque = current_state

        if not np.isfinite(target_torque) or not np.all(np.isfinite(current_state)):
            raise ValueError("Target torque and current state must be finite.")

        target_torque = float(np.clip(target_torque, -self.max_torque_Nm, self.max_torque_Nm))
        current_torque = float(np.clip(current_torque, -self.max_torque_Nm, self.max_torque_Nm))
        # First-Order Dynamics for Torque
        d_torque_dt = (target_torque - current_torque) / self.tau_s

        if current_speed >= self.max_omega_rad_s and current_torque > 0:
            current_torque = 0.0
        elif current_speed <= -self.max_omega_rad_s and current_torque < 0:
            current_torque = 0.0

        angular_acceleration = (current_torque / self.moment_of_inertia_kg_m2)

        return (np.array([angular_acceleration, d_torque_dt]), np.array([0.0, 0.0, -current_torque])) # Torque applied to spacecraft is equal and opposite to wheel torque along local z-axis of wheel


class VirtualMTQ(VirtualActuator):
    """
    Generic magnetorquer class.
    """

    def command(self, input: np.ndarray) -> np.ndarray:
        """Input current, output magnetic field / torque vector in sensor frame"""


