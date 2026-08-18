import numpy as np

class PID:
    def __init__(self, P_coeff: float = 0.0, I_coeff: float = 0.0, D_coeff: float = 0.0, integration_type: str = "R_rect"):
        """
        Initialize PID controller class. Parameters P_coeff, I_coeff, and D_coeff represent the controller's three
        coefficients, while integration_type is the integration method used ('R_rect' for right-hand rectangular integration, 
        'L_rect' for left-hand rectangular integration, or 'trap' for trapezoidal integration). If a P/PI/PD controller
        is desired, do not initialize the corresponding coefficients with values.
        """
        if integration_type not in ["R_rect", "L_rect", "trap"]:
            raise ValueError("integration_type must be one of 'R_rect', 'L_rect', or 'trap'.")
        
        self.PID_LIST = ["P", "I", "D"]
        self.PID_coeffs = [P_coeff, I_coeff, D_coeff]
        self.status_list = [True, True, True]
        self.integ_type = integration_type

        for i in range(3):
            if round(self.PID_coeffs[i], 12) == 0.0:
                self.status_list[i] = False

    def update_coeffs(self, P_coeff: float = None, I_coeff: float = None, D_coeff: float = None):
        """
        Update PID coefficients to new values P_coeff, I_coeff, and D_coeff.
        """
        if P_coeff != None:
            self.PID_coeffs[0] = P_coeff

        if I_coeff != None:
            self.PID_coeffs[1] = I_coeff

        if D_coeff != None:
            self.PID_coeffs[2] = D_coeff

        for i in range(3):
            if round(self.PID_coeffs[i], 12) == 0.0:
                self.status_list[i] = False
            else:
                self.status_list[i] = True

    def set_mode(self, mode: str):
        """
        Conditionally activate/deactivate the PID controller's modes based on whether characters 'P', 'I', or 'D'
        are present within given string mode. Does not reset coefficients corresponding with deactivated 
        terms to 0.0, enabling straightforward isolation of each term's impact without coefficent re-assignment.
        """
        for i in range(3):
            if self.PID_LIST[i] in mode:
                self.status_list[i] = True
            else:
                self.status_list[i] = False

    def return_coeffs(self) -> tuple:
        """
        Retrieve the controller's current coefficients.
        """
        return tuple(self.PID_coeffs)

    def begin_sim(self, starting_state_vec: np.ndarray, desired_state_vec: np.ndarray, default_timestep: float = 1.0, init_time: float = 0.0):
        """
        Initialize the PID controller with a starting state starting_state_vec, desired state desired_state_vec, 
        default timestep default_timestep, and initial timestamp init_time. Note that the controller
        does not return a signal until after the first step() operation.
        """
        starting_state_vec = np.asarray(starting_state_vec, dtype=float)
        desired_state_vec = np.asarray(desired_state_vec, dtype=float)
        
        self.goal = desired_state_vec
        self.def_timestep = default_timestep
        self.prev_timestep = default_timestep

        self.time = init_time

        self.integrated_error = np.zeros(np.shape(self.goal)[0])
        self.prev_integ_error = self.integrated_error.copy()
        self.error = self.goal - starting_state_vec
        self.prev_error = self.error.copy()

    def reset_integrated_error(self):
        """
        Reset the PID controller's integrated error to be the zero vector.
        """
        self.integrated_error = np.zeros(np.shape(self.goal)[0])
        self.prev_integ_error = self.integrated_error.copy()

    def _integrate_error(self, error: np.ndarray, prev_error: np.ndarray, step_duration: float):
        """
        Helper function which determines the controller's integral response. 
        This method should not be called outside of the class.
        """
        self.prev_integ_error = self.integrated_error.copy()

        if self.integ_type == "R_rect":
            self.integrated_error += error * step_duration
        elif self.integ_type == "L_rect":
            self.integrated_error += prev_error * step_duration
        elif self.integ_type == "trap":
            self.integrated_error += ((error + prev_error) * step_duration) / 2

    def step(self, state_vec: np.ndarray, step_timestamp: float = None) -> np.ndarray:
        """
        Return the PID controller's produced output given system state state_vec, 
        which describes the system after self.def_timestep has elapsed, 
        or at time step_timestamp if a value is provided. If the current step's timestamp
        is the same as the prior step's timestamp, the controller will redo the prior 
        step operation using the newly provided state_vec.
        """
        state_vec = np.asarray(state_vec, dtype=float)

        prev_time = self.time
  
        if step_timestamp == None:
            step_duration = self.def_timestep
        else:
            step_duration = step_timestamp - prev_time

        if round(step_duration, 8) != 0.0:
            self.time += step_duration
            self.prev_timestep = step_duration
            self.prev_error = self.error.copy()
        else:
            step_duration = self.prev_timestep
            self.integrated_error = self.prev_integ_error.copy()

        self.error = self.goal - state_vec
        signal = np.zeros(np.shape(self.goal)[0])

        if self.status_list[0]:
            signal += self.PID_coeffs[0] * self.error

        if self.status_list[1]:
            self._integrate_error(self.error, self.prev_error, step_duration)
            signal += self.PID_coeffs[1] * self.integrated_error

        if self.status_list[2] and round(step_duration, 8) != 0.0:
            deriv_error = (self.error - self.prev_error) / step_duration
            signal += self.PID_coeffs[2] * deriv_error

        return signal
