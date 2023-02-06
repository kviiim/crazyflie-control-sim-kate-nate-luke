import numpy as np

from utils import State

class StateEstimator1D():
    """
    This class estimates the state (position and velocity) of the system based 
    on noisy sensor measurements using a kalman filter

    You are to implement the "compute" method.
    """
    def __init__(self, params, init_state):
        """
        Inputs:
        - params (CrazyflieParams dataclass):       model parameter class for the crazyflie
        - init_state (State dataclass):             initial state of the system
        """
        self.params = params
        self.pos = init_state.z_pos
        self.pos_var = 0.01 

        self.vel = init_state.z_vel
        self.vel_var = 0.01

        #NUMBERS? HOW TO CHOOSE?
        self.pos_sensor_var = .2
        self.pos_process_var = 0.01
        self.vel_sensor_var = .2
        self.vel_process_var = 0.01

    def compute(self, z_meas, U, time_delta):
        """
        Estimates the current system state given the noisy measurement and control input

        Inputs:
        - z_meas (float):       current noisy measurement from the system sensor
        - U (float):            computed control input (thrust)
        - time_delta (float):   discrete time interval for simulation

        Returns:
        - filtered_state (State): estimated system state (z_pos, z_vel) using the kalman filter

        """
        vel_est = self.vel + U/self.params.mass*time_delta
        vel_var_est = self.vel_var + self.vel_process_var

        vel_meas = (z_meas - self.pos)/time_delta

        kalman_gain_vel = vel_var_est / (vel_var_est + self.vel_sensor_var)

        vel_new = vel_est + kalman_gain_vel*(vel_meas - vel_est)
        vel_var_new = vel_var_est * self.vel_sensor_var / (vel_var_est + self.vel_sensor_var)

        pos_est = self.pos + vel_new*time_delta
        pos_var_est = self.pos_var + self.pos_process_var

        kalman_gain_pos = pos_var_est / (pos_var_est + self.pos_sensor_var)

        #calculating new mean
        pos_new = pos_est + kalman_gain_pos*(z_meas - pos_est)
        #calculating new variance
        pos_var_new = pos_var_est * self.pos_sensor_var / (pos_var_est + self.pos_sensor_var)

        self.pos_var = pos_var_new
        self.pos = pos_new
        self.vel = vel_new
        self.vel_var = vel_var_new

        filtered_state = State(z_pos=pos_new, z_vel=vel_new)

        return filtered_state
