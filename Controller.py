
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd
        self.accumulated_error = 0.

    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        error = setpoint.z_pos-state.z_pos
        self.accumulated_error += error

        gain = 1
        self.kp_z = .52
        self.ki_z = 0.000001
        self.kd_z = -0.28
        p = self.kp_z* error
        i = self.ki_z*self.accumulated_error
        d = self.kd_z*state.z_vel
    
        # your code here
        command = gain* (p + d + i) + self.params.mass*self.params.g
        return command
