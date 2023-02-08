import numpy as np
from math import sin, cos

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains, dt):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """
        self.params = cfparams

        # translational
        self.kp_x = pid_gains['kp_x']
        self.kp_y = pid_gains['kp_y']
        self.kp_z = pid_gains['kp_z']
        #-----
        self.ki_x = pid_gains['ki_x']
        self.ki_y = pid_gains['ki_y']
        self.ki_z = pid_gains['ki_z']
        #-----
        self.kd_x = pid_gains['kd_x']
        self.kd_y = pid_gains['kd_y']
        self.kd_z = pid_gains['kd_z']
        #-----
        # rotational 
        self.kp_phi = pid_gains['kp_phi']
        self.kp_theta = pid_gains['kp_theta']
        self.kp_psi = pid_gains['kp_psi']
        #-----
        self.ki_phi = pid_gains['ki_phi']
        self.ki_theta = pid_gains['ki_theta']
        self.ki_psi = pid_gains['ki_psi']
        #-----
        self.kd_p = pid_gains['kd_p']
        self.kd_q = pid_gains['kd_q']
        self.kd_r = pid_gains['kd_r']

        self.e_z_total = 0
        self.e_x_total = 0
        self.e_y_total = 0
        self.e_phi_total = 0
        self.e_theta_total = 0
        self.e_psi_total = 0


    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (TrajPoint dataclass):   the desired control setpoint
        - state (State dataclass):          the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        N.B. TrajPoint is a new dataclass. Please check it out from the utils.py script
        """
        U = np.array([0.,0.,0.,0.])

        print(setpoint.z_acc)

        # Translation
        e_z = setpoint.z_pos - state.z_pos
        e_z_dot = setpoint.z_vel - state.z_vel
        self.e_z_total += e_z
        e_x = setpoint.x_pos - state.x_pos
        e_x_dot = setpoint.x_vel - state.x_vel
        self.e_x_total += e_x
        e_y = setpoint.y_pos - state.y_pos
        e_y_dot = setpoint.y_vel - state.y_vel
        self.e_y_total += e_y

        z_dotdot = self.kp_z * e_z + self.ki_z * self.e_z_total + self.kd_z * e_z_dot

        U[0] = self.params.mass * (z_dotdot + self.params.g)

        x_dotdot = self.kp_x * e_x + self.ki_x * self.e_x_total + self.kd_x * e_x_dot
        y_dotdot = self.kp_y * e_y + self.ki_y * self.e_y_total + self.kd_y * e_y_dot

        phi_d = (1/self.params.g) * (x_dotdot * np.sin(state.psi) - y_dotdot * np.cos(state.psi))
        theta_d = (1/self.params.g) * (x_dotdot * np.cos(state.psi) + y_dotdot * np.sin(state.psi))

        # psi_d?

        e_phi = phi_d - state.phi
        e_p = -1 * state.p
        self.e_phi_total += e_phi
        e_theta = theta_d - state.theta
        e_q = -1 * state.q
        self.e_theta_total += e_theta
        e_psi = setpoint.psi - state.psi
        e_r = setpoint.r - state.r
        self.e_psi_total += e_psi

        U[1] = self.kp_phi * e_phi + self.kd_p * e_p + self.ki_phi * self.e_phi_total
        U[2] = self.kp_theta * e_theta + self.kd_q * e_q + self.ki_theta * self.e_theta_total
        U[3] = self.kp_psi * e_psi + self.kd_r * e_r + self.ki_psi * self.e_psi_total

        # print(U)

        # U = np.array([self.params.mass * self.params.g,0.,0.,0.])

        return U
