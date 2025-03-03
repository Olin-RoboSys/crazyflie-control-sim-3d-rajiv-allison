# 3D Crazyflie Control Simulation
## Task 1: Implement the controller class 
```
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

    z_dotdot = setpoint.z_acc + self.kp_z * e_z + self.ki_z * self.e_z_total + self.kd_z * e_z_dot

    U[0] = self.params.mass * (z_dotdot + self.params.g)

    x_dotdot = setpoint.x_acc + self.kp_x * e_x + self.ki_x * self.e_x_total + self.kd_x * e_x_dot
    y_dotdot = setpoint.y_acc + self.kp_y * e_y + self.ki_y * self.e_y_total + self.kd_y * e_y_dot

    phi_d = (1/self.params.g) * (x_dotdot * np.sin(state.psi) - y_dotdot * np.cos(state.psi))
    theta_d = (1/self.params.g) * (x_dotdot * np.cos(state.psi) + y_dotdot * np.sin(state.psi))

    e_phi = phi_d - state.phi
    e_p = -1 * state.p
    self.e_phi_total += e_phi
    e_theta = theta_d - state.theta
    e_q = -1 * state.q
    self.e_theta_total += e_theta
    e_psi = setpoint.psi - state.psi
    e_r = setpoint.r - state.r
    self.e_psi_total += e_psi

    U[1] = self.params.I[0,0] * (self.kp_phi * e_phi + self.kd_p * e_p + self.ki_phi * self.e_phi_total)
    U[2] = self.params.I[1,1] * (self.kp_theta * e_theta + self.kd_q * e_q + self.ki_theta * self.e_theta_total)
    U[3] = self.params.I[2,2] * (setpoint.r_dot + self.kp_psi * e_psi + self.kd_r * e_r + self.ki_psi * self.e_psi_total)

    return U
```

The controller was an implementation of the equations for the linearized 3D dynamics of a quadrotor that we derived as a group on day 6.


## Task 2: PID Controller Family Tuning (Cascades)
The values that we chose for our .yml file were as follows:
```
# translational 
kp_x: 0.
kp_y: 0.
kp_z: 16
#-----
ki_x: 0
ki_y: 0
ki_z: 0.
#-----
kd_x: 0.
kd_y: 0.
kd_z: 8
#-----

# rotational 
kp_phi: 25.
kp_theta: 25.
kp_psi: 25.
#-----
ki_phi: 0.
ki_theta: 0.
ki_psi: 0.
#-----
kd_p: 10.
kd_q: 10.
kd_r: 10.
```
Our first attempts with no outer-loop tuning achieved each waypoint except the final with relative ease.
This is because there is no disturbance in this system, and our trajectory was pre-planned and fed into the inner loop. The only part of the control where inner-loop-only tuning went wrong was between waypoints 4 and 5 with a z and x translation simultaneously: because the z-thrust for dropping could affect the acceleration almost instantaneously, the drone would deviate from the expected path in an arc because the drone was forced to change its pitch before it could move in the x-direction (which takes longer than dropping from the sky!).


Our cascaded tuning was more troublesome, as the inner loop alone could be tuned to achieve the waypoints. Because the k values for the outer loop were added on top of the already sufficient control, they merely added overshoot to the system. In a system with more disturbance, the outer-loop tuning may have had more of an impact. 

This module was not centered around trajectory generation, but with an understanding of trajectory generation in future modules we will have a greater ability to customize the inner/outer loop balance of cascaded control.


![4/5 PID](/media/3d_pid_4_outof_5_2.png)
**4/5 waypoints reached with inner loop only**. With only inner loop PID tuned, the drone can reach all waypoints except the final one.

![Overshooting PID](/media/3d_pid_overshoot.png)
**Overshooting PID with outer loop control**. Adding the outer loop PID (x and y acceleration control) caused the system to overshoot. We were not able to find values that would result in the drone following the final path from waypoints 4 to 5 without overshooting previous waypoints.

<!-- Submit: A write-up on what you find from this exercise. Include plots (or screen captures of the sim fig). Be creative. How would you explain this to your future self? -->

## Reflection: 
The most difficult aspect of this assignment was tuning the second loop once the first loop was in place - the first loop was behaving as expected, but when combined these two loops behaved in ways that were difficult to account for. 

The most straightforward part of this assignment was tuning the inner control loop - because the trajectory has already been planned out with ideal accelerations, velocities, and positions on the way to each waypoint. We could use mathematically critically damped values for P-D controllers for altitude, roll, pitch, and yaw without any I, and achieve 4/5 waypoints. Coding the controller function was also relatively straightforward because we had gone over all of the equations necessary in class.

This assignment took ~5 hours. Tuning the PID took the longest.

Learning about cascading controllers was interesting and understanding how the outer controller affects the inner controller and vice versa. We still don't have a complete understanding of this but were able to get some intuition for it while tuning.

We would like to learn more about how to get a controller that is robust enough to work both in an idealized situation and with noise and disturbances. We would also like to understand how to tune a controller to respond well when used with a trajectory generator. We had more intuition about what the controller was doing when the setpoints were fixed in space, but with the trajectory also dictating accelerations and velocities we were less clear about what changes to our controller and tuning were doing.

<!-- - 
Question
- What did you learn from this? What did you not know before this assignment?
- What was the most difficult aspect of the assignment?
- What was the easiest or most straightforward aspect of the assignment?
- How long did this assignment take? What took the most time (Setup? Figuring out the codebase? Coding in Python? Exploring the questions?)?
- What did you learn about PID controllers that we didn’t explicitly cover in class or in this assignment?
- What more would you like to learn about controls? -->