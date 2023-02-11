# 3D Crazyflie Control Simulation
## Task 1: Implement the controller class 
```
Insert updated controller.py here
```

The controller was an implementation of the equations for Theta, Phi, and Psi that we derived as a group on day 6. 


## Task 2: PID Controller Family Tuning (Cascades)
The values that we chose for our .yml file were as follows:
```
<!-- insert the values from our yml here -->
```
Our first attempts with no outer-loop tuning achieved each waypoint except the final with relative ease.
This is because there is no disturbance in this system, and our trajectory was pre-planned and fed into the inner loop. The only part of the control where inner-loop-only tuning went wrong was between waypoints 4 and 5 with a z and x translation simultaneously: because the z-thrust for dropping could affect the acceleration almost instantaneously, the drone would deviate from the expected path in an arc because the drone was forced to change its pitch before it could move in the x-direction (which takes longer than dropping from the sky!).


Our cascaded tuning was more troublesome, as the inner loop alone could be tuned to achieve the waypoints. Because the k values for the outer loop were added on top of the already sufficient control, they merely added overshoot to the system. In a system with more disturbance, the outer-loop tuning may have had more of an impact. 

This module was not centered around trajectory generation, but with an understanding of trajectory generation in future modules we will have a greater ability to customize the inner/outer loop balance of cascaded control.


<!-- insert screenshot of the first test here, the named one -->


<!-- Submit: A write-up on what you find from this exercise. Include plots (or screen captures of the sim fig). Be creative. How would you explain this to your future self? -->

## Reflection: 
The most difficult aspect of this assignment was tuning the second loop once the first loop was in place - the first loop was behaving as expected, but when combined these two loops behaved in ways that were difficult to account for. 

The most straightforward part of this assignment was coding the inner control loop - because the trajectory


<!-- - 
Question
- What did you learn from this? What did you not know before this assignment?
- What was the most difficult aspect of the assignment?
- What was the easiest or most straightforward aspect of the assignment?
- How long did this assignment take? What took the most time (Setup? Figuring out the codebase? Coding in Python? Exploring the questions?)?
- What did you learn about PID controllers that we didn’t explicitly cover in class or in this assignment?
- What more would you like to learn about controls? -->