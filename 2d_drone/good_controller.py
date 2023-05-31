import numpy as np
from pid import PID
from datetime import datetime

# Do not change the name of the class or the signature of the functions!
class Controller():
    def __init__(self, maxthrust):
        self.maxthrust = maxthrust # just to know what the max thrust is. Could be useful

        # EDIT HERE: Create whatever objects you need
        self.y_pid = PID(10, 0.1, 10)  #PID(10, 0.1, 10)
        self.x_pid = PID(0.2, 0, 0.3) # this defines our desired theta angle
        self.theta_pid = PID(10, 0 , 10)


    def step(self, 
             x, y, theta, 
             target_x, target_y, 
             dt):        

        # EDIT HERE: do whatever you want here
        desired_thrust = self.y_pid.step(target_y - y, dt)
        desired_x_acc = self.x_pid.step(target_x - x, dt)
        target_theta = -desired_x_acc
        maxangle = np.deg2rad(45) # we do not want to tilt more than this
        target_theta = np.clip(target_theta, -maxangle, maxangle)
        desired_thrust_difference = self.theta_pid.step(target_theta-theta, dt)
        desired_thrust = np.clip(desired_thrust, self.maxthrust*0.2, self.maxthrust*0.8)
        lt = desired_thrust - desired_thrust_difference
        rt = desired_thrust + desired_thrust_difference

        # Return the desired left and right thrusts.
        # Consider that values will be clamped between 0 and maxthrust
        lt = np.clip(lt, 0, self.maxthrust)
        rt = np.clip(rt, 0, self.maxthrust)
        return lt, rt