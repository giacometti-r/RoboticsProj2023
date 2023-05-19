import numpy as np
from pid import PID

# Do not change the name of the class or the signature of the functions!
class Controller():
    def __init__(self, maxthrust):
        self.maxthrust = maxthrust
        self.y_pid = PID(10, 1, 10) 
        self.x_pid = PID(.1, 0, .1) 
        self.theta_pid = PID(10, 0, 4  ) 

    def step(self, 
             x, y, theta, 
             target_x, target_y, 
             dt):
        
        e_y = target_y - y
        tt = self.y_pid.step(e_y, dt)
        
        e_x = target_x - x
        target_xa = self.x_pid.step(e_x, dt)

        target_theta = -target_xa
        e_theta = target_theta - theta
        tdiff = self.theta_pid.step(e_theta, dt)

        lt, rt = tt/2-tdiff/2, tt/2+tdiff/2

        # Return the desired left and right thrusts.
        # Consider that values will be clamped between 0 and maxthrust
        lt = np.clip(lt, 0, self.maxthrust)
        rt = np.clip(rt, 0, self.maxthrust)
        
        return lt, rt