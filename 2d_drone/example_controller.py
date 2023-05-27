import numpy as np
from pid import PID

# Do not change the name of the class or the signature of the functions!
class Controller():
    def __init__(self, maxthrust):
        self.maxthrust = maxthrust # just to know what the max thrust is. Could be useful

        # EDIT HERE: Create whatever objects you need
        self.y_pid = PID(1,0,0)
        
    def step(self, 
             x, y, theta, 
             target_x, target_y, 
             dt):
        
        # EDIT HERE: do whatever you want
        e = target_y - y
        thrust = self.y_pid.step(e, dt)
        lt = thrust
        rt = thrust

        # Return the desired left and right thrusts.
        # Consider that values will be clamped between 0 and maxthrust
        lt = np.clip(lt, 0, self.maxthrust)
        rt = np.clip(rt, 0, self.maxthrust)
        return lt, rt