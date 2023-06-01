import numpy as np
import matplotlib.pyplot as plt
from linear_controller import LinearController
import siso_controller_type as SISO_CONTROLLER_TYPE
from proportional_integral_derivative import ProportionalIntegralDerivative
from DPID import DiscreteProportionalIntegralDerivative
from LLC import LeadLagCompensator

#Cascaded PID for Planar Dynamical Systems with q = [x,y,theta]
class CascadedPlanarController(LinearController):
    def __init__(self, K, dt, mingain=-float("inf"), maxgain=float("inf"), ctype=SISO_CONTROLLER_TYPE.PID):
        super.__init__(self, dt, mingain, maxgain)
        assert K.shape[0] > 4, "Gain matrix too large"

        #check for type of cascade (thrust coupling???): [x,y,thetaX, thetaY]
        for i in range(K.shape[0]):
            #Check for controller type
            if ctype == SISO_CONTROLLER_TYPE.PID:
                self.controllers[i] = ProportionalIntegralDerivative(K[i], self.dt, self.mingain, self.maxgain)
            elif ctype == SISO_CONTROLLER_TYPE.DPID:
                self.controllers[i] = DiscreteProportionalIntegralDerivative(K[i], self.dt, self.mingain, self.maxgain)
            elif ctype == SISO_CONTROLLER_TYPE.LLC:
                self.controllers[i] = LeadLagCompensator(K[i], self.dt, self.mingain, self.maxgain)

    #Change timestep
    def dtUpdate(self, dt):
        self.dt = dt
        for controller in self.controllers:
            controller.dtUpdate(self.dt)

    #Define cascading
    def step(self, error):
        #Coupled vs. uncoupled thrust
        gain = 0.0
        if len(self.controllers) == 3:
            #compute x and y controllers
            target_theta = self.controllers[0].step(error[0])
            thrust = self.controllers[2].step(error[1])

            #compute theta controller
            dthrust = self.controllers[2].step(target_theta - error[2])

            #compute gain
            gain = np.array([thrust - dthrust, thrust + dthrust])
        else:
            #compute x and y controllers
            target_theta = self.controllers[0].step(error[0])
            thrust = self.controllers[2].step(error[1])

            #compute theta controller
            dthrustx = self.controllers[2].step(target_theta - error[2])
            dthrusty = self.controllers[3].step(target_theta - error[2])

            #compute gain
            gain = np.array([thrust - dthrustx, thrust + dthrusty])

        return gain