import numpy as np
import matplotlib.pyplot as plt
from linear_controller import LinearController

#Proportional Integral Derivative Controller
#   -computes PID in discrete space using bilinear transform
class DiscreteProportionalIntegralDerivative(LinearController):
    def __init__(self, kp, ki, kd, dt, mingain=-float("inf"), maxgain=float("inf")):
        super.__init__(self, dt, mingain, maxgain)

        #define parameter from kp, kd, ki
        self.k = kp + (2 * ki) / self.dt + (0.5 * ki * self.dt)

        #define previous errors
        self.e1 = None
        self.e2 = None

        #define previous controls
        self.c1 = None
        self.c2 = None

    def step(self, error):
        #compute gain
        gain = (self.k * error) + (2 * self.k * self.e1) + (self.k * self.e2) - self.c2

        #store 2nd error and control signals
        if self.e1 and self.c1:
            self.e2 = self.e1
            self.c2 = self.c1

        #define 1st error and control signals
        self.e1 = error
        self.c1 = gain

        return min(max(self.mingain, gain), self.maxgain)