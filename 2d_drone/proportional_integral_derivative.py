import numpy as np
import matplotlib.pyplot as plt
from linear_controller import LinearController

#Proportional Integral Derivative Controller
class ProportionalIntegralDerivative(LinearController):
    def __init__(self, kp, ki, kd, dt, mingain=-float("inf"), maxgain=float("inf")):
        super().__init__(dt, mingain, maxgain)
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def step(self, error):
        #D: compute dedt
        if self.e_last is not None:
            dedt = (error - self.e_last) / self.dt
        else:
            dedt = 0

        #I: compute integral
        self.int_e += error * self.dt

        #PID: compute gain
        gain = (self.kp * error) + (self.ki * self.int_e) + (self.kd * dedt)
        self.e_last = error

        return min(max(self.mingain, gain), self.maxgain)