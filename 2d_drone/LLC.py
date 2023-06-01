import numpy as np
import matplotlib.pyplot as plt
from linear_controller import LinearController

#Lead-Lag Compensator with params [K0, K1, K2, K3, gamma]
class LeadLagCompensator(LinearController):
    def __init__(self, k, dt, mingain=-float("inf"), maxgain=float("inf"), discrete=True):
        super().__init__(dt, mingain, maxgain)

        #define parameters
        self.k = k

        #define previous errors
        self.e1 = None
        self.e2 = None

        #define previous controls
        self.c1 = None
        self.c2 = None

    #Run one step of the general lead lag compensator
    #   -derived by taking the LLC transfer function and discretizing with bilinear transform
    def step(self, error):
        #define terms
        a = (4 / self.dt**2)
        b = (2 * self.k[3]) / (self.dt * self.k[4])
        c = (2 * self.k[4]) / (self.dt * self.k[2])
        d = 2 / (self.dt * self.k[0])
        e = 2 / (self.dt * self.k[1])

        #define parameters
        A = a + b + c
        B = 8 / self.dt**2
        C = a + b + c + self.k[2] * self.k[3]
        D = a + d + e
        E = a + d + e + 1 / (self.k[0] ( self.k[1]))

        #compute LLC control signal
        gain = ((E / C) * error) + ((B / C) * (self.e1 - self.c1)) + ((D / C) * self.e2) - ((A / C) * self.c2)

        #store error and control signals
        if not self.e1 or not self.c1:
            self.e1 = error
            self.c1 = gain
        else:
            self.e2 = self.e1
            self.c2 = self.c1
            self.e1 = error
            self.c1= gain

        return min(max(self.mingain, gain), self.maxgain)