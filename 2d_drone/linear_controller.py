import numpy as np
import matplotlib.pyplot as plt

#Template class for controllers
class LinearController:
    def __init__(self, dt, mingain=-float("inf"), maxgain=float("inf")):
        #set timestep
        self.dt = dt

        #define min and max gains
        self.mingain = mingain
        self.maxgain = maxgain

    #Change timestep
    def dtUpdate(self, dt):
        self.dt = dt

    #Initialize controller
    def initialize(self, state=None, control=None):
        pass

    #Run step of controller given error
    def step(self, error, equilibrium=None):
        pass