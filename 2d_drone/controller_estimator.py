import numpy as np
import matplotlib.pyplot as plt

#Implements feedback loop for controller and estimator
class ControllerEstimator:
    def __init__(self, controller, estimator):
        self.controller = controller
        self.estimator = estimator

    #initialize controller and observer
    def initialize(self, state=None, control=None):
        self.controller.initialize(self, state, control)
        self.estimator.initialize(self, state, control)

    #Take state and target and return control signal
    def step(self, state, target):
       #Get state estimate
        estimation = self.estimator.step(state)

        #get control signal given state estimate
        error = target - estimation
        control = self.controller.step(error)

        return control
    
    #get trajectory
    def getTrajectory(self):
        return self.estimator.getTrajectory()