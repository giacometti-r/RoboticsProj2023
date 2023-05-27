import numpy as np

def mktr(x, y):
    return np.array([[1, 0, x],
                     [0, 1, y],
                     [0, 0, 1]])

def mkrot(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

class Drone2D:
    """ Simulates a 2D drone """

    def __init__(self, initial_pose, mass, L, maxthrust):
        """
        params:
            - initial_pose: 3x3 homogenenous matrix (pose transform)
            - mass: float [kg]
            - L: distance between thrusters [m]
            - maxthrust: maximum force that each thruster can exert [N]
        """
        self.pose = initial_pose
        self.mass = mass
        self.moment_of_inertia = mass * L**2
        self.L = L
        self.v = np.array([0.0, 0.0])  # in the world reference frame
        self.omega = 0.0
        self.maxthrust = maxthrust
        self.lt = 0
        self.rt = 0

    def getxy(self):
        """ returns (x,y) pose of drone (in meters) """
        return self.pose[0:2, 2] 

    def gettheta(self):
        """ returns the drone attitude (angle, in radians). 0 means horizontal.
            positive is counterclockwise
        """
        #                 sin(theta)       cos(theta)
        return np.arctan2(self.pose[1, 0], self.pose[0, 0])
    
    def __str__(self):
        x, y = self.getxy()
        theta = np.rad2deg(self.gettheta())
        return f"Drone ({x:.2f}, {y:.2f}) ∠ {theta:.1f}°"

    def step(self, thrustl_f, thrustr_f, dt):
        """ Steps the simulation assuming the left and right thrusters exert a
        constant force during the timestep.

        params:
            - thrustl_f: force exerted by the left thruster [N]
            - thrustr_f: force exerted by the right thruster [N]
            - dt: timestep duration [s]
        """
        thrustl_f = np.clip(thrustl_f, 0, self.maxthrust)
        thrustr_f = np.clip(thrustr_f, 0, self.maxthrust)
        gravity_force = np.array([0, -9.8 * self.mass])
        theta = self.gettheta()
        thrust_force = (np.array([np.cos(theta+np.pi/2), np.sin(theta+np.pi/2)])
                        * (thrustr_f + thrustl_f))
        force = gravity_force + thrust_force
        torque = self.L * (thrustr_f - thrustl_f)
        acceleration = force / self.mass
        self.v = self.v + acceleration * dt
        angular_acceleration = torque / self.moment_of_inertia
        self.omega = self.omega + angular_acceleration * dt
        self.pose = mktr(*(self.v * dt)) @ self.pose @ mkrot(self.omega * dt)
        self.lt = thrustl_f
        self.rt = thrustr_f

class ControlledDrone():
    """
    Represents a drone controlled by a controller in order to reach a given target x,y position
    """

    def __init__(self, drone, controller, target_x=0, target_y=0):
        """
        params:
            - drone: a Drone2D instance
            - controller: a controller instance
        """

        self.drone=drone
        self.controller=controller
        self.target_x = target_x
        self.target_y = target_y
        
    def step(self, dt):
        """
        This function first steps the controller, passing as parameters:
            - the drone x, y, theta
            - the current target_x and target_y
            - dt
        The controller returns the desired left and right thrust.
        Then, the controller steps the drone, passing the left and right thrust.

        params:
            - dt: timestep duration [seconds]
        """
        x, y = self.drone.getxy()
        theta = self.drone.gettheta()
        lt, rt = self.controller.step(x, y, theta, 
             self.target_x, self.target_y, 
             dt)
        self.drone.step(lt, rt, dt)