import numpy as np
import arcade
from datetime import datetime
import importlib
import dronesim
from bspline_trajectory import Waypoints, BSplineTrajectory
import matplotlib.pyplot as plt
import siso_controller_type as SISO_CONTROLLER_TYPE

class DroneViz(arcade.Window):
    """ Main application class. """
    
    def draw_controlled_drone(self, controlled_drone):
        drone = controlled_drone.drone
        L = drone.L
        shape_list = arcade.ShapeElementList()        
        shape_list.append(arcade.create_polygon(
            [(-L,0),(L,0),(L*0.9,L/2),(-L*0.9,L/2)],
            arcade.color.GRAY))
        shape_list.append(arcade.create_polygon(
           [(-L*1.1,0),(-L*0.9,0),(-L,-drone.lt/drone.maxthrust*L)],
           arcade.color.YELLOW))
        shape_list.append(arcade.create_polygon(
           [(+L*1.1,0),(+L*0.9,0),(+L,-drone.rt/drone.maxthrust*L)],
           arcade.color.YELLOW))
        shape_list.angle = np.rad2deg(drone.gettheta())
        shape_list.center_x, shape_list.center_y = drone.getxy()
        return shape_list
        #arcade.draw_text("test", shape_list.center_x, shape_list.center_y, arcade.color.WHITE, rotation=shape_list.angle,
        #                 font_size = 5, align="center")
        
    
    def __init__(self, setpoints, waypoints):
        super().__init__(1024, 768, "Drone simulator", resizable=True)
        self.controlled_drones = []
        self.waypoints_list = waypoints
        self.setpoints_list = setpoints
        self.t_sim = 0
        self.t_real = 0
        self.count = 0
        self.worldwidth = 80

    
    def on_draw(self):
        """
        Render the screen.
        """
        width, height = self.get_size()
        self.set_viewport(-self.worldwidth/2, self.worldwidth/2, -0.5*height/width*self.worldwidth, +0.5*height/width*self.worldwidth)
        
        # This command has to happen before we start drawing
        arcade.start_render()
        for i,setpoint in enumerate(setpoints_list):
            arcade.draw_points(setpoint, arcade.color.RED, .4)
            arcade.draw_points(waypoints_list[i].samples, arcade.color.GREEN, .2)
                
        start_t = datetime.now()
        shape_lists = []
        for controlled_drone in self.controlled_drones: 
            self.draw_controlled_drone(controlled_drone).draw()

    def euclidean_dist(self, x, y, target_x, target_y):
        return np.sqrt((x-target_x)**2 + (y-target_y)**2)
        
    def on_update(self, delta_time):
        """ Movement and game logic """

        # for controlled_drone in self.controlled_drones:
        #     controlled_drone.target_x = self.target_x
        #     controlled_drone.target_y = self.target_y

        dt = 0.02
        self.t_real += delta_time
        while self.t_sim < self.t_real: # advance simulation time in steps of dt until it matches real time
            for controlled_drone in self.controlled_drones:
                if self.count == 0:
                    print('INITIAL DEFINE')
                    self.target_x, self.target_y = self.waypoints.step()
                try:
                    x, y = controlled_drone.drone.getxy()
                    dist = self.euclidean_dist(x,y,self.target_x, self.target_y)
                    print(dist)
                    controlled_drone.step(dt, self.target_x, self.target_y)
                    if dist<1:
                        print('CLOSE ENOUGH...')
                        self.target_x, self.target_y = self.waypoints.step()
                        print('CHANGING WAYPOINTS')
                        print(self.target_x, self.target_y)
                    self.count+=1

                except Exception as e:
                    print(f"error in controller step: {e}")
            self.t_sim = self.t_sim + dt
        
    def window2viewport(self, x, y):
        width, height = self.get_size()
        x_min, x_max, y_min, y_max = self.get_viewport()
        xv = x_min + (x/width)*(x_max-x_min)
        yv = y_min + (y/height)*(y_max-y_min)
        print(xv,yv)
        return xv, yv    
    
    def on_mouse_press(self, x, y, button, modifiers):
        """
        Called whenever the mouse button is clicked.
        """
        # x, y = self.window2viewport(x, y)
        # self.target_x, self.target_y = x, y
        # for cd in self.controlled_drones:
        #     cd.target_x, cd.target_y = x, y
    
    def on_key_press(self, symbol, modifiers):
        if symbol == arcade.key.Q:
            # Quit immediately
            arcade.close_window()
        elif symbol == arcade.key.E:
            self.controlled_drones.clear()
        elif symbol == arcade.key.A:
            self.perturb_all_angles(-30)
        elif symbol == arcade.key.S:
            self.perturb_all_angles(+30)
        elif symbol == arcade.key.SPACE:
            self.spawn_drone()
        elif symbol == arcade.key.UP:
            self.worldwidth *= 1.1
        elif symbol == arcade.key.DOWN:
            self.worldwidth /= 1.1
    
    def perturb_all_angles(self, angle_deg):
        for cd in self.controlled_drones:
            cd.drone.pose = cd.drone.pose @ dronesim.mkrot(np.deg2rad(angle_deg))
            print(cd.drone.pose)

    def spawn_drone(self):
        ctype_dict = {
            0: SISO_CONTROLLER_TYPE.PID,
            1: SISO_CONTROLLER_TYPE.DPID,
            2: SISO_CONTROLLER_TYPE.LLC
        }
        try:
            for i in range(3):
                import cascaded_planar_controller as cont
                importlib.reload(cont)
                initial_pose = dronesim.mktr(self.initial_x,self.initial_y) @ dronesim.mkrot(np.deg2rad(0))
                d = dronesim.Drone2D(initial_pose=initial_pose, mass=1, L=1, maxthrust=20)
                c = cont.CascadedPlanarController(mingain=0, maxgain=20, ctype=ctype_dict[i])
                cd = dronesim.ControlledDrone(drone=d, controller=c, waypoints=waypoints)
                self.controlled_drones.append(cd)
        except:
            print("error")

if __name__ == "__main__":
    setpoints_list = []
    waypoints_list = []
    for i in range(3):
        setpoints = np.array([[0,0],[0,1],[1,1],[1,0],[0,0]])*10 + np.array([((i+1)*20),0])
        trajectory = BSplineTrajectory(setpoints)
        waypoints = Waypoints(trajectory, 10)
        setpoints_list.append(setpoints)
        waypoints_list.append(waypoints)
    g = DroneViz(setpoints=setpoints_list, waypoints=waypoints_list)

    arcade.run()