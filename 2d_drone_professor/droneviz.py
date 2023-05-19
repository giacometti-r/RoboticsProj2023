import numpy as np
import arcade
from datetime import datetime
import importlib
import dronesim

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
        
    
    def __init__(self):
        super().__init__(1024, 768, "Drone simulator", resizable=True)
        self.controlled_drones = []
        self.target_x, self.target_y = 0, 0
        self.t_sim = 0
        self.t_real = 0
        self.worldwidth = 40

    
    def on_draw(self):
        """
        Render the screen.
        """
        width, height = self.get_size()
        self.set_viewport(-self.worldwidth/2, self.worldwidth/2, -0.5*height/width*self.worldwidth, +0.5*height/width*self.worldwidth)
        
        # This command has to happen before we start drawing
        arcade.start_render()
        arcade.draw_rectangle_outline(self.target_x, self.target_y, 2, 2, border_width=0.1, color=arcade.color.RED)
        
        start_t = datetime.now()
        shape_lists = []
        for controlled_drone in self.controlled_drones:
            self.draw_controlled_drone(controlled_drone).draw()
        #print(f"Step time: {datetime.now()-start_t}")
        
    def on_update(self, delta_time):
        """ Movement and game logic """

        for controlled_drone in self.controlled_drones:
            controlled_drone.target_x = self.target_x
            controlled_drone.target_y = self.target_y

        dt = 0.02
        self.t_real += delta_time
        while self.t_sim < self.t_real: # advance simulation time in steps of dt until it matches real time
            for controlled_drone in self.controlled_drones:
                try:
                    controlled_drone.step(dt)
                except Exception as e:
                    print(f"error in controller step {e}")
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
        x, y = self.window2viewport(x, y)
        self.target_x, self.target_y = x, y
        for cd in self.controlled_drones:
            cd.target_x, cd.target_y = x, y
    
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
        try:
            import controller as cont
            importlib.reload(cont)
            initial_pose = dronesim.mktr(0,0) @ dronesim.mkrot(np.deg2rad(0))
            d = dronesim.Drone2D(initial_pose=initial_pose, mass=1, L=1, maxthrust=10)
            c = cont.Controller(maxthrust=d.maxthrust)
            cd = dronesim.ControlledDrone(drone=d, controller=c)
            self.controlled_drones.append(cd)
        except:
            print("error")

if __name__ == "__main__":
    g = DroneViz()
    arcade.run()