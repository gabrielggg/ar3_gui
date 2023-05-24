"""
Script for visualizing a robot from a URDF.
"""
import io
import pathlib

import glooey
import numpy as np

import pyglet
import trimesh
import trimesh.viewer
import trimesh.transformations as tf
import PIL.Image


here = pathlib.Path(__file__).resolve().parent

####
import sys
import time
import logging
import argparse

#from urdf_ar3 import __version__
from urdf_ar3 import URDF





def parse_args(args):
    """Parse command line parameters

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--help"]``).

    Returns:
      :obj:`argparse.Namespace`: command line parameters namespace
    """
    parser = argparse.ArgumentParser(description="Visualize a URDF model.")
    
    parser.add_argument(
        "input",
        help="URDF file name.",
    )
    
    return parser.parse_args(args)


def setup_logging(loglevel):
    """Setup basic logging.

    Args:
      loglevel (int): minimum loglevel for emitting messages
    """
    logformat = "[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(
        level=loglevel, stream=sys.stdout, format=logformat, datefmt="%Y-%m-%d %H:%M:%S"
    )


def generate_joint_limit_trajectory(self, urdf_model, loop_time):
    """Generate a trajectory for all actuated joints that interpolates between joint limits.
    For continuous joint interpolate between [0, 2 * pi].

    Args:
        urdf_model (yourdfpy.URDF): _description_
        loop_time (float): Time in seconds to loop through the trajectory.

    Returns:
        dict: A dictionary over all actuated joints with list of configuration values.
    """
    trajectory_via_points = {}
    for joint_name in urdf_model.actuated_joint_names:
        if urdf_model.joint_map[joint_name].type.lower() == "continuous":
            via_point_0 = 0.0
            via_point_2 = 2.0 * np.pi
            via_point_1 = (via_point_2 - via_point_0) / 2.0
            print("1")
        else:
            print("2")
            limit_lower = (
                urdf_model.joint_map[joint_name].limit.lower
                if urdf_model.joint_map[joint_name].limit.lower is not None
                else -np.pi
            )
            limit_upper = (
                urdf_model.joint_map[joint_name].limit.upper
                if urdf_model.joint_map[joint_name].limit.upper is not None
                else +np.pi
            )
            via_point_0 = limit_lower
            via_point_1 = limit_upper
            via_point_2 = limit_lower

        trajectory_via_points[joint_name] = np.array(
            [
                via_point_0,
                via_point_1,
                via_point_2,
            ]
        )
    times = np.linspace(0.0, 1.0, int(loop_time * 100.0))
    bins = np.arange(3) / 2.0

    # Compute alphas for each time
    inds = np.digitize(times, bins, right=True)
    inds[inds == 0] = 1
    alphas = (bins[inds] - times) / (bins[inds] - bins[inds - 1])

    # Create the new interpolated trajectory
    trajectory = {}
    for k in trajectory_via_points:
        trajectory[k] = (
            alphas * trajectory_via_points[k][inds - 1]
            + (1.0 - alphas) * trajectory_via_points[k][inds]
        )
        self.kont.append(0)

    return trajectory


def viewer_callback(scene, urdf_model, trajectory, loop_time, kont):
    frame = int(100.0 * (time.time() % loop_time))
    #solo mover un eje
    cfg1 = {'joint_1': trajectory['joint_1'][kont[0]], 'joint_2': trajectory['joint_2'][kont[1]], 'joint_3': trajectory['joint_3'][kont[2]], 'joint_4': trajectory['joint_4'][kont[3]], 'joint_5': trajectory['joint_5'][kont[4]], 'joint_6': trajectory['joint_6'][kont[5]]}
    #mover todos los ejes
    cfg = {k: trajectory[k][kont[index]] for index,k in enumerate(trajectory)}
    print(cfg)
    print(cfg1)

    urdf_model.update_cfg(configuration=cfg)



##############################



class Application:

    """
    Example application that includes moving camera, scene and image update.
    """

    def __init__(self, robot_widget, urdf_model):
        # create window with padding
        self.width, self.height = 1000, 625
        self.cont = 0
        self.grafi = 0
        self.kont = []
        self.robot_widget = robot_widget
        self.urdf_model = urdf_model
        self.pressedButton = False
        self.pressedButton2 = False
        self.pressedButton3 = False
        self.pressedButton4 = False
        self.pressedButton5 = False
        self.pressedButton6 = False

        self.pressedButtonb = False
        self.pressedButton2b = False
        self.pressedButton3b = False
        self.pressedButton4b = False
        self.pressedButton5b = False
        self.pressedButton6b = False
        window = self._create_window(width=self.width, height=self.height)

        gui = glooey.Gui(window)

        hbox = glooey.HBox()
        hbox2 = glooey.HBox()
        hbox3 = glooey.HBox()
        hbox4 = glooey.HBox()
        hbox5 = glooey.HBox()
        hbox6 = glooey.HBox()
        grid = glooey.Grid(default_row_height=220, default_col_width=200)
        hbox.set_padding(3)
        hbox2.set_padding(3)
        hbox3.set_padding(3)
        hbox4.set_padding(3)
        hbox5.set_padding(3)
        hbox6.set_padding(3)
        grid.set_row_height(0, 0)
        grid.set_row_height(2, 0)
        grid.set_row_height(1, 550)
        grid.set_col_width(2, 550)

        

        self.button = glooey.Button("DIR1")
        self.buttonLabel = glooey.Button("J1")
        self.button.push_handlers(on_click=lambda w: self.pressed(w))
        self.buttonb = glooey.Button("DIR2")
        self.buttonb.push_handlers(on_click=lambda w: self.pressedb(w))
        #hbox.pack(self.button)
        hbox.pack(self.buttonLabel)
        hbox.pack(self.button)
        hbox.pack(self.buttonb)
        

        self.button2 = glooey.Button("DIR1")
        self.button2Label = glooey.Button("J2")
        self.button2.push_handlers(on_click=lambda w: self.pressed2(w))
        self.button2b = glooey.Button("DIR2")
        self.button2b.push_handlers(on_click=lambda w: self.pressed2b(w))
        hbox2.pack(self.button2Label)
        hbox2.pack(self.button2)
        hbox2.pack(self.button2b)

        #self.button22 = glooey.Button("Click here!")
        #self.button22.push_handlers(on_click=lambda w: self.pressed2(w))
        #self.button23 = glooey.Button("Click here!")
        #self.button23.push_handlers(on_click=lambda w: self.pressed2(w))
        #hbox.pack(self.button22)
        #hbox.pack(self.button23)

        self.button3 = glooey.Button("DIR1")
        self.button3Label = glooey.Button("J3")
        self.label3= glooey.Label("J3")
        self.button3.push_handlers(on_click=lambda w: self.pressed3(w))
        self.button3b = glooey.Button("DIR2")
        self.button3b.push_handlers(on_click=lambda w: self.pressed3b(w))
        hbox3.pack(self.button3Label)
        hbox3.pack(self.button3)
        hbox3.pack(self.button3b)

        self.button4 = glooey.Button("DIR1")
        self.button4Label = glooey.Button("J4")
        self.button4.push_handlers(on_click=lambda w: self.pressed4(w))
        self.button4b = glooey.Button("DIR2")
        self.button4b.push_handlers(on_click=lambda w: self.pressed4b(w))
        
        hbox4.pack(self.button4Label)
        hbox4.pack(self.button4)
        hbox4.pack(self.button4b)
        

        self.button5 = glooey.Button("DIR1")
        self.button5Label = glooey.Button("J5")
        self.button5.push_handlers(on_click=lambda w: self.pressed5(w))
        self.button5b = glooey.Button("DIR2")
        self.button5b.push_handlers(on_click=lambda w: self.pressed5b(w))
        hbox5.pack(self.button5Label)
        hbox5.pack(self.button5)
        hbox5.pack(self.button5b)

        self.button6 = glooey.Button("DIR1")
        self.button6Label = glooey.Button("J6")
        self.button6.push_handlers(on_click=lambda w: self.pressed6(w))
        self.button6b = glooey.Button("DIR2")
        self.button6b.push_handlers(on_click=lambda w: self.pressed6b(w))
        hbox6.pack(self.button6Label)
        hbox6.pack(self.button6)
        hbox6.pack(self.button6b)


        #hbox.add(self.robot_widget)

        # integrate with other widget than SceneWidget
        self.image_widget = glooey.Image()
        self.image_widget2 = glooey.Image()
        #hbox.add(self.image_widget)
        #hbox2.add(self.image_widget2)

        grid[0,0] = hbox
        grid[0,1] = hbox2
        grid[1,0] = hbox3
        grid[1,1] = hbox4
        grid[2,0] = hbox5
        grid[2,1] = hbox6
        grid[1,2] = self.robot_widget
        #grid.do_resize_children()

        #gui.add(hbox)
        #gui.add(hbox2)

        gui.add(grid)

        pyglet.clock.schedule_interval(self.callback, 1. / 20)
        
        pyglet.app.run()
        
        
    def pressed(self,button):
        print(f"{button} clicked!") 
        if(self.pressedButton):
            self.pressedButton = False
        else:
            self.pressedButton = True
            self.pressedButtonb = False

    def pressed2(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton2):
            self.pressedButton2 = False
        else:
            self.pressedButton2 = True
            self.pressedButton2b = False

    def pressed3(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton3):
            self.pressedButton3 = False
        else:
            self.pressedButton3 = True
            self.pressedButton3b = False

    def pressed4(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton4):
            self.pressedButton4 = False
        else:
            self.pressedButton4 = True
            self.pressedButton4b = False

    def pressed5(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton5):
            self.pressedButton5 = False
        else:
            self.pressedButton5 = True
            self.pressedButton5b = False

    def pressed6(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton6):
            self.pressedButton6 = False
        else:
            self.pressedButton6 = True
            self.pressedButton6b = False

    ########################
    def pressedb(self,button):
        print(f"{button} clicked!") 
        if(self.pressedButtonb):
            self.pressedButtonb = False
        else:
            self.pressedButtonb = True
            self.pressedButton = False

    def pressed2b(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton2b):
            self.pressedButton2b = False
        else:
            self.pressedButton2b = True
            self.pressedButton2 = False

    def pressed3b(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton3b):
            self.pressedButton3b = False
        else:
            self.pressedButton3b = True
            self.pressedButton3 = False

    def pressed4b(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton4b):
            self.pressedButton4b = False
        else:
            self.pressedButton4b = True
            self.pressedButton4 = False

    def pressed5b(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton5b):
            self.pressedButton5b = False
        else:
            self.pressedButton5b = True
            self.pressedButton5 = False

    def pressed6b(self,button):
        print(f"{button} clicked!")
        if(self.pressedButton6b):
            self.pressedButton6b = False
        else:
            self.pressedButton6b = True
            self.pressedButton6 = False

    #######################

    def callback(self, dt):
        # change camera location
        loop_time = 6.0
        if (self.grafi == 0):
            self.trajectory=generate_joint_limit_trajectory(
                       self, urdf_model=self.urdf_model, loop_time=loop_time
                    )
        self.grafi = 1
        if(self.pressedButton):
            speedknob = 1  #use this constant to control speed
            self.kont[0] = self.kont[0] + speedknob 
            if (self.kont[0] >= 600):
                self.kont[0] = 0
        
        if(self.pressedButton2):
            speedknob = 1  #use this constant to control speed
            self.kont[1] = self.kont[1] + speedknob 
            if (self.kont[1] >= 600):
                self.kont[1] = 0

        if(self.pressedButton3):
            speedknob = 1  #use this constant to control speed
            self.kont[2] = self.kont[2] + speedknob 
            if (self.kont[2] >= 600):
                self.kont[2] = 0

        if(self.pressedButton4):
            speedknob = 1  #use this constant to control speed
            self.kont[3] = self.kont[3] + speedknob 
            if (self.kont[3] >= 600):
                self.kont[3] = 0

        if(self.pressedButton5):
            speedknob = 1  #use this constant to control speed
            self.kont[4] = self.kont[4] + speedknob 
            if (self.kont[4] >= 600):
                self.kont[4] = 0

        if(self.pressedButton6):
            speedknob = 1  #use this constant to control speed
            self.kont[5] = self.kont[5] + speedknob 
            if (self.kont[5] >= 600):
                self.kont[5] = 0

        ################

        if(self.pressedButtonb):
            speedknob = 1  #use this constant to control speed
            self.kont[0] = self.kont[0] - speedknob 
            if (self.kont[0] == -600):
                self.kont[0] = 0
        
        if(self.pressedButton2b):
            speedknob = 1  #use this constant to control speed
            self.kont[1] = self.kont[1] - speedknob 
            if (self.kont[1] == -600):
                self.kont[1] = 0

        if(self.pressedButton3b):
            speedknob = 1  #use this constant to control speed
            self.kont[2] = self.kont[2] - speedknob 
            if (self.kont[2] == -600):
                self.kont[2] = 0

        if(self.pressedButton4b):
            speedknob = 1  #use this constant to control speed
            self.kont[3] = self.kont[3] - speedknob 
            if (self.kont[3] == -600):
                self.kont[3] = 0

        if(self.pressedButton5b):
            speedknob = 1  #use this constant to control speed
            self.kont[4] = self.kont[4] - speedknob 
            if (self.kont[4] == -600):
                self.kont[4] = 0

        if(self.pressedButton6b):
            speedknob = 1  #use this constant to control speed
            self.kont[5] = self.kont[5] - speedknob 
            if (self.kont[5] == -600):
                self.kont[5] = 0
        ###############


        viewer_callback(..., self.urdf_model,self.trajectory, loop_time, self.kont )
        #self.robot_widget.scene.set_camera([1.57, 0, 0])
        self.robot_widget._draw()
        #self.robot_widget.scene.set_camera([1.57, 0, 0])


        # change image
        image = np.random.randint(0,
                                  255,
                                  (self.height - 10, self.width // 3 - 10, 3),
                                  dtype=np.uint8)
        with io.BytesIO() as f:
            PIL.Image.fromarray(image).save(f, format='JPEG')
            self.image_widget.image = pyglet.image.load(filename=None, file=f)
            self.image_widget2.image = pyglet.image.load(filename=None, file=f)

    def _create_window(self, width, height):
        try:
            config = pyglet.gl.Config(sample_buffers=1,
                                      samples=4,
                                      depth_size=24,
                                      double_buffer=True)
            window = pyglet.window.Window(resizable=True, config=config,
                                          width=width,
                                          height=height)
        except pyglet.window.NoSuchConfigException:
            config = pyglet.gl.Config(double_buffer=True)
            window = pyglet.window.Window(config=config,
                                          width=width,
                                          height=height)

        @window.event
        def on_key_press(symbol, modifiers):
            if modifiers == 0:
                if symbol == pyglet.window.key.Q:
                    window.close()

        return window


##############################

def main(args):
    """Wrapper allowing string arguments in a CLI fashion.

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--verbose", "42"]``).
    """
    args = parse_args(args)
    #setup_logging(args.loglevel)


    urdf_model = URDF.load(args.input)
    #urdf_model.apply_translation((0, 0, -0.005))
    scenex = urdf_model.scene
    print(scenex)
    scene_widget2 = trimesh.viewer.SceneWidget(scenex)
    print(scene_widget2)


    return scene_widget2, urdf_model
    


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`.

    This function can be used as entry point to create console scripts with setuptools.
    """
    np.random.seed(0)
    robot_widget, urdf_model = main(sys.argv[1:])
    Application(robot_widget, urdf_model)


if __name__ == "__main__":
    run()
