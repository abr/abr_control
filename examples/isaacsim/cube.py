import numpy as np
from abr_control.arms import ur5 as arm

from abr_control.interfaces.isaacsim import IsaacSim
from abr_control.utils import transformations

# Sim step size
dt = 0.005

# Initialize our robot config
robot_config = arm.Config()


# Create our interface
# interface = CoppeliaSim(robot_config, dt=dt)
interface = IsaacSim(robot_config, dt=dt)
interface.connect()
# Imports must be done after instantiating the SimulationApp, which is done by the setup in the isaacsim interface
from isaacsim.core.api import World # type: ignore
from isaacsim.core.api.objects import DynamicCuboid # type: ignore



#TODO add this in interface setup
#world = World(physics_dt=dt,rendering_dt=dt)

world = World()
world.scene.add_default_ground_plane()
fancy_cube =  world.scene.add(

    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.5015, 0.5015, 0.5015]),
        color=np.array([0, 0, 1.0]),
    ))


world.reset()
for i in range(500):

    position, orientation = fancy_cube.get_world_pose()
    linear_velocity = fancy_cube.get_linear_velocity()

    # will be shown on terminal
    print("Cube position is : " + str(position))
    print("Cube's orientation is : " + str(orientation))
    print("Cube's linear velocity is : " + str(linear_velocity))

    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step


interface.disconnect()