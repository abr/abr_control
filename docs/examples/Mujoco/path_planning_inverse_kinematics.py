"""
Running the joint controller with an inverse kinematics path planner
for a Mujoco simulation. The path planning system will generate
a trajectory in joint space that moves the end effector in a straight line
to the target, which changes every n time steps.
"""
import numpy as np
import glfw

from abr_control.interfaces.mujoco import Mujoco
from abr_control.arms.mujoco_config import MujocoConfig as arm
from abr_control.controllers import path_planners
from abr_control.utils import transformations

# initialize our robot config for the jaco2
robot_config = arm("ur5", use_sim_state=False)

# create our path planner
n_timesteps = 2000
path_planner = path_planners.InverseKinematics(robot_config)

# create our interface
dt = 0.001
interface = Mujoco(robot_config, dt=dt)
interface.connect()
interface.send_target_angles(robot_config.START_ANGLES)
feedback = interface.get_feedback()

try:
    print("\nSimulation starting...")
    print("Click to move the target.\n")

    count = 0
    while 1:
        if interface.viewer.exit:
            glfw.destroy_window(interface.viewer.window)
            break

        if count % n_timesteps == 0:
            feedback = interface.get_feedback()
            target_xyz = np.array(
                [
                    np.random.random() * 0.5 - 0.25,
                    np.random.random() * 0.5 - 0.25,
                    np.random.random() * 0.5 + 0.5,
                ]
            )
            R = robot_config.R("EE", q=feedback["q"])
            target_orientation = transformations.euler_from_matrix(R, "sxyz")
            # update the position of the target
            interface.set_mocap_xyz("target", target_xyz)

            # can use 3 different methods to calculate inverse kinematics
            # see inverse_kinematics.py file for details
            path_planner.generate_path(
                position=feedback["q"],
                target_position=np.hstack([target_xyz, target_orientation]),
                method=3,
                dt=0.005,
                n_timesteps=n_timesteps,
                plot=False,
            )

        # returns desired [position, velocity]
        target = path_planner.next()[0]

        # use position control
        print("target angles: ", target[: robot_config.N_JOINTS])
        interface.send_target_angles(target[: robot_config.N_JOINTS])
        interface.viewer.render()

        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")
