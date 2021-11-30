"""
Running the joint controller with an inverse kinematics path planner
using the PyGame display. The path planning system will generate
a trajectory in joint space that moves the end effector in a straight line
to the target, which changes every n time steps.
"""
import numpy as np

from abr_control.arms import threejoint as arm
from abr_control.controllers import Joint, path_planners

# from abr_control.arms import twojoint as arm
from abr_control.interfaces.pygame import PyGame
from abr_control.utils import transformations

# change this flag to False to use position control
use_force_control = True

# initialize our robot config
robot_config = arm.Config()

# create our arm simulation
arm_sim = arm.ArmSim(robot_config)

if use_force_control:
    # create an operational space controller
    ctrlr = Joint(robot_config, kp=300, kv=20)

# create our path planner
n_timesteps = 2000
path_planner = path_planners.InverseKinematics(robot_config)

# create our interface
dt = 0.001
interface = PyGame(robot_config, arm_sim, dt=dt)
interface.connect()
feedback = interface.get_feedback()

try:
    print("\nSimulation starting...")
    print("Click to move the target.\n")

    count = 0
    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        hand_xyz = robot_config.Tx("EE", feedback["q"])

        if count % n_timesteps == 0:
            target_xyz = np.array(
                [np.random.random() * 2 - 1, np.random.random() * 2 + 1, 0]
            )
            R = robot_config.R("EE", q=feedback["q"])
            target_orientation = transformations.euler_from_matrix(R, "sxyz")
            # update the position of the target
            interface.set_target(target_xyz)

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
        target, _ = path_planner.next()

        if use_force_control:
            # generate an operational space control signal
            u = ctrlr.generate(
                q=feedback["q"],
                dq=feedback["dq"],
                target=target,
            )

            # apply the control signal, step the sim forward
            interface.send_forces(u, update_display=(count % 20 == 0))
        else:
            # use position control
            interface.send_target_angles(
                target[: robot_config.N_JOINTS],
                update_display=(count % 20 == 0),
            )

        count += 1

finally:
    # stop and reset the simulation
    interface.disconnect()

    print("Simulation terminated...")
