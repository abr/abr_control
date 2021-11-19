"""
A basic script for connecting and moving the arm to a target
configuration in joint space, offset from its starting position.
The simulation simulates 1500 time steps and then plots the results.
"""
import traceback
import numpy as np

# from abr_control.arms import ur5 as arm
from abr_control.arms import jaco2 as arm

# from abr_control.arms import onejoint as arm
from abr_control.controllers import Joint
from abr_control.interfaces import CoppeliaSim


# initialize our robot config
robot_config = arm.Config()

# instantiate the REACH controller for the jaco2 robot
ctrlr = Joint(robot_config, kp=50)

# create interface and connect
interface = CoppeliaSim(robot_config=robot_config, dt=0.005)
interface.connect()

# make the target an offset of the current configuration
feedback = interface.get_feedback()
target = feedback["q"] + np.ones(robot_config.N_JOINTS) * 0.3

# For CoppeliaSim files that have a shadow arm to show the target configuration
# get joint handles for shadow
names = [f"joint{ii}_shadow" for ii in range(robot_config.N_JOINTS)]
joint_handles = []
for name in names:
    interface.get_xyz(name)  # this loads in the joint handle
    joint_handles.append(interface.misc_handles[name])
# move shadow to target position
interface.send_target_angles(target, joint_handles)

# set up arrays for tracking end-effector and target position
q_track = []


try:
    count = 0
    print("\nSimulation starting...\n")
    while count < 1500:
        # get joint angle and velocity feedback
        feedback = interface.get_feedback()

        # calculate the control signal
        u = ctrlr.generate(
            q=feedback["q"],
            dq=feedback["dq"],
            target=target,
        )

        # send forces into CoppeliaSim, step the sim forward
        interface.send_forces(u)

        # track joint angles
        q_track.append(np.copy(feedback["q"]))
        count += 1

except:
    print(traceback.format_exc())

finally:
    # close the connection to the arm
    interface.disconnect()

    print("Simulation terminated...")

    q_track = np.array(q_track)
    if q_track.shape[0] > 0:
        import matplotlib.pyplot as plt

        plt.plot((q_track + np.pi) % (np.pi * 2) - np.pi)
        plt.gca().set_prop_cycle(None)
        plt.plot(
            np.ones(q_track.shape) * ((target + np.pi) % (np.pi * 2) - np.pi), "--"
        )
        plt.legend(range(robot_config.N_LINKS))
        plt.tight_layout()
        plt.show()
